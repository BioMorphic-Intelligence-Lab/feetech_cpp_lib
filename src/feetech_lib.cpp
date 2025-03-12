#include "feetech_lib.hpp"


#define TICKS_PER_REVOLUTION 4096
#define RADIANS_PER_TICK 0.00153398078 // 2 * pi / TICKS_PER_REVOLUTION
#define TICKS_PER_RADIAN 651.898646904 // TICKS_PER_REVOLUTION / 2 * pi
#define RADIANS_PER_REVOLUTION 6.28318530718
#define DEGREES_PER_TICK 0.087890625 // 360 / TICKS_PER_REVOLUTION
#define AMPERE_PER_TICK 0.0065

namespace instruction
{
    uint8_t const PING_      = 0x01;
    uint8_t const READ       = 0x02;
    uint8_t const WRITE      = 0x03;
    uint8_t const REGWRITE   = 0x04;
    uint8_t const ACTION     = 0x05;
    uint8_t const SYNCWRITE  = 0x83;
    uint8_t const RESET      = 0x06;
};

FeetechServo::FeetechServo(std::string port, long const &baud, const double frequency, const std::vector<uint8_t>& servo_ids) : 
    serial(nullptr), 
    referencePositions_(servo_ids.size()),
    referenceVelocities_(servo_ids.size()),
    referenceAccelerations_(servo_ids.size())
    {
    // Write servo IDs to member data structure
    servoIds_ = servo_ids;

    for (size_t i = 0; i < servoIds_.size(); ++i) {
        idToIndex_[servoIds_[i]] = i;
        gearRatios_.push_back(1.0);
        proportionalGains_.push_back(1.0);
        derivativeGains_.push_back(0.1);
        integralGains_.push_back(0.0);

        referencePositions_[i].store(0);  // Default constructs std::atomic<double>
        referenceVelocities_[i].store(0);
        referenceAccelerations_[i].store(0);

        currentPositions_.push_back(0.0);
        currentVelocities_.push_back(0.0);
        currentTemperatures_.push_back(0.0);
        currentCurrents_.push_back(0.0);
        homePositions_.push_back(0.0);

        servoType_.push_back(ServoType::UNKNOWN);
    }

    /* Open the serial port for communication */
    boost::asio::io_service io;
    try{
        this->serial = new boost::asio::serial_port(io, port);
    }
    catch (const boost::wrapexcept<boost::system::system_error>& e){
        std::cerr << "\033[31m" << "[ERROR] Could not open serial port, is the USB connected?" << "\033[0m" << std::endl;
        exit(1);
    } 
    this->serial->set_option(boost::asio::serial_port_base::baud_rate(baud));
    this->serial->set_option(boost::asio::serial_port_base::character_size(8 /* data bits */));
    this->serial->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    this->serial->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    std::cout << "Serial port opened" << std::endl;

    // Read all data once to populate data structs
    std::cout << "Reading all servo data..." << std::endl;
    bool success=false;
    int fail_counter = 0;
    while(!success)
    {
        success = readAllServoData();
        fail_counter++;
        if (fail_counter>10)
        {
            std::cerr << "\033[31m" << "[ERROR] Failed to read all servo data for 10 attempts. Are all servos connected?" << "\033[0m" << std::endl;
            exit(-1);
        }
    }
    std::cout << "Done reading all servo data..." << std::endl;

    // Controller
    std::cout << "Starting controller..." << std::endl;
    timer_ = std::make_unique<BoostTimer>(frequency, std::bind(&FeetechServo::execute, this));

    // Set all servos to velocity mode and torque enable
}

bool FeetechServo::execute()
{
    // Update current servo data
    readAllServoData();

    // Position mode
    if (settings_.mode == STSMode::POSITION)
    {
        // Calculate rotational error on servo horn
        double servo_rads_to_go;
        double velocity_ref;
        for (size_t i = 0; i < servoIds_.size(); ++i)
        {
            servo_rads_to_go = (referencePositions_[i].load(std::memory_order_relaxed) - currentPositions_[i])*gearRatios_[i];

            velocity_ref = std::clamp(proportionalGains_[i]*servo_rads_to_go + derivativeGains_[i]*currentVelocities_[i], 
                -settings_.max_speed, settings_.max_speed);
            // Set target speed

            if (abs(servo_rads_to_go)<settings_.position_tolerance)
            {
                // Stop servo
                writeTargetVelocity(servoIds_[i], 0, false);
            }
            else
            {
                // Set target velocity
                writeTargetVelocity(servoIds_[i], velocity_ref*gearRatios_[i], false);
            }
        }
    }
    // Velocity mode
    else if (settings_.mode == STSMode::VELOCITY)
    {
        for (size_t i = 0; i < servoIds_.size(); ++i)
        {
            // Set target velocity
            writeTargetVelocity(servoIds_[i], referenceVelocities_[i].load(), true);
        }
    }

    return true;
}

bool FeetechServo::close()
{
    if (this->serial != nullptr)
    {
        this->serial->close();
        delete this->serial;
        this->serial = nullptr;
        return true;
    }
    return false;
}

bool FeetechServo::ping(uint8_t const &servoId)
{
    uint8_t response[1] = {0xFF};
    int send = sendMessage(servoId,
                           instruction::PING_,
                           0,
                           response);
    // Failed to send
    if (send != 6)
        return false;
    // Read response
    int rd = receiveMessage(servoId, 1, response);
    if (rd < 0)
        return false;
    return response[0] == 0x00;
}

void FeetechServo::setDriverSettings(const DriverSettings& settings)
{
    settings_ = settings;
}

bool FeetechServo::readAllServoData()
{
    bool success = true;

    success &= readAllCurrentPositions();
    success &= readAllCurrentSpeeds();
    success &= readAllCurrentCurrents(); 

    if (!success)
        // Make error appear in red
        std::cerr << "\033[31m" << "[ERROR] Failed to read all servo data" << "\033[0m" << std::endl;

    return success;
}


bool FeetechServo::setId(uint8_t const &oldServoId, uint8_t const &newServoId)
{
    if (servoType_[oldServoId] == ServoType::UNKNOWN)
    {
        determineServoType(oldServoId);
    }

    if (oldServoId >= 0xFE || newServoId >= 0xFE)
        return false;
    if (ping(newServoId))
        return false; // address taken

    unsigned char lockRegister = STSRegisters::WRITE_LOCK;
    if (servoType_[oldServoId] == ServoType::SCS)
    {
        lockRegister = STSRegisters::TORQUE_LIMIT; // On SCS, this has been remapped.
    }
    // Unlock EEPROM
    if (!writeRegister(oldServoId, lockRegister, 0))
        return false;
    // Write new ID
    if (!writeRegister(oldServoId, STSRegisters::ID, newServoId))
        return false;
    // Lock EEPROM
    if (!writeRegister(newServoId, lockRegister, 1))
        return false;
    // Update servo type cache.
    servoType_[newServoId] = servoType_[oldServoId];
    servoType_[oldServoId] = ServoType::UNKNOWN;
    return ping(newServoId);
}

bool FeetechServo::setPositionOffset(uint8_t const &servoId, int const &positionOffset)
{
    if (!writeRegister(servoId, STSRegisters::WRITE_LOCK, 0))
        return false;
    // Write new position offset
    if (!writeTwouint8_tsRegister(servoId, STSRegisters::POSITION_CORRECTION, positionOffset))
        return false;
    // Lock EEPROM
    if (!writeRegister(servoId, STSRegisters::WRITE_LOCK, 1))
        return false;
    return true;
}

double FeetechServo::readCurrentPosition(uint8_t const &servoId)
{
    // Calculate wrapped position at servo horn (in radians)
    double previous_wrapped_position = wrap_to_2pi(currentPositions_[idToIndex_[servoId]]*gearRatios_[idToIndex_[servoId]]);
    
    // Get current position at servo horn in radianss
    double position = readTwouint8_tsRegister(servoId, STSRegisters::CURRENT_POSITION)*RADIANS_PER_TICK;
    if (position==0)
        return 0;
    double position_diff = position - previous_wrapped_position;
    double change_rads;

    if (position_diff > 5.5) // 5.5 is a threshold value over which we assume the servo has wrapped around
    {
        change_rads = position_diff - RADIANS_PER_REVOLUTION;
    }
    else if (position_diff < -5.5)
    {
        change_rads = position_diff + RADIANS_PER_REVOLUTION;
    }
    else
    {
        change_rads = position_diff;
    }
    
    // Add change in position to previous absolute position and correct for gear ratio
    currentPositions_[idToIndex_[servoId]] = (previous_wrapped_position + change_rads)/gearRatios_[idToIndex_[servoId]];
    return currentPositions_[idToIndex_[servoId]];
}

bool FeetechServo::readAllCurrentPositions()
{
    bool ret = true;
    double position;

    // Loop over servo IDs and read current position
    for (size_t i = 0; i < servoIds_.size(); ++i)
    {
        std::cout << "Reading position for servo (all current positions)" << servoIds_[i] << std::endl;
        position = readCurrentPosition(servoIds_[i]);
        // If 0 is returned, position is not read correctly, so return value of function becomes false
        if (position == 0)
            ret = false;
    }
    return ret;
}

double FeetechServo::readCurrentSpeed(uint8_t const &servoId)
{
    // Get velocity in counts/second
    int16_t velocity_ticks = readTwouint8_tsRegister(servoId, STSRegisters::CURRENT_SPEED);

    // If 0 is returned, velocity is not read correctly, return and keep old value
    if (velocity_ticks == 0)
        return 0;
    
    double velocity_rads = velocity_ticks*RADIANS_PER_TICK;
    currentVelocities_[idToIndex_[servoId]] = velocity_rads;
    return velocity_rads;
}

bool FeetechServo::readAllCurrentSpeeds()
{
    bool ret = true;
    double velocity;

    // Loop over servo IDs and read current speed
    for (size_t i = 0; i < servoIds_.size(); ++i)
    {
        velocity = readCurrentSpeed(servoIds_[i]);
        // If 0 is returned, velocity is not read correctly, so return value of function becomes false
        if (velocity == 0)
            ret = false;
    }
    return ret;
}

int FeetechServo::readCurrentTemperature(uint8_t const &servoId)
{
    return readTwouint8_tsRegister(servoId, STSRegisters::CURRENT_TEMPERATURE);
}

float FeetechServo::readCurrentCurrent(uint8_t const &servoId)
{
    // Get velocity in counts/second
    int16_t current_ticks = readTwouint8_tsRegister(servoId, STSRegisters::CURRENT_CURRENT);

    // If 0 is returned, velocity is not read correctly, return and keep old value
    if (current_ticks == 0)
        return 0;
    
    double current_amps = current_ticks*AMPERE_PER_TICK;
    currentCurrents_[idToIndex_[servoId]] = current_amps;
    return current_amps;
}

bool FeetechServo::readAllCurrentCurrents()
{
    bool ret = true;
    double current;
    // Loop over servo IDs and read current speed
    for (size_t i = 0; i < servoIds_.size(); ++i)
    {
        current = readCurrentCurrent(servoIds_[i]);
        // If 0 is returned, velocity is not read correctly, so return value of function becomes false
        if (current == 0)
            ret = false;
    }
    return ret;
}

bool FeetechServo::isMoving(uint8_t const &servoId)
{
    uint8_t const result = readRegister(servoId, STSRegisters::MOVING_STATUS);
    return result > 0;
}

bool FeetechServo::writeTargetPosition(uint8_t const &servoId, int const &position, int const &speed, bool const &asynchronous)
{
    uint8_t params[6] = {0, 0, // Position
        0, 0, // Padding
        0, 0}; // Velocity
    convertIntTouint8_ts(servoId, position, &params[0]);
    convertIntTouint8_ts(servoId, speed, &params[4]);
    return writeRegisters(servoId, STSRegisters::TARGET_POSITION, sizeof(params), params, asynchronous);
}

bool FeetechServo::writeTargetVelocity(uint8_t const &servoId, int const &velocity, bool const &asynchronous)
{
    int velocity_ticks = velocity * TICKS_PER_RADIAN;
    return writeTwouint8_tsRegister(servoId, STSRegisters::RUNNING_SPEED, velocity_ticks, asynchronous);
}

bool FeetechServo::writeTargetAcceleration(uint8_t const &servoId, uint8_t const &acceleration, bool const &asynchronous)
{
    return writeRegister(servoId, STSRegisters::TARGET_ACCELERATION, acceleration, asynchronous);
}

bool FeetechServo::writeMode(unsigned char const& servoId, STSMode const& mode)
{
    return writeRegister(servoId, STSRegisters::OPERATION_MODE, static_cast<unsigned char>(mode));
}

void FeetechServo::setReferencePosition(uint8_t const &servoId, double const &position)
{
    referencePositions_[idToIndex_[servoId]].store(position, std::memory_order_relaxed);
}

void FeetechServo::setReferenceVelocity(uint8_t const &servoId, double const &velocity)
{
    referenceVelocities_[idToIndex_[servoId]].store(velocity);
}

void FeetechServo::setReferenceAcceleration(uint8_t const &servoId, double const &acceleration)
{
    referenceAccelerations_[idToIndex_[servoId]].store(acceleration);
}

std::vector<double> FeetechServo::getCurrentPositions()
{
    return currentPositions_;
}

std::vector<double> FeetechServo::getCurrentVelocities()
{
    return currentVelocities_;
}

std::vector<double> FeetechServo::getCurrentTemperatures()
{
    return currentTemperatures_;
}

std::vector<double> FeetechServo::getCurrentCurrents()
{
    return currentCurrents_;
}

STSMode FeetechServo::getOperatingMode()
{
    return settings_.mode;
}


bool FeetechServo::trigerAction()
{
    uint8_t noParam = 0;
    int send = sendMessage(0xFE, instruction::ACTION, 0, &noParam);
    return send == 6;
}

int FeetechServo::sendMessage(uint8_t const &servoId,
    uint8_t const &commandID,
    uint8_t const &paramLength,
    uint8_t *parameters)
{
    std::vector<uint8_t> message(6 + paramLength);
    uint8_t checksum = servoId + paramLength + 2 + commandID;
    message[0] = 0xFF;
    message[1] = 0xFF;
    message[2] = servoId;
    message[3] = paramLength + 2;
    message[4] = commandID;

    for (int i = 0; i < paramLength; i++)
    {
        message[5 + i] = parameters[i];
        checksum += parameters[i];
    }
    message[5 + paramLength] = ~checksum;

    // Todo implement message sending via boost (?)
    int ret = this->writeCommand(message.data(), 6 + paramLength);
    // Give time for the message to be processed.
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    return ret;
}

bool FeetechServo::writeRegisters(uint8_t const &servoId,
                                    uint8_t const &startRegister,
                                    uint8_t const &writeLength,
                                    uint8_t const *parameters,
                                    bool const &asynchronous)
{
    std::vector<uint8_t> param(writeLength + 1);
    param[0] = startRegister;
    for (int i = 0; i < writeLength; i++)
        param[i + 1] = parameters[i];
    int rc = sendMessage(servoId,
                         asynchronous ? instruction::REGWRITE : instruction::WRITE,
                         writeLength + 1,
                         param.data());
    return rc == writeLength + 7;
}

bool FeetechServo::writeRegister(uint8_t const &servoId,
                                   uint8_t const &registerId,
                                   uint8_t const &value,
                                   bool const &asynchronous)
{
    return writeRegisters(servoId, registerId, 1, &value, asynchronous);
}

bool FeetechServo::writeTwouint8_tsRegister(uint8_t const &servoId,
                                           uint8_t const &registerId,
                                           int16_t const &value,
                                           bool const &asynchronous)
{
    uint8_t params[2] = {0, 0};
    convertIntTouint8_ts(servoId, value, params);
    return writeRegisters(servoId, registerId, 2, params, asynchronous);
}

uint8_t FeetechServo::readRegister(uint8_t const &servoId, uint8_t const &registerId)
{
    uint8_t result = 0;
    int rc = readRegisters(servoId, registerId, 1, &result);

    if (rc < 0)
        return 0;
    return result;
}

int16_t FeetechServo::readTwouint8_tsRegister(uint8_t const &servoId, uint8_t const &registerId)
{
    if (servoType_[idToIndex_[servoId]] == ServoType::UNKNOWN)
    {
        determineServoType(servoId);
    }

    unsigned char result[2] = {0, 0};
    int16_t value = 0;
    int16_t signedValue = 0;

    int rc = readRegisters(servoId, registerId, 2, result);

    if (rc < 0)
        return 0;
    switch(servoType_[idToIndex_[servoId]])
    {
        case ServoType::SCS:
            value = static_cast<int16_t>(result[1] +  (result[0] << 8));
            // Bit 15 is sign
            signedValue = value & ~0x8000;
            if (value & 0x8000)
                signedValue = -signedValue;
            return signedValue;
        case ServoType::STS:
            value = static_cast<int16_t>(result[0] +  (result[1] << 8));
            // Bit 15 is sign
            signedValue = value & ~0x8000;
            if (value & 0x8000)
                signedValue = -signedValue;
            return signedValue;
        default:
            return 0;
    }
}

int FeetechServo::readRegisters(uint8_t const &servoId,
                                  uint8_t const &startRegister,
                                  uint8_t const &readLength,
                                  uint8_t *outputBuffer)
{
    uint8_t readParam[2] = {startRegister, readLength};
    // Flush read buffer
    int fd = this->serial->native_handle();
    tcflush(fd, TCIFLUSH);

    // Send read command
    int send = sendMessage(servoId, instruction::READ, 2, readParam);

    // Failed to send
    if (send != 8)
    {
        return -1;
    }
    // Read
    std::vector<uint8_t> result(readLength + 1);
    int rd = receiveMessage(servoId, readLength + 1, result.data());
    if (rd < 0)
    {
        return rd;
    }
    for (int i = 0; i < readLength; i++)
    {
        outputBuffer[i] = result[i + 1];
    }
    return 0;
}

int FeetechServo::receiveMessage(uint8_t const& servoId,
                                   uint8_t const& readLength,
                                   uint8_t *outputBuffer)
{   
    std::vector<uint8_t> result(readLength + 5);
    boost::system::error_code error;
    int serial_timeout_ms = (int)(settings_.tx_time_per_byte * (readLength + 5) + 10);

    // Handler for async read
    size_t rd = 0;
    auto handler = [&](const boost::system::error_code& error, std::size_t bytes_transferred)
      {
        rd = bytes_transferred;
        if (error) {
            this->serial->cancel(); // Stop the io_context from running
            return;
        }
        return;
    };

    size_t bytes_read = 0;
    auto start = std::chrono::high_resolution_clock::now();
    while (bytes_read < (uint8_t)(readLength + 5)) {
        // Check if timeout occurred
        if (std::chrono::high_resolution_clock::now() - start > std::chrono::milliseconds(serial_timeout_ms)) {
            //std::cout << "Timeout occurred in while" << std::endl;
            this->serial->cancel(); // Cancel read operation
            return -1;
        }
        // Wait for data or timeout (non-blocking check)
        this->serial->async_read_some(boost::asio::buffer(result.data() + bytes_read, readLength + 5 - bytes_read), handler);
        // Return invalid message on error
        if (error) {
            std::cout << "Error reading message" << std::endl;
            this->serial->cancel(); // Cancel read operation
            return -2;
        }

        bytes_read += rd;
    }

    // Check if read length is correct
    if (rd != (unsigned short)(readLength + 5))
        return -1;

    // Check message integrity
    if (result[0] != 0xFF || result[1] != 0xFF || result[2] != servoId || result[3] != readLength + 1)
        return -2;

    uint8_t checksum = 0;
    for (int i = 2; i < readLength + 4; i++)
        checksum += result[i];
    checksum = ~checksum;
    if (result[readLength + 4] != checksum)
        return -3;

    // Copy result to output buffer
    for (int i = 0; i < readLength; i++)
        outputBuffer[i] = result[i + 4];

    return 0;
}


void FeetechServo::convertIntTouint8_ts(uint8_t const& servoId, int const &value, uint8_t result[2])
{
    uint16_t servoValue = 0;
    if (servoType_[idToIndex_[servoId]] == ServoType::UNKNOWN)
    {
        determineServoType(servoId);
    }

    // Handle different servo type.
    switch(servoType_[idToIndex_[servoId]])
    {
        case ServoType::SCS:
            // Little endian ; uint8_t 10 is sign.
            servoValue = abs(value);
            if (value < 0)
                servoValue = 0x0400  | servoValue;
            // Invert endianness
            servoValue = (servoValue >> 8) + ((servoValue & 0xFF) << 8);
            break;
        case ServoType::STS:
        default:
            servoValue = abs(value);
            if (value < 0)
                servoValue = 0x8000  | servoValue;
            break;
    }
    result[0] = static_cast<unsigned char>(servoValue & 0xFF);
    result[1] = static_cast<unsigned char>((servoValue >> 8) & 0xFF);
}

void FeetechServo::sendAndUpdateChecksum(uint8_t convertedValue[], uint8_t &checksum)
{
    this->serial->write_some(boost::asio::buffer(convertedValue, 2));
    checksum += convertedValue[0] + convertedValue[1];
}

void FeetechServo::writeTargetPositions(uint8_t const &numberOfServos, const uint8_t servoIds[],
                                        const int positions[],
                                        const int speeds[])
{   
    // Check if number of servos is within limits for SYNCWRITE
    if (numberOfServos > 35)
    {
        throw std::invalid_argument("Too many servos to send in one message. Maximum number of servos for SYNCWRITE is 35.");
    }
    uint8_t servoSpace = numberOfServos * 7 + 4;
    const std::vector<uint8_t> data = {0xFF, 0xFF, 0xFE, servoSpace, instruction::SYNCWRITE, STSRegisters::TARGET_POSITION, 6};
    size_t ret = this->serial->write_some(boost::asio::buffer(data, data.size()));
    if (ret != data.size())
    {
        throw std::runtime_error("Failed to write to serial port.");
    }

    uint8_t checksum = 0xFE + numberOfServos * 7 + 4 + instruction::SYNCWRITE + STSRegisters::TARGET_POSITION + 6;
    uint8_t zeros[2] = {0, 0};
    for (int index = 0; index < numberOfServos; index++)
    {
        checksum += servoIds[index];
        this->serial->write_some(boost::asio::buffer(&servoIds[index], 1));
        uint8_t intAsuint8_t[2];
        convertIntTouint8_ts(servoIds[index], positions[index], intAsuint8_t);
        sendAndUpdateChecksum(intAsuint8_t, checksum);
        this->serial->write_some(boost::asio::buffer(zeros, 2));
        convertIntTouint8_ts(servoIds[index], speeds[index], intAsuint8_t);
        sendAndUpdateChecksum(intAsuint8_t, checksum);
    }
    checksum = ~checksum;
    this->serial->write_some(boost::asio::buffer(&checksum, 1));
}

void FeetechServo::determineServoType(uint8_t const& servoId)
{
    switch(readRegister(servoId, STSRegisters::SERVO_MAJOR))
    {
        case 9: servoType_[idToIndex_[servoId]] = ServoType::STS; break;
        case 5: servoType_[idToIndex_[servoId]] = ServoType::SCS; break;
    }
}

int FeetechServo::writeCommand(const uint8_t *cmd, int cmd_length)
{
    std::size_t ret = this->serial->write_some(boost::asio::buffer(cmd, cmd_length));
    return static_cast<int>(ret);
}

double FeetechServo::wrap_to_2pi(double angle_rad) {
    const double TWO_PI = 2.0 * M_PI;
    
    angle_rad = std::fmod(angle_rad, TWO_PI);
    if (angle_rad < 0)
        angle_rad += TWO_PI;
    return angle_rad;
}