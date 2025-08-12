#include "feetech_lib.hpp"
#include <chrono>
#include <bitset>
#include <iomanip>


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

FeetechServo::FeetechServo(std::string port, long const &baud,
                           const double frequency, const std::vector<uint8_t>& servo_ids,
                           bool homing, bool logging) : 
    serial_(nullptr), 
    servoData_(servo_ids.size()),
    logger_(nullptr)
    {

    // Initialize Driver settings
    settings_.port = port;
    settings_.baud = baud;
    settings_.frequency = frequency;
    settings_.homing = homing;
    settings_.logging = logging;

    for (size_t i = 0; i < servo_ids.size(); ++i) {
        // Write servo IDs to member data structure
        servoData_[i].servoId = servo_ids[i];
        // Setup the map
        idToIndex_[servo_ids[i]] = i;

        servoData_[i].referencePosition.store(0);  // Default constructs std::atomic<double>
        servoData_[i].referenceVelocity.store(0);
        servoData_[i].referenceAcceleration.store(0);
        servoData_[i].previousHornPosition = 0;
        servoData_[i].fullRotation = 0;
        servoData_[i].currentPosition = 0.0;
        servoData_[i].currentVelocity = 0.0;
        servoData_[i].currentTemperature = 0.0;
        servoData_[i].currentCurrent = 0.0;
        servoData_[i].currentPWM = 0.0;
        servoData_[i].homePosition = 0; // In ticks at horn
        servoData_[i].homingMode = 0; // Default no homing

        servoData_[i].gearRatio = 1.0; // From horn to output, i.e. if horn:output = 2:1, gear ratio is 2
        servoData_[i].operatingMode = DriverMode::CONTINUOUS_POSITION; // Initialize in position mode
        servoData_[i].direction = 1; // Default direction is 1
        servoData_[i].maxSpeed = 5.0; // Observed max speed of servo at gear ratio 1
        servoData_[i].servoType = ServoType::UNKNOWN;
    }

    /* Open the serial port for communication */
    try{
        this->io_context_ = new boost::asio::io_context();
        this->serial_ = new boost::asio::serial_port(*io_context_, port);
    }
    catch (const boost::wrapexcept<boost::system::system_error>& e){
        std::cerr << "\033[31m" << "[ERROR] Could not open serial port, is the USB connected and baud rate correct?" << "\033[0m" << std::endl;
        exit(1);
    } 
    this->serial_->set_option(boost::asio::serial_port_base::baud_rate(baud));
    this->serial_->set_option(boost::asio::serial_port_base::character_size(8 /* data bits */));
    this->serial_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    this->serial_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));

    // Create logger if specified
    if (settings_.logging)
    {
        std::cerr << "\033[31m" << "Creating logging file" << "\033[0m" << std::endl;
        this-> logger_ = std::make_shared<ServoSerialLogger>("/ros2_ws/aerial_tactile_servoing/data/serial_log.txt"); // TODO: Where is this file created?
    }
    // Read all data once to populate data structs
    bool success=false;
    int fail_counter = 0;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // Populate the previous horn positions before starting to read data
    for (size_t i = 0; i < servoData_.size(); ++i)
    {
        servoData_[i].previousHornPosition = readCurrentPositionTicks(servo_ids[i]);
    }

    for(int i = 0; i<5; i++)
    {

        std::cout<<"Checking servo connections..."<<std::endl;
        success = readAllServoData();
        if (!success)
            fail_counter++;

        if (fail_counter>10)
        {
            std::cerr << "\033[31m" << "[ERROR] Failed to read all servo data for 10 attempts. Are all servos connected?" << "\033[0m" << std::endl;
            exit(-1);
        }
    }
    // Set servos to velocity at velocity 0 and home, set maximum angle to 0 to enable multi-turn
    for (size_t i = 0; i < servoData_.size(); ++i)
    {
        if (homing)
        {
            // Set position offset to 0 to omit position shift when switching mode
            writePositionOffset(servoData_[i].servoId, 0);
            writeMaxAngle(servoData_[i].servoId, 0);
            writeMinAngle(servoData_[i].servoId, 0);
            resetHomePosition(servoData_[i].servoId);
            while(readCurrentPosition(servoData_[i].servoId)<0); // Make sure you have a good position reading
            writeTargetPosition(servoData_[i].servoId, servoData_[i].homePosition);
            setReferencePosition(servoData_[i].servoId, 0.0);
        }
        else
        {
            setReferencePosition(servoData_[i].servoId, servoData_[i].currentPosition);
        }
        
        writeReturnDelayTime(servoData_[i].servoId, 0);
        writeTorqueEnable(servoData_[i].servoId, true);
        writeMode(servoData_[i].servoId, STSMode::STS_POSITION); // Reset mode to reset internal position counter
        writeMode(servoData_[i].servoId, STSMode::STS_VELOCITY);
        writeMode(servoData_[i].servoId, STSMode::STS_POSITION);
    }
    
    std::cout << "Starting timer "<< std::endl;
    timer_ = std::make_unique<BoostTimer>(frequency, std::bind(&FeetechServo::execute, this));
}

FeetechServo::~FeetechServo()
{
    // Set velocity to zero
    stopAll();

    // Set torque disable
    for (size_t i = 0; i < servoData_.size(); ++i)
    {
        writeTorqueEnable(servoData_[i].servoId, false);
    }
    
    // Close serial port
    close();
    // Delete serial port and io_context
    delete this->serial_;
    delete this->io_context_;
}

bool FeetechServo::execute()
{
    // Update current servo data
    if(readAllServoData())
    {
        // Calculate rotational error on servo horn
        for (size_t i = 0; i < servoData_.size(); ++i)
        {
            // Position mode
            if (servoData_[i].operatingMode == DriverMode::CONTINUOUS_POSITION)
            {
                // std::cout<< "[ID: " << static_cast<int>(servoData_[i].servoId)<<"] "<< "Reference position output in rad " << referencePositions_[i].load(std::memory_order_relaxed) << std::endl;

                int position = static_cast<int>(servoData_[i].direction * 
                    servoData_[i].referencePosition.load(std::memory_order_relaxed) * 
                    servoData_[i].gearRatio * 
                    TICKS_PER_RADIAN + servoData_[i].homePosition);
                writeTargetPosition(
                    servoData_[i].servoId,
                    position,
                    servoData_[i].maxSpeed * servoData_[i].gearRatio * TICKS_PER_RADIAN
                );                
                // std::cout << "[ID: " << static_cast<int>(servoData_[i].servoId)<<"] "<< "Wrote target position as ticks: " << position << std::endl;
            }
            // Velocity mode
            else if (servoData_[i].operatingMode == DriverMode::VELOCITY)
            {
                // Set target velocity
                double velocity = std::clamp(
                    servoData_[i].referenceVelocity.load(std::memory_order_relaxed) * servoData_[i].gearRatio,
                    -servoData_[i].maxSpeed, servoData_[i].maxSpeed);
                writeTargetVelocity(servoData_[i].servoId, velocity, false);
            }
        }
        return true;
    }
    else
    {
        return false;
    }
}

bool FeetechServo::close()
{
    if (this->serial_ != nullptr)
    {
        this->serial_->close();
        delete this->serial_;
        this->serial_ = nullptr;
        return true;
    }
    return false;
}

bool FeetechServo::stopAll()
{
    for (size_t i = 0; i < servoData_.size(); ++i)
    {
        writeTargetVelocity(servoData_[i].servoId, 0, false);
    }
    return true;
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

DriverSettings FeetechServo::getDriverSettings() const
{
    return settings_;
}

bool FeetechServo::readAllServoData()
{
    bool success = true;

    success &= readAllCurrentPositions();
    success &= readAllCurrentSpeeds();
    success &= readAllCurrentCurrents(); 
    success &= readAllCurrentPWMs();

    if (!success)
        // Make error appear in red
        std::cerr << "\033[31m" << "[ERROR] Failed to read all servo data" << "\033[0m" << std::endl;

    return success;
}

bool FeetechServo::writePositionOffset(uint8_t const &servoId, int const &positionOffset)
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

bool FeetechServo::writeReturnDelayTime(uint8_t const &servoId, int const &returnDelayTime)
{
    if (!writeRegister(servoId, STSRegisters::WRITE_LOCK, 0))
        return false;
    // Write new position offset
    if (!writeTwouint8_tsRegister(servoId, STSRegisters::RESPONSE_DELAY, returnDelayTime))
        return false;
    // Lock EEPROM
    if (!writeRegister(servoId, STSRegisters::WRITE_LOCK, 1))
        return false;
    return true;
}

double FeetechServo::readCurrentPosition(uint8_t const &servoId)
{
    int16_t absolute_position_ticks = readTwouint8_tsRegister(servoId, STSRegisters::CURRENT_POSITION);
    double speed = servoData_[idToIndex_[servoId]].currentVelocity;
    int direction = servoData_[idToIndex_[servoId]].direction;

    // Handle errors
    if (absolute_position_ticks == -1)
    {
        std::cerr << "\033[31m" << "[ID "<< static_cast<int>(servoId)<< "] "<<"[ERROR] Failed to read current position (pos == -1)" << "\033[0m" << std::endl;
        return -1;
    }
    else if (absolute_position_ticks == -2)
    {
        std::cerr << "\033[31m" << "[ID "<< static_cast<int>(servoId)<< "] "<< "[ERROR] Failed to read current position (pos == -2)" << "\033[0m" << std::endl;
        return -2;
    }

    // Note: Magic numbers here have been obtained by trial and error and depend on sampling speed
    // Note: Velocity is stored in the specified direction, so needs to be 'un-reversed' when comparing to absolute values
    // Note: Under the hood rotations are always counted in the positive direction, independent of direction setting
    // If speed is sufficiently negative and the new position is sufficiently larger than the previous position
    if(speed * direction < -0.25 && absolute_position_ticks > servoData_[idToIndex_[servoId]].previousHornPosition + 200)
    {
        servoData_[idToIndex_[servoId]].fullRotation--;
    }
    // If speed is sufficiently positive and the new position is sufficiently smaller than the previous position
    else if(speed * direction > 0.25 && absolute_position_ticks + 200 < servoData_[idToIndex_[servoId]].previousHornPosition)
    {
        servoData_[idToIndex_[servoId]].fullRotation++;
    }
    // In case of low speed, if previous position is small compared to current position the horn went backwards
    else if(abs(speed) <= 0.25 && absolute_position_ticks - servoData_[idToIndex_[servoId]].previousHornPosition > 3500)
    {
        servoData_[idToIndex_[servoId]].fullRotation--;
    }
    // In case of low speed, if previosu position is large compared to current position the horn went forwards
    else if(abs(speed) <= 0.25 && absolute_position_ticks - servoData_[idToIndex_[servoId]].previousHornPosition < -3500)
    {
        servoData_[idToIndex_[servoId]].fullRotation++;
    }

    double current_position_rads = (
        (absolute_position_ticks - servoData_[idToIndex_[servoId]].homePosition) 
        + servoData_[idToIndex_[servoId]].fullRotation * TICKS_PER_REVOLUTION) * direction
        * RADIANS_PER_TICK / servoData_[idToIndex_[servoId]].gearRatio;

    // Uncomment for debugging ctrl + /
    // std::cout << "[ID: " << static_cast<int>(servoId)<<"]"<<" Full rotations registered: " << servoData_[idToIndex_[servoId]].fullRotation << " revs "<< std::endl;
    // std::cout << "[ID: " << static_cast<int>(servoId)<<"]"<<" Previous absolute position ticks: " << servoData_[idToIndex_[servoId]].previousHornPosition << " ticks "<< std::endl;
    // std::cout << "[ID: " << static_cast<int>(servoId)<<"]"<<" Current absolute position ticks: " << absolute_position_ticks << " ticks "<< std::endl;
    // std::cout << "[ID: " << static_cast<int>(servoId)<<"]"<<" Current velocity: " << speed << " rad/s "<< std::endl;
    // std::cout << "[ID: " << static_cast<int>(servoId)<<"]"<<" Current position: " << current_position_rads << " rads "<< std::endl;
    servoData_[idToIndex_[servoId]].currentPosition = current_position_rads;

    servoData_[idToIndex_[servoId]].previousHornPosition = absolute_position_ticks;

    return servoData_[idToIndex_[servoId]].currentPosition;
}

int16_t FeetechServo::readCurrentPositionTicks(uint8_t const &servoId)
{

    return readTwouint8_tsRegister(servoId, STSRegisters::CURRENT_POSITION);
}

bool FeetechServo::readAllCurrentPositions()
{
    bool ret = true;
    double position;

    // Loop over servo IDs and read current position
    for (size_t i = 0; i < servoData_.size(); ++i)
    {
        position = readCurrentPosition(servoData_[i].servoId);
        // If 0 is returned, position is not read correctly, so return value of function becomes false
        if (position == -1)
        {
            std::cerr << "\033[31m" << "[ID "<< static_cast<int>(servoData_[i].servoId)<< "] "<< "[ERROR] Failed to read all positions (pos == -1)" << "\033[0m" << std::endl;
            ret = false;
        }
    }
    return ret;
}

double FeetechServo::readCurrentSpeed(uint8_t const &servoId)
{
    // Get velocity in counts/second
    int16_t velocity_ticks = readTwouint8_tsRegister(servoId, STSRegisters::CURRENT_SPEED);

    // If 0 is returned, velocity is not read correctly, return and keep old value
    // TODO fix, if the servo is still this does not hold
    if (velocity_ticks == -1)
        return -1;
    else if (velocity_ticks == -2)
        return -2;
    
    double velocity_rads = velocity_ticks * RADIANS_PER_TICK / servoData_[idToIndex_[servoId]].gearRatio * servoData_[idToIndex_[servoId]].direction;
    servoData_[idToIndex_[servoId]].currentVelocity = velocity_rads;
    return velocity_rads;
}

bool FeetechServo::readAllCurrentSpeeds()
{
    bool ret = true;
    double velocity;

    // Loop over servo IDs and read current speed
    for (size_t i = 0; i < servoData_.size(); ++i)
    {
        velocity = readCurrentSpeed(servoData_[i].servoId);
        // If -1 or -2 is returned, velocity is not read correctly, so return value of function becomes false
        if (velocity == -1 || velocity == -2) // TODO: Do something about the error codes, the velocity can actually be -1 or -2 rad/s, 
        //although this would be very unlikely
        {
            std::cerr << "\033[31m" << "[ID "<< static_cast<int>(servoData_[i].servoId)<< "] "<< "[ERROR] Failed to read all speeds (vel == -1 or -2)"  << "\033[0m" << std::endl;
            ret = false;
        }
    }
    return ret;
}

int FeetechServo::readCurrentTemperature(uint8_t const &servoId)
{
    return readTwouint8_tsRegister(servoId, STSRegisters::CURRENT_TEMPERATURE);
}

float FeetechServo::readCurrentCurrent(uint8_t const &servoId)
{
    // Get current in counts
    uint8_t signBit = 15;
    int16_t current_ticks = readTwouint8_tsRegister(servoId, STSRegisters::CURRENT_CURRENT, signBit);

    // std::cout << "Raw hex: 0x"
    //         << std::setw(4) << std::setfill('0') << std::hex << std::uppercase << current_ticks
    //         << " | Bin: " << std::bitset<16>(current_ticks)
    //         << " | int16_t: " << std::dec << current_ticks
    //         << " | uint16_t: " << current_ticks
    //         << std::endl;

    // If 0 is returned, current is not read correctly, return and keep old value
    if (current_ticks == -1 || current_ticks == -2)
        return current_ticks; // Return the error code
    
    double current_amps = current_ticks*AMPERE_PER_TICK;
    servoData_[idToIndex_[servoId]].currentCurrent = current_amps;
    return current_amps;
}

bool FeetechServo::readAllCurrentCurrents()
{
    bool ret = true;
    double current;
    // Loop over servo IDs and read current speed
    for (size_t i = 0; i < servoData_.size(); ++i)
    {
        current = readCurrentCurrent(servoData_[i].servoId);
        // If 0 is returned, velocity is not read correctly, so return value of function becomes false
        if (current == -1 || current == -2)
        {
            std::cerr << "\033[31m" << "[ID "<< static_cast<int>(servoData_[i].servoId)<< "] "<< "[ERROR] Failed to read all currents (current == -1 or -2)" << "\033[0m" << std::endl;
            ret = false;
        }
    }
    return ret;
}

double FeetechServo::readCurrentPWM(uint8_t const &servoId)
{
    uint8_t signBit = 10; // BIT10 is the direction bit
    int16_t PWM_ticks = readTwouint8_tsRegister(servoId, STSRegisters::CURRENT_DRIVE_VOLTAGE, signBit);

    // std::cout << "Raw hex: 0x"
    //         << std::setw(4) << std::setfill('0') << std::hex << std::uppercase << PWM_ticks
    //         << " | Bin: " << std::bitset<16>(PWM_ticks)
    //         << " | int16_t: " << std::dec << PWM_ticks
    //         << " | uint16_t: " << PWM_ticks
    //         << std::endl;

    // If 0 is returned, PWM is not read correctly, return and keep old value
    if (PWM_ticks == -1 || PWM_ticks == -2)
        return PWM_ticks; // Return the error code

    double PWM_percentage = 0.1 * PWM_ticks;
    servoData_[idToIndex_[servoId]].currentPWM = PWM_percentage;
    // std::cout<< "PWM %: "<< std::setprecision(3) << PWM_percentage << std::endl;
    return PWM_percentage;
}

bool FeetechServo::readAllCurrentPWMs()
{
    bool ret = true;
    double PWM;
    // Loop over servo IDs and read current speed
    for (size_t i = 0; i < servoData_.size(); ++i)
    {
        PWM = readCurrentPWM(servoData_[i].servoId);
        // If 0 is returned, velocity is not read correctly, so return value of function becomes false
        if (PWM == -1 || PWM== -2)
        {
            std::cerr << "\033[31m" << "[ID "<< static_cast<int>(servoData_[i].servoId)<< "] "<< "[ERROR] Failed to read all PWMs (PWM == -1 or -2)" << "\033[0m" << std::endl;
            ret = false;
        }
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
    // std::cout << "[ID: " << static_cast<int>(servoId)<<"] "<< "Writing target position: " << position << std::endl;
    uint8_t params[6] = {0, 0, // Position
        0, 0, // Padding
        0, 0}; // Velocity
    convertIntTouint8_ts(servoId, position, &params[0]);
    convertIntTouint8_ts(servoId, speed, &params[4]);
    return writeRegisters(servoId, STSRegisters::TARGET_POSITION, sizeof(params), params, asynchronous);
}

// If passing a double, assumes rad/s
bool FeetechServo::writeTargetVelocity(uint8_t const &servoId, double const &velocity, bool const &asynchronous)
{
    int velocity_ticks = static_cast<int>(velocity * TICKS_PER_RADIAN * servoData_[idToIndex_[servoId]].gearRatio);
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

bool FeetechServo::writeMinAngle(uint8_t const &servoId, double const &minAngle)
{
    int minPosition_ticks = static_cast<int>(minAngle * TICKS_PER_RADIAN * servoData_[idToIndex_[servoId]].gearRatio);
    return writeTwouint8_tsRegister(servoId, STSRegisters::MINIMUM_ANGLE, minPosition_ticks);
}

bool FeetechServo::writeMaxAngle(uint8_t const &servoId, int16_t const &maxAngle)
{
    return writeTwouint8_tsRegister(servoId, STSRegisters::MAXIMUM_ANGLE, maxAngle);
}

bool FeetechServo::writeTorqueEnable(uint8_t const &servoId, bool const enable)
{
    std::cout<< "[ID: " << static_cast<int>(servoId) << "]: Writing torque enable " << static_cast<int8_t>(enable) << std::endl;
    return writeRegister(servoId, STSRegisters::TORQUE_SWITCH, static_cast<int8_t>(enable));
}

void FeetechServo::setReferencePosition(uint8_t const &servoId, double const &position)
{
    servoData_[idToIndex_[servoId]].referencePosition.store(position, std::memory_order_relaxed);
}

void FeetechServo::setReferenceVelocity(uint8_t const &servoId, double const &velocity)
{
    servoData_[idToIndex_[servoId]].referenceVelocity.store(velocity * servoData_[idToIndex_[servoId]].direction, std::memory_order_relaxed);
}

void FeetechServo::setReferenceAcceleration(uint8_t const &servoId, double const &acceleration)
{
    servoData_[idToIndex_[servoId]].referenceAcceleration.store(acceleration);
}

std::vector<double> FeetechServo::getCurrentPositions()
{
    std::vector<double> currentPositions(servoData_.size(), 0);
    for (size_t i = 0; i < servoData_.size(); ++i)
    {
        currentPositions[i] = servoData_[i].currentPosition;
    }
    return currentPositions;
}

std::vector<double> FeetechServo::getCurrentVelocities()
{
    std::vector<double> currentVelocities(servoData_.size(), 0);
    for (size_t i = 0; i < servoData_.size(); ++i)
    {
        currentVelocities[i] = servoData_[i].currentVelocity;
    }
    return currentVelocities;
}

std::vector<double> FeetechServo::getCurrentTemperatures()
{
    std::vector<double> currentTemperatures(servoData_.size(), 0);
    for (size_t i = 0; i < servoData_.size(); ++i)
    {
        currentTemperatures[i] = servoData_[i].currentTemperature;
    }
    return currentTemperatures;
}

std::vector<double> FeetechServo::getCurrentCurrents()
{
    std::vector<double> currentCurrents(servoData_.size(), 0);
    for (size_t i = 0; i < servoData_.size(); ++i)
    {
        currentCurrents[i] = servoData_[i].currentCurrent;
    }
    return currentCurrents;
}

std::vector<double> FeetechServo::getCurrentPWMs()
{
    std::vector<double> currentPWMs(servoData_.size(), 0);
    for (size_t i = 0; i < servoData_.size(); ++i)
    {
        currentPWMs[i] = servoData_[i].currentPWM;
    }
    return currentPWMs;
}

DriverMode FeetechServo::getOperatingMode(uint8_t const &servoId)
{
    return servoData_[idToIndex_[servoId]].operatingMode;
}

void FeetechServo::setOperatingMode(uint8_t const &servoId, DriverMode const &mode)
{
    int index = idToIndex_[servoId];
    servoData_[index].operatingMode = mode;

    if (mode == DriverMode::VELOCITY)
    {
        // First set zero velocity on the servo
        writeTargetVelocity(servoId, 0.0);
        setReferenceVelocity(servoId, 0.0);
        writeMode(servoId, STSMode::STS_VELOCITY);
    }
    else if (mode == DriverMode::CONTINUOUS_POSITION)
    {
        // First set a suitable target position on the servo
        int position = static_cast<int>(servoData_[index].direction * 
            servoData_[index].currentPosition * 
            servoData_[index].gearRatio * 
            TICKS_PER_RADIAN + servoData_[index].homePosition);
        // std::cout<< "[ID: " << static_cast<int>(servoId)<<"] " << "Current target position ticks: " << position << std::endl;
        // std::cout<< "[ID: " << static_cast<int>(servoId)<<"] " << "Current position as: " << servoData_[index].currentPosition << " rad" << std::endl;
        // std::cout<< "[ID: " << static_cast<int>(servoId)<<"] " << "Current home position ticks: " << servoData_[index].homePosition << std::endl;
        bool res = writeTargetPosition(servoId, 
            position,
            servoData_[index].maxSpeed * servoData_[index].gearRatio * TICKS_PER_RADIAN);
        setReferencePosition(servoId, servoData_[index].currentPosition);
        // Switch mode
        if (res)
        {
            writeMode(servoId, STSMode::STS_POSITION);
            writeMinAngle(servoId, 0); // Set min angle to 0 to dusable multi-turn
            writeMaxAngle(servoId, 0); // Set max angle to 0 to enable multi-turn
            std::cout<< "[ID: " << static_cast<int>(servoId)<<"] " << "Mode succesfully set to continuous position " << mode << std::endl;
        }
    }
    else if (mode == DriverMode::POSITION)
    {
        writeMode(servoId, STSMode::STS_POSITION);
        writeMinAngle(servoId, 0); // Set min angle to 0 to dusable multi-turn
        writeMaxAngle(servoId, 4095); // Set max angle to 4095 to disable multi-turn
        std::cout<< "[ID: " << static_cast<int>(servoId)<<"] " << "Mode succesfully set to velocity " << mode << std::endl;
    }
    else
    {
        std::cerr << "\033[31m" << "[ID "<< static_cast<int>(servoId)<< "] "<< "[ERROR] Invalid operating mode" << "\033[0m" << std::endl;
    }
}

std::vector<DriverMode> FeetechServo::getOperatingModes()
{
    std::vector<DriverMode> modes(servoData_.size(), DriverMode::CONTINUOUS_POSITION);
    for (size_t i = 0; i < servoData_.size(); ++i)
    {
        modes[i] = servoData_[i].operatingMode;
    }
    return modes;
}

void FeetechServo::setOperatingModes(std::vector<DriverMode> const &modes)
{
    for (size_t i = 0; i < modes.size(); ++i)
    {
        servoData_[i].operatingMode = modes[i];
        setOperatingMode(servoData_[i].servoId, modes[i]);
    }
}

double FeetechServo::getGearRatio(uint8_t const &servoId)
{
    return servoData_[idToIndex_[servoId]].gearRatio;
}

void FeetechServo::setGearRatio(uint8_t const &servoId, double const &ratio)
{
    servoData_[idToIndex_[servoId]].gearRatio = ratio;
}

std::vector<double> FeetechServo::getGearRatios()
{
    std::vector<double> ratios(servoData_.size(), 1.0);
    for (size_t i = 0; i < servoData_.size(); ++i)
    {
        ratios[i] = servoData_[i].gearRatio;
    }
    return ratios;
}

void FeetechServo::setGearRatios(std::vector<double> const &ratios)
{
    for(size_t i = 0; i < ratios.size(); ++i)
    {
        servoData_[i].gearRatio = ratios[i];
    }
}

double FeetechServo::getMaxSpeed(uint8_t const &servoId)
{
    return servoData_[idToIndex_[servoId]].maxSpeed;
}

void FeetechServo::setMaxSpeed(uint8_t const &servoId, double const &speed)
{
    servoData_[idToIndex_[servoId]].maxSpeed = speed;
}

std::vector<double> FeetechServo::getMaxSpeeds()
{
    std::vector<double> maxSpeeds(servoData_.size(), -1.0); // Negative value to indicate not set
    for (size_t i = 0; i < servoData_.size(); ++i)
    {
        maxSpeeds[i] = servoData_[i].maxSpeed;
    }
    return maxSpeeds;
}

void FeetechServo::setMaxSpeeds(std::vector<double> const &speeds)
{
    for(size_t i = 0; i < speeds.size(); ++i)
    {
        if (speeds[i] < 0)
        {
            std::cerr << "\033[31m" << "[ID "<< static_cast<int>(servoData_[i].servoId)<< "] "<< "[ERROR] Speed must be positive. Keeping old speed." << "\033[0m" << std::endl;
            continue;
        }
        servoData_[i].maxSpeed = speeds[i];
    }
}

double FeetechServo::getHomePosition(uint8_t const &servoId)
{
    return servoData_[idToIndex_[servoId]].homePosition;
}

void FeetechServo::resetHomePosition(uint8_t const &servoId)
{
    // Make sure position is read correctly
    int16_t current_position = -1;
    while(current_position < 0)
    {
        current_position = readCurrentPositionTicks(servoId);
    }
    // Print setting home position
    std::cout<< "[ID: " << static_cast<int>(servoId)<<"] " << "Setting home position as: " << current_position << std::endl;

    // Assign current position as home
    servoData_[idToIndex_[servoId]].homePosition = current_position;

    // Update servo position with new home
    double current_position_rads = (
        (current_position - servoData_[idToIndex_[servoId]].homePosition) 
        + servoData_[idToIndex_[servoId]].fullRotation * TICKS_PER_REVOLUTION) * servoData_[idToIndex_[servoId]].direction
        * RADIANS_PER_TICK / servoData_[idToIndex_[servoId]].gearRatio;

    servoData_[idToIndex_[servoId]].currentPosition = current_position_rads;
}

std::vector<int16_t> FeetechServo::getHomePositions()
{
    std::vector<int16_t> homePositions(servoData_.size(), 0);
    for (size_t i = 0; i < servoData_.size(); ++i)
    {
        homePositions[i] = servoData_[i].homePosition;
    }
    return homePositions;
}

void FeetechServo::setHomePositions()
{
    for (size_t i = 0; i < servoData_.size(); ++i)
    {
        servoData_[i].homePosition = servoData_[i].currentPosition + servoData_[i].homePosition;
        // Set reference positions to 
        servoData_[i].referencePosition.store(0.0, std::memory_order_relaxed);
        servoData_[i].currentPosition = 0.0;
    }
    // TODO: make current servo data structs atomic because when setting home they can be written by other threads.
    // TODO: Add setting home in servo registers when mode == POSITION  (i.e. not CONTINUOUS_POSITION)
    // TODO: currentPosition in rad vs homePosition in ticks?
}


void FeetechServo::setHomePosition(uint8_t const &servoId, int16_t ticks)
{
    servoData_[idToIndex_[servoId]].homePosition = ticks;
}

int FeetechServo::getVelocityDirection(uint8_t const &servoId)
{
    return servoData_[idToIndex_[servoId]].direction;
}

void FeetechServo::setVelocityDirection(uint8_t const &servoId, int const &direction)
{
    if (direction != 1 && direction != -1)
    {
        std::cerr << "\033[31m" << "[ID "<< static_cast<int>(servoId)<< "] "<< "[ERROR] Direction must be 1 for default and -1 for inverted. Keeping old direction." << "\033[0m" << std::endl;
        return;
    }
    // set direction
    servoData_[idToIndex_[servoId]].direction = direction;

    // update reference velocity
    servoData_[idToIndex_[servoId]].referenceVelocity.store(servoData_[idToIndex_[servoId]].referenceVelocity.load(std::memory_order_relaxed) * direction, std::memory_order_relaxed);
}

std::vector<int> FeetechServo::getVelocityDirections()
{
    std::vector<int> directions(servoData_.size(), 1); // Default direction is 1
    for (size_t i = 0; i < servoData_.size(); ++i)
    {
        directions[i] = servoData_[i].direction;
    }
    return directions;
}

void FeetechServo::setVelocityDirections(std::vector<int> const &directions)
{
    for (size_t i = 0; i < directions.size(); ++i)
    {
        if (directions[i] != 1 && directions[i] != -1)
        {
            std::cerr << "\033[31m" << "[ID "<< static_cast<int>(servoData_[i].servoId)<< "] "<< "[ERROR] Direction must be 1 for default and -1 for inverted. Keeping old directions." << "\033[0m" << std::endl;
            return;
        }
        servoData_[i].direction = directions[i];
    }

    // update reference velocities
    for (size_t i = 0; i < directions.size(); ++i)
    {
        servoData_[i].referenceVelocity.store(servoData_[i].referenceVelocity.load(std::memory_order_relaxed) * servoData_[i].direction, std::memory_order_relaxed);
    }
}

bool FeetechServo::triggerAction(uint8_t const &servoId)
{
    uint8_t noParam = 0;
    int send = sendMessage(servoId, instruction::ACTION, 0, &noParam);
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

int16_t FeetechServo::readTwouint8_tsRegister(uint8_t const &servoId, uint8_t const &registerId, uint8_t signBit)
{
    if (servoData_[idToIndex_[servoId]].servoType == ServoType::UNKNOWN)
    {
        determineServoType(servoId);
    }

    unsigned char result[2] = {0, 0};
    int16_t value = 0;
    int16_t signedValue = 0;

    int rc = readRegisters(servoId, registerId, 2, result);

    if (rc < 0)
        return -1;
    switch(servoData_[idToIndex_[servoId]].servoType)
    {
        case ServoType::SCS:
            if (signBit == 15)
            {
                value = static_cast<int16_t>((result[0] << 8) + result[1] ); // SCS
                // Bit 15 is sign
                signedValue = value & ~0x8000;
                if (value & 0x8000)
                    signedValue = -signedValue;
                return signedValue;
            }
            else if (signBit == 10)
            {
                value = static_cast<int16_t>((result[0] << 8) + result[1] ); // SCS
                // Bit 10 is sign
                signedValue = value & ~0x0400;
                if (value & 0x0400)
                    signedValue = -signedValue;
                return signedValue;
            }
            else {return -3;}
            break;
        case ServoType::STS:
            if (signBit == 15)
            {
                value = static_cast<int16_t>((result[1] << 8) + result[0]); // STS
                // Bit 15 is sign
                signedValue = value & ~0x8000;
                if (value & 0x8000)
                    signedValue = -signedValue;
                return signedValue;
            }
            else if (signBit == 10)
            {
                value = static_cast<int16_t>((result[1] << 8) + result[0]); // STS
                // Bit 10 is sign
                signedValue = value & ~0x0400;
                if (value & 0x0400)
                    signedValue = -signedValue;
                return signedValue;
            }
            else {return -3;}
            break;
        default:
            return -2;
    }
}

int FeetechServo::readRegisters(uint8_t const &servoId,
                                  uint8_t const &startRegister,
                                  uint8_t const &readLength,
                                  uint8_t *outputBuffer)
{
    uint8_t readParam[2] = {startRegister, readLength};
    // Flush read buffer
    int fd = this->serial_->native_handle();
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
    boost::system::error_code read_ec, timer_ec;
    std::size_t bytes_read = 0;

    int serial_timeout_ms = static_cast<int>(settings_.tx_time_per_byte * (readLength + 5) + 25);

    boost::asio::steady_timer timer(*io_context_);
    bool read_done = false, timer_expired = false;

    // Start timer
    timer.expires_after(std::chrono::milliseconds(serial_timeout_ms));
    timer.async_wait([&](const boost::system::error_code& ec) {
        if (!ec) {
            timer_expired = true;
            serial_->cancel(); // Cancel read
        }
    });

    // Start async read
    boost::asio::async_read(
        *serial_,
        boost::asio::buffer(result),
        boost::asio::transfer_exactly(readLength + 5),
        [&](const boost::system::error_code& ec, std::size_t len) {
            read_ec = ec;
            bytes_read = len;
            read_done = true;
            timer.cancel(); // Stop the timer
        });

    io_context_->reset(); // Prepare io_context for a new run
    io_context_->run();

    if (timer_expired) {
        std::cout << "Timeout while reading serial " << serial_timeout_ms << "ms\n";
        return -1;
    }

    if (read_ec || bytes_read != static_cast<long unsigned int>(readLength + 5)) {
        std::cout << "Error during serial read\n";
        return -2;
    }

    // Check message integrity
    if (result[0] != 0xFF || result[1] != 0xFF || result[2] != servoId || result[3] != readLength + 1)
        return -3;

    uint8_t checksum = 0;
    for (int i = 2; i < readLength + 4; i++)
        checksum += result[i];
    checksum = ~checksum;
    if (result[readLength + 4] != checksum)
        return -4;

    // Log the received message
    if (this->logger_) {
        this->logger_->logRx(result);
    }

    // Copy to output buffer
    std::memcpy(outputBuffer, result.data() + 4, readLength);
    return 0;
}


void FeetechServo::convertIntTouint8_ts(uint8_t const& servoId, int const &value, uint8_t result[2])
{
    uint16_t servoValue = 0;
    if (servoData_[idToIndex_[servoId]].servoType == ServoType::UNKNOWN)
    {
        determineServoType(servoId);
    }

    // Handle different servo type.
    switch(servoData_[idToIndex_[servoId]].servoType)
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
    this->serial_->write_some(boost::asio::buffer(convertedValue, 2));
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
        exit(-2);
    }
    uint8_t servoSpace = numberOfServos * 7 + 4;
    const std::vector<uint8_t> data = {0xFF, 0xFF, 0xFE, servoSpace, instruction::SYNCWRITE, STSRegisters::TARGET_POSITION, 6};
    size_t ret = this->serial_->write_some(boost::asio::buffer(data, data.size()));
    if (ret != data.size())
    {
        throw std::runtime_error("Failed to write to serial port.");
        exit(-1);
    }

    uint8_t checksum = 0xFE + numberOfServos * 7 + 4 + instruction::SYNCWRITE + STSRegisters::TARGET_POSITION + 6;
    uint8_t zeros[2] = {0, 0};
    for (int index = 0; index < numberOfServos; index++)
    {
        checksum += servoIds[index];
        this->serial_->write_some(boost::asio::buffer(&servoIds[index], 1));
        uint8_t intAsuint8_t[2];
        convertIntTouint8_ts(servoIds[index], positions[index], intAsuint8_t);
        sendAndUpdateChecksum(intAsuint8_t, checksum);
        this->serial_->write_some(boost::asio::buffer(zeros, 2));
        convertIntTouint8_ts(servoIds[index], speeds[index], intAsuint8_t);
        sendAndUpdateChecksum(intAsuint8_t, checksum);
    }
    checksum = ~checksum;
    this->serial_->write_some(boost::asio::buffer(&checksum, 1));
}

void FeetechServo::determineServoType(uint8_t const& servoId)
{
    uint8_t response = readRegister(servoId, STSRegisters::SERVO_MAJOR);
    switch(response)
    {
        case 10: servoData_[idToIndex_[servoId]].servoType = ServoType::STS; break; // STS3025BL has type 10 for some reason
        case 9: servoData_[idToIndex_[servoId]].servoType = ServoType::STS; break;
        case 5: servoData_[idToIndex_[servoId]].servoType = ServoType::SCS; break;
    }
}

int FeetechServo::writeCommand(const uint8_t *cmd, int cmd_length)
{
    std::size_t ret = this->serial_->write_some(boost::asio::buffer(cmd, cmd_length));

    // Log the data
    if (this->logger_) {
        std::vector<uint8_t> data(cmd, cmd + ret);
        this->logger_->logTx(data);
    }

    return static_cast<int>(ret);
}

double FeetechServo::wrap_to_2pi(double angle_rad)
{
    //return atan2(sin(angle_rad), cos(angle_rad))+M_PI; singularity at 0
    double twoPi = 2*M_PI;
    return angle_rad - twoPi*std::floor(angle_rad/twoPi);
}

double FeetechServo::wrap_to_pi(double angle_rad) 
{
    double twoPi = 2*M_PI;
    return angle_rad - twoPi*std::floor((angle_rad + M_PI)/twoPi);
}