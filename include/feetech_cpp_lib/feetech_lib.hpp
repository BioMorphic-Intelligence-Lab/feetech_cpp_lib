/// \file feetech_lib.hpp
/// \brief A simple driver for the STS-serie TTL servos made by Feetech
///
/// \details This code is meant to work as a minimal example for communicating with
///          the STS servos, in particular the low-cost STS-3215 servo.
///          These servos use a communication protocol identical to
///          the Dynamixel serie, but with a different register mapping
///          (due in part to different functionalities like step mode, multiturn...)
#ifndef FEETECH_LIB_HPP
#define FEETECH_LIB_HPP

#include <boost/asio.hpp>
#include <stdexcept>
#include <chrono>
#include <thread>
#include <atomic>

#include "boost_timer.hpp"
#include "serial_logger.hpp"

namespace STSRegisters
{
    uint8_t const FIRMWARE_MAJOR           = 0x00;
    uint8_t const FIRMWARE_MINOR           = 0x01;
    uint8_t const SERVO_MAJOR              = 0x03;
    uint8_t const SERVO_MINOR              = 0x04;
    uint8_t const ID                       = 0x05;
    uint8_t const BAUDRATE                 = 0x06;
    uint8_t const RESPONSE_DELAY           = 0x07;
    uint8_t const RESPONSE_STATUS_LEVEL    = 0x08;
    uint8_t const MINIMUM_ANGLE            = 0x09;
    uint8_t const MAXIMUM_ANGLE            = 0x0B;
    uint8_t const MAXIMUM_TEMPERATURE      = 0x0D;
    uint8_t const MAXIMUM_VOLTAGE          = 0x0E;
    uint8_t const MINIMUM_VOLTAGE          = 0x0F;
    uint8_t const MAXIMUM_TORQUE           = 0x10;
    uint8_t const UNLOADING_CONDITION      = 0x13;
    uint8_t const LED_ALARM_CONDITION      = 0x14;
    uint8_t const POS_PROPORTIONAL_GAIN    = 0x15;
    uint8_t const POS_DERIVATIVE_GAIN      = 0x16;
    uint8_t const POS_INTEGRAL_GAIN        = 0x17;
    uint8_t const MINIMUM_STARTUP_FORCE    = 0x18;
    uint8_t const CK_INSENSITIVE_AREA      = 0x1A;
    uint8_t const CCK_INSENSITIVE_AREA     = 0x1B;
    uint8_t const CURRENT_PROTECTION_TH    = 0x1C;
    uint8_t const ANGULAR_RESOLUTION       = 0x1E;
    uint8_t const POSITION_CORRECTION      = 0x1F;
    uint8_t const OPERATION_MODE           = 0x21;
    uint8_t const TORQUE_PROTECTION_TH     = 0x22;
    uint8_t const TORQUE_PROTECTION_TIME   = 0x23;
    uint8_t const OVERLOAD_TORQUE          = 0x24;
    uint8_t const SPEED_PROPORTIONAL_GAIN  = 0x25;
    uint8_t const OVERCURRENT_TIME         = 0x26;
    uint8_t const SPEED_INTEGRAL_GAIN      = 0x27;
    uint8_t const TORQUE_SWITCH            = 0x28;
    uint8_t const TARGET_ACCELERATION      = 0x29;
    uint8_t const TARGET_POSITION          = 0x2A;
    uint8_t const RUNNING_TIME             = 0x2C;
    uint8_t const RUNNING_SPEED            = 0x2E;
    uint8_t const TORQUE_LIMIT             = 0x30;
    uint8_t const WRITE_LOCK               = 0x37;
    uint8_t const CURRENT_POSITION         = 0x38;
    uint8_t const CURRENT_SPEED            = 0x3A;
    uint8_t const CURRENT_DRIVE_VOLTAGE    = 0x3C;
    uint8_t const CURRENT_VOLTAGE          = 0x3E;
    uint8_t const CURRENT_TEMPERATURE      = 0x3F;
    uint8_t const ASYNCHRONOUS_WRITE_ST    = 0x40;
    uint8_t const STATUS                   = 0x41;
    uint8_t const MOVING_STATUS            = 0x42;
    uint8_t const CURRENT_CURRENT          = 0x45;
};

enum DriverMode{
    POSITION = 0,
    VELOCITY = 1,
    PWM = 2,
    STEP = 3,
    CONTINUOUS_POSITION = 4
};

enum STSMode{
    STS_POSITION = 0,
    STS_VELOCITY = 1,
    STS_PWM = 2,
    STS_STEP = 3
};

enum ServoType
{
    UNKNOWN = 0,
    STS = 1,
    SCS = 2
};

enum UNITS
{
    COUNTS = 0,
    RAD = 1,
    DEG = 2
};

struct DriverSettings
{
    std::string port = "/dev/ttyUSB0";
    long baud = 1000000;
    double frequency = 100;
    UNITS unit = RAD;
    int max_servos = 35;
    double position_tolerance = 0.01; // rad
    int tx_time_per_byte = 1000./(float)baud*10; // 10 bits per byte for some overhead
    bool debug = false;
    bool logging = false;
};

/// \brief Driver for STS servos, using UART
class FeetechServo
{
public:
    /// \brief Constructor. Initialize the servo driver.
    /// \param port port name, default /dev/ttyUSB0
    /// \param baud Baud rate, default 1Mbps
    /// \param frequency Frequency of the servo driver loop, default 250 Hz
    /// \param servoIds IDs of servos to control, default 1
    FeetechServo(std::string port="/dev/ttyUSB0", long const &baud=1000000, const double frequency=250, const std::vector<uint8_t>& servo_ids = {1}, bool debug = false, bool logging=false);


    /// \brief Destructor. Close the serial port.
    ~FeetechServo();
    
    /// \brief Execute the servo driver loop (to be called in the timer)
    /// \returns True on success
    bool execute();


    /// \brief Ping servo
    /// \param[in] servoId ID of the servo
    /// \return True if servo responded to ping
    bool ping(uint8_t const &servoId);

    /// \brief Close the serial port
    /// \return True on success
    bool close();

    /// \brief Stop all servos, sets velocity to 0
    /// \return True on success
    bool stopAll();

    /// \brief Change the ID of a servo.
    /// \note If the desired ID is already taken, this function does nothing and returns false.
    /// \param[in] oldServoId old servo ID
    /// \param[in] newServoId new servo ID
    /// \return True if servo could successfully change ID
    bool setId(uint8_t const &oldServoId, uint8_t const &newServoId);

    /// \brief Change the position offset of a servo.
    /// \param[in] servoId servo ID
    /// \param[in] positionOffset new position offset
    /// \return True if servo could successfully change position offset
    bool writePositionOffset(uint8_t const &servoId, int const &positionOffset);

    /// \brief Read the position offset of a servo.
    /// \param[in] servoId servo ID
    /// \param[out] positionOffset position offset
    /// \return Position offset, -1 on read failure, -2 on servo type failure.
    bool readPositionOffset(uint8_t const &servoId, int16_t &positionOffset);
    
    /// @brief Set new driver settings.
    /// @param settings 
    void setDriverSettings(const DriverSettings& settings);

    /// @brief Get current driver settings.
    /// @return Current driver settings.
    DriverSettings getDriverSettings() const;

    /****************************************************************************************/
    /********************************* DATA GETTERS *****************************************/
    /****************************************************************************************/

    /// @brief Get current position, speed, and current for all servos.
    /// @return true on success, false on failure
    bool readAllServoData();

    bool readAllCurrentPositions();
    bool readAllCurrentSpeeds();
    bool readAllCurrentTemperatures();
    bool readAllCurrentCurrents(); 

    /// \brief Get current servo position at output.
    /// \note This function assumes that the amplification factor ANGULAR_RESOLUTION is set to 1.
    /// \param[in] servoId ID of the servo
    /// \return Position, in rad, -1 on read failure, -2 on servo type failure.
    double readCurrentPosition(uint8_t const &servoId);

    /// \brief Get current servo position in ticks.
    /// \note This function assumes that the amplification factor ANGULAR_RESOLUTION is set to 1.
    /// \param[in] servoId ID of the servo
    /// \return Position, in ticks, -1 on read failure, -2 on servo type failure.
    int16_t readCurrentPositionTicks(uint8_t const &servoId);

    /// \brief Get current servo speed.
    /// \note This function assumes that the amplification factor ANGULAR_RESOLUTION is set to 1.
    /// \param[in] servoId ID of the servo
    /// \return Speed, in rad/s, -1 on read failure, -2 on servo type failure.
    double readCurrentSpeed(uint8_t const &servoId);

    /// \brief Get current servo temperature.
    /// \param[in] servoId ID of the servo
    /// \return Temperature, in degC, -1 on read failure, -2 on servo type failure.
    int readCurrentTemperature(uint8_t const &servoId);

    /// \brief Get current servo current.
    /// \param[in] servoId ID of the servo
    /// \return Current, in A, -1 on read failure, -2 on servo type failure.
    float readCurrentCurrent(uint8_t const &servoId);

    /// \brief Check if the servo is moving
    /// \param[in] servoId ID of the servo
    /// \return True if moving, false otherwise.
    bool isMoving(uint8_t const &servoId);

    /// \brief Set target servo position.
    /// \note This function assumes that the amplification factor ANGULAR_RESOLUTION is set to 1.
    /// \param[in] servoId ID of the servo
    /// \param[in] position Target position, in counts.
    /// \param[in] speed speed of the servo.
    /// \param[in] asynchronous If set, write is asynchronous (ACTION must be send to activate)
    /// \return True on success, false otherwise.
    bool writeTargetPosition(uint8_t const &servoId, int const &position, int const &speed = 4095, bool const &asynchronous = false);

    /// \brief Set target servo velocity in rad/s.
    /// \note This function assumes that the amplification factor ANGULAR_RESOLUTION is set to 1.
    /// \param[in] servoId ID of the servo
    /// \param[in] velocity Target velocity, in rad/s.
    /// \param[in] asynchronous If set, write is asynchronous (ACTION must be send to activate)
    /// \return True on success, false otherwise.
    bool writeTargetVelocity(uint8_t const &servoId, double const &velocity, bool const &asynchronous= false);

    /// \brief Change the target acceleration of a servo.
    /// \param[in] servoId servo ID
    /// \param[in] acceleration target acceleration
    /// \return True if servo could successfully set target acceleration
    bool writeTargetAcceleration(uint8_t const &servoId, uint8_t const &acceleration, bool const &asynchronous = false);


    /// \brief Set servo working mode: position, velocity or step.
    /// \param[in] servoId ID of the servo
    /// \param[in] mode Desired mode
    bool writeMode(unsigned char const& servoId, STSMode const& mode);

    /// \brief Set the minimum angle of a servo.
    /// \param[in] servoId ID of the servo
    /// \param[in] minAngle Minimum angle in radians
    /// \note This function assumes that the amplification factor ANGULAR_RESOLUTION is set to 1.
    bool writeMinAngle(uint8_t const &servoId, double const &minAngle);

    /// \brief Set the maximum angle of a servo.
    /// \param[in] servoId ID of the servo
    /// \param[in] maxAngle Maximum angle in radians
    /// \note This function assumes that the amplification factor ANGULAR_RESOLUTION is set to 1.
    bool writeMaxAngle(uint8_t const &servoId, int16_t const &maxAngle);

    /// @brief Set torque enable
    /// @param servoId ID of the servo
    /// @param enable True for enabling, false for disabling
    /// @return True if success
    bool writeTorqueEnable(uint8_t const &servoId, bool const enable);

    /// \brief Trigger the action previously stored by an asynchronous write on all servos.
    /// \return True on success
    bool trigerAction();

    /// \brief Write to a single uint8_t register.
    /// \param[in] servoId ID of the servo
    /// \param[in] registerId Register id.
    /// \param[in] value Register value.
    /// \param[in] asynchronous If set, write is asynchronous (ACTION must be send to activate)
    /// \return True if write was successful
    bool writeRegister(uint8_t const &servoId,
                       uint8_t const &registerId,
                       uint8_t const &value,
                       bool const &asynchronous = false);

    /// \brief Write a two-uint8_ts register.
    /// \param[in] servoId ID of the servo
    /// \param[in] registerId Register id (LSB).
    /// \param[in] value Register value.
    /// \param[in] asynchronous If set, write is asynchronous (ACTION must be send to activate)
    /// \return True if write was successful
    bool writeTwouint8_tsRegister(uint8_t const &servoId,
                               uint8_t const &registerId,
                               int16_t const &value,
                               bool const &asynchronous = false);

    /// \brief Read a single register
    /// \param[in] servoId ID of the servo
    /// \param[in] registerId Register id.
    /// \return Register value, 0 on failure.
    uint8_t readRegister(uint8_t const &servoId, uint8_t const &registerId);

    /// \brief Read two uint8_ts, interpret result as <LSB> <MSB>
    /// \param[in] servoId ID of the servo
    /// \param[in] registerId LSB register id.
    /// \return Register value, -1 on read failure, -2 on servo type failure.
    int16_t readTwouint8_tsRegister(uint8_t const &servoId, uint8_t const &registerId);

    /// @brief Sets the target positions for multiple servos simultaneously.
    /// @param[in] NumberOfServos Number of servo.
    /// @param[in] servoIds Array of servo IDs to control.
    /// @param[in] positions Array of target positions (corresponds to servoIds).
    /// @param[in] speeds Array of target speeds (corresponds to servoIds).
    void writeTargetPositions(uint8_t const &numberOfServos,
                            const uint8_t servoIds[],
                            const int positions[],
                            const int speeds[]);
    
    /// @brief Set the reference position for a servo.
    /// @param[in] servoId ID of the servo
    /// @param[in] position Reference absolute position in radians
    void setReferencePosition(uint8_t const &servoId, double const &position);

    /// @brief Set the reference velocity for a servo.
    /// @param[in] servoId ID of the servo
    /// @param[in] velocity Reference velocity in rad/s
    void setReferenceVelocity(uint8_t const &servoId, double const &velocity);

    /// @brief Set the reference acceleration for a servo.
    /// @param[in] servoId ID of the servo
    /// @param[in] acceleration Reference velocity in rad/s^2
    void setReferenceAcceleration(uint8_t const &servoId, double const &acceleration);

    std::vector<double> getCurrentPositions();

    std::vector<double> getCurrentVelocities();

    std::vector<double> getCurrentTemperatures();
    
    std::vector<double> getCurrentCurrents();

    // Operating modes
    DriverMode getOperatingMode(uint8_t const &servoId);

    void setOperatingMode(uint8_t const &servoId, DriverMode const &mode);

    std::vector<DriverMode> getOperatingModes();

    void setOperatingModes(std::vector<DriverMode> const &modes);

    // Gear ratios
    double getGearRatio(uint8_t const &servoId);

    void setGearRatio(uint8_t const &servoId, double const &ratio);

    std::vector<double> getGearRatios();

    void setGearRatios(std::vector<double> const &ratios);
    
    // Max speeds
    double getMaxSpeed(uint8_t const &servoId);

    void setMaxSpeed(uint8_t const &servoId, double const &speed);

    std::vector<double> getMaxSpeeds();

    void setMaxSpeeds(std::vector<double> const &speeds);

    /// @brief Get the reference position for a servo.
    /// @param[in] servoId ID of the servo
    /// @return Absolute home position in radians
    double getHomePosition(uint8_t const &servoId);

    /// @brief Set the current position as servo home position. Adjust current positions and reference positions 
    ///         to account for new home.
    /// @param[in] servoId ID of the servo
    void resetHomePosition(uint8_t const &servoId);

    /// @brief Get the home positions for all servos.
    /// @return Vector of home positions in radians.
    std::vector<int16_t> getHomePositions();

    /// @brief Set the current positions for all servos as their home positions. Adjust current positions and reference positions
    ///         to account for new home.
    void setHomePositions();

    int getVelocityDirection(uint8_t const &servoId);

    void setVelocityDirection(uint8_t const &servoId, int const &direction);

    std::vector<int> getVelocityDirections();

    void setVelocityDirections(std::vector<int> const &directions);

private:
    /// \brief Send a message to the servos.
    /// \param[in] servoId ID of the servo
    /// \param[in] commandID Command id
    /// \param[in] paramLength length of the parameters
    /// \param[in] parameters parameters
    /// \return Result of write.
    int sendMessage(uint8_t const &servoId,
                    uint8_t const &commandID,
                    uint8_t const &paramLength,
                    uint8_t *parameters);

    /// \brief Recieve a message from a given servo.
    /// \param[in] servoId ID of the servo
    /// \param[in] readLength Message length
    /// \param[in] paramLength length of the parameters
    /// \param[in] outputBuffer Buffer where the data is placed.
    /// \return 0 on success
    ///         -1 if read failed due to timeout
    ///         -2 if invalid message (no 0XFF, wrong servo id)
    ///         -3 if invalid checksum
    int receiveMessage(uint8_t const &servoId,
                       uint8_t const &readLength,
                       uint8_t *outputBuffer);
    
    size_t read_bytes(std::vector<char>& buffer, std::size_t bytes_to_read);

    /// \brief Write to a sequence of consecutive registers
    /// \param[in] servoId ID of the servo
    /// \param[in] startRegister First register
    /// \param[in] writeLength Number of registers to write
    /// \param[in] parameters Value of the registers
    /// \param[in] asynchronous If set, write is asynchronous (ACTION must be send to activate)
    /// \return True if write was successful
    bool writeRegisters(uint8_t const &servoId,
                        uint8_t const &startRegister,
                        uint8_t const &writeLength,
                        uint8_t const *parameters,
                        bool const &asynchronous = false);

    /// \brief Read a sequence of consecutive registers.
    /// \param[in] servoId ID of the servo
    /// \param[in] startRegister First register
    /// \param[in] readLength Number of registers to write
    /// \param[out] outputBuffer Buffer where to read the data (must have been allocated by the user)
    /// \return 0 on success, -1 if write failed, -2 if read failed, -3 if checksum verification failed
    int readRegisters(uint8_t const &servoId,
                      uint8_t const &startRegister,
                      uint8_t const &readLength,
                      uint8_t *outputBuffer);

    /// @brief Send two uint8_ts and update checksum
    /// @param[in] convertedValue Converted int value
    /// @param[out] checksum Update the checksum
    void sendAndUpdateChecksum(uint8_t convertedValue[], uint8_t &checksum);

    /// @brief Convert int to pair of uint8_ts
    /// @param[in] value
    /// @param[out] result
    void convertIntTouint8_ts(uint8_t const& servoId, int const &value, uint8_t result[2]);

    /// \brief Determine servo type (STS or SCS, they don't use exactly the same protocol)
    void determineServoType(uint8_t const& servoId);
    
    /// @brief Write a command to the serial port
    /// @param[in] cmd Command to write
    /// @return Number of uint8_ts written
    int writeCommand(const uint8_t *cmd, int cmd_length);
    
    /// @brief Wrap angle to 2pi
    /// @param[in] angle_rad Angle in radians
    /// @return Wrapped angle in radians
    double wrap_to_2pi(double angle_rad);

    /// @brief Wrap angle to -pi to pi
    /// @param[in] angle_rad Angle in radians
    /// @return Wrapped angle in radians
    double wrap_to_pi(double angle_rad);

    boost::asio::io_context* io_context_;
    boost::asio::serial_port* serial_;

    std::unique_ptr<BoostTimer> timer_;
    
    // Servo data
    std::vector<uint8_t> servoIds_; // IDs of servos to control
    std::unordered_map<int, size_t> idToIndex_; // Map of servo IDs to index in servoIds_
    std::vector<double> gearRatios_;
    std::vector<uint16_t> previousHornPositions_;
    std::vector<int> fullRotations_;

    // Use pointers because vectors cannot be made atomic
    std::vector<std::atomic<double>> referencePositions_;
    std::vector<std::atomic<double>> referenceVelocities_;
    std::vector<std::atomic<double>> referenceAccelerations_;
    std::vector<double> currentPositions_;
    std::vector<double> currentVelocities_;
    std::vector<double> currentTemperatures_;
    std::vector<double> currentCurrents_;
    std::vector<int16_t> homePositions_; // In ticks at horn

    // Servo settings
    std::vector<DriverMode> operatingModes_;
    std::vector<double> maxSpeeds_;
    std::vector<int> directions_;

    std::vector<ServoType> servoType_; // Map of servo types - STS/SCS servos have slightly different protocol.

    DriverSettings settings_;

    std::shared_ptr<ServoSerialLogger> logger_;
};

#endif