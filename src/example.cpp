#include "feetech_lib.hpp"

void main()
{
    // Create a FeetechServo object
    FeetechServo servo;

    // Initialize the servo object
    std::string port = "/dev/ttyUSB0";
    long baud = 1000000;
    double frequency = 100;
    std::vector<uint8_t> servo_ids = {0x01};
    servo.init(port, baud, frequency, servo_ids);

    // Execute the servo object
    double position;
    double new_position;
    int count = 0;
    while(true)
    {
        position = servo.getCurrentPositions()[0];
        std::cout << "Current position: " << position << std::endl;
        std::cout << "Enter new position: ";
        std::cin >> new_position;

        servo.setReferencePosition(servo_ids[0], new_position);
    }
}