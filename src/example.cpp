#include "feetech_lib.hpp"

int main()
{
    std::cout << "Feetech Servo Library Example" << std::endl;
    // Create a FeetechServo object
    FeetechServo servo;

    // Initialize the servo object
    std::string port = "/dev/ttyUSB0";
    long baud = 1000000;
    double frequency = 100;
    std::vector<uint8_t> servo_ids = {0x21};
    std::cout << "Initializing servo object..." << std::endl;
    servo.init(port, baud, frequency, servo_ids);


    // Execute the servo object
    double position;
    double new_position;
    int count = 0;
    std::cout << "Starting loop" << std::endl;
    while(true)
    {
        position = servo.getCurrentPositions()[0];
        std::cout << "Current position: " << position << std::endl;
        std::cout << "Enter new position: ";
        std::cin >> new_position;

        servo.setReferencePosition(servo_ids[0], new_position);
    }

    return 1;
}