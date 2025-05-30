#include "feetech_lib.hpp"

int main()
{
    std::cout << "Feetech Servo Library Example" << std::endl;
    // Create a FeetechServo object
    

    // Initialize the servo object
    std::string port = "/dev/ttyUSB0";
    long baud = 1000000;
    double frequency = 25;
    std::vector<uint8_t> servo_ids = {0x21};
    std::cout << "Initializing servo object..." << std::endl;
    FeetechServo servo(port, baud, frequency, servo_ids, false);
    // Set driver to velocity mode
    std::cout << "Setting driver to velocity mode" << std::endl;
    servo.setOperatingMode(servo_ids[0], DriverMode::VELOCITY);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // Execute the servo object
    int sleep = 3000;

    // Move servo to 0.5 rad/s
    std::cout << "Moving servo to 0.5 rad/s" << std::endl;
    double velocity = 0.5;
    servo.setReferenceVelocity(servo_ids[0], velocity);   
    std::this_thread::sleep_for(std::chrono::milliseconds(sleep));
    
    // Move servo at 1.5 rad/s
    std::cout << "Moving servo at 1.5 rad/s" << std::endl;
    velocity = 1.5;
    servo.setReferenceVelocity(servo_ids[0], velocity);
    std::this_thread::sleep_for(std::chrono::milliseconds(sleep));

    // Move servo at -1.0 rad/s
    std::cout << "Moving servo at -1.0 rad/s" << std::endl;
    velocity = -1.0;
    servo.setReferenceVelocity(servo_ids[0], velocity);
    std::this_thread::sleep_for(std::chrono::milliseconds(sleep));

    // Change direction
    std::cout << "Changing direction" << std::endl;
    servo.setVelocityDirection(servo_ids[0], -1);
    std::this_thread::sleep_for(std::chrono::milliseconds(sleep));

    // Move servo at 0.5 rad/s
    std::cout << "Moving servo at 0.5 rad/s" << std::endl;
    velocity = 0.5;
    servo.setReferenceVelocity(servo_ids[0], velocity);
    std::this_thread::sleep_for(std::chrono::milliseconds(sleep));
    

    // Stop servo
    std::cout << "Stopping servo" << std::endl;
    servo.setReferenceVelocity(servo_ids[0], 0);

    // Close serial connection and destroy driver
    servo.close();
    return 0;
}