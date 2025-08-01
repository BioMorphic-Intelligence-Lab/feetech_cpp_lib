#include "feetech_lib.hpp"

int main()
{
    std::cout << "Feetech Servo Library Example" << std::endl;
    // Create a FeetechServo object
    

    // Initialize the servo object
    std::string port = "/dev/ttyUSB0";
    long baud = 115200;
    double frequency = 25;
    std::vector<uint8_t> servo_ids = {0x0C};
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

    // Set servo to position mode
    servo.setOperatingMode(servo_ids[0], DriverMode::CONTINUOUS_POSITION);
    std::vector<double> current_position = servo.getCurrentPositions();
    std::cout<< "Got position: " << current_position[0] << std::endl;
    servo.setReferencePosition(servo_ids[0], current_position[0]);

    std::vector<double> current;
    for(int i = 0; i<1000; i++)
    {
        current = servo.getCurrentCurrents();
        //std::cout << "Read current " << current[0] << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    

    // Stop servo
    std::cout << "Stopping servo" << std::endl;
    servo.setReferenceVelocity(servo_ids[0], 0.0);

    // Close serial connection and destroy driver
    return 0;
}