#include "feetech_lib.hpp"

int main()
{
    std::cout << "Feetech Servo Library Example" << std::endl;
    // Create a FeetechServo object
    

    // Initialize the servo object
    std::string port = "/dev/ttyUSB0";
    long baud = 1000000;
    double frequency = 50;
    std::vector<uint8_t> servo_ids = {0x21};
    std::cout << "Initializing servo object..." << std::endl;
    FeetechServo servo(port, baud, frequency, servo_ids, false);

    // Execute the servo object
    double position;
    double new_position;
    int sleep = 7000;

    std::this_thread::sleep_for(std::chrono::milliseconds(sleep));

    // Move servo to 0 rad
    std::cout << "Moving servo to 0 rad" << std::endl;
    position = 0.0;
    servo.setReferencePosition(servo_ids[0], position);
    std::this_thread::sleep_for(std::chrono::milliseconds(sleep));

    // Move servo to pi/2 rad
    std::cout << "Moving servo to 3pi rad" << std::endl;
    position = 3.14159*3.0;
    servo.setReferencePosition(servo_ids[0], position);
    std::this_thread::sleep_for(std::chrono::milliseconds(sleep));

    // Set home at pi/2 rad
    std::cout << "Setting home position at 3pi rad" << std::endl;
    servo.setHomePosition(servo_ids[0]);
    std::this_thread::sleep_for(std::chrono::milliseconds(sleep));

    // Move servo to -pi/2 rad
    std::cout << "Moving servo to 0 rad" << std::endl;
    position = 0.0;
    servo.setReferencePosition(servo_ids[0], position);
    std::this_thread::sleep_for(std::chrono::milliseconds(sleep));
    
    // Move servo to pi/2 rad
    std::cout << "Moving servo to pi/2 rad" << std::endl;
    position = 1.5708;
    servo.setReferencePosition(servo_ids[0], position);
    std::this_thread::sleep_for(std::chrono::milliseconds(sleep));
    

    // Close serial connection and destroy driver
    servo.close();
    return 0;
}