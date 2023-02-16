#include "main.h"
#include "pros/adi.hpp"
#include "pros/llemu.hpp"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include <cmath>
#include <iostream>
#include <string>

#define LEFT_MTR1_PORT 1
#define LEFT_MTR2_PORT 2
#define LEFT_MTR3_PORT 3
#define LEFT_MTR4_PORT 14
#define RIGHT_MTR1_PORT 8
#define RIGHT_MTR2_PORT 9
#define RIGHT_MTR3_PORT 10
#define RIGHT_MTR4_PORT 6
#define SLIPGEAR_BUMPER 'h'
#define LEFT_CATAPULT_PORT 11
#define RIGHT_CATAPULT_PORT 12
#define INTAKE_PORT_1 5
#define INTAKE_PORT_2 7
#define CATAPULT_MAX 600
#define LAUNCHER_PORT 'g'
#define GYRO_PORT 13
#define VISION_PORT 19
enum intakeDirection { intake, outtake, stopped };
intakeDirection intakeState = stopped;

enum turnType{left, right};
enum direction{forward, backward};
enum color{red, blue};
enum intakeSetting{on, off};



    pros::Controller controller(pros::E_CONTROLLER_MASTER);
    pros::Motor right_mtr1(RIGHT_MTR1_PORT, true);
    pros::Motor right_mtr2(RIGHT_MTR2_PORT);
    pros::Motor right_mtr3(RIGHT_MTR3_PORT,true);
    pros::Motor right_mtr4(RIGHT_MTR4_PORT);
    pros::Motor left_mtr1(LEFT_MTR1_PORT);
    pros::Motor left_mtr2(LEFT_MTR2_PORT,true);
    pros::Motor left_mtr3(LEFT_MTR3_PORT);
    pros::Motor left_mtr4(LEFT_MTR4_PORT, true);
    pros::Motor left_catapult(LEFT_CATAPULT_PORT,MOTOR_GEAR_RED);
    pros::Motor right_catapult(RIGHT_CATAPULT_PORT,MOTOR_GEAR_RED,true);
    pros::Motor Intake_1(INTAKE_PORT_1);
    pros::Motor Intake_2(INTAKE_PORT_2, true);
    pros::Imu gyro (GYRO_PORT);
    pros::ADIDigitalOut launcher(LAUNCHER_PORT);
    pros::Optical vision(VISION_PORT);
    
    //Bumpers and switches example
    pros::ADIDigitalIn SlipGearSensor (SLIPGEAR_BUMPER);

    bool launcherState = true;
    int cataFlagAuto = 1;
    
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
    static bool pressed = false;
    pressed = !pressed;
    if (pressed) {
        pros::lcd::set_text(2, "I was pressed!");
    } else {
        pros::lcd::clear_line(2);
    }
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize();

    /*
    Sets necessary motor values to start, like locking the catapult motor, resetting all motor positions
    to zero, and defining rotation units of the motor
    */

    pros::c::motor_tare_position(LEFT_CATAPULT_PORT);
    pros::c::motor_tare_position(RIGHT_CATAPULT_PORT);
    
    pros::c::motor_set_encoder_units(LEFT_CATAPULT_PORT, pros::E_MOTOR_ENCODER_ROTATIONS);
    pros::c::motor_set_encoder_units(RIGHT_CATAPULT_PORT, pros::E_MOTOR_ENCODER_ROTATIONS);
    
    left_catapult.set_brake_mode(MOTOR_BRAKE_HOLD);
    right_catapult.set_brake_mode(MOTOR_BRAKE_HOLD);
    Intake_1.set_brake_mode(MOTOR_BRAKE_COAST);
    Intake_2.set_brake_mode(MOTOR_BRAKE_COAST);

    left_mtr1.tare_position();
    left_mtr2.tare_position();
    left_mtr3.tare_position();
    right_mtr1.tare_position();
    right_mtr2.tare_position();
    right_mtr3.tare_position();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {

}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}


void tareMotors(){
    // Resets all drive train motor positions to 0
    left_mtr1.tare_position();
    left_mtr2.tare_position();
    left_mtr3.tare_position();
    left_mtr4.tare_position();
    right_mtr1.tare_position();
    right_mtr2.tare_position();
    right_mtr3.tare_position();
    right_mtr4.tare_position();
}

void drive(int power, float distance){
    // Resets motor position to ensure accuracy in autonomous
    tareMotors();
    // Drives specified distance
    // Multiplies distance by the conversion factor for inches
	left_mtr1.move_absolute(distance*13.37*2.5 - 5, power);
	left_mtr2.move_absolute(distance*13.37*2.5 - 5, power);
	left_mtr3.move_absolute(distance*13.37*2.5 - 5, power);
	left_mtr4.move_absolute(distance*13.37*2.5 - 5, power);
	right_mtr1.move_absolute(distance*13.37*2.5, power);
	right_mtr2.move_absolute(distance*13.37*2.5, power);
	right_mtr3.move_absolute(distance*13.37*2.5, power);
	right_mtr4.move_absolute(distance*13.37*2.5, power);
	pros::lcd::set_text(3, "driving");
    // Waits a second to allow motors to build speed
	pros::delay(1000);
    // Continue running this loop as long as the motor is moving, locking the program in the function
	while (!(left_mtr1.get_actual_velocity() == 0)) {
        pros::delay(10);
        // Prints motor voltage values
		pros::lcd::set_text(4, std::to_string((left_mtr1.get_voltage()+left_mtr2.get_voltage()+left_mtr3.get_voltage()+left_mtr4.get_voltage())/4));
		pros::lcd::set_text(5, std::to_string((right_mtr1.get_voltage()+right_mtr2.get_voltage()+right_mtr3.get_voltage()+right_mtr4.get_voltage())/4));
    }
	pros::lcd::set_text(3, "done driving");
}


void turn(int32_t deg, double precision) {
    // PID loop to control turning
    /* 
    Turntype is legacy code that used to determine if the robot was turning left or right,
    this code was bugged to where left did not work so we defaulted to turning right, and
    using negative values to turn left
    */
    turnType dir = right;
    tareMotors();
    // Gets initial gyro position
    float initialValue = gyro.get_rotation();
    // Sets error to the degree that we are turning to, degree is based off starting rotation at 
    // the beginning of the match, not current rotation
    float error = deg - initialValue;
    float prevError = error;
    float totalError = 0;
    const float threshold = precision;
    const float kp = 1.45; //was 1.4
    const float ki = .7; //was .3
    const float kd = .78; //was .8
    std::string first = std::to_string(gyro.get_rotation());
    pros::lcd::set_text(7, "");
    pros::lcd::set_text(4, "Gyro Value: " + first);
    // Loops PID until the error is within the threshold for 2 cycles
    while (fabs(error) > threshold || fabs(prevError) > threshold) {
        int speed = (kp * error + kd * (error - prevError) + ki * totalError) * 9 / 10;
        // Move motors
        left_mtr1.move(dir == left ? -speed : speed);
        left_mtr2.move(dir == left ? -speed : speed);
        left_mtr3.move(dir == left ? -speed : speed);
        left_mtr4.move(dir == left ? -speed : speed);
        right_mtr1.move(dir == right ? -speed : speed);
        right_mtr2.move(dir == right ? -speed : speed);
        right_mtr3.move(dir == right ? -speed : speed);
        right_mtr4.move(dir == right ? -speed : speed);
        pros::delay(200);
        prevError = error;
        std::string second = std::to_string((float)gyro.get_rotation());
        pros::lcd::set_text(5, "Gyro Value In while: " + second);
        // Updates new error value
        error = deg - (gyro.get_rotation());
        std::string errorInWhile = std::to_string(error);
        pros::lcd::set_text(6, "Error Value: " + errorInWhile);
        // Increments total error while error is less than 20, allows for fine turns
        totalError += (fabs(error) < 20 ? error : 0); // was 10
        // Flips direction of total error and reduces it if the goal is overshot
        totalError *= (std::signbit(error) == std::signbit(prevError)) ? 1 : -3/4;
  }
    pros::lcd::set_text(7, "I'm out of PID");
    right_mtr1.brake();
    right_mtr2.brake();
    right_mtr3.brake();
    right_mtr4.brake();
    left_mtr1.brake();
    left_mtr2.brake();
    left_mtr3.brake();
    left_mtr4.brake();
}

color get_color(double hue) {
    // Returns red or blue based off of the hue value seen by the vision sensor
    return (hue > 300 || hue < 20) ? red : blue;
}

void runRoller(int speed = 75){
    // Scores roller if the roller is at competition start position
    // Turns on flashlight
    vision.set_led_pwm(100);
    pros::lcd::set_text(4, std::to_string(vision.get_hue()));

    // Slowly backs into roller
    left_mtr1 = -20;
    left_mtr2 = -20;
    left_mtr3 = -20;
    left_mtr4 = -20;
    right_mtr1 = -20;
    right_mtr2 = -20;
    right_mtr3 = -20;
    right_mtr4 = -20;

    int counter = 0;
    bool startColor;
    startColor = get_color(vision.get_hue());

    // Spins roller until it sees a different color than what it started at    

    while (startColor == get_color(vision.get_hue()) && counter < 200) {
        Intake_1.move(speed);
        Intake_2.move(speed);
        counter++;
        pros::delay(10);
    }

    // Stops the intake, robot movement, and turns off flashlight
    Intake_1.brake();
    Intake_2.brake();
    left_mtr1 = 0;
    left_mtr2 = 0;
    left_mtr3 = 0;
    left_mtr4 = 0;
    right_mtr1 = 0;
    right_mtr2 = 0;
    right_mtr3 = 0;
    right_mtr4 = 0;
    vision.set_led_pwm(0);
}

// Scores roller in skills autonomous
void skills_roller(int speed = -50) {
    // Turns on flashlight and slowly backs into roller
    vision.set_led_pwm(100);
    pros::lcd::set_text(4, std::to_string(vision.get_hue()));
    left_mtr1 = -20;
    left_mtr2 = -20;
    left_mtr3 = -20;
    left_mtr4 = -20;
    right_mtr1 = -20;
    right_mtr2 = -20;
    right_mtr3 = -20;
    right_mtr4 = -20;
    int counter = 0;
    
    // 2 while loops spin until color changes twice
    while(get_color(vision.get_hue()) == red && counter < 200) {
        Intake_1.move(speed);
        Intake_2.move(speed);
        counter++;
        pros::delay(10);
    }

    while(get_color(vision.get_hue()) != red && counter < 200) {
        Intake_1.move(speed);
        Intake_2.move(speed);
        counter++;
        if (counter == 80)
            speed += 40;
        pros::delay(10);
    }

    // Stops the intake, robot movement, and turns off flashlight
    Intake_1.brake();
    Intake_2.brake();
    left_mtr1 = 0;
    left_mtr2 = 0;
    left_mtr3 = 0;
    left_mtr4 = 0;
    right_mtr1 = 0;
    right_mtr2 = 0;
    right_mtr3 = 0;
    right_mtr4 = 0;
    vision.set_led_pwm(0);
}

void shoot(){
    // Moves catapult for 1.5 seconds, which launches the catapult
    left_catapult.move_velocity(600);
    right_catapult.move_velocity(600);
    pros::delay(1500);
    // Keeps spinning the catapult until button is pressed
    while (!SlipGearSensor.get_value()) {
        left_catapult.move_velocity(600);
        right_catapult.move_velocity(600);
    } 
    left_catapult.brake();
    right_catapult.brake();
}

void intakeSetting(intakeSetting setting) {
    // Turns intake either on or off
    if (setting == off) {
        Intake_1 = 0;
        Intake_2 = 0;
    }
    else if (setting == on) {
        Intake_1 = -112;
        Intake_2 = -112;
    }
}

void startCatapult() {
    // Starts catapult at start of match
    while (!SlipGearSensor.get_value()) {
        left_catapult.move_velocity(600);
        right_catapult.move_velocity(600);
    } 
    left_catapult.brake();
    right_catapult.brake();
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

void autonomous() {
    int cataFlag;
    // For testing turn
    gyro.set_rotation(0);
    while(gyro.is_calibrating()) {
        pros::delay(10);
    };

    while (!SlipGearSensor.get_value()) {
        left_catapult.move_velocity(600);
        right_catapult.move_velocity(600);
        pros::lcd::set_text(3, "up" + std::to_string(SlipGearSensor.get_value()));
    }

            left_catapult.brake();
            right_catapult.brake();

    // shoot();
    // pros::delay(300);
    // turn(90, 1);
    // pros::delay(5000);
    // turn(right, -90);
    // pros::delay(3000);
    // drive(50, 70);
    // turn(right, 180);
    // pros::delay(3000);
    // turn(right, -90);
    // pros::delay(3000);
    // turn(right, -180);
    // pros::delay(3000);
    
    // shoot();
    // pros::delay(1000);
    // turn(right, 6);
    // pros::delay(200);
    // pros::lcd::set_text(2, "auton started");

    // Competition auton
    // Get First roller
    drive(20, -15);
    pros::delay(200);
    drive(40, 2);
    pros::delay(200);
    runRoller();
    // Drive to second roller
    pros::delay(200);
    drive(20, 5);
    pros::delay(200);
    turn(-89, 1);
    intakeSetting(on);
    pros::delay(200);
    drive(50, -50);
    pros::delay(50);
    turn(-90, 1);
    pros::delay(50);
    drive(40, -70);
    pros::delay(200);
    drive(30, 5);
    pros::delay(200);
    turn(-180, 6);
    pros::delay(200);
    drive(20, 30);
    pros::delay(400);
    drive(50, -99);
    pros::delay(200);
    intakeSetting(off);
    // Get second roller
    turn(-90, 1);
    pros::delay(200);
    drive(40, -15);
    pros::delay(200);
    drive(40, 2);
    pros::delay(200);
    runRoller();
    pros::delay(200);

    // Skills Auton 

    // First roller
    // startCatapult();
    // drive(20, -2);
    // pros::delay(50);
    // drive(40, 1);
    // pros::delay(50);
    // skills_roller();
    // pros::delay(50); 
    // drive(20, 4);
    // pros::delay(50); 
    // // Pick up first disk
    // turn(150, 1);
    // intakeSetting(on);
    // drive(40, -12);
    // pros::delay(50);
    // drive(60, -2);
    // pros::delay(50);
    // drive(40, -8);
    // pros::delay(200);
    // intakeSetting(off);
    // // Second Roller
    // turn(89, 2);
    // pros::delay(50);
    // drive(20, -15);
    // pros::delay(50);
    // drive(40, 2);
    // pros::delay(50);
    // skills_roller();
    // pros::delay(50);
    // // Drive towards goal
    // drive(20, 5);
    // pros::delay(200);
    // turn(0, 3);
    // pros::delay(50);
    // drive(40, 40);
    // pros::delay(200);
    // turn(-2, 3);
    // pros::delay(50);
    // drive(40, 16);
    // pros::delay(50);
    // turn(12, 2);
    // pros::delay(50);
    // shoot();
    // // Line up for more disks
    // pros::delay(200);
    // turn(-2, 3);
    // pros::delay(50);
    // drive(40, -40);
    // pros::delay(200);
    // turn(-90, 5);
    // pros::delay(50);
    // drive(50, 15);
    // pros::delay(50);
    // gyro.set_rotation(-90);
    // // Get more disks
    // drive(40, -16);
    // pros::delay(200);
    // turn(-135, 1);
    // pros::delay(50);
    // intakeSetting(on);
    // pros::delay(50);
    // drive(30, -12);
    // pros::delay(50);
    // drive(40, -3);
    // pros::delay(50);
    // drive(30, -12);
    // pros::delay(50);
    // drive(40, -3);
    // pros::delay(50);
    // drive(30, -12);
    // pros::delay(50);
    // drive(40, -3);
    // pros::delay(50);
    // drive(50, -2);
    // pros::delay(200);
    // intakeSetting(off);
    // // Shoot again
    // turn(-45, 1);
    // pros::delay(50);
    // drive(40, 20);
    // pros::delay(50);
    // shoot();
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

void toggle() { intakeState = intakeState == intake ? stopped : intake; }

int driveTask(){
    while(true){
        //Base Tank Controls
        int left = controller.get_analog(ANALOG_LEFT_Y);
        int right = controller.get_analog(ANALOG_RIGHT_Y);
        if(abs(right) > 5){
            right = right -5 * (127/122);
            right_mtr1.move(right);
            right_mtr2.move(right);
            right_mtr3.move(right);
            right_mtr4.move(right);
        } else {
            right_mtr1.brake();
            right_mtr2.brake();
            right_mtr3.brake();
            right_mtr4.brake();
        }
        if(abs(left) > 5){
            left = left - 5 * (127/122);
            left_mtr1.move(left);
            left_mtr2.move(left);
            left_mtr3.move(left);
            left_mtr4.move(left);
        } else {
            left_mtr1.brake();
            left_mtr2.brake();
            left_mtr3.brake();
            left_mtr4.brake();
        }

    
    }
    
    return 0;
}

void opcontrol() {
    int cataFlag = 0;
    int intakeLock = 0;
    int i = 0;
    std::string buttonNum;
    pros::Task drive(driveTask);
    while (true) {
        pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
                         (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
                         (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);

        //Base SplitArcade Controls with pros
        /*
        int power = controller.get_analog(ANALOG_LEFT_Y);
        int turn = controller.get_analog(ANALOG_RIGHT_Y);
        int right = power - turn;
        int left = power+turn;
        right_mtr1.move(right);
        right_mtr2.move(right);
        right_mtr3.move(right);
        left_mtr1.move(left);
        left_mtr2.move(left);
        left_mtr3.move(left);*/
        buttonNum = std::to_string(SlipGearSensor.get_value());
        pros::screen::print(TEXT_MEDIUM, 1, "Sensor value: %3d", SlipGearSensor.get_value());
        
        //Catapult pull back on Button Sensor
        if (!SlipGearSensor.get_value()) {
            cataFlag = 1;
            left_catapult.move_velocity(600);
            right_catapult.move_velocity(600);
            pros::lcd::set_text(3, "up" + std::to_string(SlipGearSensor.get_value()));
        } else if (SlipGearSensor.get_value()) {
            cataFlag = 0;
            intakeLock = 0;
            left_catapult.brake();
            right_catapult.brake();
            pros::lcd::set_text(3, "down" + std::to_string(SlipGearSensor.get_value()));
        }
        
        pros::lcd::set_text(3, std::to_string(SlipGearSensor.get_value()));

        if(controller.get_digital(DIGITAL_R1) && cataFlag == 0){
            cataFlag = 0;
            intakeLock = 1;
            left_catapult.move_velocity(600);
            right_catapult.move_velocity(600);
        }


        double value = left_catapult.get_position();
        double value3 = left_catapult.get_target_position();
        std::string thingy = std::to_string(value);
        std::string thingy3 = std::to_string(value3);
        double value2 = right_catapult.get_position();
        double value4 = right_catapult.get_target_position();
        std::string thingy2 = std::to_string(value2);
        std::string thingy4 = std::to_string(value4);
        pros::lcd::set_text(4, "LeftPos: " + thingy + " Target: " + thingy3);
        pros::lcd::set_text(6, "RightPos: " + thingy2+ " Target: " + thingy4);
        
        if(fabs(left_catapult.get_position()- left_catapult.get_target_position()) >= 0.0 && fabs(right_catapult.get_position() - right_catapult.get_target_position()) >= 0.0 && cataFlag == 1){
            cataFlag = 0;
            intakeLock = 0;
            i++;
            std::string thingggggg = std::to_string(i);
            pros::lcd::set_text(1, "InsideIf: " + thingggggg);
            //left_catapult.tare_position();
            //right_catapult.tare_position();
        }

        //Intake
        if(controller.get_digital(DIGITAL_L1) && intakeLock == 0){
            Intake_1.move(-127);
            Intake_2.move(-127);
        }else if (controller.get_digital(DIGITAL_L2) && intakeLock == 0){
            Intake_1.move(127);
            Intake_2.move(127);
        } else {
            Intake_1.brake();
            Intake_2.brake();
        }

        //String Launcher
        
        if(controller.get_digital(DIGITAL_RIGHT) && controller.get_digital(DIGITAL_UP) && controller.get_digital(DIGITAL_DOWN)
            && controller.get_digital(DIGITAL_LEFT) && controller.get_digital(DIGITAL_Y) && controller.get_digital(DIGITAL_X) 
            && controller.get_digital(DIGITAL_B) && controller.get_digital(DIGITAL_A)){
                launcher.set_value(launcherState);
        }
        
        //For toggle
        /*
        if(controller.get_digital(DIGITAL_L1)){
            intakeState = outtake;
        } else if (intakeState == outtake){
            intakeState = stopped;
        }

        if (intakeState == stopped) {
            Intake.brake();
        } else if (intakeState == outtake) {
            Intake.move(-127);
        } else if (intakeState == intake) {
            Intake.move(80);
        } */


        pros::delay(20);
    }
}