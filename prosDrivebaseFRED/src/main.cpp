#include "main.h"
#include "pros/adi.hpp"
#include "pros/llemu.hpp"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include <iostream>
#include <string>

#define LEFT_MTR1_PORT 1
#define LEFT_MTR2_PORT 2
#define LEFT_MTR3_PORT 3
#define RIGHT_MTR1_PORT 8
#define RIGHT_MTR2_PORT 9
#define RIGHT_MTR3_PORT 10
#define SLIPGEAR_BUMPER 'h'
#define LEFT_CATAPULT_PORT 11
#define RIGHT_CATAPULT_PORT 12
#define INTAKE_PORT_1 5
#define INTAKE_PORT_2 7
#define CATAPULT_MAX 600
#define LAUNCHER_PORT 'g'
#define GYRO_PORT 13
#define VISION_PORT 20
enum intakeDirection { intake, outtake, stopped };
intakeDirection intakeState = stopped;

enum turnType{left, right};
enum direction{forward, backward};
enum color{red, blue};



	pros::Controller controller(pros::E_CONTROLLER_MASTER);
	pros::Motor right_mtr1(RIGHT_MTR1_PORT, true);
	pros::Motor right_mtr2(RIGHT_MTR2_PORT);
	pros::Motor right_mtr3(RIGHT_MTR3_PORT,true);
	pros::Motor left_mtr1(LEFT_MTR1_PORT);
	pros::Motor left_mtr2(LEFT_MTR2_PORT,true);
	pros::Motor left_mtr3(LEFT_MTR3_PORT);
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
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);

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

	// while (cataFlagAuto == 1) {
	// 	if (!SlipGearSensor.get_value()) {
	// 		cataFlagAuto = 1;
	// 		left_catapult.move_velocity(600);
	// 		right_catapult.move_velocity(600);
	// 		pros::lcd::set_text(3, "up" + std::to_string(SlipGearSensor.get_value()));
	// 	}
	// 	else if (SlipGearSensor.get_value()) {
	// 		cataFlagAuto = 0;
	// 		//intakeLock = 0;
	// 		left_catapult.brake();
	// 		right_catapult.brake();
	// 		pros::lcd::set_text(3, "down" + std::to_string(SlipGearSensor.get_value()));
	// 	}
	// }
		
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

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
void tareMotors(){
	left_mtr1.tare_position();
	left_mtr2.tare_position();
	left_mtr3.tare_position();
	right_mtr1.tare_position();
	right_mtr2.tare_position();
	right_mtr3.tare_position();
}

void drive(int power, float distance){
	tareMotors();
	left_mtr1.move_absolute(distance, power);
	left_mtr2.move_absolute(distance, power);
	left_mtr3.move_absolute(distance, power);
	right_mtr1.move_absolute(distance, power);
	right_mtr2.move_absolute(distance,power);
	right_mtr3.move_absolute(distance, power);
	while (!((left_mtr1.get_position() < distance+5) && (left_mtr1.get_position() > distance-5))) {
    // Continue running this loop as long as the motor is not within +-5 units of its goal
    pros::delay(2);
  }
	
}

void drivePID(std::string dir, float distance){

}

void turn(turnType dir, int32_t deg) {
	
  float initialValue = gyro.get_rotation();
  float error = deg;
  float prevError = deg;
  float totalError = 0;
  const float threshold = 2.0;
  const float kp = 2.1;
  const float kd = 1.65;
  const float ki = 0.09;
  std::string first = std::to_string(gyro.get_rotation());
  pros::lcd::set_text(4, "Gyro Value: " + first);
  while (fabs(error) > threshold || fabs(prevError) > threshold) {
    int speed = kp * error + kd * (error - prevError) + ki * totalError;
    left_mtr1.move(dir == left ? -speed : speed);
    left_mtr2.move(dir == left ? -speed : speed);
    left_mtr3.move(dir == left ? -speed : speed);
    right_mtr1.move(dir == right ? -speed : speed);
    right_mtr2.move(dir == right ? -speed : speed);
    right_mtr3.move(dir == right ? -speed : speed);
    pros::delay(200);
    prevError = error;
	std::string second = std::to_string((float)gyro.get_rotation());
  	pros::lcd::set_text(5, "Gyro Value In while: " + second);
    error = deg - (fabs((float)gyro.get_rotation()) - fabs(initialValue));
	std::string errorInWhile = std::to_string(error);
  	pros::lcd::set_text(6, "Error Value: " + errorInWhile);
    totalError += (fabs(error) < 10 ? error : 0);
  }
  pros::lcd::set_text(7, "Im out of turn PID");
  	left_mtr1.move(0);
	left_mtr2.move(0);
    left_mtr3.move(0);
    right_mtr1.move(0);
    right_mtr2.move(0);
    right_mtr3.move(0);
  right_mtr1.brake();
  right_mtr2.brake();
  right_mtr3.brake();
  left_mtr1.brake();
  left_mtr2.brake();
  left_mtr3.brake();
}

void runRoller(color color){
	vision.set_led_pwm(100);
	pros::lcd::set_text(4, std::to_string(vision.get_hue()));
	left_mtr1 = -20;
	left_mtr2 = -20;
	left_mtr3 = -20;
	right_mtr1 = -20;
	right_mtr2 = -20;
	right_mtr3 = -20;

	if (color == blue) {
		// Rolls roller until it sees red
		while (!(vision.get_hue() > 300 || vision.get_hue() < 20)) {
			Intake_1.move(127);
			Intake_2.move(127);
			pros::lcd::set_text(5, std::to_string(vision.get_hue()));
			left_mtr1 = -20;
			left_mtr2 = -20;
			left_mtr3 = -20;
			right_mtr1 = -20;
			right_mtr2 = -20;
			right_mtr3 = -20;
		}
		// Rolls roller until it sees blue, this means that red is in place
		while (vision.get_hue() > 300 || vision.get_hue() < 20) {
			Intake_1.move(127);
			Intake_2.move(127);
			pros::lcd::set_text(5, std::to_string(vision.get_hue()));
			left_mtr1 = -20;
			left_mtr2 = -20;
			left_mtr3 = -20;
			right_mtr1 = -20;
			right_mtr2 = -20;
			right_mtr3 = -20;
		}
	}
	if (color == red) {
		// Rolls roller until it sees blue
		while (!(vision.get_hue() < 300 && vision.get_hue() > 100)) {
			Intake_1.move(127);
			Intake_2.move(127);
			pros::lcd::set_text(5, std::to_string(vision.get_hue()));
			left_mtr1 = -20;
			left_mtr2 = -20;
			left_mtr3 = -20;
			right_mtr1 = -20;
			right_mtr2 = -20;
			right_mtr3 = -20;
		}
		// Rolls roller until it sees red, this means that blue is in place
		while (vision.get_hue() < 300 && vision.get_hue() > 100) {
			Intake_1.move(127);
			Intake_2.move(127);
			pros::lcd::set_text(5, std::to_string(vision.get_hue()));
			left_mtr1 = -20;
			left_mtr2 = -20;
			left_mtr3 = -20;
			right_mtr1 = -20;
			right_mtr2 = -20;
			right_mtr3 = -20;
		}
	}
	Intake_1.brake();
	Intake_2.brake();
	left_mtr1 = 0;
	left_mtr2 = 0;
	left_mtr3 = 0;
	right_mtr1 = 0;
	right_mtr2 = 0;
	right_mtr3 = 0;

	vision.set_led_pwm(0);
}

void runIntake(direction dir, float dis){
	Intake_1 = -127;
	Intake_2 = -127;
	if (dir == forward){
		drive(127, dis);
	} else if (dir == backward){
		drive(-127, -dis);
	}

	Intake_1 = 0;
	Intake_2 = 0;
	
}

void shoot(){
	left_catapult.move_velocity(600);
	right_catapult.move_velocity(600);
	cataFlagAuto = 0;

	pros::delay(300);
	while(cataFlagAuto == 1){
		if (!SlipGearSensor.get_value()) {
			cataFlagAuto = 1;
			left_catapult.move_velocity(600);
			right_catapult.move_velocity(600);
			pros::lcd::set_text(3, "up" + std::to_string(SlipGearSensor.get_value()));
			
		}
		else if (SlipGearSensor.get_value()) {
			cataFlagAuto = 0;
			left_catapult.brake();
			right_catapult.brake();
			pros::lcd::set_text(3, "down" + std::to_string(SlipGearSensor.get_value()));
		}
	}
	

}

void autonomous() {

	pros::lcd::set_text(2, "auton started");
	runRoller(red);
	pros::lcd::set_text(3, "red");
	pros::delay(3000);
	pros::lcd::set_text(3, "blue");
	runRoller(blue);

	/* 	
	drive(-120, -50);
	runRoller();
	pros::delay(200);
	drive(50, 50);
	turn(right, 90);
	
	drive(127, 900);
	pros::delay(200);
	turn(left,270);
	pros::delay(1000);
	 */
	//pros::delay(20000);
	/*
	drive(127,9000);
	turn(right, 45);
	drive(127,2500);
	turn(left,90);
	shoot();*/



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
		} else {
			right_mtr1.brake();
			right_mtr2.brake();
			right_mtr3.brake();
		}
		if(abs(left) > 5){
			left = left - 5 * (127/122);
			left_mtr1.move(left);
			left_mtr2.move(left);
			left_mtr3.move(left);
		} else {
			left_mtr1.brake();
			left_mtr2.brake();
			left_mtr3.brake();
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
