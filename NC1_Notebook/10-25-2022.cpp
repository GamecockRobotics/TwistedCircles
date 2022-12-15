#include "main.h"
#include "pros/adi.hpp"
#include "pros/llemu.hpp"
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
enum intakeDirection { intake, outtake, stopped };
intakeDirection intakeState = stopped;


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
	
	//Bumpers and switches example
	pros::ADIDigitalIn SlipGearSensor (SLIPGEAR_BUMPER);

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
void autonomous() {


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
			//When analog right stick is moved, control right side of chassis 
			right = right -5 * (127/122);
			right_mtr1.move(right);
			right_mtr2.move(right);
			right_mtr3.move(right);
		} else {
			//if the right analog stick is centered, don't move the right side of the chassis
			right_mtr1.brake();
			right_mtr2.brake();
			right_mtr3.brake();
		}
		if(abs(left) > 5){
			//When left analog stick is moved, control left side of chassis 
			left = left - 5 * (127/122);
			left_mtr1.move(left);
			left_mtr2.move(left);
			left_mtr3.move(left);
		} else {
			//if the left analog stick is centered, don't move left side of chassis
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

		

		// Catapult pull back on Button Sensor
		if (!SlipGearSensor.get_value()) {
			cataFlag = 1;
			left_catapult.move_velocity(600);
			right_catapult.move_velocity(600);
			pros::lcd::set_text(3, "up" + std::to_string(SlipGearSensor.get_value()));
		}
		else if (SlipGearSensor.get_value()) {
		//stops the catapult if sensor finds value
			cataFlag = 0;
			intakeLock = 0;
			left_catapult.brake();
			right_catapult.brake();
			pros::lcd::set_text(3, "down" + std::to_string(SlipGearSensor.get_value()));
		}
		
		pros::lcd::set_text(3, std::to_string(SlipGearSensor.get_value()));

		if(controller.get_digital(DIGITAL_B) && cataFlag == 0){
		// if the "B" button is pressed then catapult motors "left and right catapult" is reset
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

		if(controller.get_digital(DIGITAL_L1) && intakeLock == 0){
			//Intake in fast
			Intake_1.move(-127);
			Intake_2.move(-127);
		}else if (controller.get_digital(DIGITAL_L2) && intakeLock == 0){
			//Intake in but slower
			Intake_1.move(-100);
			Intake_2.move(-100);
		} else if (controller.get_digital(DIGITAL_DOWN) && intakeLock == 0){
			//reverse intake - outake
			Intake_1.move(90);
			Intake_2.move(90);
		} else {
			//stops intake
			Intake_1.brake();
			Intake_2.brake();
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