#include "main.h"

#define LEFT_MTR1_PORT 1
#define LEFT_MTR2_PORT 2
#define LEFT_MTR3_PORT 3
#define RIGHT_MTR1_PORT 8
#define RIGHT_MTR2_PORT 9
#define RIGHT_MTR3_PORT 10
#define SLIPGEAR_BUMPER 'a'
#define LEFT_CATAPULT_PORT 11
#define RIGHT_CATAPULT_PORT 12
#define INTAKE_PORT 5

enum intakeDirection { intake, outtake, stopped };
intakeDirection intakeState = stopped;

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
void autonomous() {}

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

void opcontrol() {
	pros::Controller controller(pros::E_CONTROLLER_MASTER);
	pros::Motor right_mtr1(RIGHT_MTR1_PORT, true);
	pros::Motor right_mtr2(RIGHT_MTR2_PORT);
	pros::Motor right_mtr3(RIGHT_MTR3_PORT,true);
	pros::Motor left_mtr1(LEFT_MTR1_PORT);
	pros::Motor left_mtr2(LEFT_MTR2_PORT,true);
	pros::Motor left_mtr3(LEFT_MTR3_PORT);
	pros::Motor left_catapult(LEFT_CATAPULT_PORT,MOTOR_GEAR_RED);
	pros::Motor right_catapult(RIGHT_CATAPULT_PORT,MOTOR_GEAR_RED,true);
	pros::Motor Intake(INTAKE_PORT);
	left_catapult.set_brake_mode(MOTOR_BRAKE_HOLD);
	right_catapult.set_brake_mode(MOTOR_BRAKE_HOLD);
	Intake.set_brake_mode(MOTOR_BRAKE_COAST);
	
	//Bumpers and switches example
	pros::ADIDigitalIn SlipGearSensor (SLIPGEAR_BUMPER);


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

		//Base Tank Controls
		int left = controller.get_analog(ANALOG_LEFT_Y);
		int right = controller.get_analog(ANALOG_RIGHT_Y);
		right_mtr1.move(right);
		right_mtr2.move(right);
		right_mtr3.move(right);
		left_mtr1.move(left);
		left_mtr2.move(left);
		left_mtr3.move(left);


		//Catapult Controls
		if(controller.get_digital(DIGITAL_R1)){
			left_catapult.move_velocity(600);
			right_catapult.move_velocity(600);
		} else if (controller.get_digital(DIGITAL_R2)){
			left_catapult.move_velocity(-600);
			right_catapult.move_velocity(-600);
		} else {
			left_catapult.brake();
			right_catapult.brake();
		}

		//Intake
		if(controller.get_digital(DIGITAL_L1)){
			Intake.move(-127);
		} else if (controller.get_digital(DIGITAL_L2)){
			Intake.move(90);
		} else {
			Intake.brake();
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
