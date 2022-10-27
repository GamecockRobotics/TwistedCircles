#include "main.h"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <string>


#define CHASSIS_L1_PORT 9
#define CHASSIS_L2_PORT 15
#define CHASSIS_L3_PORT 13
#define CHASSIS_R1_PORT 1
#define CHASSIS_R2_PORT 6
#define CHASSIS_R3_PORT 20
#define INTAKE_PORT 5
#define FLYWHEEL_PORT 16
#define ROLLER_PORT 4
#define INDEX_PORT 18


#define FLYWHEEL_EPSILON 20
enum IntakeDirection { in, stop, out };
enum FlywheelSpeed { low, med, high };
FlywheelSpeed flywheelSpeed = low;
IntakeDirection intakeState = stop;
int flywheelTarget = 0, indexerTarget = 0;


pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::Motor chassis_r1(CHASSIS_R1_PORT);
pros::Motor chassis_r2(CHASSIS_R2_PORT);
pros::Motor chassis_r3(CHASSIS_R3_PORT);
pros::Motor chassis_l1(CHASSIS_L1_PORT, true);
pros::Motor chassis_l2(CHASSIS_L2_PORT, true);
pros::Motor chassis_l3(CHASSIS_L3_PORT, true);
pros::Motor intake(INTAKE_PORT, true);
pros::Motor flywheel(FLYWHEEL_PORT, true);
pros::Motor indexer(INDEX_PORT);
pros::Motor roller(ROLLER_PORT);

	

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::c::motor_tare_position(CHASSIS_L1_PORT);
	pros::c::motor_tare_position(CHASSIS_R1_PORT);
	
	pros::c::motor_set_encoder_units(CHASSIS_L1_PORT, pros::E_MOTOR_ENCODER_DEGREES);
	pros::c::motor_set_encoder_units(CHASSIS_R1_PORT, pros::E_MOTOR_ENCODER_DEGREES);
	
	flywheel.set_brake_mode(MOTOR_BRAKE_HOLD);
	intake.set_brake_mode(MOTOR_BRAKE_COAST);
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

void driverFlywheel() {
	if (controller.get_digital(DIGITAL_B)) {
		
	} else if (controller.get_digital(DIGITAL_A)) {

	} else if (controller.get_digital(DIGITAL_X)) {

	}
	flywheel.move_velocity(100);
	if (fabs(flywheelTarget-flywheel.get_actual_velocity()) < FLYWHEEL_EPSILON) {

	}
}

bool checkFlywheelSpeed(int speed) {
	return 18*flywheel.get_actual_velocity() > speed;
}

void shoot() {
	indexer = indexerTarget;
	if (indexerTarget == 0 && checkFlywheelSpeed(2500)) {
		indexerTarget = 30;
		intakeState = out;
	}
	if (indexerTarget == 30 && indexer.get_efficiency() < 5) {
		indexerTarget = -30;
		intakeState = in;
	}
	if (indexerTarget == -30 && indexer.get_efficiency() < 5) {
		indexerTarget = 0;
	}
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
void opcontrol() {
	bool intakeFlag = true;
	int leftSpeed = 0, rightSpeed = 0;
	while (true) {
		
		/** Arcade Controls */
		// int power = controller.get_analog(ANALOG_LEFT_Y);
		// int turn = controller.get_analog(ANALOG_RIGHT_Y);
		// int rightSpeed = power - turn;
		// int leftSpeed = power + turn;

		/** Base Tank Controls */
		leftSpeed = abs(controller.get_analog(ANALOG_LEFT_Y)) > 5 ? controller.get_analog(ANALOG_LEFT_Y) : 0;
		rightSpeed = abs(controller.get_analog(ANALOG_RIGHT_Y)) > 5 ? controller.get_analog(ANALOG_RIGHT_Y) : 0;

		chassis_l1.move(leftSpeed);
		chassis_l2.move(leftSpeed);
		chassis_l3.move(leftSpeed);
		chassis_r1.move(rightSpeed);
		chassis_r2.move(rightSpeed);
		chassis_r3.move(rightSpeed);

		// Intake and Indexer
		if (controller.get_digital(DIGITAL_A) || indexerTarget != 0){
			shoot();
			intakeFlag = true;
		} else if (controller.get_digital(DIGITAL_R1)){
			intakeState = (intakeState == in) == intakeFlag ? stop : in;
			intakeFlag = false;
		} else {
			intakeFlag = true;
		}



		if (intakeState == in) {
			intake.move(127);
			indexer.move(30);
		} else if (intakeState == out) {
			intake.move(-127);
			indexer.move(-30);
		} else {
			intake.brake();
		}
		pros::lcd::set_text(7, std::to_string(flywheel.get_actual_velocity()));
		flywheel.move(127);

		pros::delay(20);
	}
}
