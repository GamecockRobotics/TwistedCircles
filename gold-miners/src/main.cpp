#include "main.h"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <chrono>
#include <ctime>


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() { 
	pros::lcd::initialize();
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
	int start = clock();
	pros::Motor fly(2);
	
	//while (elapsed_seconds.count() <= 10) {
//		pros::delay(10);
//	}
	while (clock() < start+150){
		pros::delay(10);
	}

	fly = 100;
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
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::Motor chassis_l1(19,true);
	pros::Motor chassis_l2(10, true);
	pros::Motor chassis_r1(3, true);
	pros::Motor chassis_r2(2, true);

	pros::Motor fly(2);
	pros::Motor in(1);

	while (true) {
		
		int left = master.get_analog(ANALOG_LEFT_Y);
		int right = master.get_analog(ANALOG_RIGHT_Y);

		left = abs(left) > 15 ? left : 0;
		right = abs(right) > 15 ? right : 0;

		chassis_l1 = left;
		chassis_l2 = left;
		
		chassis_r1 = right;
		chassis_r2 = right;

		if (master.get_digital(DIGITAL_L1)) {
			fly = 254;
		} else if (master.get_digital(DIGITAL_L2)) {
			fly = -254;
		} else {
			fly = 0;
		}

		if (master.get_digital(DIGITAL_R1)) {
			in = 127;
		} else if (master.get_digital(DIGITAL_R2)) {
			in = -127;
		} else {
			in = 0;
		}

		pros::delay(20);
	}
}
