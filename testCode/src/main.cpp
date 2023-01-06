#include "main.h"
#include "pros/colors.h"
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"
#include <cstdint>
#include <memory>
#include <string>
#include "gui.h"
#include "pros/screen.h"
#include "pros/screen.hpp"


pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::Motor motor(1);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {}

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
void opcontrol() {
	pros::lcd::initialize();
	// Draws Gui after short delay to allow brain time to start up
	pros::delay(3);
	
	// Main Control Loop
	int slew = 100;
	int actual_voltage = 0;
	int target_speed;
	int maintenance_voltage;
	int target_voltage;
	int error, prev_error = 0;
	float kp = 20, kd = 10;
	while (true) {
		target_speed = controller.get_analog(ANALOG_LEFT_Y)*210/127;
		error = target_speed - motor.get_actual_velocity();
		maintenance_voltage = target_speed*50 + (target_speed*50 < 0 ? -1250 : 1250);
		maintenance_voltage = abs(target_speed) > 15 ? maintenance_voltage : 0;
		target_voltage = maintenance_voltage + kp*error + kd*(prev_error - error);
		prev_error = error;
		if (abs(actual_voltage - target_voltage) <= slew) actual_voltage = target_voltage;
		else actual_voltage += abs(actual_voltage - target_voltage)/(target_voltage - actual_voltage)*slew;
		if (actual_voltage > 12000) actual_voltage = 12000;
		else if (actual_voltage < -12000) actual_voltage = -12000;
		motor.move_voltage(actual_voltage);
		pros::delay(10);
	}
}
