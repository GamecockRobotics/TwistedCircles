#include "main.h"
#include "pros/colors.h"
#include "pros/rtos.hpp"
#include <memory>
#include "gui.h"
#include "pros/screen.hpp"


enum GUI {alliance, auton, user, debug};
GUI prevGUI = debug;
GUI guiState = alliance;

bool allianceColor;

int redX0 = 20,  redX1 = 230, redY0 = 20, redY1 = 220;
int bluX0 = 250, bluX1 = 460, bluY0 = 20, bluY1 = 220;

int padding = 20, boxWidth = 95, boxHeight = 90;

NamedFunction autons[] = {
	NamedFunction{"Auton 1", auton1},
	NamedFunction{"auton 2", auton2},
	NamedFunction{"auton 3.0", auton3},
};

NamedFunction userControls = {

};


void draw_gui () {
	pros::screen::erase();
	switch (guiState) {
		case alliance:
			pros::screen::set_pen(COLOR_RED);
			pros::screen::fill_rect(redX0, redY0, redX1, redY1);
			pros::screen::set_pen(COLOR_BLUE);
			pros::screen::fill_rect(bluX0, bluY0, bluX1, bluY1);
		break;
		case auton:
			for (int i = 0; i < sizeof(autons)/sizeof(autons[0]); i++) {
				pros::screen::set_pen(COLOR_GREEN);
				pros::screen::fill_rect((i/2+1) *padding + i/2*boxWidth, padding + (i%2)*(padding+boxHeight), (i/2+1)*(padding + boxWidth), padding + (i%2)*(padding+boxHeight)+boxHeight);
				pros::screen::set_pen(COLOR_WHITE);

			}
		break;
		case user:
		break;
		case debug:
		break;
	}
}

void touch_gui () {
	pros::screen_touch_status_s_t status = pros::screen::touch_status();
	switch (guiState) {
		case alliance:
			if (redX0 < status.x && status.x < redX1 && redY0 < status.y && status.y < redY1) {
				pros::screen::set_pen(COLOR_WHITE);
				pros::screen::fill_rect(redX0, redY0, redX1, redY1);
				guiState = auton;
			} else if (bluX0 < status.x && status.x < bluX1 && bluY0 < status.y && status.y < bluY1) {
				guiState = auton;
			}
			break;
		case auton:
		break;
		case user:
		break;
		case debug:
		break;
	}
	if (prevGUI != guiState) {
		draw_gui();
		prevGUI = guiState;
	}

}

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
	pros::delay(20);
	draw_gui();
	while (true) {
   		pros::screen::touch_callback(touch_gui, TOUCH_PRESSED);
		pros::delay(20);
	}
}
