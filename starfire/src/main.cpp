#include "main.h"
#include "pros/misc.h"

pros::Controller Controller(pros::E_CONTROLLER_MASTER);

pros::Motor front_right_1(2);
pros::Motor front_right_2(1,true);
pros::Motor front_left_1(12);
pros::Motor front_left_2(11,true);
pros::Motor back_right_1(10);
pros::Motor back_right_2(9,true);
pros::Motor back_left_1(20);
pros::Motor back_left_2(19,true);

void initialize() {}


void disabled() {}


void competition_initialize() {}


void autonomous() {}


void opcontrol() {
	

	while (true) {
		

		back_left_1 = Controller.get_analog(ANALOG_LEFT_X) - Controller.get_analog(ANALOG_RIGHT_X) - Controller.get_analog(ANALOG_LEFT_Y);
    	front_left_1 = Controller.get_analog(ANALOG_LEFT_X) + Controller.get_analog(ANALOG_RIGHT_X) + Controller.get_analog(ANALOG_LEFT_Y);
    	back_right_1 = Controller.get_analog(ANALOG_LEFT_X) - Controller.get_analog(ANALOG_RIGHT_X) + Controller.get_analog(ANALOG_LEFT_Y);
    	front_right_1 = Controller.get_analog(ANALOG_LEFT_X) + Controller.get_analog(ANALOG_RIGHT_X) - Controller.get_analog(ANALOG_LEFT_Y);
		back_left_2 = Controller.get_analog(ANALOG_LEFT_X) - Controller.get_analog(ANALOG_RIGHT_X) - Controller.get_analog(ANALOG_LEFT_Y);
    	front_left_2 = Controller.get_analog(ANALOG_LEFT_X) + Controller.get_analog(ANALOG_RIGHT_X) + Controller.get_analog(ANALOG_LEFT_Y);
    	back_right_2 = Controller.get_analog(ANALOG_LEFT_X) - Controller.get_analog(ANALOG_RIGHT_X) + Controller.get_analog(ANALOG_LEFT_Y);
    	front_right_2 = Controller.get_analog(ANALOG_LEFT_X) + Controller.get_analog(ANALOG_RIGHT_X) - Controller.get_analog(ANALOG_LEFT_Y);

		pros::delay(10);
	}
}
