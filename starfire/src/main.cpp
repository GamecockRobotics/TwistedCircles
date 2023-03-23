#include "main.h"
#include "pros/misc.h"

pros::Controller Controller(pros::E_CONTROLLER_MASTER);

pros::Motor front_right_1(1);
pros::Motor front_right_2(2,true);
pros::Motor front_left_1(11);
pros::Motor front_left_2(12,true);
pros::Motor back_right_1(8);
pros::Motor back_right_2(9,true);
pros::Motor back_left_1(19);
pros::Motor back_left_2(20,true);

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
