#include "main.h"
#include "pros/serial.hpp"
#include "pros/apix.h"
#include <cstdio>
#include <string> 



void initialize() {
	pros::c::serctl(SERCTL_DISABLE_COBS, NULL);

	pros::Mutex maplock = pros::Mutex();
}

void disabled() {}
void competition_initialize() {}
void autonomous() {}



void opcontrol() {
	for(int i = 65; i <= 90; ){
		std::cout << "Sup Motha fucka";

		int a;

		std::cin >> a;

		std::cout << a;

		
		
		pros::delay(69);
	}
}