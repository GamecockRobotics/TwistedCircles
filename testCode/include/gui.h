#ifndef GUI_H_
#define GUI_H_

#include <string>
#include "api.h"
#include "pros/screen.h"
#include "pros/screen.hpp"


typedef struct{std::string name; void (*Functions)();} NamedFunction;

struct Button {
	std::uint32_t color; 
	int x, y, width, height;
	bool displayed = false;

	Button(std::uint32_t color, int x, int y, int width, int height):x(x),y(y),width(width), height(height), color(color){
		pros::screen::set_pen(color);
		pros::screen::fill_rect(x, y, x+width, y+height);
	}

	bool contains(int px, int py) {
		return px >= x && px <= x+width && py >= y && py <= py+width;
	}
};

void auton1();
void auton2();
void auton3();

#endif