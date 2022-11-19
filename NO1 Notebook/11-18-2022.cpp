#include "main.h"
#include "pros/colors.h"
#include "pros/rtos.hpp"
#include <cstdint>
#include <memory>
#include <string>
#include "gui.h"
#include "pros/screen.h"
#include "pros/screen.hpp"

// Bitwise operator to toggle to and from Blue and Red
std::uint32_t const color_toggle = 0XFF00FF;

/**
 * Color stored as Int32
 * Color is used to determine which alliance we are on
 * Color is used to detect which color the rollers are with the optical sensor
 * 
 * Default alliance color is red
 */
std::uint32_t alliance = 0XFF0000;

/**
 * Enumerated value to select walls
 * Possible values are "left", "right", "top", "bottom", "none"
 */
enum Wall {left, right, bottom, top, none};

/**
 * Default values for sensors have the back sensors pointed towards the left 
 * wall, the left sensor toward the top wall and the right sensor disabled
 */
Wall sensor_tl = top, sensor_tr = none, sensor_bl = left, sensor_br = left;

/**
 * Pointer to select which wall is being selected
 * pointer points to no sensor when no sensor is selected
 * no sensor is the default value for the pointer
 */
Wall no_sensor;
Wall* selected_wall = &no_sensor;

/**
 * Function to convert a wall enum into a char array
 * 
 * @param Wall the wall enum to convert to a char array
 * @return a string equivalent of the wall enum
 */
const char * sensor_to_string(Wall wall) {
	return (const char *[]) {"LEFT","RIGHT","BOTTOM","TOP","NONE"}[wall];
}

/** 
 * Default robot starting position is on the left wall
 * If false, starting position is on the bottom wall
 */
bool left_wall = true;

// Bool to determine if gui is active or if the brain is in debug mode.
bool gui_debug = false;

/**
 * Draws the Graphical User Interface
 * 
 * Draws a Field to select starting position on the left of the display.
 * Draws a Robot with adjustable sensor values to detect which walls the sensors
 * are detecting
 */
void draw_gui() {
	// Set Background color of text to transparent
	pros::screen::set_eraser(0X222222);
	pros::screen::erase_rect(0,0,480,240);

	// If the gui is in debug mode it erases the screen
	if (gui_debug) {
		pros::screen::set_pen(0XFFFFFF);
		return;
	}

	// Draw Background of Field
	pros::screen::set_pen(0XAAAAAA);
	pros::screen::fill_rect(0,0,240,240);

	// Highlight Robot Starting Position in Magenta
	pros::screen::set_pen(0XFF00FF);
	if (left_wall) {
		pros::screen::fill_rect(0,37,43,163);
	} else {
		pros::screen::fill_rect(77,197,163,240);
	}

	// Draw Field Starting Locations
	pros::screen::set_pen(0X555555);
	// Left Starting Zone
	pros::screen::fill_rect(0,40,40,160);
	// Top Starting Zone
	pros::screen::fill_rect(80,0,160,40);
	// Right Starting Zone
	pros::screen::fill_rect(200,80,240,200);
	// Bottom Starting Zone
	pros::screen::fill_rect(80,200,160,240);

	// Draw Autonomous Lines
	pros::screen::set_pen(0XFFFFFF);
	pros::screen::draw_line(0,4,236,240);
	pros::screen::draw_line(0,5,235,240);
	pros::screen::draw_line(0,6,234,240);
	pros::screen::draw_line(6,0,240,234);
	pros::screen::draw_line(5,0,240,235);
	pros::screen::draw_line(4,0,240,236);

	// Draw Bottom Goal Cross Bar
	pros::screen::draw_line(0,179,61,240);
	pros::screen::draw_line(0,180,60,240);
	pros::screen::draw_line(0,181,59,240);
	// Draw Top Goal Cross Bar
	pros::screen::draw_line(179,0,240,61);
	pros::screen::draw_line(180,0,240,60);
	pros::screen::draw_line(181,0,240,59);

	// Draw Alliance Field Elements
	pros::screen::set_pen(alliance);
	pros::screen::fill_circle(30,210,12);
	pros::screen::fill_rect(158,40,162,82);
	pros::screen::fill_rect(158,78,200,82);

	// Draw Opponent Field Elements
	pros::screen::set_pen(alliance ^ color_toggle);
	pros::screen::fill_circle(210,30,12);
	pros::screen::fill_rect(40,158,82,162);
	pros::screen::fill_rect(78,158,82,200);

	// Draw Robot and Sensors
	pros::screen::set_pen(0XFFFFFF);
	pros::screen::fill_rect(311,45,409,143);
	// Top Left Sensor Information
	pros::screen::set_pen(selected_wall == &sensor_tl ? 0XFF0000 : 0XFFFFFF);
	pros::screen::print(pros::E_TEXT_SMALL, 250, 10, sensor_to_string(sensor_tl));
	pros::screen::draw_line(250, 24, 300, 24);
	pros::screen::draw_line(300,24,320,44);
	pros::screen::fill_rect(310,44,330,64);
	pros::screen::set_pen(0X000000);
	pros::screen::draw_rect(310,44,330,64);
	// Top Right Sensor Information
	pros::screen::set_pen(selected_wall == &sensor_tr ? 0XFF0000 : 0XFFFFFF);
	pros::screen::print(pros::E_TEXT_SMALL, 420, 10, sensor_to_string(sensor_tr));
	pros::screen::draw_line(420, 24, 470, 24);
	pros::screen::draw_line(400,44,420,24);
	pros::screen::fill_rect(390,44,410,64);
	pros::screen::set_pen(0X000000);
	pros::screen::draw_rect(390,44,410,64);
	// Bottom Left Sensor Information
	pros::screen::set_pen(selected_wall == &sensor_bl ? 0XFF0000 : 0XFFFFFF);
	pros::screen::print(pros::E_TEXT_SMALL, 250, 150, sensor_to_string(sensor_bl));
	pros::screen::draw_line(250, 164, 300, 164);
	pros::screen::draw_line(300,164,320,144);
	pros::screen::fill_rect(310,124,330,144);
	pros::screen::set_pen(0X000000);
	pros::screen::draw_rect(310,124,330,144);
	// Bottom Right Sensor Information
	pros::screen::set_pen(selected_wall == &sensor_br ? 0XFF0000 : 0XFFFFFF);
	pros::screen::print(pros::E_TEXT_SMALL, 420, 150, sensor_to_string(sensor_br));
	pros::screen::draw_line(420, 164, 470, 164);
	pros::screen::draw_line(400,144,420,164);
	pros::screen::fill_rect(390,124,410,144);
	pros::screen::set_pen(0X000000);
	pros::screen::draw_rect(390,124,410,144);

	// Boolean to determine if the menu is on the right or the left of the screen
	bool lorr = selected_wall == &sensor_tr || selected_wall == &sensor_br;
	if (selected_wall != &no_sensor) {
		// Draw Menu to select wall
		pros::screen::set_pen(0X888888);
		pros::screen::set_eraser(0X888888);
		pros::screen::fill_rect(lorr?250:360,10,lorr?360:470,230);
		pros::screen::set_pen(0X000000);
		pros::screen::draw_rect(lorr?260:370,20,lorr?350:460,52);
		pros::screen::print(pros::E_TEXT_MEDIUM,lorr?290:400,30,"TOP");
		pros::screen::draw_rect(lorr?260:370,62,lorr?350:460,94);
		pros::screen::print(pros::E_TEXT_MEDIUM,lorr?275:385,72,"BOTTOM");
		pros::screen::draw_rect(lorr?260:370,104,lorr?350:460,136);
		pros::screen::print(pros::E_TEXT_MEDIUM,lorr?286:396,114,"LEFT");
		pros::screen::draw_rect(lorr?260:370,146,lorr?350:460,178);
		pros::screen::print(pros::E_TEXT_MEDIUM,lorr?282:392,156,"RIGHT");
		pros::screen::draw_rect(lorr?260:370,188,lorr?350:460,220);
		pros::screen::print(pros::E_TEXT_MEDIUM,lorr?286:396,198,"NONE");
	} else {
		// Draw Confirm Button
		pros::screen::set_pen(0X71A32D);
		pros::screen::fill_rect(380,190, 470, 230);
		pros::screen::set_eraser(0X71A32D);
		pros::screen::set_pen(0XFFFFFF);
		pros::screen::print(pros::E_TEXT_MEDIUM, 390, 204, "CONFIRM");
	}
}

/**
 * Detects if touch is within set rectangle
 * 
 * @param status contains the status of the touch
 * @param x0 x coordinate of left side of rectangle being checked
 * @param y0 y coordinate of top of rectangle being checked
 * @param x1 x coordinate of right side of rectangle being checked
 * @param y1 y coordinate of bottom of rectangle being checked
 * @return true if touch is within rectangle, otherwise, false
 */
bool screen_contains(pros::screen_touch_status_s_t status, int x0, int y0, int x1, int y1) {
	return x0 <= status.x && status.x <= x1 && y0 <= status.y && status.y <= y1;
}

// Handles Interactions with GUI
void touch_gui () {
	// Gets the status to get the coordinates of where the user touched the screen
	pros::screen_touch_status_s_t status = pros::screen::touch_status();
	// Determines if the menu is on the left or the right side
	bool lorr = selected_wall == &sensor_tr || selected_wall == &sensor_br;
	// Determines if the menu is currently open
	if (selected_wall != &no_sensor) {
		if (screen_contains(status,lorr?260:370,20,lorr?350:460,52)) {
			// If pressing the button labeled top, set the sensor to top wall
			*selected_wall = top;
			selected_wall = &no_sensor;
		} else if (screen_contains(status,lorr?260:370,62,lorr?350:460,94)) {
			// If pressing the button labeled bottom, set the sensor to bottom wall
			*selected_wall = bottom;
			selected_wall = &no_sensor;
		} else if (screen_contains(status,lorr?260:370,104,lorr?350:460,136)) {
			// If pressing the button labeled left, set the sensor to left wall
			*selected_wall = left;
			selected_wall = &no_sensor;
		} else if (screen_contains(status,lorr?260:370,146,lorr?350:460,178)) {
			// If pressing the button labeled right, set the sensor to right wall
			*selected_wall = right;
			selected_wall = &no_sensor;
		} else if (screen_contains(status,lorr?260:370,188,lorr?350:460,220)) {
			// If pressing the button labeled none, set the sensor to none
			*selected_wall = none;
			selected_wall = &no_sensor;
		}
	} else if (!gui_debug) {
		if (screen_contains(status, 380, 190, 470, 230)) {
			// Go into debug mode if confirm button is hit
			gui_debug = true;
		} else if (screen_contains(status,240,0,330,64)) {
			// Select top left sensor
			selected_wall = &sensor_tl;
		} else if (screen_contains(status,390,0,480,64)) {
			// Select top right sensor
			selected_wall = &sensor_tr;
		} else if (screen_contains(status,240,124,330,164)) {
			// Select bottom left sensor
			selected_wall = &sensor_bl;
		} else if (screen_contains(status,390,124,480,164)) {
			// Select bottom right sensor
			selected_wall = &sensor_br;
		} else if (screen_contains(status,0,40,40,160)) {
			// Select start position on the left wall
			left_wall = true;
		} else if (screen_contains(status,80,0,160,40)) {
			// Select start position on the bottom wall and toggles the alliance color
			left_wall = false;
			alliance = alliance ^ color_toggle;
		} else if (screen_contains(status,200,40,240,160)) {
			// Select start position on the left wall and toggles the alliance color
			left_wall = true;
			alliance = alliance ^ color_toggle;
		} else if (screen_contains(status,80,200,160,240)) {
			// Select start position on the bottom wall
			left_wall = false;
		}
	}
	// Redraws the GUI
	draw_gui();
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
	// Draws Gui after short delay to allow brain time to start up
	pros::delay(3);
	draw_gui();
	// Sends touch status to function whenever brain is clicked
	pros::screen::touch_callback(touch_gui, TOUCH_RELEASED);
	// Main Control Loop
	while (true) {
		pros::delay(20);
	}
}
