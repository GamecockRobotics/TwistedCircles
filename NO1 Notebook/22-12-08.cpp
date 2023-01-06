#include "main.h"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"



// Define Ports for Motors
#define CHASSIS_L1_PORT 1
#define CHASSIS_L2_PORT 2
#define CHASSIS_L3_PORT 3
#define CHASSIS_R1_PORT 4
#define CHASSIS_R2_PORT 5
#define CHASSIS_R3_PORT 6
#define INTAKE_PORT 7
#define FLYWHEEL_A1_PORT 8
#define FLYWHEEL_A2_PORT 9
#define FLYWHEEL_PORT 10
#define ROLLER_PORT 11

// Define Ports for Sensors
#define DISTANCE_BR_PORT 1
#define DISTANCE_TR_PORT 11
#define DISTANCE_TL_PORT 20
#define ROLLER_SENSOR_PORT 17
#define TRACKING_B_PORT 18
#define TRACKING_F_PORT 19
#define TRACKING_P_PORT 20

// Define Ports for sensors and pistons on the Analog Ports
#define FLYWHEEL_ANGLE_PORT 2
#define DISTANCE_ABR_PORT 1
#define DISTANCE_ATR_PORT 3
#define DISTANCE_ATL_PORT 7
#define INDEXER_PORT 5
#define ENDGAME_PORT 7


// The value of pi
static constexpr double pi = 3.1415926535897932;
// The value of pi divided by 2
static constexpr double pi2 = 1.5707963267948966;
// Converts inches to millimeters
static constexpr double inch_to_mm = 25.4;
// Converts millimeters to inches
static constexpr double mm_to_inch = 0.0393700787;
// Converts degrees to radians
static constexpr double degree_to_radian = 0.01745329252;
// Converts radians to degrees
static constexpr double radian_to_degree = 57.2957795;


/**
 * Enumerated value to select walls
 * Possible values are "left", "right", "top", "bottom", "none"
 */
enum Wall {left, bottom, right, top, none};

/**
 * Class for the Calibration Sensor
 * 
 * Combines the following information
 *  - Distance Sensor
 *  - Potentiometer
 *  - Location of Sensor Relative to Robot
 *  - Direction Sensor is facing
 * 
 * Uses the information to calibrate the initial location of the robot
 */
struct CalibrationSensor {
	// Positional values are public so they can be accessed
	public:
		// Direction the sensor is facing
		Wall direction;
		// x position of sensor relative to the measuring point of the robot
		int x_offset;
		// y position of sensor relative to the measuring point of the robot
		int y_offset;
		// angle of the potentiometer
		double angle;
		// distance from the distance sensor to the wall
		double distance;

		/**
		 * Constructor method of the Calibration Sensor
		 * 
		 * @param wall the direction the sensor is pointed at
		 * @param x x position of sensor relative to the measuring point of the robot
		 * @param y y position of sensor relative to the measuring point of the robot
		 * @param a the angle reading of the potentiometer
		 * @param offset the offset of the potentiometer
		 * @param d the distance reading of the distance sensor
		 */
		CalibrationSensor(Wall wall, int x, int y, double d, double a) {
			// Assigns parameters to struct's variables
			
			angle = (a);
			distance = d;
			x_offset = x;
			y_offset = y;
			direction = wall;
		}
};

// Varaiables to keep track of the Location of the Robot
int original_x_loc, original_y_loc;
// Variable to keep track of the Orientation of the Robot
float original_theta;
// Measurement of tracking Wheel Circumference
float const track_wheel_circumference = 2.1944;

// Define Controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);
// Define Motors
pros::Motor chassis_r1(CHASSIS_R1_PORT);
pros::Motor chassis_r2(CHASSIS_R2_PORT);
pros::Motor chassis_r3(CHASSIS_R3_PORT);
pros::Motor chassis_l1(CHASSIS_L1_PORT, true);
pros::Motor chassis_l2(CHASSIS_L2_PORT, true);
pros::Motor chassis_l3(CHASSIS_L3_PORT, true);
pros::Motor intake(INTAKE_PORT);
pros::Motor flywheel(FLYWHEEL_PORT);
pros::Motor roller(ROLLER_PORT);
pros::Motor flywheel_a1(FLYWHEEL_A1_PORT);
pros::Motor flywheel_a2(FLYWHEEL_A2_PORT);
// Define Pistons
pros::ADIDigitalOut indexer(INDEXER_PORT);
// Define Sensors
pros::Optical roller_sensor(ROLLER_SENSOR_PORT);
pros::Distance distance_br(DISTANCE_BR_PORT);
pros::Distance distance_tl(DISTANCE_TL_PORT);
pros::Distance distance_tr(DISTANCE_TR_PORT);
pros::ADIPotentiometer potentiometer_br(DISTANCE_ABR_PORT);
pros::ADIPotentiometer potentiometer_tl(DISTANCE_ATL_PORT);
pros::ADIPotentiometer potentiometer_tr(DISTANCE_ATR_PORT);
pros::Rotation tracking_b(TRACKING_B_PORT);
pros::Rotation tracking_f(TRACKING_F_PORT);
pros::Rotation tracking_p(TRACKING_P_PORT);
pros::ADIPotentiometer flywheel_angle(FLYWHEEL_ANGLE_PORT);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	
	pros::delay(1000);

	
	// Calculate offset from wall and theta
	
	

	

// 	// Based on the wall that the two sensors are on edit theta accordingly
// 	// Walls - Left: 0 Bottom: +90 Right: +180 Top: +270
// 	// Robot - Back: 0 Left: +90 Front: +180 Right: +270
// 	//calibrate_location();

// 	delete (close_offset);
// 	delete (far_offset);
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
void opcontrol() {
	// Variables to store the target speed for each side of the chassis
	int leftSpeed = 0, rightSpeed = 0;
	// Variables to store the position of the robot
	int x_loc, y_loc;
	// Variable to store teh direction the robot is facing
	float theta;
	// Flag to set the position of the indexing piston
	bool indexing_flag = false;
	// Main Control Loop
int count = 0;
	while (true) {
	CalibrationSensor sensor_tl(top, 216, 216, distance_tl.get(), potentiometer_tl.get_angle()+90);
	CalibrationSensor sensor_tr(none, -216, 216, distance_tr.get(), potentiometer_tr.get_angle()-115.43);
	CalibrationSensor sensor_br(left, -216, -114, distance_br.get(), potentiometer_br.get_angle()-8.65);	
	double hb  = sensor_tl.y_offset - sensor_tr.y_offset;
	double hd  = sensor_tl.x_offset - sensor_tr.x_offset;
	double hg  = sensor_tr.x_offset;
	double de  = sensor_tl.distance;
	double bc  = sensor_tr.distance;
	double hde = sensor_tl.angle*degree_to_radian;
	double cbi = sensor_tr.angle*degree_to_radian;

    double hdb = atan(hb/hd);
	double cbd = cbi - hdb;
	double bd  = sqrt(hb*hb+hd*hd);
	double bde = hde + hdb; 
	double be  = sqrt(bd*bd+de*de-2*bd*de*cos(bde));
	double ebd = acos((be*be+bd*bd-de*de)/(2*be*bd))*(bde>pi?-1:1);
	double bed = pi - ebd - bde;
	double cbe = cbd - ebd;
	double ce  = sqrt(bc*bc+be*be-2*bc*be*cos(cbe));
	double bec = acos((be*be+ce*ce-bc*bc)/(2*be*ce));



	double original_theta = pi - hde - bed - bec;
            
    original_x_loc = be*sin(bec) -hg*sin(original_theta) + sensor_tr.y_offset*cos(original_theta);
    original_y_loc = sensor_br.distance*cos((90-sensor_br.angle)*pi/180-original_theta) - cos(original_theta)*sensor_br.x_offset - sin(original_theta)*sensor_br.y_offset;
        



	pros::lcd::set_text(0, "sense 1: " + std::to_string(sensor_br.distance) + "   @   " + std::to_string(sensor_br.angle));
	pros::lcd::set_text(1, "sense 2: " + std::to_string(sensor_tr.distance) + "   @   " + std::to_string(sensor_tr.angle));
	pros::lcd::set_text(2, "sense 3: " + std::to_string(sensor_tl.distance) + "   @   " + std::to_string(sensor_tl.angle));

	pros::lcd::set_text(6, "x: " + std::to_string(original_x_loc*mm_to_inch));
	pros::lcd::set_text(7, "y: " + std::to_string(original_y_loc*mm_to_inch));
	pros::lcd::set_text(5, "theta: " + std::to_string(original_theta*radian_to_degree));

	pros::delay(100);
	





		if (controller.get_digital_new_press(DIGITAL_A)) {
			indexing_flag = !indexing_flag;
			indexer.set_value(indexing_flag);
		}
		flywheel = 127;
		/** 
         * Arcade Controls 
         *
         * Commented out Arcade Controls
         * Easy to change based on user preference
         *
         * Sets the power of the chassis based on the left joysticks forward value
         * Increases and reduces power of left and right sides of the chassis based on the horizontal
         * value of the left joystick. Respectively turns chassis left and right based on value.
         */ 
		// int power = controller.get_analog(ANALOG_LEFT_Y); 
		// int turn = controller.get_analog(ANALOG_LEFT_X); 
		// int rightSpeed = power - turn; 
		// int leftSpeed = power + turn; 
 
		/** 
         * Base Tank Controls 
         *
         * Sets left side of Chassis to left joystick value
         * Sets right side of chassis to right joystick value
         * 
         * Joystick has a small deadzone to prevent accidental movements when 
         * the joysticks are not perfectly centered
         */ 
		leftSpeed = abs(controller.get_analog(ANALOG_LEFT_Y)) > 5 ? controller.get_analog(ANALOG_LEFT_Y) : 0; 
		rightSpeed = abs(controller.get_analog(ANALOG_RIGHT_Y)) > 5 ? controller.get_analog(ANALOG_RIGHT_Y) : 0; 

        // Set Chassis Motors to Target Speed
		chassis_l1 = leftSpeed;
		chassis_l2 = leftSpeed;
		chassis_l3 = leftSpeed;
		chassis_r1 = rightSpeed;
		chassis_r2 = rightSpeed;
		chassis_r3 = rightSpeed;

		
		x_loc = original_x_loc + tracking_p.get_angle()*cos(theta)*track_wheel_circumference;
		y_loc = original_y_loc + tracking_p.get_angle()*cos(theta)*track_wheel_circumference;
		theta = original_theta + (tracking_f.get_angle() - tracking_b.get_angle())/2.0*track_wheel_circumference;

		/** 
		 * Delay so other processes can run
		 * 10 ms is the refresh rate of the Rotation sensors
		 */
	}
}
