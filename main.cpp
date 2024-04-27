#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/misc.h"

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Drive motors
pros::Motor lF(8, pros::E_MOTOR_GEARSET_06);
pros::Motor lM(-10, pros::E_MOTOR_GEARSET_06);
pros::Motor lR(7, pros::E_MOTOR_GEARSET_06);
pros::Motor lB(6, pros::E_MOTOR_GEARSET_06);

pros::Motor rF(-18, pros::E_MOTOR_GEARSET_06);
pros::Motor rM(20, pros::E_MOTOR_GEARSET_06);
pros::Motor rR(-17, pros::E_MOTOR_GEARSET_06);
pros::Motor rB(-16, pros::E_MOTOR_GEARSET_06);

// Motor groups
pros::MotorGroup leftMotors({lF, lM, lR, lB}); // left motor group
pros::MotorGroup rightMotors({rF, rM, rR, rB}); // right motor group

// Pneumatics
pros::ADIDigitalOut wings1('C');
pros::ADIDigitalOut wings2('A');
pros::ADIDigitalOut intake('B');

// Inertial Sensor on port 2
pros::Imu imu(1);

// Auton Route
int autonRoute = 2;

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
pros::Rotation verticalEnc(9, false);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed (negative signs don't work due to a pros bug)
pros::Rotation verticalEnc2(19, true);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
lemlib::TrackingWheel vertical1(&verticalEnc, lemlib::Omniwheel::NEW_325, 5.5);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical2(&verticalEnc2, lemlib::Omniwheel::NEW_325, -5.5);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              11, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
                              600, // drivetrain rpm is 343
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(10, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            3, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             15, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
// note that in this example we use internal motor encoders (IMEs), so we don't pass vertical tracking wheels
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

// LVGL Variables
lv_obj_t * myLabel;
lv_obj_t * txtInfo;
 
lv_style_t labelStyle;
 
lv_obj_t * imgLogo;
lv_obj_t * imgLogo2;
 
// button 1
lv_obj_t * button1;
lv_obj_t * button1Label;
 
lv_style_t button1StyleREL; //relesed style
lv_style_t button1StylePR; //pressed style
 
// button 2
lv_obj_t * button2;
lv_obj_t * button2Label;
 
lv_style_t button2StyleREL; //relesed style
lv_style_t button2StylePR; //pressed style
 
// button 3
lv_obj_t * button3;
lv_obj_t * button3Label;
 
lv_style_t button3StyleREL; //relesed style
lv_style_t button3StylePR; //pressed style
 
// button 4
lv_obj_t * button4;
lv_obj_t * button4Label;
 
lv_style_t button4StyleREL; //relesed style
lv_style_t button4StylePR; //pressed style
 
LV_IMG_DECLARE(spflogo);
LV_IMG_DECLARE(spflogoBW);
 
bool showInfo = false;
 
char buffer[100];
 
static lv_res_t btn_click_action(lv_obj_t * btn)
{
    uint8_t id = lv_obj_get_free_num(btn); //id usefull when there are multiple buttons
 
    if (id == 0) {
		sprintf(buffer, "Auton Route [1] Selected (Launching)");
        autonRoute = 1;
        labelStyle.text.color = LV_COLOR_LIME;
		lv_label_set_text(myLabel, buffer);
    }
    else if (id == 1) {
		sprintf(buffer, "Auton Route [2] Selected (Offense)");
        autonRoute = 2;
        labelStyle.text.color = LV_COLOR_LIME;
		lv_label_set_text(myLabel, buffer);
    }
    else if (id == 2) {
		sprintf(buffer, "Auton Route [3] Selected (Skills)");
        autonRoute = 3;
        labelStyle.text.color = LV_COLOR_LIME;
		lv_label_set_text(myLabel, buffer);
    }
    else if (id == 3) {
        showInfo = !showInfo;
        if (showInfo) {
            lv_obj_set_hidden(imgLogo, true);
            lv_obj_set_hidden(imgLogo2, false);
            lv_obj_set_hidden(myLabel, false);
            lv_obj_set_hidden(txtInfo, false);
        } else {
            lv_obj_set_hidden(imgLogo, false);
            lv_obj_set_hidden(imgLogo2, true);
            lv_obj_set_hidden(myLabel, true);
            lv_obj_set_hidden(txtInfo, true);
        }
    }
 
    return LV_RES_OK;
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    chassis.calibrate(); // calibrate sensors

    
    // Images
    imgLogo = lv_img_create(lv_scr_act(), NULL);
    lv_img_set_src(imgLogo, &spflogo);
    lv_obj_align(imgLogo, NULL, LV_ALIGN_IN_LEFT_MID, 20, 0);
 
    imgLogo2 = lv_img_create(lv_scr_act(), NULL);
    lv_img_set_src(imgLogo2, &spflogoBW);
    lv_obj_align(imgLogo2, NULL, LV_ALIGN_IN_LEFT_MID, 20, 0);
    lv_obj_set_hidden(imgLogo2, true);
 
    // BUTTON 1
    lv_style_copy(&button1StyleREL, &lv_style_plain);
    button1StyleREL.body.main_color = LV_COLOR_MAKE(90, 0, 150);
    button1StyleREL.body.grad_color = LV_COLOR_MAKE(75, 0, 75);
    button1StyleREL.body.radius = 5;
    button1StyleREL.text.color = LV_COLOR_MAKE(255, 255, 255);
    //button1StyleREL.text.font = &extonFont;
 
    lv_style_copy(&button1StylePR, &lv_style_plain);
    button1StylePR.body.main_color = LV_COLOR_MAKE(180, 0, 255);
    button1StylePR.body.grad_color = LV_COLOR_MAKE(127, 0, 127);
    button1StylePR.body.radius = 5;
    button1StylePR.body.shadow.width = 15;
    button1StylePR.text.color = LV_COLOR_MAKE(255, 255, 255);
 
    button1 = lv_btn_create(lv_scr_act(), NULL); //create button, lv_scr_act() is deafult screen object
    lv_obj_set_free_num(button1, 0); //set button is to 0
    lv_btn_set_action(button1, LV_BTN_ACTION_CLICK, btn_click_action); //set function to be called on button click
    lv_btn_set_style(button1, LV_BTN_STYLE_REL, &button1StyleREL); //set the relesed style
    lv_btn_set_style(button1, LV_BTN_STYLE_PR, &button1StylePR); //set the pressed style
    lv_obj_set_size(button1, 150, 50); //set the button size
    lv_obj_align(button1, NULL, LV_ALIGN_IN_TOP_RIGHT, -35, 10); //set the position to top mid
 
    button1Label = lv_label_create(button1, NULL); //create label and puts it inside of the button
    lv_label_set_text(button1Label, "Auton 1"); //sets label text
 
    // BUTTON 2
    lv_style_copy(&button2StyleREL, &lv_style_plain);
    button2StyleREL.body.main_color = LV_COLOR_MAKE(150, 150, 0);
    button2StyleREL.body.grad_color = LV_COLOR_MAKE(150, 100, 0);
    button2StyleREL.body.radius = 5;
    button2StyleREL.text.color = LV_COLOR_MAKE(255, 255, 255);
 
    lv_style_copy(&button2StylePR, &lv_style_plain);
    button2StylePR.body.main_color = LV_COLOR_MAKE(255, 255, 0);
    button2StylePR.body.grad_color = LV_COLOR_MAKE(255, 200, 0);
    button2StylePR.body.radius = 5;
    button2StylePR.body.shadow.width = 15;
    button2StylePR.text.color = LV_COLOR_MAKE(255, 255, 255);
 
    button2 = lv_btn_create(lv_scr_act(), NULL); //create button, lv_scr_act() is deafult screen object
    lv_obj_set_free_num(button2, 1); //set button is to 0
    lv_btn_set_action(button2, LV_BTN_ACTION_CLICK, btn_click_action); //set function to be called on button click
    lv_btn_set_style(button2, LV_BTN_STYLE_REL, &button2StyleREL); //set the relesed style
    lv_btn_set_style(button2, LV_BTN_STYLE_PR, &button2StylePR); //set the pressed style
    lv_obj_set_size(button2, 150, 50); //set the button size
    lv_obj_align(button2, NULL, LV_ALIGN_IN_RIGHT_MID, -35, -30); //set the position to top mid
 
    button2Label = lv_label_create(button2, NULL); //create label and puts it inside of the button
    lv_label_set_text(button2Label, "Auton 2"); //sets label text
 
    // BUTTON 3
    lv_style_copy(&button3StyleREL, &lv_style_plain);
    button3StyleREL.body.main_color = LV_COLOR_MAKE(90, 0, 150);
    button3StyleREL.body.grad_color = LV_COLOR_MAKE(75, 0, 75);
    button3StyleREL.body.radius = 5;
    button3StyleREL.text.color = LV_COLOR_MAKE(255, 255, 255);
    //button3StyleREL.text.font = &extonFont;
 
    lv_style_copy(&button3StylePR, &lv_style_plain);
    button3StylePR.body.main_color = LV_COLOR_MAKE(180, 0, 255);
    button3StylePR.body.grad_color = LV_COLOR_MAKE(127, 0, 127);
    button3StylePR.body.radius = 5;
    button3StylePR.body.shadow.width = 15;
    button3StylePR.text.color = LV_COLOR_MAKE(255, 255, 255);
 
    button3 = lv_btn_create(lv_scr_act(), NULL); //create button, lv_scr_act() is deafult screen object
    lv_obj_set_free_num(button3, 2); //set button is to 0
    lv_btn_set_action(button3, LV_BTN_ACTION_CLICK, btn_click_action); //set function to be called on button click
    lv_btn_set_style(button3, LV_BTN_STYLE_REL, &button3StyleREL); //set the relesed style
    lv_btn_set_style(button3, LV_BTN_STYLE_PR, &button3StylePR); //set the pressed style
    lv_obj_set_size(button3, 150, 50); //set the button size
    lv_obj_align(button3, NULL, LV_ALIGN_IN_BOTTOM_RIGHT, -35, -70); //set the position to top mid
 
    button3Label = lv_label_create(button3, NULL); //create label and puts it inside of the button
    lv_label_set_text(button3Label, "Auton 3"); //sets label text
 
    // BUTTON 4
    lv_style_copy(&button4StyleREL, &lv_style_plain);
    button4StyleREL.body.main_color = LV_COLOR_MAKE(150, 150, 0);
    button4StyleREL.body.grad_color = LV_COLOR_MAKE(150, 100, 0);
    button4StyleREL.body.radius = 5;
    button4StyleREL.text.color = LV_COLOR_MAKE(255, 255, 255);
    //button4StyleREL.text.font = &extonFont;
 
    lv_style_copy(&button4StylePR, &lv_style_plain);
    button4StylePR.body.main_color = LV_COLOR_MAKE(255, 255, 0);
    button4StylePR.body.grad_color = LV_COLOR_MAKE(255, 200, 0);
    button4StylePR.body.radius = 5;
    button4StylePR.body.shadow.width = 15;
    button4StylePR.text.color = LV_COLOR_MAKE(255, 255, 255);
 
    button4 = lv_btn_create(lv_scr_act(), NULL); //create button, lv_scr_act() is deafult screen object
    lv_obj_set_free_num(button4, 3); //set button is to 0
    lv_btn_set_action(button4, LV_BTN_ACTION_CLICK, btn_click_action); //set function to be called on button click
    lv_btn_set_style(button4, LV_BTN_STYLE_REL, &button4StyleREL); //set the relesed style
    lv_btn_set_style(button4, LV_BTN_STYLE_PR, &button4StylePR); //set the pressed style
    lv_obj_set_size(button4, 150, 50); //set the button size
    lv_obj_align(button4, NULL, LV_ALIGN_IN_BOTTOM_RIGHT, -35, -15); //set the position to top mid
 
    button4Label = lv_label_create(button4, NULL); //create label and puts it inside of the button
    lv_label_set_text(button4Label, SYMBOL_SETTINGS" More Info"); //sets label text
 
    // Labels
    myLabel = lv_label_create(lv_scr_act(), NULL); //create label and puts it on the screen
    lv_style_copy(&labelStyle, &lv_style_plain_color);
    labelStyle.text.color = LV_COLOR_RED;
    lv_label_set_style(myLabel, &labelStyle);
    lv_label_set_text(myLabel, "Button has not been clicked yet"); //sets label text
    lv_obj_align(myLabel, NULL, LV_ALIGN_IN_TOP_LEFT, 10, 10); //set the position to center
 
 
    txtInfo = lv_label_create(lv_scr_act(), NULL); //create label and puts it on the screen
    lv_label_set_text(txtInfo, "null"); //sets label text
    lv_obj_align(txtInfo, NULL, LV_ALIGN_IN_TOP_LEFT, 10, 35); //set the position to center
 
    lv_obj_set_hidden(myLabel, true);
    lv_obj_set_hidden(txtInfo, true);

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        lemlib::Pose pose(0, 0, 0);
 
        while (true) {
 
            // Temperature (130, 140, 150, 160 °F)
            int motorTemps[6];
            motorTemps[0] = lF.get_temperature();
            motorTemps[1] = lM.get_temperature();
            motorTemps[2] = lB.get_temperature();
            motorTemps[3] = rF.get_temperature();
            motorTemps[4] = rM.get_temperature();
            motorTemps[5] = rB.get_temperature();
 
            int *maxElement = std::max_element(std::begin(motorTemps), std::end(motorTemps));
 
            // print info
            char txtBuffer[200];
            sprintf(txtBuffer, "X: %.2f \nY: %.2f \nTheta: %.2f \nTemperature: %.1f°F \
                    \n\n\nSingle Point Failure \nCatholic High School For Boys \n72116A "SYMBOL_HOME,
                    chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta, static_cast<double>(*maxElement)*9/5+32);
 
		    lv_label_set_text(txtInfo, txtBuffer);
 
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous() {
    //chassis.moveToPose(0, 20, 0, 5000);
    //chassis.turnToHeading(90, 1000, {.minSpeed = 100});
    int def = 2000;
 
    switch (autonRoute) {
        // Launching (Near side)
        case 1:
            chassis.setPose(-44.5, -59.13, 135);
            chassis.moveToPose(-56.64, -46.93, 135, def, {.forwards = false});
            chassis.waitUntilDone();
            wings1.set_value(true);
            pros::delay(800);
            chassis.moveToPose(-44.5, -59.13, 135, def);
            chassis.waitUntilDone();
            pros::delay(200);
            wings1.set_value(false);
            chassis.moveToPose(-9, -59.55, 90, def);
            intake.set_value(true);
            chassis.moveToPose(-15, -59.55, 90, def, {.forwards = false});
            intake.set_value(false);
            chassis.moveToPose(-9, -59.55, 90, def);
            wings1.set_value(true);
            chassis.moveToPose(-9, -59.55, 110, def);
            break;

        // Offense (Far side)
        case 2:
            chassis.setPose(35.5, -61.63, 0);

            chassis.moveToPose(35.5, -9.63, 0, def);

            chassis.moveToPose(35.5, -9.63, 90, def);
            chassis.waitUntilDone();
            intake.set_value(true);

            chassis.moveToPose(50.5, -7.63, 90, def, {.minSpeed = 100});
            chassis.moveToPose(35.5, -7.63, 90, def, {.forwards = false});

            // Next triball
            chassis.moveToPose(35.5, -7.63, 270, def);
            chassis.moveToPose(9.5, -23.39, 270, def);
            chassis.waitUntilDone();
            intake.set_value(false);

            //from old
            chassis.setPose(9.7, chassis.getPose().y, chassis.getPose().theta);
            chassis.moveToPose(15.5, -23.39, 270, def, {.forwards = false});
            chassis.moveToPose(15.5, -23.39, 355, def);
            
            // Final push
            chassis.moveToPose(7.0, -8.2, 355, def);
            chassis.moveToPose(7.0, -8.2, 80, def, {.maxSpeed = 20});
            chassis.waitUntil(5);
            wings1.set_value(true);
            intake.set_value(true);

            
            chassis.moveToPose(42.45, -4.85, 84, 3000);

            chassis.moveToPose(32.25, -4.85, 84, def, {.forwards = false});
            chassis.waitUntilDone();
            wings1.set_value(false);
            intake.set_value(false);


            /*chassis.moveToPose(35.5, -55.63, 0, def);
            chassis.moveToPose(31.32, -12.47, 0, def);
            chassis.waitUntilDone();
            intake.set_value(true);
            pros::delay(500);
            chassis.moveToPose(31.32, -21.47, 0, def, {.forwards = false});

            chassis.moveToPose(31.32, -21.47, 270, def, {.maxSpeed = 60});
            chassis.moveToPose(9.5, -23.39, 270, def);
            chassis.waitUntilDone();
            intake.set_value(false);
            chassis.setPose(9.7, chassis.getPose().y, chassis.getPose().theta);
            chassis.moveToPose(15.5, -23.39, 270, def, {.forwards = false});
            
            // Final push
            chassis.moveToPose(10.0, -8.2, 0, def);
            chassis.moveToPose(10.0, -8.2, 90, def);
            chassis.waitUntil(1);
            wings1.set_value(true);
            intake.set_value(true);

            chassis.setPose(chassis.getPose().x, chassis.getPose().y, 0);
            chassis.moveToPose(42.45, -4.85, 90, 3000);

            chassis.moveToPose(32.25, -4.85, 90, def, {.forwards = false});
            chassis.waitUntilDone();
            wings1.set_value(false);
            intake.set_value(false);*/

            break;

        case 4:
            chassis.moveToPose(0, 15, 0, def);
            //chassis.turnToHeading(90, def);
            //chassis.turnToPoint(0, 0, 2000);
            //chassis.moveToPose(0, 0, 180, 2000);
            //chassis.turnToHeading(0, def);
            //chassis.setPose(39.96, -61.222, 0);
            //chassis.follow(example1_txt, 10, 30000);
            break;
        case 5:
            chassis.setPose(46.92, -59.02, 45);
            //wings.set_value(true);
            chassis.moveToPose(59.5, -47.1, 45, def);
            //wings.set_value(false);
            break;
 
        // Testing
        case 6:
            chassis.moveToPose(0, 25, 0, def);
 
            chassis.moveToPose(25, 25, 90, def);
 
            chassis.moveToPose(25, 0, 180, def);
 
            chassis.moveToPose(0, 0, 270, def);
 
            chassis.turnToHeading(360, def);
            break;
 
        case 7:
            chassis.setPose(39.37, -61.02, 0);
            chassis.follow(example_txt, 10, 30000);
            break;
    }

    /*// Move to x: 20 and y: 15, and face heading 90. Timeout set to 4000 ms
    chassis.moveToPose(20, 15, 90, 4000);
    // Move to x: 0 and y: 0 and face heading 270, going backwards. Timeout set to 4000ms
    chassis.moveToPose(0, 0, 270, 4000, {.forwards = false});
    // cancel the movement after it has traveled 10 inches
    chassis.waitUntil(10);
    chassis.cancelMotion();
    // Turn to face the point x:45, y:-45. Timeout set to 1000
    // dont turn faster than 60 (out of a maximum of 127)
    chassis.turnToPoint(45, -45, 1000, {.maxSpeed = 60});
    // Turn to face a direction of 90º. Timeout set to 1000
    // will always be faster than 100 (out of a maximum of 127)
    // also force it to turn clockwise, the long way around
    chassis.turnToHeading(90, 1000, {.direction = AngularDirection::CW_CLOCKWISE, .minSpeed = 100});
    // Follow the path in path.txt. Lookahead at 15, Timeout set to 4000
    // following the path with the back of the robot (forwards = false)
    // see line 116 to see how to define a path
    chassis.follow(example_txt, 15, 4000, false);
    // wait until the chassis has traveled 10 inches. Otherwise the code directly after
    // the movement will run immediately
    // Unless its another movement, in which case it will wait
    chassis.waitUntil(10);
    pros::lcd::print(4, "Traveled 10 inches during pure pursuit!");
    // wait until the movement is done
    chassis.waitUntilDone();
    pros::lcd::print(4, "pure pursuit finished!");*/
}

/**
 * Runs in driver control
 */

bool toggle = false;
bool toggle2 = false;
bool toggle3 = true;
 
void opcontrol() {
    // set up
    wings1.set_value(false);
    wings2.set_value(false);
    intake.set_value(false);
 
    while (true) {
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
 
        // chassis.curvature(leftY, rightX);
        chassis.arcade(leftY, rightX, 2.7);
 
        // Pneumatics
        if (controller.get_digital_new_press(DIGITAL_B)) {
            wings1.set_value(!toggle);    // When false go to true and in reverse
            toggle = !toggle;    // Flip the toggle to match piston state
        }
 
        if (controller.get_digital_new_press(DIGITAL_RIGHT)) {
            wings2.set_value(!toggle2);    // When false go to true and in reverse
            toggle2 = !toggle2;    // Flip the toggle to match piston state
        }
 
        if (controller.get_digital_new_press(DIGITAL_Y)) {
            intake.set_value(!toggle3);    // When false go to true and in reverse
            toggle3 = !toggle3;    // Flip the toggle to match piston state
        }
 
        pros::delay(10);
    }
}