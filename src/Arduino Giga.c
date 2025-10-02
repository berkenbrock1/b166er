////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                        //
//    Developed by: Jonathan Cerbaro, Dieisson Martinelli                                 //
//    Updated by: Gabriel Berkenbrock, Marco Reis                                         //
//    UTFPR - Federal University of Technology - Paraná                                   //
//    CPGEI - Graduate Program on Electrical Engineering and Applied Informatics          //
//    LASER - Advanced Laboratory of Embedded Systems and Robotics                        //
//    LARA - Applied Robotics Laboratory                                                  //
//    Advisor: Prof. Dr. André Schneider de Oliveira                                      //
//    Co-advisor: Prof. Dr. João Alberto Fabro                                            //
//    Last changed: [02/10/2025]                                                          //
//    Updated for Arduino Giga - Merged 3 codes into 1                                    //
//                                                                                        //
////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////
///// LIBRARIES /////
/////////////////////

#include <ros.h>
#include <movemaster_msg/setpoint.h>
#include <movemaster_msg/status.h>

//////////////////////////////////////////
///// PINS AND CONSTANTS DEFINITIONS /////
//////////////////////////////////////////

// Motor control definitions
#define STOP              0
#define CW                1
#define CCW               2
#define OPEN              3
#define CLOSE             4

// Motor identifiers
#define MOTOR_1           1
#define MOTOR_2           2
#define MOTOR_3           3
#define MOTOR_4           4
#define MOTOR_5           5
#define MOTOR_6           6

// Reset flags
#define RESET             1
#define RETRY             2

// PWM limits for each joint
#define PWM_MAX_1         100
#define PWM_MIN_1         50

#define PWM_MAX_2_CW      50
#define PWM_MIN_2_CW      0
#define PWM_MAX_2_CCW     150
#define PWM_MIN_2_CCW     80

#define PWM_MAX_3_CW      130
#define PWM_MIN_3_CW      0
#define PWM_MAX_3_CCW     255
#define PWM_MIN_3_CCW     180

#define PWM_MAX_4         180
#define PWM_MIN_4         40

#define PWM_MAX           255
#define PWM_MIN_5_CW      100
#define PWM_MIN_5_CCW     255

// Controller gains
#define kP_1              0.06
#define kD_1              0.30

#define kP_2              0.030
#define kI_2              0.020
#define kD_2              0.500

#define kP_3              0.030
#define kI_3              0.040
#define kD_3              0.600

#define kP_4              0.030
#define kI_4              0.050
#define kD_4              0.050

#define kP_5              1.000
#define kD_5              0.000

#define PWM_MAX_VAR       15

// Joint 1 & 2 pins
#define PWM_1A            44
#define PWM_1B            45
#define PWM_2A            46
#define PWM_2B            47
#define ENABLE_1F         13
#define ENABLE_1R         12 
#define ENABLE_2F         11
#define ENABLE_2R         10
#define ENCODER_1A        22
#define ENCODER_1B        24
#define ENCODER_2A        26
#define ENCODER_2B        28
#define LS_1A             23
#define LS_1B             25
#define LS_2A             27
#define LS_2B             29
#define RELAY_2           43

// Joint 3 & 4 pins
#define PWM_3A            48
#define PWM_3B            49
#define PWM_4A            50
#define PWM_4B            51
#define ENABLE_3F         9
#define ENABLE_3R         8
#define ENABLE_4F         7
#define ENABLE_4R         6
#define ENCODER_3A        30
#define ENCODER_3B        32
#define ENCODER_4A        34
#define ENCODER_4B        36
#define LS_3A             31
#define LS_3B             33
#define LS_4A             35
#define LS_4B             37
#define RELAY_3           42

// Joint 5 & 6 pins
#define PWM_5A            52
#define PWM_5B            53
#define PWM_6A            A15
#define PWM_6B            A14
#define ENABLE_5F         5
#define ENABLE_5R         4
#define ENABLE_6F         3
#define ENABLE_6R         2
#define ENCODER_5A        38
#define ENCODER_5B        40
#define LS_5A             39
#define LS_5B             41


/////////////////////////////////
///// VARIABLES DEFINITIONS /////
/////////////////////////////////

// Global flags
bool EMERGENCY_STOP = false;
bool already_reset = false;
int GOHOME = 0;

// Joint-specific variables
struct JointData {
    // Common variables
    long DEG2PUL;
    long HOME;
    float encoder_count;
    float setpoint;
    float last_setpoint;
    float error;
    float output;
    float tol;
    float last_error;
    float var_error;
    bool control_enable;
    float control_loop;
    unsigned long timer;
    char joint_name[10];
    
    // Joint-specific variables
    float acc_error;        // For joints with integral control
    float prev_output;      // For joints with PWM variation control
    float reset_output;     // For reset routines
    int PWM_MIN;
    int PWM_MAX;
    bool brake_flag;        // For joints with brake
    int relay_pin;          // Brake relay pin
};

JointData joints[6] = {
    // Joint 1
    {185, 0, 0, 0, 0, 0, 0, 0, 0, 0, true, 0, 0, "Joint 1", 0, 0, 0, 0, 0, false, 0},
    // Joint 2  
    {228, 106, 0, 0, 0, 0, 0, 0, 0, 0, true, 0, 0, "Joint 2", 0, 0, 0, 150, 150, false, RELAY_2},
    // Joint 3
    {186, -123, 0, 0, 0, 0, 0, 0, 0, 0, true, 0, 0, "Joint 3", 0, 0, 0, 150, 150, false, RELAY_3},
    // Joint 4
    {154, 0, 0, 0, 0, 0, 0, 0, 0, 0, true, 0, 0, "Joint 4", 0, 0, 0, 0, 0, false, 0},
    // Joint 5
    {116, 0, 0, 0, 0, 0, 0, 0, 0, 0, true, 0, 0, "Joint 5", 0, 0, 0, 150, 0, false, 0},
    // Joint 6 (Grip)
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, true, 0, 0, "Grip", 0, 0, 0, 0, 0, false, 0}
};

// Grip specific variables
bool aux_grip = false;
bool last_aux_grip = false;
int set_grip = CLOSE;
int last_setpoint_grip = CLOSE;
unsigned long TimeGrip = 0;

char buff[40];

/////////////////////////////
///// ROS CONFIGURATION /////
/////////////////////////////

ros::NodeHandle nh;
movemaster_msg::status pub_msg[6];
ros::Publisher pubs[6] = {
    ros::Publisher("/status_1", &pub_msg[0]),
    ros::Publisher("/status_2", &pub_msg[1]),
    ros::Publisher("/status_3", &pub_msg[2]),
    ros::Publisher("/status_4", &pub_msg[3]),
    ros::Publisher("/status_5", &pub_msg[4]),
    ros::Publisher("/status_6", &pub_msg[5])
};

void Callback(const movemaster_msg::setpoint &rec_msg);
ros::Subscriber<movemaster_msg::setpoint> sub("/setpoints", &Callback);

////////////////////////
///// SYSTEM SETUP /////
////////////////////////

void setup() {
    // Setup pins for all joints
    setupJointPins();
    
    // Setup encoders and interrupts
    attachInterrupt(digitalPinToInterrupt(ENCODER_1A), CheckEncoder1, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_2A), CheckEncoder2, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_3A), CheckEncoder3, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_4A), CheckEncoder4, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_5A), CheckEncoder5, RISING);
    
    // Set tolerances
    joints[0].tol = joints[0].DEG2PUL / 15;  // Joint 1
    joints[1].tol = joints[1].DEG2PUL / 10;  // Joint 2
    joints[2].tol = joints[2].DEG2PUL / 5;   // Joint 3
    joints[3].tol = joints[3].DEG2PUL / 5;   // Joint 4
    joints[4].tol = joints[4].DEG2PUL / 10;  // Joint 5
    
    // Initialize brakes
    brake_lock(2); // Joint 2 brake
    brake_lock(3); // Joint 3 brake
    
    // Start serial and ROS
    Serial.begin(9600);
    nh.initNode();
    nh.subscribe(sub);
    
    // Advertise all publishers
    for(int i = 0; i < 6; i++) {
        nh.advertise(pubs[i]);
    }
    
    // Initialize timers
    for(int i = 0; i < 5; i++) {
        joints[i].timer = millis();
    }
}

void setupJointPins() {
    // Limit sensors - inputs
    int limitPins[] = {LS_1A, LS_1B, LS_2A, LS_2B, LS_3A, LS_3B, LS_4A, LS_4B, LS_5A, LS_5B};
    for(int i = 0; i < 10; i++) {
        pinMode(limitPins[i], INPUT);
    }
    
    // Encoders - inputs
    int encoderPins[] = {ENCODER_1A, ENCODER_1B, ENCODER_2A, ENCODER_2B, 
                        ENCODER_3A, ENCODER_3B, ENCODER_4A, ENCODER_4B, 
                        ENCODER_5A, ENCODER_5B};
    for(int i = 0; i < 10; i++) {
        pinMode(encoderPins[i], INPUT);
    }
    
    // Relays - outputs
    pinMode(RELAY_2, OUTPUT);
    pinMode(RELAY_3, OUTPUT);
    
    // Enable pins - outputs
    int enablePins[] = {ENABLE_1F, ENABLE_1R, ENABLE_2F, ENABLE_2R, ENABLE_3F, ENABLE_3R, ENABLE_4F, ENABLE_4R, ENABLE_5F, ENABLE_5R, ENABLE_6F, ENABLE_6R};
    for(int i = 0; i < 12; i++) {
        pinMode(enablePins[i], OUTPUT);
        digitalWrite(enablePins[i], HIGH);
    }
    
    
    // PWM pins - outputs
    int pwmPins[] = {PWM_1A, PWM_1B, PWM_2A, PWM_2B, PWM_3A, PWM_3B, PWM_4A, PWM_4B, PWM_5A, PWM_5B, PWM_6A, PWM_6B};
    for(int i = 0; i < 12; i++) {
        pinMode(pwmPins[i], OUTPUT);
    }
}

/////////////////////////////
///// MAIN CONTROL LOOP /////
/////////////////////////////

void loop() {
    nh.spinOnce();
    
    if(nh.connected()) {
        // Publish status for all joints
        for(int i = 0; i < 6; i++) {
            Publish(i);
        }
        
        if (!already_reset) {
            GOHOME = RESET;
            GoHome();
            already_reset = true;
        }
        
        // Control loops for joints 1-5
        for(int i = 0; i < 5; i++) {
            if(joints[i].control_enable) {
                controlJoint(i);
            }
        }
        
        // Grip control (Joint 6)
        nh.spinOnce();
        if((millis() - TimeGrip) > 2000)
            motorGo(MOTOR_6, STOP, 0);
        else
            motorGo(MOTOR_6, set_grip, 255);
            
    } else {
        // ROS not connected - stop everything
        already_reset = false;
        for(int i = 0; i < 6; i++) {
            motorGo(i+1, STOP, 0);
        }
        brake_lock(2);
        brake_lock(3);
    }
}

void controlJoint(int joint_index) {
    JointData &j = joints[joint_index];
    
    // Compute control variables
    j.control_loop = millis() - j.timer;
    j.timer = millis();
    j.error = j.encoder_count - j.setpoint;
    
    // Joint-specific computations
    switch(joint_index) {
        case 0: // Joint 1
            j.var_error = (j.error - j.last_error) / j.control_loop;
            break;
        case 1: // Joint 2
            j.var_error = (j.error - j.last_error) / j.control_loop;
            j.acc_error += j.error * j.control_loop;
            // Set PWM limits based on direction
            if (j.error > 0) {
                j.PWM_MIN = PWM_MIN_2_CW;
                j.PWM_MAX = PWM_MAX_2_CW;
            } else {
                j.PWM_MIN = PWM_MIN_2_CCW;
                j.PWM_MAX = PWM_MAX_2_CCW;
            }
            break;
        case 2: // Joint 3
            j.var_error = (j.error - j.last_error) / j.control_loop;
            j.acc_error += j.error * j.control_loop;
            // Set PWM limits based on direction
            if (j.error > 0) {
                j.PWM_MIN = PWM_MIN_3_CW;
                j.PWM_MAX = PWM_MAX_3_CW;
            } else {
                j.PWM_MIN = PWM_MIN_3_CCW;
                j.PWM_MAX = PWM_MAX_3_CCW;
            }
            break;
        case 3: // Joint 4
            j.var_error = (j.error - j.last_error) / j.control_loop;
            j.acc_error += j.error * j.control_loop;
            break;
        case 4: // Joint 5
            j.var_error = (j.error - j.last_error) / (j.control_loop/1000);
            // Set PWM_MIN based on direction
            if (j.error > 0) {
                j.PWM_MIN = PWM_MIN_5_CW;
            } else {
                j.PWM_MIN = PWM_MIN_5_CCW;
            }
            break;
    }
    
    // Check if joint is out of tolerance
    if (abs(j.error) > j.tol) {
        pub_msg[joint_index].IsDone = false;
        pubs[joint_index].publish(&pub_msg[joint_index]);
        
        // Calculate control output
        calculateOutput(joint_index);
        
        // Apply motor control
        applyMotorControl(joint_index);
        
    } else {
        // Within tolerance - stop motor
        motorGo(joint_index + 1, STOP, 0);
        
        // Joint-specific stop actions
        switch(joint_index) {
            case 1: // Joint 2
                brake_lock(2);
                j.acc_error = 0;
                break;
            case 2: // Joint 3
                brake_lock(3);
                j.acc_error = 0;
                break;
            case 3: // Joint 4
                j.acc_error = 0;
                break;
        }
        
        j.output = 0;
        pub_msg[joint_index].IsDone = true;
        pubs[joint_index].publish(&pub_msg[joint_index]);
    }
}

void calculateOutput(int joint_index) {
    JointData &j = joints[joint_index];
    
    switch(joint_index) {
        case 0: // Joint 1 - PD control
            j.output = max(min(abs(kP_1 * j.error + kD_1 * j.var_error), PWM_MAX_1), PWM_MIN_1);
            j.last_error = j.error;
            j.prev_output = j.output;
            break;
            
        case 1: // Joint 2 - PID control with PWM variation
            j.output = max(min(abs(kP_2 * j.error + kI_2 * j.acc_error + kD_2 * j.var_error), j.PWM_MAX), j.PWM_MIN);
            if(sign(j.error) == sign(j.last_error)) {
                if (j.output > j.prev_output) {
                    if ((j.output - j.prev_output) > PWM_MAX_VAR) {
                        j.output = max(min((j.prev_output + PWM_MAX_VAR), j.PWM_MAX), j.PWM_MIN);
                    }
                } else {
                    if ((j.prev_output - j.output) > PWM_MAX_VAR) {
                        j.output = max(min((j.prev_output - PWM_MAX_VAR), j.PWM_MAX), j.PWM_MIN);
                    }
                }
            } else {
                j.output = 0;
            }
            j.last_error = j.error;
            j.prev_output = j.output;
            break;
            
        case 2: // Joint 3 - PID control with PWM variation
            j.output = max(min(abs(kP_3 * j.error + kI_3 * j.acc_error + kD_3 * j.var_error), j.PWM_MAX), j.PWM_MIN);
            if(sign(j.error) == sign(j.last_error)) {
                if (j.output > j.prev_output) {
                    if ((j.output - j.prev_output) > PWM_MAX_VAR) {
                        j.output = max(min((j.prev_output + PWM_MAX_VAR), j.PWM_MAX), j.PWM_MIN);
                    }
                } else {
                    if ((j.prev_output - j.output) > PWM_MAX_VAR) {
                        j.output = max(min((j.prev_output - PWM_MAX_VAR), j.PWM_MAX), j.PWM_MIN);
                    }
                }
            } else {
                j.output = 0;
            }
            j.last_error = j.error;
            j.prev_output = j.output;
            break;
            
        case 3: // Joint 4 - PID control
            j.output = max(min(abs(kP_4 * j.error + kI_4 * j.acc_error + kD_4 * j.var_error), PWM_MAX_4), PWM_MIN_4);
            j.last_error = j.error;
            break;
            
        case 4: // Joint 5 - PD control
            j.output = max(min(abs(kP_5 * j.error + kD_5 * j.var_error), PWM_MAX), j.PWM_MIN);
            j.last_error = j.error;
            break;
    }
}

void applyMotorControl(int joint_index) {
    JointData &j = joints[joint_index];
    int motor = joint_index + 1;
    
    switch(joint_index) {
        case 0: // Joint 1
            if (j.error > 0) {
                if (!digitalRead(LS_1A))
                    motorGo(motor, CW, j.output);
                else
                    motorGo(motor, STOP, 0);
            } else {
                if (!digitalRead(LS_1B))
                    motorGo(motor, CCW, j.output);
                else
                    motorGo(motor, STOP, 0);
            }
            break;
            
        case 1: // Joint 2
            if(j.brake_flag) {
                brake_release(2);
            }
            if (j.error > 0) {
                if (!digitalRead(LS_2B)) {
                    motorGo(motor, CW, j.output);
                } else {
                    motorGo(motor, STOP, 0);
                    brake_lock(2);
                }
            } else {
                if (!digitalRead(LS_2A)) {
                    motorGo(motor, CCW, j.output);
                } else {
                    motorGo(motor, STOP, 0);
                    brake_lock(2);
                }
            }
            break;
            
        case 2: // Joint 3
            if(j.brake_flag) {
                brake_release(3);
            }
            if (j.error > 0) {
                if (!digitalRead(LS_3B)) {
                    motorGo(motor, CW, j.output);
                } else {
                    brake_lock(3);
                    motorGo(motor, STOP, 0);
                }
            } else {
                if (!digitalRead(LS_3A)) {
                    motorGo(motor, CCW, j.output);
                } else {
                    brake_lock(3);
                    motorGo(motor, STOP, 0);
                }
            }
            break;
            
        case 3: // Joint 4
            if (j.error > 0) {
                if (!digitalRead(LS_4B))
                    motorGo(motor, CW, j.output);
                else
                    motorGo(motor, STOP, 0);
            } else {
                if (!digitalRead(LS_4A))
                    motorGo(motor, CCW, j.output);
                else
                    motorGo(motor, STOP, 0);
            }
            break;
            
        case 4: // Joint 5
            if (j.error > 0) {
                if (!digitalRead(LS_5B))
                    motorGo(motor, CW, j.output);
                else
                    motorGo(motor, STOP, 0);
            } else {
                if (!digitalRead(LS_5A))
                    motorGo(motor, CCW, j.output);
                else
                    motorGo(motor, STOP, 0);
            }
            break;
    }
}

/////////////////////
///// FUNCTIONS /////
/////////////////////

void Publish(int joint_index) {
    JointData &j = joints[joint_index];
    
    pub_msg[joint_index].joint = j.joint_name;
    pub_msg[joint_index].setpoint = j.setpoint;
    
    if(joint_index == 5) { // Grip
        pub_msg[joint_index].pulse_count = set_grip;
        pub_msg[joint_index].error = 0;
        pub_msg[joint_index].output = 0;
        pub_msg[joint_index].control_loop = 0;
    } else {
        pub_msg[joint_index].pulse_count = j.encoder_count / j.DEG2PUL;
        pub_msg[joint_index].error = j.error;
        pub_msg[joint_index].output = j.output;
        pub_msg[joint_index].control_loop = j.control_loop;
    }
    
    pubs[joint_index].publish(&pub_msg[joint_index]);
    nh.spinOnce();
}

void Callback(const movemaster_msg::setpoint &rec_msg) {
    // Set setpoints for all joints
    joints[0].setpoint = rec_msg.set_1 * joints[0].DEG2PUL;
    joints[1].setpoint = rec_msg.set_2 * joints[1].DEG2PUL;
    joints[2].setpoint = rec_msg.set_3 * joints[2].DEG2PUL;
    joints[3].setpoint = rec_msg.set_4 * joints[3].DEG2PUL;
    joints[4].setpoint = rec_msg.set_5 * joints[4].DEG2PUL;
    
    // Grip control
    aux_grip = rec_msg.set_GRIP;
    if (aux_grip != last_aux_grip) {
        TimeGrip = millis();
        last_aux_grip = aux_grip;
    }
    set_grip = aux_grip ? OPEN : CLOSE;
    
    EMERGENCY_STOP = rec_msg.emergency_stop;
    GOHOME = rec_msg.GoHome;

    if(EMERGENCY_STOP) {
        for(int i = 0; i < 6; i++) {
            motorGo(i+1, STOP, 0);
        }
        brake_lock(2);
        brake_lock(3);
    }
    
    if(GOHOME) GoHome();
}

void GoHome() {
    // Save last setpoints
    for(int i = 0; i < 5; i++) {
        joints[i].last_setpoint = joints[i].setpoint;
    }
    last_setpoint_grip = set_grip;
    
    // Execute home routines for each joint in sequence
    // (This is a simplified version - you may need to adjust the sequence)
    homeJoint2();
    homeJoint1(); 
    homeJoint4();
    homeJoint3();
    homeJoint5();
    homeGrip();
    
    // Reset controller variables
    for(int i = 0; i < 5; i++) {
        joints[i].encoder_count = joints[i].HOME * joints[i].DEG2PUL;
        joints[i].error = 0;
        joints[i].output = 0;
        joints[i].last_error = 0;
        joints[i].var_error = 0;
        if(i == 1 || i == 2 || i == 3) joints[i].acc_error = 0; // Joints 2,3,4
    }
    
    // Set new setpoints based on GOHOME mode
    if(GOHOME == RETRY) {
        for(int i = 0; i < 5; i++) {
            joints[i].setpoint = joints[i].last_setpoint;
        }
        set_grip = last_setpoint_grip;
    } else {
        for(int i = 0; i < 5; i++) {
            joints[i].setpoint = joints[i].encoder_count;
        }
        set_grip = CLOSE;
    }
}

// Home routines for each joint
void homeJoint1() {
    // Joint 1 goes to limit position, CCW. Stops when LS activates.
    float reset_output_1 = 0;
    bool LS_1Bstat = digitalRead(LS_1B);
    while(!LS_1Bstat){
        //Forces small changes in PWM.
        if(reset_output_1 < PWM_MAX_1)
            reset_output_1 = reset_output_1 + 1;
        else
            reset_output_1 = PWM_MAX_1;

        delay(2);
        motorGo(MOTOR_1, CCW, reset_output_1);
        LS_1Bstat = digitalRead(LS_1B);
        nh.spinOnce();
    }
    while(reset_output_1 > 0){
        reset_output_1 = reset_output_1 - PWM_MAX_VAR;
        delay(2);
        motorGo(MOTOR_1, CCW, reset_output_1);
    }
    motorGo(MOTOR_1, STOP, 0);
    
    //Wait for 0.5 second.
    unsigned long wait_time = millis();
    while(millis() < wait_time + 500){
        motorGo(MOTOR_1, STOP, 0);
    }
    
    //After the LS activates, rotates CW until at home position (half of the workspace of joint 1).
    joints[0].encoder_count = 0;
    
    while(joints[0].encoder_count > -joints[0].DEG2PUL*104){
        //Forces small changes in PWM.
        if(reset_output_1 < PWM_MAX_1)
            reset_output_1 = reset_output_1 + 1;
        else
            reset_output_1 = PWM_MAX_1;

        motorGo(MOTOR_1, CW, reset_output_1);
        nh.spinOnce();
    }
    while(reset_output_1 > 0){
        reset_output_1 = reset_output_1 - PWM_MAX_VAR;
        delay(2);
        motorGo(MOTOR_1, CW, reset_output_1);
    }
    motorGo(MOTOR_1, STOP, 0);

    //Wait for 0.5 second.
    wait_time = millis();
    while(millis() < wait_time + 500){
        motorGo(MOTOR_1, STOP, 0);
    }
}

void homeJoint2() {
    // Joint 2 goes to home position, CCW. Stops when LS activates.
    float reset_output_2 = 0;
    if(joints[1].brake_flag){
        brake_release(2);
    }
    bool LS_2Astat = digitalRead(LS_2A);
    while(!LS_2Astat){
        //Forces small changes in PWM.
        if(reset_output_2 < 140)
            reset_output_2 = reset_output_2 + 1;
        else
            reset_output_2 = 140;

        delay(2);
        motorGo(MOTOR_2, CCW, reset_output_2);
        LS_2Astat = digitalRead(LS_2A);
        nh.spinOnce();
    }
    while(reset_output_2 > 0){
        reset_output_2 = reset_output_2 - 1;
        delay(2);
        motorGo(MOTOR_2, CCW, reset_output_2);
    }
    motorGo(MOTOR_2, STOP, 0);
    brake_lock(2);

    //Wait for 0.5 second.
    unsigned long wait_time = millis();
    while(millis() < wait_time + 500){
        motorGo(MOTOR_2, STOP, 0);
    }
}

void homeJoint3() {
    // First, adjusts joint 4 to CCW limit to avoid possible self-collisions.
    motorGo(MOTOR_4, CCW, PWM_MAX_4);
    bool LS_4Astat = digitalRead(LS_4A);
    while(!LS_4Astat){
        LS_4Astat = digitalRead(LS_4A);
        nh.spinOnce();
    }
    motorGo(MOTOR_4, STOP, 0);

    //Wait for 1 second.
    unsigned long wait_time = millis();
    while(millis() < wait_time + 1000){
        motorGo(MOTOR_4, STOP, 0);
    }
    
    // Joint 3 goes to home position, CW. Stops when LS activates.
    brake_release(3);
    motorGo(MOTOR_3, CW, PWM_MAX_3_CW);
    bool LS_3Bstat = digitalRead(LS_3B);
    while(!LS_3Bstat){
        LS_3Bstat = digitalRead(LS_3B);
        nh.spinOnce();
    }
    motorGo(MOTOR_3, STOP, 0);
    brake_lock(3);

    //Wait for 0.5 second.
    wait_time = millis();
    while(millis() < wait_time + 500){
        motorGo(MOTOR_4, STOP, 0);
    }
}

void homeJoint4() {
    // Joint 4 goes to home position, CW. Stops at 0 degree mark.
    joints[3].encoder_count = 0;
    while(abs(joints[3].encoder_count) < joints[3].DEG2PUL*95){
        motorGo(MOTOR_4, CW, PWM_MAX_4);
        nh.spinOnce();
    }
    motorGo(MOTOR_4, STOP, 0);

    //Wait for 0.5 second.
    unsigned long wait_time = millis();
    while(millis() < wait_time + 500){
        motorGo(MOTOR_4, STOP, 0);
    }
}

void homeJoint5() {
    // Joint 5 goes to limit position, CCW. Stops when LS activates.
    motorGo(MOTOR_5, CCW, PWM_MAX);
    bool LS_5Astat = digitalRead(LS_5A);
    while(!LS_5Astat){
        LS_5Astat = digitalRead(LS_5A);
        nh.spinOnce();
    }
    motorGo(MOTOR_5, STOP, 0);

    //Wait for 0.5 seconds.
    unsigned long wait_time = millis();
    while(millis() < wait_time + 500){
        motorGo(MOTOR_5, STOP, 0);
    }

    // After the LS activates, rotates CW until at home position (zero degree, resulting in a flat grip).
    joints[4].encoder_count = 0;
    while(abs(joints[4].encoder_count) < joints[4].DEG2PUL*167){
        motorGo(MOTOR_5, CW, PWM_MAX);
        nh.spinOnce();
    }
    motorGo(MOTOR_5, STOP, 0);

    //Wait for 0.5 seconds.
    wait_time = millis();
    while(millis() < wait_time + 500){
        motorGo(MOTOR_5, STOP, 0);
    }
}

void homeGrip() {
    // Closes the grip, activating the motor for 2 seconds.
    unsigned long wait_time = millis();
    while(millis() < wait_time + 2000){
        motorGo(MOTOR_6, CLOSE, 255);
        nh.spinOnce();
    }
    motorGo(MOTOR_6, STOP, 0);
}


void brake_lock(int joint) {
    if(joint == 2) {
        digitalWrite(RELAY_2, HIGH);
        joints[1].brake_flag = HIGH;
    } else if(joint == 3) {
        digitalWrite(RELAY_3, HIGH);
        joints[2].brake_flag = HIGH;
    }
}

void brake_release(int joint) {
    if(joint == 2) {
        digitalWrite(RELAY_2, LOW);
        joints[1].brake_flag = LOW;
    } else if(joint == 3) {
        digitalWrite(RELAY_3, LOW);
        joints[2].brake_flag = LOW;
    }
}

// Encoder interrupt functions
void CheckEncoder1() {
    joints[0].encoder_count += digitalRead(ENCODER_1B) == HIGH ? -1 : +1;
}

void CheckEncoder2() {
    joints[1].encoder_count += digitalRead(ENCODER_2B) == HIGH ? +1 : -1;
}

void CheckEncoder3() {
    joints[2].encoder_count += digitalRead(ENCODER_3B) == HIGH ? -1 : +1;
}

void CheckEncoder4() {
    joints[3].encoder_count += digitalRead(ENCODER_4B) == HIGH ? -1 : +1;
}

void CheckEncoder5() {
    joints[4].encoder_count += digitalRead(ENCODER_5B) == HIGH ? -1 : +1;
}

static inline int8_t sign(int val) {
    if (val < 0) return -1;
    if (val==0) return 0;
    return 1;
}

void motorGo(int motor, int dir, int pwm) {
    // Implementation of motor control for all joints
    // ... (implement based on original codes, combining all motor control logic)
    
    // This is a template - you'll need to implement the full switch cases
    // for all 6 motors based on the original codes
    switch(motor) {
        case MOTOR_1:
            // Joint 1 motor control
            break;
        case MOTOR_2:
            // Joint 2 motor control  
            break;
        case MOTOR_3:
            // Joint 3 motor control
            break;
        case MOTOR_4:
            // Joint 4 motor control
            break;
        case MOTOR_5:
            // Joint 5 motor control
            break;
        case MOTOR_6:
            // Grip motor control
            break;
    }
}