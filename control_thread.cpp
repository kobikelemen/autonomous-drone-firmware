// #include <Arduino.h>
//#if defined (ARDUINO_AVR_UNO)

// #include <Servo.h>
// #include <Wire.h>
// #include <MPU9250.h>
#include "control_thread.h"


// 9 & 10 are esc pings...
//Control controller(9, 10);

float _map(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

Position::Position(float _x, float _y, float _z, float _roll, float _pitch, float _yaw)
{
    x = _x;
    y = _y;
    z = _z;
    roll = _roll;
    pitch = _pitch;
    yaw = _yaw;
}

Position::Position(){}



Drone::Drone(int pinfr, int pinbl)
{
    esc_fr_pin = pinfr;
    // esc_fl_pin = pinfl;
    // esc_br_pin = pinbr;
    // esc_bl_pin = pinbl;
    esc_val_fr = 1000;
    // esc_val_fl = 1000;
    // esc_val_br = 1000;
    // esc_val_bl = 1000;
    Position p(0,0,0,0,0,0);
    pos = p;
    t = millis();
}

void Drone::set_mode(String _mode)
{
    drone_mode = _mode;
}

void Drone::update_position(float x, float y, float z, float r, float p, float ya)
{
    Position po(x, y, z, r, p, ya);
    pos = po;
}

void Drone::set_target(float x, float y, float z, float r, float p, float ya)
{
    Position po(x, y, z, r, p, ya);
    target_pos = po;
}

Control::Control(int pin1, int pin2) : Drone(pin1, pin2)
{
    float roll_error = 0, pitch_error = 0, altitude_error = 0, integral_roll = 0, integral_pitch = 0, integral_altitude = 0;
    // start esc:
    esc_fr.attach(esc_fr_pin);
    // esc_fl.attach(esc_fl_pin);
    // esc_br.attach(esc_br_pin);
    // esc_bl.attach(esc_bl_pin);
    esc_fr.writeMicroseconds(900);
    // esc_fl.writeMicroseconds(900);
    // esc_br.writeMicroseconds(900);
    // esc_bl.writeMicroseconds(900);
    delay(8000);
    esc_fr.writeMicroseconds(1100);
    // esc_fl.writeMicroseconds(1100);
    // esc_br.writeMicroseconds(1100);
    // esc_bl.writeMicroseconds(1100);
    delay(5000);
    //Serial.print("2");
}





float Control::PID() {
  // float roll, float pitch, float altitude, float desired_roll, float desired_pitch, float desired_altitude, dt
    
    long dt = 1.0;
    float prev_roll_error = roll_error;
    float prev_pitch_error = pitch_error;
    float prev_altitude_error = altitude_error;

    float roll_error = pos.roll - target_pos.roll;
    float pitch_error = pos.pitch - target_pos.pitch;
    float altitude_error = pos.z - target_pos.z;
    
    float proportional_roll = Kp_r * roll_error;
    float proportional_pitch = Kp_p * pitch_error;
    float proportional_altitude = Kp_a * altitude_error;
    
    // integral_roll += Ki_r * roll_error * dt;
    // integral_pitch += Ki_p * pitch_error * dt;
    // integral_altitude += Ki_a * altitude_error * dt;

    // float derivative_roll = Kd_r * (roll_error - prev_roll_error) / dt;
    // float derivative_pitch = Kd_p * (pitch_error - prev_pitch_error) / dt;
    // float derivative_altitude = Kd_a * (altitude_error - prev_altitude_error) / dt;

    roll_motor_command = proportional_roll;// + integral_roll + derivative_roll;
    pitch_motor_command = proportional_pitch;// + integral_pitch + derivative_pitch;
    thrust_motor_command = proportional_altitude;// + integral_altitude + derivative_altitude;

    // if (thrust_motor_command >= max_thrust){
    //     thrust_motor_command = max_thrust;
    // } else if (thrust_motor_command < 0){
    //     thrust_motor_command = 0;
    // }
    // if (pitch_motor_command >= max_thrust){
    //     pitch_motor_command = max_thrust;
    // }
    // if (roll_motor_command >= max_thrust){
    //     roll_motor_command = max_thrust;
    // }
    //   Serial.println("thrust command: ");
    //   Serial.print(thrust_motor_command);
    //   Serial.println("pitch command: ");
    //   Serial.print(pitch_motor_command);
    //   Serial.println("roll command: ");
    //   Serial.print(roll_motor_command);
    // esc_commands[0] = roll_motor_command;
    // esc_commands[1] = pitch_motor_command;
    // esc_commands[2] = thrust_motor_command;
    //esc_commands = {roll_motor_command, pitch_motor_command, thrust_motor_command};
}



void Control::run_control()
{
    long new_t = millis();
    dt = new_t - t;
    t = new_t;
    // if (drone_mode == "HOVER"){
    //     target_pos = pos;
    // } else if (drone_mode == "TAKEOFF"){
    //     float takeoff_height = 2;
    //     Position p(0,0,takeoff_height,0,0,0);
    //     target_pos = p;
    // }

    PID();
    
    motor_command();

    // Serial.print("\n roll: ");
    // Serial.print(atitude[0]);
    // Serial.print("\n pitch: ");
    // Serial.print(atitude[1]);
    // prev_roll = atitude[0];
    // prev_pitch = atitude[1];
    // run esc:
    // esc_val_1 = map(prev_roll, 0, 90, 1000, 1500); 
    // esc_val_2 = map(prev_roll, 0, 90, 1000, 1500); 
        
    // Serial.print("\n ESC_1 VALUE: ");
    // Serial.print(esc_val_1);
    // Serial.print("\n ESC_2 VALUE: ");
    // Serial.print(esc_val_2);
    // esc_1.writeMicroseconds(esc_val_1);
    // esc_2.writeMicroseconds(esc_val_2);
    delay(50);
}


void Control::motor_command()
{  
    // Serial.println('|');
    // Serial.println(thrust_motor_command);
    // Serial.println(roll_motor_command);
    // Serial.println(pitch_motor_command);
    float throttle_ratio = thrust_motor_command / max_thrust;
    float pitch_ratio = pitch_motor_command / max_thrust;
    float roll_ratio = roll_motor_command / max_thrust;
    esc_val_fr = _map(throttle_ratio + pitch_ratio + roll_ratio, -2, 3, 1000, 1300);
    Serial.println(esc_val_fr);
    // esc_val_fl = map(throttle_ratio + pitch_ratio - roll_ratio, -2, 3, 1000, 1500);
    // esc_val_br = map(throttle_ratio - pitch_ratio - roll_ratio, -2, 3, 1000, 1500);
    // esc_val_bl = map(throttle_ratio - pitch_ratio - pitch_ratio, -2, 3, 1000, 1500);
    esc_fr.writeMicroseconds(esc_val_fr);
    // esc_fl.writeMicroseconds(esc_val_fl); // change these to esc_1, esc_2 etc..
    // esc_br.writeMicroseconds(esc_val_br);
    // esc_bl.writeMicroseconds(esc_val_bl);
}


//#endif