#ifndef CONTROL_H
#define CONTROL_H
// #if defined (ARDUINO_AVR_UNO)
#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <MPU9250.h>


class Position
{
public:
    float x, y, z, roll, pitch, yaw;
    Position();
    Position(float _x, float _y, float _z, float _roll, float _pitch, float _yaw);
};


class Drone
{
public: 
    Position pos;
    Position target_pos;
    int esc_fr_pin, esc_fl_pin, esc_br_pin, esc_bl_pin;
    float esc_val_fr, esc_val_fl, esc_val_br, esc_val_bl;
    String drone_mode;
    double t;
    double dt;
    Servo esc_fr, esc_fl, esc_br, esc_bl;
    Drone(int pinfr, int pinbl);
    void set_mode(String _mode);
    void update_position(float x, float y, float z, float r, float p, float ya);
    void set_target(float x, float y, float z, float r, float p, float ya);
};

float _map(float x, float in_min, float in_max, float out_min, float out_max);


class Control : public Drone
{
public:
    float desired_roll, desired_pitch, desired_altitude, throttle, yaw, current_height, desired_height;
    float thrust_motor_command, pitch_motor_command, roll_motor_command;
    float Kp_r = 1, Kp_p = 1, Kp_a = 1, Ki_r = 1, Ki_p = 1, Ki_a = 1, Kd_r = 1, Kd_p = 1, Kd_a = 1;
    float roll_error = 0, pitch_error = 0, altitude_error = 0, integral_roll = 0, integral_pitch = 0, integral_altitude = 0; // for PID loop
    float prev_roll = 0;
    float prev_pitch = 0;
    float prev_roll_angle = 0;
    float prev_pitch_angle = 0;
    //float esc_commands[3];
    float max_thrust = 20; // Newtons
    //Control(int esc_1_pin, int esc_2_pin);
    Control(int pin1, int pin2);
    void run_control();
    void motor_command();
    float PID();
};



// #endif
#endif