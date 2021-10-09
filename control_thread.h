#ifndef CONTROL_H
#define CONTROL_H
#if defined (ARDUINO_AVR_UNO)
#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <MPU9250.h>


struct Position
{
public:
    float x, y, z, roll, pitch, yaw;
    Position(float _x, float _y, float _z, float _roll, float _pitch, float _yaw)
    {
        x = _x;
        y = _y;
        z = _z;
        roll = _roll;
        pitch = _pitch;
        yaw = _yaw;
    }
};


class Drone
{
    Position pos;
    Positon target_pos;
    Servo esc_fr, esc_fl, esc_br, esc_bl;
    int esc_fr_pin, esc_fl, esc_br, esc_bl;
    float esc_val_fr, esc_val_fl, esc_val_br, esc_val_bl;
    String drone_mode;
    long t;
    long dt;
public:
    Drone(int pin1, int pin2);
    void set_mode(String _mode);
    void update_position(Position& p);
    void set_target(Position& p)
};




class Control : Drone
{
    float desired_roll, desired_pitch, desired_altitude, throttle, yaw, current_height, desired_height;
    float thrust_motor_command, pitch_motor_command, roll_motor_command;
    float Kp_r = 1, Kp_p = 1, Kp_a = 1, Ki_r = 1, Ki_p = 1, Ki_a = 1, Kd_r = 1, Kd_p = 1, Kd_a = 1;
    float roll_error = 0, pitch_error = 0, altitude_error = 0, integral_roll = 0, integral_pitch = 0, integral_altitude = 0; // for PID loop
    long dt;
    long startTime = millis();
    float prev_roll = 0;
    float prev_pitch = 0;
    float prev_roll_angle = 0;
    float prev_pitch_angle = 0;
    float esc_commands[3];
    float max_thrust = 20; // Newtons
    //Control(int esc_1_pin, int esc_2_pin);
public:
    Control();
    void run_control(float roll, float pitch);
    void set_params(int esc_1_pin, int esc_2_pin);
    void motor_command(float throttle, float roll, float pitch, float yaw);
    float PID(float roll, float pitch, float altitude, float desired_roll, float desired_pitch, float desired_altitude, long dt);
};



#endif
#endif