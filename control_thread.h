#ifndef CONTROL_H
#define CONTROL_H
#if defined (ARDUINO_AVR_UNO)
#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <MPU9250.h>


class Control
{
public:
    float xacc, yacc, zacc, pitch_accel, roll_accel, xgyro, ygyro, zgyro, filtered_roll_angle, filtered_pitch_angle, angle, ang_vel, prev_angle, measured_angle, altitude;
    float desired_roll, desired_pitch, desired_altitude, throttle, yaw, current_height, desired_height;
    float thrust_motor_command, pitch_motor_command, roll_motor_command;
    float P_prev[2][2] = {{10,0},{0,10}};
    float Q[2][2] = {{0.1,0},{0,0.3}};
    float R = 0.03, roll = 0, pitch = 0;
    float Kp_r = 1, Kp_p = 1, Kp_a = 1, Ki_r = 1, Ki_p = 1, Ki_a = 1, Kd_r = 1, Kd_p = 1, Kd_a = 1;
    float roll_error = 0, pitch_error = 0, altitude_error = 0, integral_roll = 0, integral_pitch = 0, integral_altitude = 0; // for PID loop
    float bias = 0;
    float prev_state[2] = {0,0};
    float prev_angle_estimate = 0;
    int state = 2;
    const int pingPin = 9; // THESE ARE SAME AS ESC PINS.. CHANGE THEM <
    const int recievePin = 10;
    int pos = 0;
    long pi = 3.14159265359;
    float kalman_output[2];
    long dt;
    long startTime = millis();
    float prev_roll = 0;
    float prev_pitch = 0;
    float prev_roll_angle = 0;
    float prev_pitch_angle = 0;
    float esc_val_1 = 1000;
    float esc_val_2 = 1000;
    String drone_mode;
    Servo esc_1;
    Servo esc_2;
    MPU9250 mpu;
    //Control(int esc_1_pin, int esc_2_pin);
    float kalman_filter(float angle, float ang_vel, float measured_angle, float bias, long dt, float P_prev[2][2], float Q[2][2], float R);
    void return_attitude(float prev_roll, float prev_pitch);
    void run_control();
    void set_mode(String new_mode);
    void Control::set_params(int esc_1_pin, int esc_2_pin);
};



#endif
#endif