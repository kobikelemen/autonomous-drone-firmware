
#ifndef SENSOR_H
#define SENSOR_H

#include <Arduino.h>
#include <math.h>
#include <MPU9250.h>

class Sensor
{
    int outpin;
    int inpin;
    long t;
    long dt;
public:
    Sensor(int opin, int ipin);
};



class IMU : Sensor
{
    float Q[2][2] = {{0.1,0},{0,0.3}};
    float P_prev[2][2] = {{10,0},{0,10}};
    float prev_state[2] = {0,0};
    float kalman_output[2];
    float prev_angle_estimate = 0;
    float R = 0.03, bias = 0;
    float roll = 0, pitch = 0;
    float atitude[2];
    float xacc, yacc, zacc, pitch_accel, roll_accel, xgyro, ygyro, zgyro, filtered_roll_angle, filtered_pitch_angle, angle, ang_vel, prev_angle, measured_angle, altitude;
    MPU9250 mpu;
public:
    float return_atitude();
    float kalman_filter(float angle, float ang_vel, float measured_angle, float bias, long dt, float P_prev[2][2], float Q[2][2], float R);
    IMU();

}; 

class Sonar : Sensor
{
public:
    float distance;
    float get_distance();
    //Sonar();
};


#endif