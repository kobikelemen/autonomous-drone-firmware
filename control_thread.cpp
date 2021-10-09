#include <Arduino.h>
#if defined (ARDUINO_AVR_UNO)
#include <Servo.h>


#include <Wire.h>
#include <MPU9250.h>
#include "control_thread.h"


// 9 & 10 are esc pings...
//Control controller(9, 10);




Drone::Drone(int pin1, int pin2)
{
    esc_fr_pin = pinfr;
    esc_fl_pin = pinfl;
    esc_br_pin = pinbr;
    esc_bl_pin = pinbl;
    esc_val_fr = 1000;
    esc_val_fl = 1000;
    esc_val_br = 1000;
    esc_val_bl = 1000;
    Position p(0,0,0,0,0,0);
    pos = p;
}

void Drone::set_mode(String _mode)
{
    drone_mode = _mode;
}

void Drone::update_position(Position& p)
{
    pos = p;
}

void Drone::set_target(Position& p)
{
    target_pos = p;
}

Control::Control() : Drone(int pin1, int pin2) {};

void Control::set_params(int esc_1_pin, int esc_2_pin)
{

    /*
    // start esc:
    esc_1.attach(esc_1_pin);
    esc_2.attach(esc_2_pin);
    esc_1.writeMicroseconds(900);
    esc_2.writeMicroseconds(900);
    delay(8000);
    esc_1.writeMicroseconds(1100);
    esc_2.writeMicroseconds(1100);
    delay(5000);
    Serial.print("2");
    */
}




float Contorl::PID(float roll, float pitch, float altitude, float desired_roll, float desired_pitch, float desired_altitude, long dt) {
  
  float prev_roll_error = roll_error;
  float prev_pitch_error = pitch_error;
  float prev_altitude_error = altitude_error;

  float roll_error = roll - desired_roll;
  float pitch_error = pitch - desired_pitch;
  float altitude_error = altitude - desired_altitude;
  
  float proportional_roll = Kp_r * roll_error;
  float proportional_pitch = Kp_p * pitch_error;
  float proportional_altitude = Kp_a * altitude_error;
  
  integral_roll += Ki_r * roll_error * dt;
  integral_pitch += Ki_p * pitch_error * dt;
  integral_altitude += Ki_a * altitude_error * dt;

  float derivative_roll = Kd_r * (roll_error - prev_roll_error) / dt;
  float derivative_pitch = Kd_p * (pitch_error - prev_pitch_error) / dt;
  float derivative_altitude = Kd_a * (altitude_error - prev_altitude_error) / dt;

  roll_motor_command = proportional_roll + integral_roll + derivative_roll;
  pitch_motor_command = proportional_pitch + integral_pitch + derivative_pitch;
  thrust_motor_command = proportional_altitude + integral_altitude + derivative_altitude;

  if (thrust_motor_commands >= max_thrust){
      thrust_motor_command = max_thrust;
  } else if (thrust_motor_command < 0){
      thrust_motor_command = 0;
  }
//   Serial.println("thrust command: ");
//   Serial.print(thrust_motor_command);
//   Serial.println("pitch command: ");
//   Serial.print(pitch_motor_command);
//   Serial.println("roll command: ");
//   Serial.print(roll_motor_command);

  esc_commands = {roll_motor_command, pitch_motor_command, thrust_motor_command};
}


void Control::set_desired_position(Position& p)
{
    target_pos = p;
}


void Control::run_control(float roll, float pitch)
{
    long new_t = millis();
    dt = new_t - t;
    p = Position(0,0,0.3,0,0,0);
    target_pos = p;
    // if (drone_mode == "HOVER"){
    //     target_pos = pos;
    // } else if (drone_mode == "TAKEOFF"){
    //     float takeoff_height = 2;
    //     Position p(0,0,takeoff_height,0,0,0);
    //     target_pos = p;
    // }
    esc_commands = PID(roll, pitch, pos.z, target_pos.roll, target_pos.pitch, target_pos.yaw, dt);
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
    float throttle_ratio = esc_commands[-1] / max_thrust;
    float pitch_ratio = pitch / 90;
    float roll_ratio = roll / 90;
    esc_val_fr = map(throttle_ratio + pitch_ratio + roll_ratio, -2, 3, 1000, 1500);
    esc_val_fl = map(throttle_ratio + pitch_ratio - roll_ratio, -2, 3, 1000, 1500);
    esc_val_br = map(throttle_ratio - pitch_ratio - roll_ratio, -2, 3, 1000, 1500);
    esc_val_bl = map(throttle_ratio - pitch_ratio - pitch_ratio, -2, 3, 1000, 1500);
    esc_fr.writeMicroseconds(esc_val_fr);
    esc_fl.writeMicroseconds(esc_val_fl); // change these to esc_1, esc_2 etc..
    esc_br.writeMicroseconds(esc_val_br);
    esc_bl.writeMicroseconds(esc_val_bl);
}


#endif