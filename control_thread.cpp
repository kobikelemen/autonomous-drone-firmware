#include <Arduino.h>
#if defined (ARDUINO_AVR_UNO)
#include <Servo.h>


#include <Wire.h>
#include <MPU9250.h>
#include "control_thread.h"


// 9 & 10 are esc pings...
//Control controller(9, 10);






// Control::Control(int esc_1_pin, int esc_2_pin)
// {

// }


void Control::set_params(int esc_1_pin, int esc_2_pin)
{
    // P_prev[2][2] = {{10,0},{0,10}};
    // Q[2][2] = {{0.1,0},{0,0.3}};
    // R = 0.03;
    // roll = 0;
    // pitch = 0;
    // Kp_r = 1;
    // Kp_p = 1;
    // Kp_a = 1;
    // Ki_r = 1;
    // Ki_p = 1;
    // Ki_a = 1;
    // Kd_r = 1;
    // Kd_p = 1;
    // Kd_a = 1;
    // roll_error = 0;
    // pitch_error = 0;
    // altitude_error = 0;
    // integral_roll = 0;
    // integral_pitch = 0;
    // ntegral_altitude = 0;
    // bias = 0;
    // prev_state[2] = {0,0};
    // prev_angle_estimate = 0;
    // state = 2;
    // pingPin = 9;
    Serial.println("start of control constructor");
    
    Wire.begin();
    delay(1000);
    if (!mpu.setup(0x68)) {
        while (1) {
        Serial.println("MPU connection failed. Please check your connection");
        delay(3000);
        }
    }
    Serial.println("Accel Gyro calibration will start in 1sec.");
    Serial.println("Please leave the device still on the flat plane.");
    mpu.verbose(true);
    delay(1000);
    mpu.calibrateAccelGyro();
    mpu.verbose(false);

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




float Control::kalman_filter(float angle, float ang_vel, float measured_angle, float bias, long dt, float P_prev[2][2], float Q[2][2], float R ){
 
    float P_new[2][2], S;
    float K[2];
    float kalman_state[2];
    
    //predict
    float predicted_angle = angle + dt*(ang_vel - bias); // think bias should be prev_state[1]
    P_new[0][0] = P_prev[0][0] + dt * (dt*P_prev[1][1] - P_prev[0][1] - P_prev[1][0] + Q[0][0]);
    P_new[0][1] = P_prev[0][1] - dt * P_prev[1][1];
    P_new[1][0] = P_prev[1][0] - dt * P_prev[1][1];
    P_new[1][1] = P_prev[1][1] + Q[1][1] * dt;
    
    //update
    S = P_new[0][0] + R;
    K[0] = P_new[0][0] / S;
    K[1] = P_new[1][0] / S;
    float Ktemp0 = K[0];
    float Ktemp1 = K[1];

    //y is difference between measured and predicted
    float y = measured_angle - predicted_angle;
    kalman_state[0] = predicted_angle + Ktemp0*y;
    kalman_state[1] = bias + Ktemp1*y;
    float P_final[2][2];
    P_final[0][0] = P_new[0][0] - K[0]*P_new[0][0];
    P_final[0][1] = P_new[0][1] - K[0]*P_new[0][1];
    P_final[1][0] = P_new[1][0] - K[1]*P_new[0][0];
    P_final[1][1] = P_new[1][1] - K[1]*P_new[0][1];
    P_prev = P_final;
    return kalman_state[0];
  
}


void Control::return_attitude(float prev_roll, float prev_pitch){
    if (mpu.update() == true){
        long endTime = millis();
        xacc = mpu.getAccX();
        yacc = mpu.getAccY();
        zacc = mpu.getAccZ();
        roll_accel=atan(yacc/zacc) * 180/pi;
        pitch_accel=atan(-xacc/pow((pow(yacc,2) + pow(zacc,2)),0.8)) * 180/pi;
        xgyro = mpu.getGyroX(); // * 180/pi;
        ygyro = mpu.getGyroY(); // * 180/pi; I think already in degrees/s
        zgyro = mpu.getGyroZ(); // * 180/pi;
        dt = endTime - startTime;
        long startTime = millis();
        filtered_roll_angle = kalman_filter(prev_roll, xgyro, roll_accel, bias, dt, P_prev, Q, R);
        filtered_pitch_angle = kalman_filter(prev_pitch, ygyro, pitch_accel, bias, dt, P_prev, Q, R);
    }
}

void Control::run_control()
{
    // Serial.println(" IN WHILE LOOP IN RUN_CONTROL");
    return_attitude(prev_roll, prev_pitch);
    // Serial.print("\n roll: ");
    // Serial.print(filtered_roll_angle);
    // Serial.print("\n pitch: ");
    // Serial.print(filtered_pitch_angle);
    prev_roll = filtered_roll_angle;
    prev_pitch = filtered_pitch_angle;
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
    //vTaskDelay(100);
}

void Control::set_mode(String new_mode)
{
    drone_mode = new_mode;
}
#endif