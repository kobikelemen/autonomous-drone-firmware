#include "Sense.h"


Sensor::Sensor(int opin, int ipin)
{
    inpin = ipin;
    opin = outpin;
    t = millis();
}



IMU::IMU(int opin, int ipin) : Sensor(opin, ipin)
{
    Wire.begin();
    delay(1000);
    if (!mpu.setup(0x68)) {
        while (1) {
        // Serial.println("MPU connection failed. Please check your connection");
        delay(3000);
        }
    }
    Serial.println("callibration in 1sec.");
    // Serial.println("Please leave the device still on the flat plane.");
    mpu.verbose(true);
    delay(1000);
    mpu.calibrateAccelGyro();
    mpu.verbose(false);
}




float IMU::kalman_filter(float angle, float ang_vel, float measured_angle, float bias, long dt, float P_prev[2][2], float Q[2][2], float R ){
 
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

void IMU::return_atitude()
{
    if (mpu.update() == true){
        //long endTime = millis();
        xacc = mpu.getAccX();
        yacc = mpu.getAccY();
        zacc = mpu.getAccZ();
        roll_accel=atan(yacc/zacc) * 180/M_PI;
        pitch_accel=atan(-xacc/pow((pow(yacc,2) + pow(zacc,2)),0.8)) * 180/M_PI;
        xgyro = mpu.getGyroX(); // * 180/pi;
        ygyro = mpu.getGyroY(); // * 180/pi; I think already in degrees/s
        zgyro = mpu.getGyroZ(); // * 180/pi;
        dt = millis() - t;
        t = millis();
        //dt = endTime - startTime;
        //long startTime = millis();
        filtered_roll_angle = kalman_filter(roll, xgyro, roll_accel, bias, dt, P_prev, Q, R);
        filtered_pitch_angle = kalman_filter(pitch, ygyro, pitch_accel, bias, dt, P_prev, Q, R);
    }
    //atitude[2] = {filtered_roll_angle, filtered_pitch_angle};
    roll = filtered_roll_angle;
    pitch = filtered_pitch_angle;
    //return atitude;
}




Sonar::Sonar(int opin, int ipin) : Sensor(opin, ipin) {};

float Sonar::get_distance()
{
    digitalWrite(outpin, LOW);
    delayMicroseconds(2);
    digitalWrite(outpin, HIGH);
    delayMicroseconds(10);
    digitalWrite(outpin, LOW);
    t = pulseIn(inpin, HIGH);
    distance = t*0.00034/2;
    return distance;
}












