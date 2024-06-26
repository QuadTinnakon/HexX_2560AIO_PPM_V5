/*
project_HEX_rotor v1.1  
1. Automatic  Takeoff 
2. 1 waypoint navigation
3. Landing
by: tinnakon kheowree  
tinnakon_za@hotmail.com
tinnakonza@gmail.com

date: 13-05-2557(2014)  V.1 HEX-X
date: 18-05-2557(2014)  V.2 Read Remote PPM
date: 19-05-2557(2014)  V.3 read sensor , baro , observer, ahrs , filter nois ,
date: 21-05-2557(2014)  V.4 Moving Average Filters , filter gyro 42HZ Sample rate = 1kHz,
date: 21-05-2557(2014)  V.5 Tuning PID , Moving Average Filters ,baro observer , read GPS
date: 20-03-2567(2024)  V.5

Automatic  Takeoff Landing waypoint navigation Remote Trim Acc
Altitude Hold, Fail_Safe()
Read Remote PPM

support:  Board AIO Pro v2
• Atmega2560
• MPU6050C Gyro Accelerometer //400kHz nois gyro +-0.05 deg/s , acc +-0.04 m/s^2
• MS561101BA Barometer
• HMC5883L Magnetometer //400kHz

HEX6-X
---------motor---------
int MOTOR_FrontL_PIN = 2;
int MOTOR_FrontR_PIN = 5;
int MOTOR_BackL_PIN = 6;
int MOTOR_BackR_PIN = 3;
int MOTOR_Left_PIN = 8;
int MOTOR_Right_PIN = 7;

----------rx-----------           
PPM pin 2
*/
#include <Arduino.h>
#include <Wire.h>
#include "MS561101BA.h"
#include "config.h"
#include "multi_rxPPM2560.h"
#include "mpu6050.h"
#include "ahrs.h"
#include "Control_PID.h"
#include "motorX6.h"
#include "GPS_multi.h"
#include "Ultrasonic.h"

float getAltitude(float pressure, float temperature)
{
  //return (1.0f - pow(press/101325.0f, 0.190295f)) * 4433000.0f;
  return ((pow((sea_press/pressure),1/5.257)-1.0)*(temperature+273.15))/0.0065;
}
void pushAvg(float val)
{
  movavg_buff[movavg_i] = val;
  movavg_i = (movavg_i + 1) % MOVAVG_SIZE;
}

float getAvg(float * buff, int size)
{
  float sum=0.0;
  for(int i=0;i<size;i++)
  {
    sum += buff[i];
  }
  return sum/size;
}
void setup()
{
  Serial.begin(38400);//38400
  pinMode(13, OUTPUT);pinMode (30, OUTPUT);pinMode (31, OUTPUT);//pinMode (30, OUTPUT);pinMode (31, OUTPUT);//(13=A=M),(31=B=STABLEPIN),(30=C,GPS FIG LEDPIN)
  digitalWrite(13, HIGH);
  //Serial1.begin(115200);//CRIUS Bluetooth Module pin code 0000
  //Serial2.begin(115200);//read gps from 1280
  //Serial3.begin(38400);//3DR Radio Telemetry Kit 433Mhz
  configureReceiver();//find multi_rx.h
  motor_initialize();//find motor.h
  ESC_calibration();//find motor.h
  GPS_multiInt();
  Wire.begin();
  delay(1);
  mpu6050_initialize();
  delay(1); 
  MagHMC5883Int();
  delay(1); 
  digitalWrite(13, HIGH);
  baro.init(MS561101BA_ADDR_CSB_LOW);
  UltrasonicInt();
  TWBR = ((F_CPU / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz 
  delay(1);
      for (uint8_t i=0; i<50; i++) 
    {
     mpu6050_Gyro_Values();
     mpu6050_Accel_Values();
     Mag5883Read();
     temperature = baro.getTemperature(MS561101BA_OSR_4096);
     presser = baro.getPressure(MS561101BA_OSR_4096);
     Altitude_baro += getAltitude(presser,temperature);
     delay(20);
    }
    Altitude_Ground = Altitude_baro/10.0;
    sea_press = presser;
    digitalWrite(13, LOW);
  sensor_Calibrate();//sensor.h
  ahrs_initialize();//ahrs.h
  //baroInitialize();//bmp085.h
  RC_Calibrate();//"multi_rxPPM2560.h"
  Serial.print("TK_Quadrotor_Run_Roop_100Hz");Serial.println("\t");
  sensorPreviousTime = micros();
  previousTime = micros();
  GPS_loopTimer = millis();
}

void loop()
{
    while(Serial2.available())
   {
     char byteGPS1=Serial2.read(); 
     GPS_UBLOX_newFrame(byteGPS1);
     GPS_LAT1 = GPS_coord[LAT]/10000000.0;
     GPS_LON1 = GPS_coord[LON]/10000000.0;
   }
  Dt_sensor = micros() - sensorPreviousTime;///////////Roop sensor/////////
  if(Dt_sensor <= 0)
  {
    Dt_sensor = 1001;
  }
    if(Dt_sensor >= 1000 && gyroSamples < 5)////Collect 3 samples = 2760 us  && gyroSamples < 5
    {  
        sensorPreviousTime = micros();
        mpu6050_readGyroSum();
        mpu6050_readAccelSum();
    }
   Dt_roop = micros() - previousTime;// 100 Hz task loop (10 ms)  , 5000 = 0.02626 ms
   if(Dt_roop <= 0)
   {
    Dt_roop = 10001; 
   }   
    if (Dt_roop >= 10000) 
    {
      previousTime = micros();
      G_Dt = Dt_roop*0.000001;
      frameCounter++;
      mpu6050_Get_accel();
      mpu6050_Get_gyro();
////////////////Moving Average Filters///////////////////////////
      GyroXf = (GyroX + GyroX2)/2.0;
      GyroYf = (GyroY + GyroY2)/2.0;
      GyroZf = (GyroZ + GyroZ2)/2.0;
      AccXf = (AccX + AccX2)/2.0;
      AccYf = (AccY + AccY2)/2.0;
      AccZf = (AccZ + AccZ2)/2.0;
      AccX2 = AccX;
      AccY2 = AccY;
      AccZ2 = AccZ;
      GyroX2 = GyroX;
      GyroY2 = GyroY;
      GyroZ2 = GyroZ;
////////////////Low pass filter/////////////////////////////////
      //GyroXf = GyroXf + (GyroX - GyroXf)*49.6*G_Dt; //29 - 49.4
      //GyroYf = GyroYf + (GyroY - GyroYf)*49.6*G_Dt;
      //GyroZf = GyroZf + (GyroZ - GyroZf)*49.6*G_Dt;
      //AccXf2 = AccXf2 + (AccX - AccXf2)*49.6*G_Dt;//15.4  //Low pass filter ,smoothing factor  α := dt / (RC + dt)
      //AccYf = AccYf + (AccY - AccYf)*49.6*G_Dt;//15.4
      //AccZf = AccZf + (AccZ - AccZf)*49.6*G_Dt;//15.4
//////////////////////////////////////////////////////////
      ahrs_updateMARG(GyroXf, GyroYf, GyroZf, AccXf, AccYf, AccZf, c_magnetom_x, c_magnetom_y, c_magnetom_z, G_Dt);//quaternion ,direction cosine matrix ,Euler angles
      //x_angle = x_angle + (GyroXf*RAD_TO_DEG*G_Dt);
      //x_angle = kalmanCalculateX(ahrs_r*RAD_TO_DEG, GyroX*RAD_TO_DEG, G_Dt);
      //y_angle = kalmanCalculateY(ahrs_p*RAD_TO_DEG, GyroY*RAD_TO_DEG, G_Dt);

      //Observer hz kalman , GPS_hz , GPS_vz
      float temp_vz = accrZ_Earth*1.09 + (baro_vzf - vz_hat)*13.72;//4.7 15.5
      vz_hat = vz_hat + temp_vz*G_Dt;
      vz_hat = constrain(vz_hat, -2, 2);//+-2 m/s
      applyDeadband(vz_hat, 0.01);//+-0.5
      float temp_hz = vz_hat + (Altitude_barof - Altitude_hat)*18.5;//12.54 4.5
      Altitude_hat = Altitude_hat + temp_hz*G_Dt;
      //Altitude_hat = constrain(Altitude_hat, 0.0, 1.2);//1.5 m By Ultrasonic
      //vz = vz + accrZ_Earth*G_Dt;
/*
      else//By Baro
      {
       float temp_vz = accrZ_cutg + 0.85*(baro_vzfilter - vz_hat);//0.85
       vz_hat = vz_hat + temp_vz*G_Dt;
       vz_hat = constrain(vz_hat, -2, 2);//+-2 m/s
       float temp_hz = vz_hat + 12.54*(baroAltitudeRunning - Altitude_hat);//12.54 4.5
       Altitude_hat = Altitude_hat + temp_hz*G_Dt;
      }
 */     
      //GPS Speed Low Pass Filter
     actual_speedXf = actual_speedXf + (actual_speedX - actual_speedXf)*10.17*G_Dt;//8.4  //cm/s
     actual_speedYf = actual_speedYf + (actual_speedY - actual_speedYf)*10.17*G_Dt;//1.17
     actual_speedXf = constrain(actual_speedXf, -200, 200);//+-200 cm/s
     actual_speedYf = constrain(actual_speedYf, -200, 200);//+-200 cm/s
//PID Control///////////
     Control_PIDRate();//Control_PID.h
     
         if (time_auto > 4 && endAuto == 0) //End waypoint quadrotor
        {
          timeOff++;
          if(timeOff >= 30)//relay 0.3 s timeOff
          {
           armed = 0;
          }
        }     
//////Out motor///////////
//armed = 1;
     motor_Mix();//"motor.h"
/////////////////////////
     motor_command(); ////////end Out motor//////
     
     presser = baro.getPressure(MS561101BA_OSR_4096);
     pushAvg(presser);
 if (frameCounter % TASK_50HZ == 0)// 50 Hz tak (20 ms)
 {
         computeRC();//multi_rx.h
         failsafeCnt++;
         //Fail_Safe();
       if (CH_THR < MINCHECK)  //////ARM and DISARM your Quadrotor///////////////
        {
            if (CH_RUD > MAXCHECK && armed == 0) 
            {
                armed = 1;
                digitalWrite(31, HIGH);
            }
            if (CH_RUD < MINCHECK && armed == 1) 
            {
                armed = 0;
                digitalWrite(31, LOW);
            }
        }//end  ARM and DISARM your helicopter///////////////       
   
}//end roop 50 Hz 
         if (frameCounter % TASK_20HZ == 0)// 20 Hz task (50 ms)
        {
          UltrasonicRead();
          temperature = baro.getTemperature(MS561101BA_OSR_4096);
          presser = getAvg(movavg_buff, MOVAVG_SIZE);
          Altitude_baro = getAltitude(presser,temperature);//Altitude_Ground
          Altitude_baro = constrain(Altitude_baro, -1, 1000);//1km
          Altitude_barof = Altitude_barof + (Altitude_baro - Altitude_barof)*0.25;
          baro_vz = (Altitude_barof - baro_vz_old)/0.05;
          baro_vz = constrain(baro_vz, -2, 2);//+-2 m/s
          baro_vzf = baro_vzf + (baro_vz - baro_vzf)*15.7*G_Dt;//2.5 Low Pass Filter vz baro
          Mag5883Read();  
        }//end roop 20 Hz
         if (frameCounter % TASK_10HZ == 0)// 10 Hz task (100 ms)
        {
          baro_vz_old = Altitude_barof;
          Automatictakeland();
        }//end roop 10 Hz
         if (frameCounter % TASK_5HZ == 0)//GPS_calc TASK_5HZ
        {
         //baroAltitudeold = baroAltitudeRunning;
         GPS_calc_positionhold();
        }
         if (frameCounter % TASK_20HZ == 0)//roop print  ,TASK_5HZ  TASK_10HZ
        {
            Serial.print(CH_THR);Serial.print("\t");
            Serial.print(CH_AIL);Serial.print("\t");  
            Serial.print(CH_ELE);Serial.print("\t");
            Serial.print(CH_RUD);Serial.print("\t");  
            //Serial.print(AUX_1);Serial.print("\t"); 
            //Serial.print(AUX_2);Serial.print("\t"); 
            //Serial.print(AUX_3);Serial.print("\t"); 
            //Serial.print(AUX_4);Serial.print("\t"); 
            //Serial.print(failsafeCnt);Serial.print("\t");
            
            //Serial.print(setpoint_rate_roll);Serial.print("\t");
            //Serial.print(setpoint_rate_pitch);Serial.print("\t"); 
             
            //Serial.print(MagX);Serial.print("\t");
            //Serial.print(MagXf);Serial.print("\t");
            //Serial.print(MagY);Serial.print("\t");
            //Serial.print(MagZ);Serial.print("\t");  
            
            //Serial.print(c_magnetom_x);Serial.print("\t");
            //Serial.print(c_magnetom_y);Serial.print("\t");
            //Serial.print(c_magnetom_z);Serial.print("\t"); 
            
            //Serial.print(GPS_FIX1);Serial.print("\t");
            //Serial.print(GPS_LAT1,9);Serial.print("\t"); 
            //Serial.print(GPS_LON1,9);Serial.print("\t");
            //Serial.print(GPS_speed);Serial.print("\t");//cm/s
            //Serial3.print(actual_speedXff);Serial3.print("\t");
            //Serial3.print(actual_speedXf);Serial3.print("\t");
            //Serial3.print(actual_speedYff);Serial3.print("\t");
            //Serial3.print(actual_speedYf);Serial3.print("\t");
            //Serial3.print(GPS_Distance);Serial3.print("\t");
            //Serial3.print(GPS_ground_course);Serial3.print("\t");
            //Serial3.print(Control_XEf);Serial3.print("\t");
            //Serial3.print(Control_YEf);Serial3.print("\t");
            //Serial3.print(Control_XBf);Serial3.print("\t");
            //Serial3.print(Control_YBf);Serial3.print("\t");
            
            //Serial.print(Control_XEf);Serial.print("\t");
            //Serial.print(Control_YEf);Serial.print("\t");
            //Serial.print(Control_XBf);Serial.print("\t");
            //Serial.print(Control_YBf);Serial.print("\t");
            
            //Serial.print(temperature);Serial.print("\t");
            //Serial.print(presser);Serial.print("\t");
            //Serial.print(Altitude_sonano);Serial.print("\t");
            //Serial.print(Altitude_barof);Serial.print("\t");
            //Serial.print(h_counter);Serial.print("\t");
            //Serial.print(GPS_hz);Serial.print("\t"); 
            //Serial.print(Altitude_hat);Serial.print("\t");
            //Serial3.print(Altitude_hat);Serial3.print("\t");
            //Serial.print(GPS_vz*10);Serial.print("\t");
            //Serial3.print(vz_hat*10);Serial3.print("\t");
            Serial.print(baro_vzf*10);Serial.print("\t");
            Serial.print(vz_hat*10);Serial.print("\t");
            //Serial.print(baro_vzfilter*10);Serial.print("\t");

            
            //Serial.print(AccX);Serial.print("\t");
            //Serial.print(AccXf);Serial.print("\t");
            //Serial.print(AccXf2);Serial.print("\t");
            //Serial.print(AccY);Serial.print("\t");  
            //Serial.print(AccYf);Serial.print("\t"); 
            //Serial.print(AccZ,3);Serial.print("\t");
            //Serial.print(AccZf);Serial.print("\t");       
            //Serial.print(accrX_Earth);Serial.print("\t");
            //Serial.print(accrY_Earth);Serial.print("\t");
            Serial.print(accrZ_Earth);Serial.print("\t");
            //Serial.print(accelRaw[XAXIS]);Serial.print("\t");
            //Serial.print(accelRaw[YAXIS]);Serial.print("\t");
            //Serial.print(accelRaw[ZAXIS]);Serial.print("\t");
            
            //Serial.print(GyroX*RAD_TO_DEG);Serial.print("\t");
            //Serial.print(GyroXf*RAD_TO_DEG);Serial.print("\t");
            //Serial.print(roll_D_rate);Serial.print("\t");
            //Serial.print(GyroY*RAD_TO_DEG,3);Serial.print("\t");
            //Serial.print(GyroYf*RAD_TO_DEG);Serial.print("\t");
            //Serial.print(GyroZf*RAD_TO_DEG);Serial.print("\t");
            //Serial.print(GyrofY);Serial.print("\t");  
            //Serial.print(GyroZ);Serial.print("\t");  
            //Serial.print(gyroRaw[XAXIS]);Serial.print("\t");
            //Serial.print(gyroRaw[YAXIS]);Serial.print("\t");
            //Serial.print(gyroRaw[ZAXIS]);Serial.print("\t");
   
            //Serial.print(ahrs_r*RAD_TO_DEG);Serial.print("\t");
            //Serial.print(ahrs_p*RAD_TO_DEG);Serial.print("\t");  
            //Serial.print(ahrs_y*RAD_TO_DEG);Serial.print("\t");  
            //Serial3.print(ahrs_y*RAD_TO_DEG);Serial3.print("\t"); 
            //Serial.print(cos_rollcos_pitch);Serial.print("\t"); 
             
            //Serial.print(x_angle);Serial.print("\t");
            
            //Serial.print(err_pitch_rate);Serial.print("\t");
            
            //Serial.print(motor_FrontL);Serial.print("\t");
            //Serial.print(motor_FrontR);Serial.print("\t");
            //Serial.print(motor_BackL);Serial.print("\t");
            //Serial.print(motor_BackR);Serial.print("\t");
            //Serial.print(motor_Left);Serial.print("\t");
            //Serial.print(motor_Right);Serial.print("\t");
            Serial.print(gyroSamples2);Serial.print("\t");
            //Serial.print(Dt_sensor);Serial.print("\t");
            Serial.print(G_Dt*1000);Serial.print("\t");
            //Serial.print(millis()/1000.0);//millis() micros()
            Serial.print("\n"); 
        }//end roop 5 Hz 
        if (frameCounter >= TASK_1HZ) { // Reset frameCounter back to 0 after reaching 100 (1s)
            frameCounter = 0;
            time_auto++;
            Remote_TrimACC();//motor.h
              if(Status_LED == LOW)
            {
            Status_LED = HIGH;
            }
            else
            {
            Status_LED = LOW;
            }
            digitalWrite(13, Status_LED);
            if(GPS_FIX == 1)
            {
            digitalWrite(30, !Status_LED);
            }
            else
            {
             digitalWrite(30, LOW);
            }
        }//end roop 1 Hz
    }//end roop 100 HZ 
}
