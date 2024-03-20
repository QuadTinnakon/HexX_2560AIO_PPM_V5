//PID By tinnakon_za@hotmail.com

int u_roll = 0;
int u_pitch = 0;
int u_yaw = 0;
float roll_I_rate = 0.0;
float roll_D_rate = 0.0;
float setpoint_rollold = 0.0;
float setpoint_rate_roll = 0.0;
float error_rollold = 0.0;
float error_rate_rollold = 0.0;

float pitch_I_rate = 0.0;
float pitch_D_rate = 0.0;
float setpoint_pitchold = 0.0;
float setpoint_rate_pitch = 0.0;
float error_pitchold = 0.0;
float error_rate_pitchold = 0.0;

float yaw_I_rate = 0.0;
float yaw_D_rate = 0.0;
float error_rate_yawold = 0.0;

//Automatic take-off and landing
int time_auto = 0;
float  h_counter = 0.03;//0.08
float hz_I = 0.0;
uint8_t takeoff = 0;
uint8_t endAuto = 0;


void Control_PIDRate(){
   if(AUX_2 < 1750)
  {
    Control_XBf = 0.0;
    Control_YBf = 0.0;
  }
//I-PD By tinnakon
//if(AUX_1 >= 1300)//Flight Modes 2.Stabilize
// ROLL CONTROL P-PID-I///////////
  float setpoint_roll = ((CH_AIL-CH_AIL_Cal)*0.12) + Control_YBf;//max +-45 deg  ////+-18  + Control_YBf
  applyDeadband(setpoint_roll, 2.4);//1.2
  setpoint_rate_roll = (0.055*setpoint_rate_roll/(0.055+G_Dt)) + ((setpoint_roll-setpoint_rollold)/(0.055+G_Dt));//Diff remote
  setpoint_rollold = setpoint_roll;
  applyDeadband(setpoint_rate_roll, 1.5);//1.5
  setpoint_rate_roll = constrain(setpoint_rate_roll, -50, 50);//+-100 deg/s
  float error_roll = setpoint_roll - ahrs_r*RAD_TO_DEG;//ahrs_r*ToDeg
  float error_rate_roll = setpoint_rate_roll - GyroXf*RAD_TO_DEG;
  float errorui_roll = error_rate_roll + (error_roll*Kpi_rateRoll);//Control ui rate By tinnakon
  roll_I_rate += errorui_roll*Ki_rateRoll*G_Dt;
  roll_I_rate = constrain(roll_I_rate, -100, 100);//+-150
  roll_D_rate = (tar*roll_D_rate/(tar+G_Dt)) + ((error_rate_roll-error_rate_rollold)/(tar+G_Dt));
  error_rate_rollold = error_rate_roll;
  u_roll = Kp_rateRoll*error_rate_roll + roll_I_rate + Kd_rateRoll*roll_D_rate + Kp_levelRoll*error_roll;
  
////// PITCH CONTROL  P-PID-I///////////
  float setpoint_pitch = ((CH_ELE-CH_ELE_Cal)*-0.12) - Control_XBf;//max +-45 deg  ////+-18 - Control_XBf
  applyDeadband(setpoint_pitch, 2.4);//1.2
  setpoint_rate_pitch = (0.055*setpoint_rate_pitch/(0.055+G_Dt)) + ((setpoint_pitch-setpoint_pitchold)/(0.055+G_Dt));//Diff remote
  setpoint_pitchold = setpoint_pitch;
  applyDeadband(setpoint_rate_pitch, 1.5);//1.5
  setpoint_rate_pitch = constrain(setpoint_rate_pitch, -50, 50);
  float error_pitch = setpoint_pitch - ahrs_p*RAD_TO_DEG;//ahrs_p*RAD_TO_DEG
  float error_rate_pitch = setpoint_rate_pitch - GyroYf*RAD_TO_DEG;
  float errorui_pitch = error_rate_pitch + (error_pitch*Kpi_ratePitch);//Control ui rate By tinnakon
  pitch_I_rate += errorui_pitch*Ki_ratePitch*G_Dt;
  pitch_I_rate = constrain(pitch_I_rate, -100, 100);//+-150
  pitch_D_rate = (tar*pitch_D_rate/(tar+G_Dt)) + ((error_rate_pitch-error_rate_pitchold)/(tar+G_Dt));
  error_rate_pitchold = error_rate_pitch;
  u_pitch = Kp_ratePitch*error_rate_pitch + pitch_I_rate + Kd_ratePitch*pitch_D_rate + Kp_levelPitch*error_pitch;
  
////// YAW CONTROL PID///////////
  float setpoint_rate_yaw = (CH_RUD-CH_RUD_Cal)*0.4;//0.35
  applyDeadband(setpoint_rate_yaw, 7.1);//6.5
  float error_yaw = 0.0 - ahrs_y*RAD_TO_DEG;
  float error_rate_yaw = setpoint_rate_yaw - GyroZf*RAD_TO_DEG;
  float errorui_yaw = error_rate_yaw + (error_yaw*Kpi_rateYaw);
  yaw_I_rate += errorui_yaw*Ki_rateYaw*G_Dt;
  yaw_I_rate = constrain(yaw_I_rate, -150, 150);//+-100
  yaw_D_rate = (tar*yaw_D_rate/(tar+G_Dt)) + ((error_rate_yaw-error_rate_yawold)/(tar+G_Dt));
  error_rate_yawold = error_rate_yaw;
  u_yaw = Kp_rateYaw*error_rate_yaw + yaw_I_rate + Kd_rateYaw*yaw_D_rate + Kp_levelyaw*error_yaw;
  u_yaw = constrain(u_yaw, -290, 290);//+-300
   
  if(AUX_2 > 1750)//Altitude control, AUX_1 > 1250 && AUX_1 <= 1750
  {
    float err_hz = h_counter - Altitude_hat;
    hz_I = hz_I + (Ki_altitude*err_hz*G_Dt);//1.9 - 2.9  18.5  22.5
    hz_I = constrain(hz_I, -200, 200);//+-200
    float uthrottle2 = (err_hz*Kp_altitude) + hz_I - (vz_hat*Kd_altitude) - (accrZ_Earth*Ka_altitude);//- (accrZ_cutg*0.129);//- (accrZ_cutg*0.0129) + 9.81; //state feedback control Altitude m = 1290 g
    //uthrottle = uthrottle2*m_quad/(cos_roll*cos_pitch);// = u*m*d/b*(cosr*cosp)     ,m*d/b = 122.33  (cos_roll*cos_pitch)
    uthrottle = uthrottle2*m_quad/cos_rollcos_pitch;
  }
  else
  {
    hz_I = 0.0;
    uthrottle = 0.0;
  }//end Altitude control
  if(AUX_2 > 1250 && AUX_2 <= 1750)//Altitude Hold, 
  {
    float err_hz = Altitude_Hold - Altitude_hat;
    hz_I = hz_I + (Ki_altitude*err_hz*G_Dt);//1.9 - 2.9  18.5  22.5
    hz_I = constrain(hz_I, -200, 200);//+-200
    float uthrottle2 = (err_hz*Kp_altitude) + hz_I - (vz_hat*Kd_altitude) - (accrZ_Earth*Ka_altitude);//- (accrZ_cutg*0.129);//- (accrZ_cutg*0.0129) + 9.81; //state feedback control Altitude m = 1290 g
    //uthrottle = uthrottle2*m_quad/(cos_roll*cos_pitch);// = u*m*d/b*(cosr*cosp)     ,m*d/b = 122.33  (cos_roll*cos_pitch)
    uthrottle = uthrottle2*m_quad/cos_rollcos_pitch;
  }
  else
  {
    Altitude_Hold = Altitude_hat;
  }//end Altitude Hold
uAltitude = CH_THR + uthrottle;//m*g = 10.8 N = 
}

void Automatictakeland(){
 //Altitude control and 1 waypoint navigation
  if(AUX_2 > 1750 && CH_THR > MINCHECK && armed == 1)
  {
    takeoff = 1;
     if(Altitude_hat >= h_control && endAuto == 1)//waypoint1
    {
      target_LAT = waypoint1_LAT;
      target_LON = waypoint1_LON;
    }
     if(time_auto > 3 && abs(error_LAT) <= 10 && abs(error_LON) <= 10 && endAuto == 1)//10 Landing and position hold mode
    {
      timeLanding++;
      if(timeLanding >= 30)//relay 3 s Landing
      {
        takeoff = 0;
      }
    }
  }
  else
  {
    takeoff = 0;
    timeLanding = 0;
    timeOff = 0;
  }     
  
      if(h_counter == 0.0 && AUX_2 < 1750)
      {
        time_auto = 0;
        target_LAT = GPS_LAT1;//GPS_LAT_HOME
        target_LON = GPS_LON1;//GPS_LON_HOME
      }
       if(h_counter < h_control && takeoff == 1)//take-off
      {
        endAuto = 1;
        h_counter = h_counter + 0.033;//0.01- 0.03  0.1 = 10 s  //ramp input hz take-off
      }
       if(takeoff == 0)//landing
      {
        h_counter = h_counter - 0.033; //ramp input hz  landing
        if(h_counter <= 0.03)
        {
         h_counter = 0.0;
         endAuto = 0;
        }
      }
}

 void GPS_distance_m_bearing(float lat1, float lon1, float lat2, float lon2, float alt){
  float a, R, c, d, dLat, dLon;
  lon1=lon1/RAD_TO_DEG;
  lat1=lat1/RAD_TO_DEG;
  lon2=lon2/RAD_TO_DEG;
  lat2=lat2/RAD_TO_DEG;
  R=6371000.0;    //m raio da terra 6371km
  a=atan2(sin(lon2-lon1)*cos(lat2), cos(lat1)*sin(lat2)-sin(lat1)*cos(lat2)*cos(lon2-lon1));
  GPS_ground_course = a*RAD_TO_DEG;
  //if (yaw<0) yaw=360+yaw;
//calculo da distancia entre modelo e home
  dLat = (lat2-lat1);
  dLon = (lon2-lon1);
  a = sin(dLat/2) * sin(dLat/2) + sin(dLon/2) * sin(dLon/2) * cos(lat1) * cos(lat2); 
  c = 2* asin(sqrt(a));  
  d = 6371000 * c;
  //alt=alt-Alt_Home;
  //pitch=atan(alt/d);
  //pitch=pitch*360/(2*PI);
  GPS_Distance = sqrt(alt*alt+d*d);
}
  void GPS_calc_positionhold(){
          GPS_distance_m_bearing(GPS_LAT1, GPS_LON1, GPS_LAT_HOME, GPS_LON_HOME, Altitude_hat);
          error_LAT = (target_LAT - GPS_LAT1)*1000000.0; // X Error
          error_LON = (target_LON - GPS_LON1)*1000000.0;// Y Error
          error_LAT = constrain(error_LAT,-100,100);//50 = +-5 m
          error_LON = constrain(error_LON,-100,100);
          float target_speedLAT = error_LAT*Kp_speed;//P Control Velocity GPS
          float target_speedLON = error_LON*Kp_speed;//P Control Velocity GPS
          target_speedLAT = constrain(target_speedLAT,-70,70);//+-100 cm/s = 1m/s
          target_speedLON = constrain(target_speedLON,-70,70);

          float error_rate_LAT = target_speedLAT - actual_speedXf;
          float error_rate_LON = target_speedLON - actual_speedYf;
          error_rate_LAT = constrain(error_rate_LAT,-200,200);//+-200 cm/s
          error_rate_LON = constrain(error_rate_LON,-200,200);
          //Control_XEf = error_rate_LAT*Kd_gps;//P Control speed 
          //Control_YEf = error_rate_LON*Kd_gps;
          Control_XEf = error_LAT*Kp_gps + error_rate_LAT*Kd_gps;//PD Control speed 
          Control_YEf = error_LON*Kp_gps + error_rate_LON*Kd_gps;
          
          //The desired roll and pitch angles by tinnakon 
          float urolldesir = (Control_YEf*m_quad)/uAltitude;
          float upitchdesir = (Control_XEf*m_quad)/uAltitude;
          urolldesir = constrain(urolldesir,-0.7,0.7);//+-0.7 = +-44.427
          upitchdesir = constrain(upitchdesir,-0.7,0.7);
          float temp_YBf = asin(urolldesir)*RAD_TO_DEG;
          float temp_XBf = asin(upitchdesir)*RAD_TO_DEG;
          Control_XBf = (DCM00*temp_XBf) + (DCM01*temp_YBf);//Control Body Frame
          Control_YBf = (DCM10*temp_XBf) + (DCM11*temp_YBf);//Control Body Frame
          //Control_XBf = constrain(Control_XBf, -20, 20);//+-20 deg
          //Control_YBf = constrain(Control_YBf, -20, 20);//+-20 deg
          
          //The desired roll and pitch angles by paper Modeling and Backstepping-based Nonlinear Control 
          //Ashfaq Ahman Mian 2008 (eq.25 and eq.26)
          //float urolldesir = ((Control_XEf*m_quad*sin(ahrs_y))/uAltitude) - ((Control_YEf*m_quad*cos_yaw)/uAltitude);
          //float upitchdesir = ((Control_XEf*m_quad)/(uAltitude*cos_roll*cos_yaw)) - ((sin(ahrs_r)*sin(ahrs_y))/(cos_roll*cos_yaw));
          //urolldesir = constrain(urolldesir,-0.7,0.7);//+-0.7 = +-44.427
          //upitchdesir = constrain(upitchdesir,-0.7,0.7);
          //Control_YBf = asin(urolldesir)*RAD_TO_DEG;//Control roll eq.25 
          //Control_XBf = asin(upitchdesir)*RAD_TO_DEG*-1.0;//Control roll eq.26
          
          Control_XBf = constrain(Control_XBf, -20, 20);//+-20 +- 44
          Control_YBf = constrain(Control_YBf, -20, 20);
          
          GPS_LAT1_old = GPS_LAT1;
          GPS_LON1_old = GPS_LON1;
  }
