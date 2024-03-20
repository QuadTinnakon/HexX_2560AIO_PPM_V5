//#include "motorX6.h"
//motor Arduino boards AIO 2560 works on pins 2, 3, 5, 6, 7, and 8. On 
int MOTOR_FrontL_PIN = 2;
int MOTOR_FrontR_PIN = 5;
int MOTOR_BackL_PIN = 6;
int MOTOR_BackR_PIN = 3;
int MOTOR_Left_PIN = 8;
int MOTOR_Right_PIN = 7;
    
#define PWM_FREQUENCY 400   //400 in Hz
#define PWM_PRESCALER 8
#define PWM_COUNTER_PERIOD (F_CPU/PWM_PRESCALER/PWM_FREQUENCY)

int16_t motor_FrontL = 1000;
int16_t motor_FrontR = 1000;
int16_t motor_Right = 1000;
int16_t motor_Left = 1000;
int16_t motor_BackL = 1000;
int16_t motor_BackR = 1000;

void motor_command_all() 
{
  for (int j = 0 ; j <= 50 ; j++)
  {
   motor_FrontL = 1000;
   motor_FrontR = 1000;
   motor_Right = 1000;
   motor_Left = 1000;
   motor_BackL = 1000;
   motor_BackR = 1000;
  OCR3B = motor_FrontL*2; //  pin 2
  OCR3C = motor_BackR*2; //  pin 3  <<3
  OCR3A = motor_FrontR*2; //  pin 5
  OCR4A = motor_BackL*2; //  pin 6
  OCR4B = motor_Right*2; //  pin 7
  OCR4C = motor_Left*2; //  pin 8
  delay(20);
}
}
void motor_initialize() 
{
  pinMode(MOTOR_FrontL_PIN,OUTPUT); 
  pinMode(MOTOR_FrontR_PIN,OUTPUT);
  pinMode(MOTOR_BackL_PIN,OUTPUT);
  pinMode(MOTOR_BackR_PIN,OUTPUT);
  pinMode(MOTOR_Right_PIN,OUTPUT); 
  pinMode(MOTOR_Left_PIN,OUTPUT); 
     // init 16bit timer 3
    // Init PWM Timer 3                                       // WGMn1 WGMn2 WGMn3  = Mode 14 Fast PWM, TOP = ICRn ,Update of OCRnx at BOTTOM
    TCCR3A = (1<<WGM31)|(1<<COM3A1)|(1<<COM3B1)|(1<<COM3C1);  // Clear OCnA/OCnB/OCnC on compare match, set OCnA/OCnB/OCnC at BOTTOM (non-inverting mode)
    TCCR3B = (1<<WGM33)|(1<<WGM32)|(1<<CS31);                 // Prescaler set to 8, that gives us a resolution of 0.5us
    ICR3 = PWM_COUNTER_PERIOD;                                // Clock_speed / ( Prescaler * desired_PWM_Frequency) #defined above.
            // init 16bit timer 4
       // Init PWM Timer 4
      TCCR4A = (1<<WGM41)|(1<<COM4A1);
      TCCR4B = (1<<WGM43)|(1<<WGM42)|(1<<CS41);
      ICR4 = PWM_COUNTER_PERIOD;
            // Init PWM Timer 4
      TCCR4A = (1<<WGM41)|(1<<COM4A1)|(1<<COM4B1)|(1<<COM4C1);
      TCCR4B = (1<<WGM43)|(1<<WGM42)|(1<<CS41);
      ICR4 = PWM_COUNTER_PERIOD;
  motor_command_all();
}

//motor command
void motor_command() 
{
  OCR3B = motor_FrontL*2; //  pin 2
  OCR3C = motor_BackR*2; //  pin 3  <<3
  OCR3A = motor_FrontR*2; //  pin 5
  OCR4A = motor_BackL*2; //  pin 6
  OCR4B = motor_Right*2; //  pin 7
  OCR4C = motor_Left*2; //  pin 8
}
  /********************************************************************/
  /****           ESCs calibration                                 ****/
  /********************************************************************/
void ESC_calibration () {
  Serial.print("ESC_Calibrate");Serial.println("\t");
   for (int i = 0; i < 5; i++)
  {
    computeRC();
    if(CH_THR > MAXCHECK)
    {
     ESC_calibra = 1; 
    }
    else
    {
     ESC_calibra = 0;
     Serial.print("Not_ESC_Calibrate");Serial.println("\t");
    }
   delay(20);
  }
  int jprint = 0;
  while(ESC_calibra == 1){
   computeRC();
   motor_FrontL = (CH_THR - 500)*1.5;
   motor_FrontR = (CH_THR - 500)*1.5;
   motor_Right = (CH_THR - 500)*1.5;
   motor_Left = (CH_THR - 500)*1.5;
   motor_BackL = (CH_THR - 500)*1.5;
   motor_BackR = (CH_THR - 500)*1.5;
   
   motor_FrontL = constrain(motor_FrontL, MINCOMMAND, MAXCOMMAND);
   motor_FrontR = constrain(motor_FrontR, MINCOMMAND, MAXCOMMAND);
   motor_Right = constrain(motor_Right, MINCOMMAND, MAXCOMMAND);
   motor_Left = constrain(motor_Left, MINCOMMAND, MAXCOMMAND);
   motor_BackL = constrain(motor_BackL, MINCOMMAND, MAXCOMMAND);
   motor_BackR = constrain(motor_BackR, MINCOMMAND, MAXCOMMAND);
   
  //analogWrite(MOTOR_FrontL_PIN, motor_FrontL/8);
  //analogWrite(MOTOR_FrontR_PIN, motor_FrontR/8);
  //analogWrite(MOTOR_BackL_PIN, motor_BackL/8);
  //analogWrite(MOTOR_BackR_PIN, motor_BackR/8);
  //analogWrite(MOTOR_Left_PIN, motor_Left/8);
  //analogWrite(MOTOR_Right_PIN, motor_Right/8);
  OCR3B = motor_FrontL*2; //  pin 2
  OCR3C = motor_BackR*2; //  pin 3  <<3
  OCR3A = motor_FrontR*2; //  pin 5
  OCR4A = motor_BackL*2; //  pin 6
  OCR4B = motor_Right*2; //  pin 7
  OCR4C = motor_Left*2; //  pin 8
  
   jprint++;
   if(jprint > 10)
   {
     jprint = 0;
   Serial.print(motor_FrontL);Serial.print("\t");
   Serial.print(motor_Right);Serial.print("\t");    
   Serial.print(motor_Left);Serial.print("\t");     
   Serial.print(motor_BackL);Serial.println("\t");    
   //Serial1.println(motor_Back);
     if(Status_LED == LOW)
     Status_LED = HIGH;
     else
     Status_LED = LOW;
     digitalWrite(13, Status_LED);
     digitalWrite(30, Status_LED);
     digitalWrite(31, Status_LED);
   }
   delay(20);
  }
}
void motor_Mix(){
#ifdef HEX_X
      motor_FrontL = uAltitude + u_pitch*0.875 + u_roll*0.5 - u_yaw;//Front L
      motor_FrontR = uAltitude + u_pitch*0.875 - u_roll*0.5 + u_yaw;
      motor_Right = uAltitude - u_roll - u_yaw;
      motor_Left = uAltitude  + u_roll + u_yaw;
      motor_BackL = uAltitude - u_pitch*0.875 + u_roll*0.5 - u_yaw;
      motor_BackR = uAltitude - u_pitch*0.875 - u_roll*0.5 + u_yaw;
#endif
       if (CH_THR < MINCHECK) 
        {
          roll_I_rate = 0;
          pitch_I_rate = 0;
          yaw_I_rate = 0;
          motor_FrontL = 1000;
          motor_FrontR = 1000;
          motor_Right = 1000;
          motor_Left = 1000;
          motor_BackL = 1000;
          motor_BackR = 1000;
        }
        if(armed == 1)
        {
         motor_FrontL = constrain(motor_FrontL, MINTHROTTLE, MAXCOMMAND);
         motor_FrontR = constrain(motor_FrontR, MINTHROTTLE, MAXCOMMAND);
         motor_Right = constrain(motor_Right, MINTHROTTLE, MAXCOMMAND);
         motor_Left = constrain(motor_Left, MINTHROTTLE, MAXCOMMAND);
         motor_BackL = constrain(motor_BackL, MINTHROTTLE, MAXCOMMAND);
         motor_BackR = constrain(motor_BackR, MINTHROTTLE, MAXCOMMAND);
        }
        else
        {
          motor_FrontL = 1000;
          motor_FrontR = 1000;
          motor_Right = 1000;
          motor_Left = 1000;
          motor_BackL = 1000;
          motor_BackR = 1000;
        }
}
