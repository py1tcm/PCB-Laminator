/*
 * Using parts of above projects:
 * Frequency controller: https://forum.arduino.cc/t/frequency-gen-1hz-900khz-controlled-by-serial-monitor/467803
 * Temperature measure: https://create.arduino.cc/projecthub/Marcazzan_M/how-easy-is-it-to-use-a-thermistor-e39321
 * PID control: http://electronoobs.com/eng_arduino_tut24.php

 * Frequqnecy calc (ICR1): F_CPU / freq / 2 + 1
 * Duty cicle calc 50% (ICR1A): ((F_CPU / freq / 2 + 1) + 2) / 2 
*/

//Motor control
int pwmPin = 9;    // Pwm out
int enaPin = 10;   // Enable motor at low level
int Hz = 0;        // variable for Freq pwm motor control
//End motor

//Temp control
#define RT0 230000             // Ω  valor do NTC a 25º
#define B 4161                 // Thermistor beta value 
#define VCC 5.00               // Supply voltage
#define R 220000               // R=220KΩ  AD divider resistor
float RT, VR, ln, TX, T0, VRT; // Variables
const int  samples = 10;       // Samples quantity for temperature calc
//End Temp

//LCD control
#include <LiquidCrystal.h>
LiquidCrystal lcd(2, 4, 5, 6, 7, 8);
unsigned long timeLcd = 0;
const long timeUpdate = 1000;  // LCD refresh time
//Array for celsuis simbol
byte grau[8] ={ B00001100,
                B00010010,
                B00010010,
                B00001100,
                B00000000,
                B00000000,
                B00000000,
                B00000000,};
//End LCD

//PID fuser control
int lamPwm = 3;               // Fuser pin
float temperature_read = 0.0;
float set_temperature = 0;    // Initial set point
float PID_error = 0;
float previous_error = 0;
float elapsedTime, Time, timePrev;
int PID_value = 0;
int kp = 9.1;
int ki = 0.3;
int kd = 1.8;
int PID_p = 0;
int PID_i = 0;
int PID_d = 0;
float last_kp = 0;
float last_ki = 0;
float last_kd = 0;
int PID_values_fixed =0;
//End PID fuser

//start / stop button control
int button = A1;
//End button

//Fan and Led control
int fanPin = 11;      // Fan pin
int redledPin = 12;   //Red Led pin
int greenledPin = 13; //Green Led pin
//End fan and led

void setup() {

//Motor
  pinMode(pwmPin, OUTPUT);
  pinMode(enaPin, OUTPUT);
  digitalWrite(enaPin, HIGH); // Avoid random motor enable
//End motor

//Serial
  Serial.begin(9600);  // ATTENTION >> don´t remove this line, prevent sketchs update issues 
  Serial.println("Laminator Version 1.0 by PY1TCM - 01/09/2022");
//End serial

//Temp
  T0 = 25 + 273.15;  //Temperature T0 from datasheet, conversion from Celsius to kelvin
//End temp

//PID fuser control
  pinMode(lamPwm, OUTPUT);
  digitalWrite(lamPwm, LOW);          // Avoid random fuser enable
  TCCR2B = TCCR2B & B11111000 | 0x03; // pin 3 and 11 PWM frequency of 928.5 Hz
//End PID fuser

//LCD
  lcd.begin(16, 2);
  lcd.setCursor(5, 0);
  lcd.print("PY1TCM");
  lcd.setCursor(1, 1);
  lcd.print("Laminator V1.0");
  delay(2000);
  lcd.clear();
  lcd.createChar(0, grau);     // Create celsius simbol
//End LCD

//start/stop button
  pinMode(button, INPUT_PULLUP);
//End button

//Fan and led control
  pinMode(fanPin, OUTPUT);
  pinMode(redledPin, OUTPUT);
  pinMode(greenledPin, OUTPUT);
  digitalWrite(redledPin, HIGH);
  digitalWrite(greenledPin, HIGH);
//End fan and led  
}

void loop() {

  Temp();

  Pid();
  
  Button_control();
}

void PWMgen() {
  ICR1 = F_CPU / Hz / 2 + 1;
  OCR1A = (F_CPU / Hz / 2 + 1) / 2 + 1;
  TCCR1A = _BV(COM1A1)    // canal A on non-inverting mode
           | _BV(COM1B0);  
  TCCR1B = _BV(WGM13)   
           | _BV(CS10);   // prescaler = 1
}

void Temp() {
  int sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(A0);
    delay (10);    
  }
  VRT = (VCC*sum)/(samples*1024.0); //voltage conversion from samples
  VR = VCC - VRT;
  RT = VRT / (VR / R);              //Resistance of RT
  ln = log(RT / RT0);
  TX = (1 / ((ln / B) + (1 / T0))); //Temperature from thermistor
  TX = TX - 273.15;                 //Conversion to Celsius
  unsigned long timeCur = millis();
  // LCD temp update
  if (timeCur - timeLcd >= timeUpdate) {
    timeLcd = timeCur;
    lcd.setCursor(0, 1);
    lcd.print("       ");
    lcd.setCursor(0, 1);
    lcd.print(TX);
    lcd.write((byte)0);
  }
}

void Pid() {
//First we read the real value of temperature
  temperature_read = TX;
//Next we calculate the error between the setpoint and the real value
  PID_error = set_temperature - temperature_read;
//Calculate the P value
  PID_p = kp * PID_error;
//Calculate the I value in a range on +-3
  if(-3 < PID_error <3) {
    PID_i = PID_i + (ki * PID_error);
  }
//For derivative we need real time to calculate speed change rate
  timePrev = Time;                            // the previous time is stored before the actual time read
  Time = millis();                            // actual time read
  elapsedTime = (Time - timePrev) / 1000; 
//Now we can calculate the D calue
  PID_d = kd*((PID_error - previous_error)/elapsedTime);
//Final total PID value is the sum of P + I + D
  PID_value = PID_p + PID_i + PID_d;
//We define PWM range between 0 and 255
  if(PID_value < 0) {
    PID_value = 0;    
  }
  if(PID_value > 255) {
    PID_value = 255;  
  }
//  analogWrite(lamPwm,255-PID_value); // PWM inverted output (0=max 255=min)
  analogWrite(lamPwm,PID_value); //PWM normal output (0=min 255=max)
  previous_error = PID_error;     //Remember to store the previous error for next loop.
  delay(200);
}

void Button_control() {
  if (digitalRead(button) == LOW) {
    set_temperature = 180;  //Setpoint temperature
    if (TX < 165) {
      Hz = 560;
      PWMgen();
      analogWrite(fanPin, 160);
      digitalWrite(enaPin, LOW);
      digitalWrite(redledPin, LOW);
      digitalWrite(greenledPin, HIGH);
      lcd.setCursor(0, 0);
      lcd.print("38 Rpm");
      lcd.setCursor(8, 0);
      lcd.print("Warming ");
      lcd.setCursor(8, 1);
      lcd.print("Fan mid ");
    }
    else  {
      Hz = 200;
      PWMgen();
      analogWrite(fanPin, 96);
      digitalWrite(enaPin, LOW);
      digitalWrite(redledPin, HIGH);
      digitalWrite(greenledPin, LOW);
      lcd.setCursor(0, 0);
      lcd.print("13 Rpm");
      lcd.setCursor(8, 0);
      lcd.print("Running ");
      lcd.setCursor(8, 1);
      lcd.print("Fan low ");
    }
  }    
  else {
    set_temperature = 0;  // Setpoint 0 for PID disable
    if (TX >= 50) {
      Hz = 840;
      PWMgen();
      analogWrite(fanPin, 255);
      digitalWrite(enaPin, LOW);
      digitalWrite(redledPin, LOW);
      digitalWrite(greenledPin, HIGH);
      lcd.setCursor(0, 0);
      lcd.print("56 Rpm");
      lcd.setCursor(8, 0);
      lcd.print("Cooling ");
      lcd.setCursor(8, 1);
      lcd.print("Fan high");
    }
    else if (TX < 50 and TX > 40){
      Hz = 0;
      PWMgen();
      analogWrite(fanPin, 192);
      digitalWrite(enaPin, HIGH);
      digitalWrite(redledPin, HIGH);
      digitalWrite(greenledPin, HIGH);
      lcd.setCursor(0, 0);
      lcd.print(" 0 Rpm");
      lcd.setCursor(8, 0);
      lcd.print("Finish  ");
      lcd.setCursor(8, 1);
      lcd.print("Fan mid ");
    }
    else {
      Hz = 0;
      PWMgen();
      analogWrite(fanPin, 0);
      digitalWrite(enaPin, HIGH);
      digitalWrite(redledPin, HIGH);
      digitalWrite(greenledPin, HIGH);
      lcd.setCursor(0, 0);
      lcd.print(" 0 Rpm");
      lcd.setCursor(8, 0);
      lcd.print("Ready!  ");
      lcd.setCursor(8, 1);
      lcd.print("Fan off ");
    }
  }
}
