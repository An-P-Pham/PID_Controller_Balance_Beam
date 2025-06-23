  //To do list:
//Mode filter
//use the 2nd function
//Tune the constants
//Make a plotter
//understand the sampletimes
//work the output limits

#include<Servo.h> //Servo library
#include<NewPing.h>  //"More accurate" Servo functions
//#include<LiquidCrystal.h>

//define the pins for sonar/
#define trigPin 13
#define echoPin 12
#define Max_Distance 28

//variables list
unsigned long lastTime;
double input, output, ITerm, lastInput;
//original k constants double kp = 0.8, ki = 0.07, kd = 0.12 <-- tips online
double kp = 2.5 , ki = 0.25 , kd = 0.3 ; //3.1 1.75 1.25 || 2, .75, 1.25 //1.5, .25, .3 (best so far)
double setpoint = 14;
double outMin = -90; //min and max output values
double outMax = 90;
const int servoPin = 9;
int SampleTime = 100; //play around to tune (100) works the best so far
/*const int pot1 = 0;
const int pot2 = 1;
const int pot3 = 2;
double sensorVal1, sensorVal2, sensorVal3;*/

//intailize objects
NewPing sonar(trigPin, echoPin, Max_Distance); 
Servo myServo;
//LiquidCrystal lcd(7, 6, 5, 4, 3, 2);

void setup() {
  Serial.begin(19200);
 // lcd.begin(16,2);
  myServo.attach(servoPin);
  readPosition();
  myServo.write(90); //degree for structure sit flat
  SetOutputLimits(outMin, outMax);
  SetTunings(kp, ki, kd);
  SetSampleTime(SampleTime);
}

//things needed tune, sampletime, gains 

void loop() {
  
  readPosition();
  Compute();
  myServo.write(90-output); 
 // delay(15); //Experiment this later for the response!!
  printData(setpoint, input, output);
  newGains();

}

/* obtain the distance of the ball */
void readPosition(){
  delay(36);
  unsigned int uS = sonar.ping_median(7); //Experiment with this affecting other constants
  input = uS/ US_ROUNDTRIP_CM;
}

// The Print Function 
void printData(double set, double in, double out)
{                       //^ those are local variables
  Serial.print("Setpoint = ");
  Serial.print(set);
  Serial.print(" \t Input = ");
  Serial.print(in);
  Serial.print(" \t Output = ");
  Serial.print(out);
  Serial.print("\n");
}


void Compute()
{
  //How long since we last calculated
   unsigned long now = millis();
   int timeChange = (now - lastTime);
    if(timeChange >= SampleTime)
    {
   //Compute all the working error variables
   double error = setpoint - input;
   ITerm += (ki * error);
  if(ITerm > outMax) ITerm = outMax;
  else if(ITerm < outMin) ITerm = outMin;
   double dInput = (input - lastInput);
  
   //Compute PID Output
   output = kp * error + ITerm - kd * dInput;
   if(output > outMax) output = outMax;
   else if(output < outMin) output = outMin;
   //Remember some variables for next time
   lastInput = input;
   lastTime = now;
    }
} 

void SetTunings(double Kp, double Ki, double Kd)
{
  double SampleTimeInSec =((double)SampleTime)/1000;
  kp = Kp;
  ki = Ki * SampleTimeInSec;
  kd = Kd/ SampleTimeInSec;
}

void SetSampleTime(int NewSampleTime)
{
  if (NewSampleTime > 0)
  {
    double ratio = (double)NewSampleTime/(double)SampleTime;

    ki *= ratio;
    kd /= ratio;
    SampleTime = (unsigned long)NewSampleTime;
  }
}

void SetOutputLimits(double Min, double Max)
{
  if(Min > Max) return;
  outMin = Min;
  outMax = Max;

  if(output > outMax) output = outMax;
  else if(output < outMin) output = outMin;

  if(ITerm > outMax) ITerm = outMax;
  else if(ITerm < outMin) ITerm = outMin;
}

void newGains() {
  if (Serial.available() > 0) {                                        //If there is any data in serial input, then it starts reading.
     delay(100);                                                       //Pause for a tenth of a second to readjust plant or anything else you need to do.
                                                                       //  Can be as long as needed.     
     for (int i = 0; i < 4; i = i + 1) {                               
       switch (i) { 
         case 0:                                                       //Reads 1st value in
           kp = Serial.parseFloat(); 
           break; 
         case 1:                                                       //Reads 2st value in 
           ki = Serial.parseFloat(); 
           break; 
         case 2:                                                       //Reads 3st value in
           kd = Serial.parseFloat(); 
           break; 
         case 3:                                                       //Clears any remaining parts.
           for (int j = Serial.available(); j == 0; j = j - 1) { 
             Serial.read();  
           }        
           break; 
       }
     }
     Serial.print(" Kp, Ki, Kd = ");                                     //Prints new gain values to Serial
     Serial.print(kp); 
     Serial.print(", "); 
     Serial.print(ki); 
     Serial.print(", "); 
     Serial.println(kd);  
     SetTunings(kp, ki, kd);
   }
   
}

/*
void plot()
{
  Serial.print(setpoint); // kp portion of error
  Serial.print(" "); //needed so plotter knows the different values to plot
  Serial.println(output); //ki portion of error
                    // println needed for the last value
}
/*
int treatValue(int data){
  return (data * 5 / 10240); // scale the pot data from 1-5
}

void pottune()
{
  sensorVal1 = analogRead(pot1); //manually tune gains with pot-meter
  sensorVal2 = analogRead(pot2);  //the analog values are then translated
  sensorVal3 = analogRead(pot3); // into 1-5 
  kp = (map(sensorVal1, 0, 1023, 0, 500)/ 100.0);
  ki = (map(sensorVal2, 0, 1023, 0, 500)/ 100.0);
  kd = (map(sensorVal3, 0, 1023, 0, 500)/ 100.0);
  
}

void lcdprint()
{
  lcd.setCursor(0,0);
  lcd.print("kp:");
  lcd.print(" ");
  lcd.print("ki:");
  lcd.print(" ");
  lcd.print("kd:");
  lcd.setCursor(0, 1);
  lcd.print(kp);
  lcd.print(" ");
  lcd.print(ki);
  lcd.print(" ");
  lcd.print(kd);
}*/

