int pot1 = 0;
int pot2 = 1;
int pot3 = 2;
double sensorval1, sensorval2, sensorval3;
double kp, ki, kd;



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
 
}

void loop() {
  delay(500);
  // put your main code here, to run repeatedly:
  Serial.println("----- analog read ----");
  sensorval1 = analogRead(pot1);
  Serial.println(sensorval1);
  sensorval2 = analogRead(pot2);
  Serial.println(sensorval2);
  sensorval3 = analogRead(pot3);
  Serial.println(sensorval3);
  Serial.println("----- end read ----");
   Serial.println("----- map val read ----");
  kp = (map(sensorval1, 0, 1023, 0, 500)/100.0); //Map function rounds 
  ki = (map(sensorval2, 0, 1023, 0, 500)/100.0);
  kd = (map(sensorval3, 0, 1023, 0, 500)/100.0);
  Serial.print(kp);
  Serial.print("\t");
  Serial.print(ki);
  Serial.print("\t");
  Serial.println(kd);
   Serial.println("----- end map read ----\n\n\n");
  
}
