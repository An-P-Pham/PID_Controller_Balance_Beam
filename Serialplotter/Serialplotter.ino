int sensorVal1, sensorVal2;
const int xaxis = 0;
const int yaxis = 1;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  sensorVal1 = analogRead(xaxis);
  sensorVal2 = analogRead(yaxis);
  plot(); 
}

void plot()
{
  Serial.print(treatValue(sensorVal1));
  Serial.print(" "); // needed for 2 values 
  Serial.println(treatValue(sensorVal2)); //last value must be println
}

int treatValue(int data) {
  return (data * 5 /1024 ); //scale the data from 1-9
}

