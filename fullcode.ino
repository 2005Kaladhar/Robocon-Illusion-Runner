/*
 Sample Line Following Code for the Robojunkies LF-2 robot
*/

#include <SparkFun_TB6612.h>

#define AIN1 4
#define BIN1 6
#define AIN2 3
#define BIN2 7
#define PWMA 9
#define PWMB 10
#define STBY 5

// these constants are used to allow you to make your motor configuration
// line up with function names like forward.  Value can be 1 or -1
const int offsetA = 1;
const int offsetB = 1;

// Initializing motors.  The library will allow you to initialize as many
// motors as you have memory for.  If you are using functions like forward
// that take 2 motors as arguements you can either write new functions or
// call the function more than once.


// motor2 is left motor
// motor1 is right motor

// Sensor Connections are : 
/*
Out5 = A1
Out4 = A2
Out3 = A3
Out2 = A4
Out1 = A5
*/


Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);


int P, D, I, previousError, PIDvalue, error;
int lsp, rsp;

//Line follow speed
int lfspeed = 80;

float Kp = 17;
float Kd = 24;
float Ki = 0.0001;

const int pushButton = A0;
const int led = 2;
int i;


int minValues[8], maxValues[8], threshold[8];

void setup()
{
  Serial.begin(9600);
 pinMode(pushButton,INPUT);
 pinMode(led,OUTPUT);
}


void loop()
{
  delay(1000);
  calibrate();

  for(i=0;i<20;i++){
  digitalWrite(led,HIGH);
  delay(50);
  digitalWrite(led,LOW);
  delay(50);

  }
  digitalWrite(led,HIGH);
  while (1){
    if (analogRead(pushButton) ==1023 ){
      digitalWrite(led,LOW);
      break;
  }
  }
  delay(1000);

  while (1)
  {
    
    // Sharp right turn at 90 degree
    if ( (analogRead(1) > threshold[1] && analogRead(5) < threshold[5] ) )
    {
      lsp = 0; rsp = lfspeed;
      delay(10);
      motor1.drive(0);
    motor2.drive(255);
    delay(30);
    }

    // Sharp left turn at 90 degree
    else if ( (analogRead(5) > threshold[5] && analogRead(1) < threshold[1]))
    { lsp = lfspeed; rsp = 0;
    delay(10);
      motor1.drive(255);
      motor2.drive(0);
      delay(37);
    }
    else if ((analogRead(3) > threshold[3]))
    {
  // error is multiplied with constant Kp - sterring value
  // Higher the Kp value more will be the oscillations and lower the kp value and it wont turn fast enough
   
      Kp = 0.0006 * (1000 - ((analogRead(3))));
      // Kp = 0.08;
     // Kp=17;
      Kd = 24;
      Ki = 0.0001;
      linefollow();
    }
  }
}

void linefollow()
{
  int error = ( (analogRead(1)+analogRead(2)) - (analogRead(5) + analogRead(4)));
  P = error;
  I = I + error;
  D = error - previousError;

  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  lsp = lfspeed - PIDvalue;
  rsp = lfspeed + PIDvalue;

  if (lsp > 255) {
    lsp = 255;
  }
  if (lsp < 0) {
    lsp = 0;
  }
  if (rsp > 255) {
    rsp = 255;
  }
  if (rsp < 0) {
    rsp = 0;
  }
  motor2.drive(rsp);
  motor1.drive(lsp);

}

void calibrate()
{
  for ( int i = 1; i < 6; i++)
  {
    minValues[i] = analogRead(i);
    maxValues[i] = analogRead(i);
  }
  
  for (int i = 0; i < 5000; i++)
  {
    motor1.drive(50);
    motor2.drive(-50);

    for ( int i = 1; i < 6; i++)
    {
      if (analogRead(i) < minValues[i])
      {
        minValues[i] = analogRead(i);
      }
      if (analogRead(i) > maxValues[i])
      {
        maxValues[i] = analogRead(i);
      }
    }
  }

  for ( int i = 1; i < 6; i++)
  {
    threshold[i] = (minValues[i] + maxValues[i]) / 2;
    Serial.print(threshold[i]);
    Serial.print("   ");
  }
  // Serial.println();
  
  motor1.drive(0);
  motor2.drive(0);
}
