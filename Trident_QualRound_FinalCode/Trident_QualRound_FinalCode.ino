

#include <PulseInZero.h>


const float SpeedOfSound       = 343.2; // ~speed of sound (m/s) in air, at 20°C         
const float  MicrosecondsToMillimetres2  = (SpeedOfSound/ 1000.0 )/2;

const int SIGNAL_PIN  = 11;   //fSonar trigg pin
                           


unsigned long lastTime  = 0;

int pingTimer     = 0;
int pingDelay     = 500; // milliseconds between ping pulses

//f is set to greater than 20 for safety
int fDistance = 25;

//nokol^

#define sensorNum 8
#define maxSpeed 255

int blackLimit[sensorNum];
float safety=0.35;
int time=5;

const int motorPin1 = 9,motorPin2 = 10;        //right motor
const int motorPin3 = 5,motorPin4 = 6;       //left motor

int sensorValue = 0;
int error, prevError = 0;
int mappedValue, targetValue;

float kp = 5.7;
float kd = 0.2;
float ki;

int motorResponse;
float correction;

int surface = 0; //defualt 9 is set to black on white

int leftSpeed, rightSpeed;

//trigger variables
int surfChange = 0; //detects surface change in order to change all black decisions 

int acuteKey = 0; //prepares for first acute angle rotate


int stopKey = 0; //prepares to stop at endpoint

int obsKey = -1;

int allBlackKey = 0; //prevents plus intersection decision 

//time delays
int danceDelay = 280;
int blackDelay = 500;
int intersectionDelay = 300;


int prev, curr, diff;






void setup()
{

  pinMode(SIGNAL_PIN, OUTPUT);
  digitalWrite(SIGNAL_PIN, LOW); 

  //initialize IR pins
  for(int i = 0; i < sensorNum; i++)
  {
    pinMode(A0 + i, INPUT);
    
  }

  //initialize motor pins
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT); 
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);



  //3sec Delay before auto calibration starts
  delay(3000);
  calibration();

  //sonar reading using interrupt
  PulseInZero::setup(pingPulseComplete);

  Serial.begin(9600);
}







void loop()
{

  unsigned long time = millis();
  unsigned long dt   = time - lastTime;
  lastTime       = time;
  
  pingTimer += dt;
  //new trigger if timer is greater than Delay
  if(pingTimer > pingDelay)
  {  
  pingTimer = 0;
  ping();
  }


//take Ir reading
sensorRead();

//mapReading for PID
sensorMapping();


//checks for obstacles
if  (fDistance < 20 && obsKey == 0) 
{ 

   //resets acuteKey variables to reverse false triggers
   curveRun();
   //fDistance is set to greater than 20 to avoid repeated curveRun
   fDistance = 100;
   //prepare for first acute turn
   acuteKey = 1;


   //stops sonar trigger
   surfChange = 0;
  


}

//reactions for normal line
if((mappedValue != 100) && (mappedValue != 111))
{
 
  pid();

  motor(leftSpeed, rightSpeed);

}


else if(mappedValue == 100)
{
   //Reaction for all-white 
   if(acuteKey == 1)
    { 
      //safe acute key move 
      dance();

      //prepare to stop at endpoint
      stopKey = 1;
    }

    else
    {
        findPath();
    }

}

else if((mappedValue == 111)&&(surface == 1))
{     
      //keep turning if line missed after a right angle turn
      //  findPath(); //turned off findpath for black
}

//stops at endpoint
else if((mappedValue == 111)&&(surface == 0)&&(stopKey == 1))
{
    delay(300);
    brake();
}



//plus intersection decision for Qualification round
else if((mappedValue == 111)&&(surface == 0)&&(surfChange == 1)&&(allBlackKey == 1))  
{

  resetAcuteKey();

  //rotates to go to right road after inverted surface has been passed
  plannedCRotate();
  delay(intersectionDelay);

}

//go forward on all black before the inverted surface
 else if((mappedValue == 111)&&(surface == 0))
 {

  mappedValue = 0;

  pid();
  motor(leftSpeed, rightSpeed);

  //registers that bot has atleast reached the loop before the inverted surface
  allBlackKey = 1;

 }

}

//store sensor reading as an 8-Bit binary value
void sensorRead(void)
{
  int digitalValue;
  sensorValue = 0;
    
  for (int i= 0; i< sensorNum; i++)
  { 
    if (analogRead(A0+i) < blackLimit[i]) digitalValue = 1;  //A7 is leftmost IR
    else digitalValue = 0;

    sensorValue |= (digitalValue << (sensorNum-1-i));
  }
  Serial.println(sensorValue,BIN);
  
}

//mapping the patterns for pid
void sensorMapping(void)
{
  if (sensorValue == 0b00000000) {mappedValue = 100; return;}
  else if (sensorValue == 0b11111111) {mappedValue = 111; return;}
  

  //detect surface
  if ((sensorValue == 0b11100111)
||(sensorValue == 0b11000111)
||(sensorValue == 0b11100011)
||(sensorValue == 0b10011111)
||(sensorValue == 0b11000111)
||(sensorValue == 0b11110001)
||(sensorValue == 0b11111001)) 
    {
      surface = 1;
      //if(allBlackKey == 1)
      surfChange=1;
    }

  else if((sensorValue == 0b00011000)
||(sensorValue == 0b00111000)
||(sensorValue == 0b00011100)
||(sensorValue == 0b01100000)
||(sensorValue == 0b00111000)
||(sensorValue == 0b00001110)
||(sensorValue == 0b00000110)) 
    {
      surface = 0;
      if(surfChange == 1) obsKey = 0;
      
    }

if(surface == 0)
{  
 

  //90 deg right turn
  if (sensorValue == 0b00011111 || sensorValue == 0b00001111 || sensorValue == 0b00111111) mappedValue = 80;         
 

  //line at right
  else if (sensorValue == 0b00000001) mappedValue = 60;
  else if (sensorValue == 0b00000011) mappedValue = 50;
  else if (sensorValue == 0b00000010) mappedValue = 40;
  else if (sensorValue == 0b00000111) mappedValue = 40;
  else if (sensorValue == 0b00000110) mappedValue = 30;
  else if (sensorValue == 0b00000100) mappedValue = 30;
  else if (sensorValue == 0b00001110) mappedValue = 30;
  else if (sensorValue == 0b00001100) mappedValue = 20;
  else if (sensorValue == 0b00001000) mappedValue = 20;
  else if (sensorValue == 0b00011100) mappedValue = 10;

  //line at left
  else if (sensorValue == 0b10000000) mappedValue = -60;
  else if (sensorValue == 0b11000000) mappedValue = -50;
  else if (sensorValue == 0b01000000) mappedValue = -40;
  else if (sensorValue == 0b11100000) mappedValue = -40;
  else if (sensorValue == 0b01100000) mappedValue = -30;
  else if (sensorValue == 0b00100000) mappedValue = -30;
  else if (sensorValue == 0b01110000) mappedValue = -30;
  else if (sensorValue == 0b00110000) mappedValue = -20;
  else if (sensorValue == 0b00010000) mappedValue = -20;
  else if (sensorValue == 0b00111000) mappedValue = -10;
 
  //90 deg left turn
  else if (sensorValue == 0b11111000 || sensorValue == 0b11110000)
  {
    mappedValue = 0; //changed due to track from -60 to 0  
  }    

  //line at middle
  else if (sensorValue == 0b00011000) 
  { 
    mappedValue = 0;        
  }

      

}

else if(surface == 1)
{  

  //90 deg right turn
  if (sensorValue == 0b11100000 || sensorValue == 0b11110000) { mappedValue = 80; plannedCRotate(); delay(blackDelay); /*sensorRead(); sensorMapping();*/}         
  
  //line at right
  else if (sensorValue == 0b11111110) mappedValue = 60;
  else if (sensorValue == 0b11111100) mappedValue = 50;
  else if (sensorValue == 0b11111101) mappedValue = 40;
  else if (sensorValue == 0b11111000) mappedValue = 40;
  else if (sensorValue == 0b11111001) mappedValue = 30;
  else if (sensorValue == 0b11111011) mappedValue = 30;
  else if (sensorValue == 0b11110001) mappedValue = 30;
  else if (sensorValue == 0b11110011) mappedValue = 20;
  else if (sensorValue == 0b11110111) mappedValue = 20;
  else if (sensorValue == 0b11100011) mappedValue = 10;

    //line at left
  else if (sensorValue == 0b01111111) mappedValue = -60;
  else if (sensorValue == 0b00111111) mappedValue = -50;
  else if (sensorValue == 0b10111111) mappedValue = -40;
  else if (sensorValue == 0b00011111) mappedValue = -40;
  else if (sensorValue == 0b10011111) mappedValue = -30;
  else if (sensorValue == 0b11011111) mappedValue = -30;
  else if (sensorValue == 0b10001111) mappedValue = -30;
  else if (sensorValue == 0b11001111) mappedValue = -20;
  else if (sensorValue == 0b11101111) mappedValue = -20;
  else if (sensorValue == 0b11000111) mappedValue = -10;
 
  //90 deg left turn
  else if (sensorValue == 0b00000111 || sensorValue == 0b00001111) {mappedValue = -80; plannedACRotate(); delay(blackDelay); /*sensorRead(); sensorMapping();*/}   

  //line at middle
  else if (sensorValue == 0b11100111) mappedValue = 0;

 
 }

}

void pid(void)
{
  
  error = targetValue - mappedValue;
  correction = (kp * error) + (kd * (error - prevError));

  prevError = error;

  motorResponse = (int)correction;

  if(motorResponse > maxSpeed) motorResponse = maxSpeed;
  if(motorResponse < -maxSpeed) motorResponse = -maxSpeed;

  if(motorResponse > 0)
  {
    rightSpeed = maxSpeed ;
    leftSpeed = maxSpeed - motorResponse;
  }

  else
  {
    rightSpeed = maxSpeed + motorResponse;
    leftSpeed = maxSpeed;
  }

}

//writes motor speed
void motor(int left, int right)
{
  analogWrite(motorPin1, right);
  analogWrite(motorPin2, 0);
  analogWrite(motorPin3, left);
  analogWrite(motorPin4, 0);

}

//stops the bot
void brake(void)
{
  analogWrite(motorPin1, 0);
  analogWrite(motorPin2, 0);
  analogWrite(motorPin3, 0);
  analogWrite(motorPin4, 0);
}



//special set of motor functions

void plannedForward()
{
  analogWrite(motorPin1, 90);
  analogWrite(motorPin2 , 0);
  analogWrite(motorPin3, 90);
  analogWrite(motorPin4, 0);

}


void plannedACRotate()
{
  analogWrite(motorPin1,100);
  analogWrite(motorPin2, 0);
  analogWrite(motorPin3, 0);
  analogWrite(motorPin4,100);

}

void plannedCRotate()
{
  analogWrite(motorPin1,0);
  analogWrite(motorPin2, 100);
  analogWrite(motorPin3, 100);
  analogWrite(motorPin4,0);

}

//avoid obstacles
void curveRun()
{
Serial.println("I am curveRunning");
plannedCRotate();
delay(600);
plannedForward();
delay(1000);
plannedACRotate();
delay(500);
plannedForward();
delay(1800);
plannedACRotate();
delay(350);

//go forward while still on all white
do
{
 sensorRead();
 sensorMapping();    

 plannedForward();
} while(mappedValue == 100);
//goes forward for a littlebit for better alignment
delay(140);
//make sure bot aligns back to the line
plannedCRotate();
delay(200);

}

//auto calibration
void calibration()
{
  plannedCRotate();

  float upSum = 0,lowSum = 0;
  int sensorArray[sensorNum][2];

  for(int i = 0; i < sensorNum; i++)
    {
      sensorArray[i][0] = analogRead(A0+i);
      sensorArray[i][1] = analogRead(A0+i);
    }
 

  int loopCounter = (int)(time * 1000 / 2.5);  
  while(loopCounter)
  {
    for(int i = 0; i < sensorNum; i++)
    {
      if(analogRead(A0+i)<sensorArray[i][0]) sensorArray[i][0]=analogRead(A0+i);
      if(analogRead(A0+i)>sensorArray[i][1]) sensorArray[i][1]=analogRead(A0+i);
    }
  loopCounter--;
  }

 for(int i=0; i < sensorNum; i++)
  blackLimit[i] = (int)(sensorArray[i][0] + safety * (sensorArray[i][1] - sensorArray[i][0]));

prev = millis();
//take sensor reading
sensorRead();

//mapReading for PID
sensorMapping();

curr = millis();

diff = curr - prev;

  brake();
  delay(1000);

}

//reset acuteKey variables
void resetAcuteKey(void)
{
  acuteKey = 0;
  
  stopKey = 0;  
}

void findPath(void)
{
       if(prevError != 0)
       {
        motor(leftSpeed, rightSpeed);
        //motor(rightSpeed, leftSpeed);
       }

}

//acute angle movement
void dance(void)
{

int loopCounter = (int) (danceDelay / diff);

  for(int i = loopCounter; i > 0; i--)
  {

  plannedCRotate();
  sensorRead();
  sensorMapping();

   if(mappedValue != 100)
   {
    pid();
    motor(leftSpeed, rightSpeed);
    return;
   }

  }

  for(int i = 2 * loopCounter; i > 0; i--)
  {
  plannedACRotate();
  sensorRead();
  sensorMapping();

   if(mappedValue != 100)
   {
    pid();
    motor(leftSpeed, rightSpeed);
    return;
   }

  }
  
}

//Allah maaf koruk

void ping(){

 // Serial.println("ping out");
  
  digitalWrite(SIGNAL_PIN, HIGH);
  delayMicroseconds(10); // I think I can cope with blocking for a whole 10us here...
  digitalWrite(SIGNAL_PIN, LOW);
  
  // start listening out for the echo pulse on interrupt 0
  PulseInZero::begin();
}


/**
* Pulse complete callback hanlder for PulseInZero 
* @param duration - pulse length in microseconds
*/
void pingPulseComplete(unsigned long duration) 
{

  fDistance = MicrosecondsToMillimetres2 * duration/ 10;
  
}
