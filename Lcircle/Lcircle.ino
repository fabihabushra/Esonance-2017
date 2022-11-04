
#define sensorNum 8
#define maxSpeed 50
int stopcounter;
 int blackLimit[sensorNum];

const int motorPin1=9,motorPin2=10;        //right motor
const int motorPin3=5,motorPin4=6;       //left motor

float error, prevError=0;
int contiBlack= 35;
float mappedValue, targetValue = 7;      //changed ferom 4.5 to 9

float safety=0.35;

float kp=50;                             //49
float kd=200 ;                       //180
float kt;                               

int motorResponse;
float correction;

int leftSpeed,rightSpeed;
int digitalValue;

float lastAct;
int blackIRCount;
int circleoutrotate=0;

int time=6;
int danceDelay = 400;

int prev, curr, diff;


void setup()
{
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
  delay(1000);
calibration();
  
  Serial.begin(9600);
}







void loop()
{
 
//sensorRead();

sensorMapping();
Serial.println(mappedValue);
Serial.println(blackIRCount);

/* For IR Reading
for(int i = 0; i < 8; i++)


{
  //Serial.print(analogRead(A0+i));
  if(analogRead(A0+i)<blackLimit[i])
 Serial.print("B");
 else Serial.print("W");
 Serial.print(" ");
}
Serial.println();
*/

  if (stopcounter>contiBlack){
    
  }

   if (circleoutrotate==1) {plannedCRotate(); delay(50); }
  
   if(mappedValue!=100)
    {
      pid();
      motor(leftSpeed,rightSpeed);
      
      lastAct = mappedValue;
    }
   
   
  else
     dance();
    /*
    {
     
      if(lastAct<targetValue) 
      {
      if(rightSpeed>leftSpeed)
       motor(leftSpeed,rightSpeed);        
      else
      motor(rightSpeed,leftSpeed); 
      }
      else  
      {
      if(leftSpeed>rightSpeed)
       motor(leftSpeed,rightSpeed);        
      else
       motor(rightSpeed,leftSpeed);
       }
    }
       */

      /*
      //use lastAct if this doesnt work
      if(mappedValue!=lastAct)
      {  
      if(rightSpeed>leftSpeed) 
            //motor(leftSpeed,rightSpeed);              //goes right
    
    /*  else if(rightSpeed<leftSpeed)
            motor(-rightSpeed,leftSpeed);               //goes left
      }
      else  if(lastAct<9) motor(-rightSpeed,leftSpeed);             //last line position was at left 
      else  motor(rightSpeed,-leftSpeed);                             
      */
    

}


void sensorMapping()
{
int sum=0;
int coun=0;
blackIRCount=0;
 
 for (int i = 0; i <sensorNum; i++)
  { 
    //if(i!=7)
    
    if (analogRead(A0+i) < blackLimit[i])              //A7 is leftmost IR 
     { 
      
      sum += i*2;
      blackIRCount++;
    }
    

  }
  if(analogRead(A0)< blackLimit[0] && analogRead(A7)< blackLimit[7]){
    circleoutrotate=1;
  } else circleoutrotate=0;
   if(blackIRCount!=0){  
  mappedValue = sum / blackIRCount;
   }
   else mappedValue=100;
   /*
  Serial.print("\tmappedValue: ");
  Serial.println(mappedValue);
  */

  if(coun<8) stopcounter=0;

  if(coun= 8) 
  {
  stopcounter++;
  }
}


void pid()
{
  
  error=targetValue-mappedValue;
  correction=(kp*error)+(kd*(error-prevError));
  prevError=error;
  motorResponse=(int)correction;
 
 if(motorResponse>maxSpeed) motorResponse=maxSpeed;
 
if(motorResponse<-maxSpeed) motorResponse=-maxSpeed;

   if(motorResponse>0)
  {
    rightSpeed=maxSpeed;
    leftSpeed=maxSpeed-motorResponse;
  }
  else 
  {
    rightSpeed=maxSpeed+ motorResponse;
    leftSpeed=maxSpeed;
  }

}

void motor(int left, int right)
{
  
  if(right>0)
  {
  analogWrite(motorPin1,right);
  analogWrite(motorPin2,0);
  }
  else
  {
    analogWrite(motorPin1,0);
    analogWrite(motorPin2,-right);
  }

  if(left>0)
  {
  analogWrite(motorPin3,left);
  analogWrite(motorPin4,0);
  }
  else
  {
   analogWrite(motorPin3,0);
   analogWrite(motorPin4,-left); 
  }

 }



void brake(void)
{
  analogWrite(motorPin1, 0);
  analogWrite(motorPin2, 0);
  analogWrite(motorPin3, 0);
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

void dance(void)
{

int loopCounter = (int) (danceDelay / diff);

  for(int i = loopCounter; i > 0; i--)
  {

  plannedCRotate();
 
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

  sensorMapping();

   if(mappedValue != 100)
   {
    pid();
    motor(leftSpeed, rightSpeed);
    return;
   }

  }
  
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

  brake();
  delay(1000);

}

