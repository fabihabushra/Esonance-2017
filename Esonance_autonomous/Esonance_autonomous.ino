
#define sensorNum 8
#define maxSpeed 100
int rotationSpeed=100;

 int blackLimit[sensorNum];

const int motorPin1=9,motorPin2=10;        //right motor
const int motorPin3=5,motorPin4=6;       //left motor

float error, prevError=0;
int contiBlack= 35000000000;
float mappedValue, targetValue = 7;      //changed ferom 4.5 to 9

float safety=0.35;

float kp=45;                             //49
float kd=0;                       //180
float kt;                               

int motorResponse;
float correction;

int leftSpeed,rightSpeed;
int digitalValue;
int allBlack=0,RightIRsBlack=0 ;
int CRotateKey=0,ACRotateKey=0;

float lastAct;
int time=5;
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


  if (stopcounter>contiBlack){
    
  }
*/
 if (RightIRsBlack==1 || allBlack==1) 

    {plannedCRotate(); delay(300); }
 

/*
  if (ThreeLeftIRsBlack==1) 
  { if(CRotateKey=0)
  {plannedACRotate(); delay(5000); ACRotateKey=1;}
  else CRotateKey=0;}
  
  else if (RightIRsBlack==1) 
  { if(ACRotateKey=0)
    {plannedCRotate(); delay(5000); CRotateKey=1;}
  else ACRotateKey=0; }
 */ 
  sensorMapping();

  
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
int sum=0,coun=0; int digitalReading[sensorNum];
 
 for (int i = 0; i <sensorNum; i++)
  { 
    //if(i!=7)
    
    if (analogRead(A0+i) < blackLimit[i])              //A7 is leftmost IR 
     { 
      
      sum += i*2;
      coun++;
      digitalReading[i]= 1;
    } else digitalReading[i]= 0;
    

  }
   if(coun!=0){  
  mappedValue = sum / coun;
   }
   else mappedValue=100;
   /*
  Serial.print("\tmappedValue: ");
  Serial.println(mappedValue);
  */


//erokom nimnomaner vabe IR reading neyar jonno khoma cheye nicchi
if (analogRead(A7)< blackLimit[7] && analogRead(A6)< blackLimit[6] && analogRead(A5)< blackLimit[5] && analogRead(A4)< blackLimit[4] && analogRead(A0)> blackLimit[0]) RightIRsBlack=1; else RightIRsBlack=0;
if(coun== 8) allBlack=1; else allBlack=0;
  
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
  analogWrite(motorPin1,rotationSpeed);
  analogWrite(motorPin2, 0);
  analogWrite(motorPin3, 0);
  analogWrite(motorPin4,rotationSpeed);

}

void plannedCRotate()
{
  analogWrite(motorPin1,0);
  analogWrite(motorPin2, rotationSpeed);
  analogWrite(motorPin3, rotationSpeed);
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
prev = millis();
sensorMapping ();
curr= millis();
diff = curr - prev;

  brake();
  delay(1000);

}

