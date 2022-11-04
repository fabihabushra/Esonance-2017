#define sensorNum 8
#define maxSpeed 160
#define rotationSpeed 160

int blackLimit[sensorNum];

const int motorPin1 = 10,motorPin2 = 9;        //right motor
const int motorPin3 = 6,motorPin4 = 5;       //left motor

float error, prevError=0;
float mappedValue, targetValue = 7;     

float safety=0.35;

float kp=45*3;                         //45*3
float kd=50*3;                      //50*3
                              

int motorResponse;
float correction;

int digitalReading[sensorNum];
int leftSpeed,rightSpeed;
int pidAllBlack=0,rightIRsBlack=0,stopAllBlack=0 ;
int leftIR,rightIR;
int contiBlack=25,stopCounter=0;


int time=4;

int danceDelay = 300;
int prev, curr, diff;

//sonar shit

const int rSonarTrig = 3;
const int rSonarEcho = 2;

const int fSonarTrig = 8;
const int fSonarEcho = 7;

int rDistance;
int fDistance;
int caveMin=40,caveMax=150;
int rWallDistance=5,fWallDistance=8;

int fIR=11;
int saidulIR=4;

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

    pinMode(rSonarTrig, OUTPUT);
  pinMode(rSonarEcho, INPUT);
  pinMode(fSonarTrig, OUTPUT);
  pinMode(fSonarEcho, INPUT);
  pinMode(fIR, INPUT);
  pinMode(saidulIR, INPUT);
  
  delay(1000);
    Serial.begin(115200);
calibration();
  

}



void loop()
{
 
//sensorRead();

sensorMapping();
/*
Serial.println(mappedValue);
Serial.println();
Serial.print(leftSpeed);
Serial.print(" ");
Serial.print(rightSpeed);
Serial.println();
*/
// For IR Reading
for(int i = 0; i < sensorNum; i++)


{
  //Serial.print(analogRead(A0+i));
  if(analogRead(A0+i)<blackLimit[i])
 Serial.print("B");
 else Serial.print("W");
 Serial.print(" ");
 Serial.print(analogRead(A0+i));
 Serial.print(" && ");
 Serial.print(blackLimit[i]);
 Serial.print(" ||| ");
}
Serial.println();
Serial.print(leftIR);
Serial.print(" & ");
Serial.println(rightIR);



//movement
//if(digitalRead(fIR==0))
//{
//  brake();
//}
if(stopAllBlack==1){
 Serial.println(stopCounter);
  if(stopCounter>contiBlack)
  {delay(100);
  while(stopAllBlack==1){brake(); sensorMapping();}
  }
}

 if (((rightIRsBlack==1 && rightIR==1) ||pidAllBlack==1) && stopAllBlack!=1)
  
    {
     //rightIR = 0; leftIR=0;
     plannedCRotate(); 
     delay(200);  sensorMapping();}
    
    //saidul's code
/*    
 if ((rightIRsBlack==1 && rightIR==1)  && stopAllBlack!=1)
    {
      //rightIR = 0; leftIR=0;
     plannedCRotate(); 
     int rusab=1;
     int raiyaan=0;
     int akash=0;
     while(rusab)
     {
      if(leftIR==1) raiyaan=1;
      if((raiyaan==1)&&rightIR==1) akash=1;
      if(akash=1&&digitalReading[3]==1) rusab=0;   
      sensorMapping();
     }
     }
     else if((pidAllBlack==1) && stopAllBlack!=1)
  {
    plannedCRotate(); 
     int rusab=1;
     int raiyaan=0;
     int akash=0;
     int bushra=0;
     while(rusab)
     {
      if(leftIR==1) raiyaan=1;
      else raiyaan=0;
      if((raiyaan==1)&&rightIR==1) bushra++;
      if(bushra==2&&digitalReading[3]==1) rusab=0;   
      sensorMapping();
     }
  }
*/
  
 if(mappedValue!=100)
   {  
      pid();
      motor(leftSpeed,rightSpeed);
   }
   
 else 
  { 
      sideSonarReadings();
      if(rDistance<rWallDistance)
      {
        brake();
        delay(50);
      for(int i=0;i<5;i++)
        {
        
        sideSonarReadings();
        if(rDistance>rWallDistance)
        break;
        }
      if(rDistance<rWallDistance)
      secondCave();
      }
     else {
       
       if (leftIR==0 && rightIR==1) 
      {{while((mappedValue==100)||digitalReading[3]!=1) 
      {plannedCRotate();  
      sensorMapping();
      } 
      pid(); 
      motor(leftSpeed,rightSpeed);
      } rightIR=0;}
      
      else  if (leftIR==1 && rightIR==0) 
      {{while(mappedValue==100) 
      {plannedACRotate();  
      sensorMapping();
      } 
      pid(); 
      motor(leftSpeed,rightSpeed);
      } leftIR=0;}
      
      else if (leftIR==1 && rightIR==1) 
      {{while((mappedValue==100)||digitalReading[3]!=1) 
      {plannedCRotate();  
      sensorMapping();
      } 
      pid(); 
      motor(leftSpeed,rightSpeed);
      } leftIR=0; rightIR=0;}
//      else if (leftIR==0 && rightIR==0){
//      while(mappedValue==100){
//      motor(maxSpeed,maxSpeed);
//      sensorMapping();
//      }
//      }
     }
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

/*
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
*/

//sonar functions
long mstocm(long microseconds)
{
 
  return (microseconds*346.3)/2/10000;
}

int trigger(int trigPin,int echopin)
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  int distance =  mstocm(pulseIn(echopin, HIGH));
  return distance;
}

void sideSonarReadings(){

 //  rDistance= trigger(rSonarTrig,rSonarEcho);
 //  fDistance= trigger(fSonarTrig,fSonarEcho);
   rDistance= 786;
   fDistance= 199;
}

void secondCave()
{ do
  {
  if(digitalRead(saidulIR)==1)
  {
    brake();
    while(digitalRead(saidulIR)==1)
   {
    motor(-maxSpeed,-maxSpeed); 
    delay(500);
    brake();
    delay(10000);}
    
  }
  if (fDistance>fWallDistance)

  {caveForward(caveMax);}
  else if (fDistance<=fWallDistance && rDistance<rWallDistance)
  {plannedACRotate(); delay(400);}
  sideSonarReadings();
  sensorMapping();
  
  } while(mappedValue == 100);
  
}

void caveForward(int beshi)
{
  analogWrite(motorPin1, beshi);
  analogWrite(motorPin2 , 0);
  analogWrite(motorPin3, beshi);
  analogWrite(motorPin4, 0);

}

void caveLeft(int beshi,int kom)
{
  analogWrite(motorPin1, beshi);
  analogWrite(motorPin2, 0);
  analogWrite(motorPin3, 0);
  analogWrite(motorPin4, kom);

}

void caveRight(int beshi,int kom)
{
  analogWrite(motorPin1, 0);
  analogWrite(motorPin2, kom);
  analogWrite(motorPin3, beshi);
  analogWrite(motorPin4, 0);

}

 //auto calibration
void sensorMapping()
{
int sum=0,pidDetect=0, stopDetect=0; 
 
    
 for (int i = 0; i <sensorNum; i++)
  { 
    
    if (analogRead(A0+i) < blackLimit[i])           
     { 

      if(i>0 && i<7) {sum += i*2; pidDetect++;}
      stopDetect++;
      digitalReading[i]= 1;
    } else digitalReading[i]= 0;
    }


    
   if(pidDetect!=0){  
  mappedValue = sum / pidDetect;
   }
   else mappedValue=100;



if (digitalReading[0]==1 || digitalReading[7]==1) 
{leftIR=digitalReading[0]; rightIR=digitalReading[7];}

if(digitalReading[1]==0 && digitalReading[5]==1 && digitalReading[6]==1)
{rightIRsBlack=1;}else rightIRsBlack=0; 

if(pidDetect>=6) 
{pidAllBlack=1;}else pidAllBlack=0; 

if(stopDetect==sensorNum)
{
stopAllBlack=1;
stopCounter++;

} else {stopAllBlack=0; stopCounter = 0;}

}


 
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

