/*
بسم الله الرحمن الرحيم
*/
#define sensorNum 8          //always even

#define maxSpeed 225//180  //180 //200
#define sSpeed 200

int blackLimit[sensorNum];
int digitalReading[sensorNum];

const int motorPin1 = 10,motorPin2 = 9;        //right motor
const int motorPin3 = 6,motorPin4 = 5;       //left motor
const int initIR=0;

int rightIRsBlack = 0;
int rotateDelay = 140;

float error, prevError=0;
float mappedValue, targetValue = 6;      

float safety=0.35;

float kp=26.7;   //26.7G //26        //38 //37  //39                //45 IF DOESN'T WORK
float kd=54;  //57.3  //50G //50    //67             //70  //42 //87

int rotationSpeed=255;

int motorResponse;
float correction;

int leftSpeed,rightSpeed;

float lastAct;
int time=5;

int allBlack=0;
int leftBlack=0;
int rightBlack=0;

int frontIR=1;                      //change
int leftIR=2;                          //change
int rightIR=3;                         //change

int insideLoop=0;
//int caveSpeed=;
//int turnSpeed=;

int left=0,right=0;                   //Left and Right most Irs
int contiBlack = 0;
int stopTrigger = 900 ; //2
int startTrigger = 3;

int fIR=11;
int saidulIR=4;

void setup()
{
  //initialize IR pins
  for(int i = initIR; i < sensorNum; i++)          //A0 is leftmost Ir
  {
    pinMode(A0 + i, INPUT);
  }

  pinMode(frontIR,INPUT);
  pinMode(rightIR,INPUT);
  pinMode(leftIR,INPUT);

  

  //initialize motor pins
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT); 
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);
  delay(1000);
  Serial.begin(9600);
  
  Serial.println("Calibrating");
  calibration();
  
  
}





void loop()
{
 
//sensorRead();
sensorMapping();
Serial.print(mappedValue);
Serial.print(" ");
  if(digitalRead(fIR)==0)
  {
    brake();
    while(digitalRead(fIR)==0)
   {
    motor(-maxSpeed,-maxSpeed); 
    delay(50);
    brake();
    delay(2000);
    }
    
  }
if(stopTrigger < contiBlack) 
{ 
  while(allBlack == 1) {brake(); startTrigger = 0; sensorMapping();}
   //if(mappedValue == 100) {delay(100); sensorMapping(); if(mappedValue == 100) brake();}
}

for(int i = 0; i < sensorNum; i++)
{
  //Serial.print(analogRead(A0+i));
  /*if(analogRead(A0+i)<blackLimit[i])
     Serial.print("1");
  else
     Serial.print("0");   
  Serial.print(" ");*/ 
}

  Serial.print("left: ");
  Serial.print(left);
  Serial.print(" right: ");
  Serial.print(right);
  Serial.println();
 /*
  while(!digitalRead(frontIRpin))    // if reading 0 object is close
  {
  	brake();
  }
 */ 


  //esonance loop turn

  if(allBlack == 1)
      {
        motor(-100,-100);
        delay(10);
        brake();
        delay(100);
      }
      
 if (rightIRsBlack==1 || allBlack == 1)
  
    {
      
     leftIR=0; rightIR = 1;
     CRotate(rotationSpeed); 
     delay(rotateDelay);  sensorMapping();
     }


 

  if(mappedValue == 100)                                 // all white
    {
      /* if(!digitalRead(leftIR)||!digitalRead(rightIR))
        {
             //bridge codes


        }*/
      /*   if(insideLoop)  //make it else if
        {
          //loop codes
          ACRotate(rotationSpeed);
          while(mappedValue==100) sensorMapping();
          
          int insideLoop=1;              //unnecessary
          int loopNo=1;
          left=0;

          while(insideLoop)
          {
            sensorMapping();
            pid();
            motor(leftSpeed,rightSpeed);
            
            if(allBlack==1) loopNo=2;
            
            if(loopNo==1)
            {
              if(left==1)
              {
                ACRotate(rotationSpeed);
                int exitLoop=1;
                int firstStep=0;

                while(exitLoop)
                {
                  if(right==1) firstStep=1;
                  if((firstStep==1)&&(right==0)) exitLoop=0;
                  sensorMapping();
                }
                insideLoop=0;
                
              }
            }
            else if(loopNo==2)
                {
                	if(right==1)
               		{
               		 CRotate(rotationSpeed);
                	int exitLoop=1;
                	int firstStep=0;

                	while(exitLoop)
                	{
                  	if(left==1) firstStep=1;
                  	if((firstStep==1)&&(left==0)) exitLoop=0;
                  	sensorMapping();
                	}
                	insideLoop=0;
                
              		}
                }
            
          }
        }

        */
        brake();
        //delay(10);
        //delayMicroseconds();
        motor(-50,-50);
        //delayMicroseconds(25);
        if((left==1)&&(right==0))
        {
          ACRotate(rotationSpeed);   
          while(mappedValue==100) sensorMapping();
          while(digitalReading[initIR+2]!=1)
          {
            sensorMapping();
          pid();
          motor(leftSpeed,rightSpeed);
          }      
           left=0;                                     //changed
        }
        else if((right==1)&&(left==0))
        {
          CRotate(rotationSpeed);   
          while(mappedValue==100) sensorMapping();
          while(digitalReading[sensorNum-3]!=1)
          {
            sensorMapping();
          pid();
          motor(leftSpeed,rightSpeed);
          }
          right=0;                                     //changed
        }
        else
        {
          motor(150,150);
          while(mappedValue==100) sensorMapping();
          pid();
          motor(leftSpeed,rightSpeed);
        }

      
    }

     else if(mappedValue!=100 || (contiBlack > stopTrigger && allBlack == 1))     // add conditions                    
    {
      pid();      
      motor(leftSpeed,rightSpeed); 
    }
    
      Serial.print("left");
      Serial.print(left);
      Serial.print(" Right: ");    
      Serial.print(right);

Serial.println();
}


void sensorMapping()
{
int sum=0,coun=0;





 
 for (int i = initIR; i <sensorNum; i++)
  { 
    if (analogRead(A0+i) < blackLimit[i])              //A0 is leftmost IR 
     { 
      
      sum += i*2;
      coun++;
      digitalReading[i]=1;
    }
    else digitalReading[i]=0;
  }

   if(digitalReading[0]||digitalReading[sensorNum-1])  //if left or right gives black saves their value
   {
     left=digitalReading[0];
     right=digitalReading[sensorNum-1];
   }

//esonance loop
if(digitalReading[6]==1 && digitalReading[7]==1) //liberal
{rightIRsBlack=1;}else rightIRsBlack=0; 


   if((left==1)&&(right==1)) 
   {
     left=0;
     right=1;//loop detection
   }
   if(coun!=0){                   
  mappedValue = sum / coun;
   }
   else mappedValue=100;         //all White detection 

   if(coun==sensorNum) 
   {
    contiBlack++;
   
   allBlack=1;
   
   //allblack detection
   //Serial.println("All black");
   }
   else if(startTrigger == 0 && mappedValue != 7 && coun != sensorNum)
   {
    allBlack = 1;
   }

    else
    {
    allBlack=0; 
    contiBlack = 0;  
    }
   /*
  int  leftAND=1;
  int  rightAND=1;
  int  leftOR=0;
  int  rightOR=0;

   int i=0;
   while(i<(sensorNum/2)-1)                      // (sensorNum/2)-1=3   and < before
   {
    
    leftAND= leftAND && digitalReading[i];
    rightAND= rightAND && digitalReading[sensorNum-i-1];
    
    leftOR= leftOR || digitalReading[i];
    rightOR= rightOR || digitalReading[sensorNum-i-1];

    i++;
   }
   
    if((leftOR==0)&&(rightAND==1)) 
    {  
   // Serial.print("All Right black");
    rightBlack=1;               //all right
    }
    else rightBlack=0;

    if((leftAND==1)&&(rightOR==0)) 
    {
    // Serial.print("All Left black");  
     leftBlack=1;                //all Left
    }
    else leftBlack=0;
   
  Serial.print("\tmappedValue: ");
  Serial.print(mappedValue);
  */
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
  else if(motorResponse<0)
  {
    rightSpeed=maxSpeed+ motorResponse;
    leftSpeed=maxSpeed;
  }
  //non aggressive straight
  else
  {
    rightSpeed= sSpeed;
    leftSpeed= sSpeed;
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

void CRotate(int rotateSpeed)
{
  analogWrite(motorPin1,0);
  analogWrite(motorPin2, rotateSpeed);
  analogWrite(motorPin3, rotateSpeed);
  analogWrite(motorPin4,0);

}

void ACRotate(int rotateSpeed)
{
  analogWrite(motorPin1,rotateSpeed);
  analogWrite(motorPin2, 0);
  analogWrite(motorPin3, 0);
  analogWrite(motorPin4,rotateSpeed);

}

void brake(void)
{
  analogWrite(motorPin1, 0);
  analogWrite(motorPin2, 0);
  analogWrite(motorPin3, 0);
  analogWrite(motorPin4, 0);
}

 //auto calibration
void calibration()
{
  CRotate(150);
  float upSum = 0,lowSum = 0;
  int sensorArray[sensorNum][2];

  for(int i = initIR; i < sensorNum; i++)
    {
      sensorArray[i][0] = analogRead(A0+i);
      sensorArray[i][1] = analogRead(A0+i);
    }
 

  int loopCounter = (int)(time * 1000 / 2.5);  
  while(loopCounter)
  {
    for(int i = initIR; i < sensorNum; i++)
    {
      if(analogRead(A0+i)<sensorArray[i][0]) sensorArray[i][0]=analogRead(A0+i);
      if(analogRead(A0+i)>sensorArray[i][1]) sensorArray[i][1]=analogRead(A0+i);
    }
  loopCounter--;

  }

 for(int i=initIR; i < sensorNum; i++)
  blackLimit[i] = (int)(sensorArray[i][0] + safety * (sensorArray[i][1] - sensorArray[i][0]));

  brake();
  delay(1000);

}
