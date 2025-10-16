#define MID_SPEED 100    
#define HIGH_SPEED 120    
#define LOW_SPEED 80

#define LONG_DELAY_TIME 70 
#define DELAY_TIME 40 
#define SHORT_DELAY_TIME 30 
         
#define speedPinR 9   //  Front Wheel PWM pin connect Model-Y M_B ENA 
#define RightMotorDirPin1  22    //Front Right Motor direction pin 1 to Model-Y M_B IN1  (K1)
#define RightMotorDirPin2  24   //Front Right Motor direction pin 2 to Model-Y M_B IN2   (K1)                                 
#define LeftMotorDirPin1  26    //Front Left Motor direction pin 1 to Model-Y M_B IN3 (K3)
#define LeftMotorDirPin2  28   //Front Left Motor direction pin 2 to Model-Y M_B IN4 (K3)
#define speedPinL 10   //  Front Wheel PWM pin connect Model-Y M_B ENB

#define speedPinRB 11   //  Rear Wheel PWM pin connect Left Model-Y M_A ENA 
#define RightMotorDirPin1B  5    //Rear Right Motor direction pin 1 to Model-Y M_A IN1 ( K1)
#define RightMotorDirPin2B 6    //Rear Right Motor direction pin 2 to Model-Y M_A IN2 ( K1) 
#define LeftMotorDirPin1B 7    //Rear Left Motor direction pin 1 to Model-Y M_A IN3  (K3)
#define LeftMotorDirPin2B 8  //Rear Left Motor direction pin 2 to Model-Y M_A IN4 (K3)
#define speedPinLB 12    //  Rear Wheel PWM pin connect Model-Y M_A ENB

#define sensor1   A4 // Left most sensor
#define sensor2   A3 // 2nd Left   sensor
#define sensor3   A2 // center sensor
#define sensor4   A1 // 2nd right sensor// Right most sensor
#define sensor5   A0 // Right most sensor

/*motor control*/

void FR_fwd(int speed)  //front-right wheel forward turn
{
  digitalWrite(RightMotorDirPin1,HIGH);
  digitalWrite(RightMotorDirPin2,LOW); 
  analogWrite(speedPinR,speed);
}
void FR_bck(int speed) // front-right wheel backward turn
{
  digitalWrite(RightMotorDirPin1,LOW);
  digitalWrite(RightMotorDirPin2,HIGH); 
  analogWrite(speedPinR,speed);
}
void FL_fwd(int speed) // front-left wheel forward turn
{
  digitalWrite(LeftMotorDirPin1,HIGH);
  digitalWrite(LeftMotorDirPin2,LOW);
  analogWrite(speedPinL,speed);
}
void FL_bck(int speed) // front-left wheel backward turn
{
  digitalWrite(LeftMotorDirPin1,LOW);
  digitalWrite(LeftMotorDirPin2,HIGH);
  analogWrite(speedPinL,speed);
}

void RR_fwd(int speed)  //rear-right wheel forward turn
{
  digitalWrite(RightMotorDirPin1B, HIGH);
  digitalWrite(RightMotorDirPin2B,LOW); 
  analogWrite(speedPinRB,speed);
}
void RR_bck(int speed)  //rear-right wheel backward turn
{
  digitalWrite(RightMotorDirPin1B, LOW);
  digitalWrite(RightMotorDirPin2B,HIGH); 
  analogWrite(speedPinRB,speed);
}
void RL_fwd(int speed)  //rear-left wheel forward turn
{
  digitalWrite(LeftMotorDirPin1B,HIGH);
  digitalWrite(LeftMotorDirPin2B,LOW);
  analogWrite(speedPinLB,speed);
}
void RL_bck(int speed)    //rear-left wheel backward turn
{
  digitalWrite(LeftMotorDirPin1B,LOW);
  digitalWrite(LeftMotorDirPin2B,HIGH);
  analogWrite(speedPinLB,speed);
}



//Simple Accelerations



void stop_bot()
{
  analogWrite(speedPinLB,0);
  analogWrite(speedPinRB,0);
  analogWrite(speedPinL,0);
  analogWrite(speedPinR,0);
  digitalWrite(RightMotorDirPin1B, LOW);
  digitalWrite(RightMotorDirPin2B,LOW);   
  digitalWrite(LeftMotorDirPin1B, LOW);
  digitalWrite(LeftMotorDirPin2B,LOW); 
  digitalWrite(RightMotorDirPin1, LOW);
  digitalWrite(RightMotorDirPin2,LOW);   
  digitalWrite(LeftMotorDirPin1, LOW);
  digitalWrite(LeftMotorDirPin2,LOW); 
  delay(40);
}
void forward(int speed_left,int speed_right)
{
   RL_fwd(speed_left);
   RR_fwd(speed_right);
   FR_fwd(speed_right);
   FL_fwd(speed_left); 
}
void reverse(int speed)
{
   RL_bck(speed);
   RR_bck(speed);
   FR_bck(speed);
   FL_bck(speed); 
}



//Direction Accelerations



void right_shift(int speed_fl_fwd,int speed_rl_bck ,int speed_rr_fwd,int speed_fr_bck) 
{
  FL_fwd(speed_fl_fwd); 
  RL_bck(speed_rl_bck); 
  RR_fwd(speed_rr_fwd);
  FR_bck(speed_fr_bck);
}
void left_shift(int speed_fl_bck,int speed_rl_fwd ,int speed_rr_bck,int speed_fr_fwd)
{
   FL_bck(speed_fl_bck);
   RL_fwd(speed_rl_fwd);
   RR_bck(speed_rr_bck);
   FR_fwd(speed_fr_fwd);
}

void left_turn(int speed)
{
   RL_bck(0);
   RR_fwd(speed);
   FR_fwd(speed);
   FL_bck(0); 
}
void right(int speed)
{
   RL_fwd(speed);
   RR_bck(0);
   FR_bck(0);
   FL_fwd(speed); 
}
void left(int speed)
{
   RL_fwd(0);
   RR_bck(speed);
   FR_bck(speed);
   FL_fwd(0); 
}
void right_back(int speed)
{
   RL_bck(speed);
   RR_fwd(0);
   FR_fwd(0);
   FL_bck(speed); 
}
void sharpRightTurn(int speed_left,int speed_right)
{
   RL_fwd(speed_left);
   RR_bck(speed_right);
   FR_bck(speed_right);
   FL_fwd(speed_left); 
}
void sharpLeftTurn(int speed_left,int speed_right){
   RL_bck(speed_left);
   RR_fwd(speed_right);
   FR_fwd(speed_right);
   FL_bck(speed_left); 
}


//Pins initialize
void init_GPIO()
{
  pinMode(RightMotorDirPin1, OUTPUT); 
  pinMode(RightMotorDirPin2, OUTPUT); 
  pinMode(speedPinL, OUTPUT);  
 
  pinMode(LeftMotorDirPin1, OUTPUT);
  pinMode(LeftMotorDirPin2, OUTPUT); 
  pinMode(speedPinR, OUTPUT);
  pinMode(RightMotorDirPin1B, OUTPUT); 
  pinMode(RightMotorDirPin2B, OUTPUT); 
  pinMode(speedPinLB, OUTPUT);  
 
  pinMode(LeftMotorDirPin1B, OUTPUT);
  pinMode(LeftMotorDirPin2B, OUTPUT); 
  pinMode(speedPinRB, OUTPUT);
  pinMode(sensor1, INPUT);
  pinMode(sensor2, INPUT);
  pinMode(sensor3, INPUT);
  pinMode(sensor4, INPUT);
  pinMode(sensor5, INPUT);
  
  stop_bot();
}



//Action Variables
bool steeringState = false;
int error = 0;
bool errorDirection = true; //false = Left, true = Right
int kp = 50;

int moveValue;
int lowMV;

int s1, s2, s3, s4, s5;
int digiRead;

//Action Functions




void setup()
{
  init_GPIO();
  Serial.begin(9600);
}

void loop(){
  s1 = !digitalRead(sensor1);
  s2 = !digitalRead(sensor2);
  s3 = !digitalRead(sensor3);
  s4 = !digitalRead(sensor4);
  s5 = !digitalRead(sensor5);

  digiRead = (1 * s1) + (2 * s2) + (4 * s3) + (8 * s4) + (16 * s5);

  steeringState = true;

  if (digiRead == 1 || digiRead == 3 || digiRead == 7) { //10000 or 11000 or 11100
    error = 2;
    errorDirection = false;
  } else if (digiRead == 2 || digiRead == 6) { //01000 or 01100
    error = 1;
    errorDirection = false;
  } else if (digiRead == 4 || digiRead == 14) { //00100 or 01110
    error = 0;
  } else if (digiRead == 8 || digiRead == 12) { //00010 or 00110
    error = 1;
    errorDirection = true;
  } else if (digiRead == 16 || digiRead == 24 || digiRead == 28) { //00001 or 00011 or 00111
    error = 2;
    errorDirection = true;
  } 
  else if (digiRead == 31) { //11111
    steeringState = false;
    errorDirection = true;
  }
  else if (digiRead == 0) { //00000
    steeringState = false;
    errorDirection = false;
  }

    
    moveValue = kp * error;
    lowMV = moveValue * 0.7;

    if (steeringState == true) {
      if (error == 0) {
      forward(MID_SPEED, MID_SPEED);
    } else if (error == 2) {
      if (errorDirection == false) {
        FL_bck(lowMV);
        FR_fwd(moveValue);
        RL_bck(lowMV);
        RR_fwd(moveValue);
      } else if (errorDirection == true) {
        FL_fwd(moveValue);
        FR_bck(lowMV);
        RL_fwd(moveValue);
        RR_bck(lowMV);
      }
    } else if (error == 1) {
      if (errorDirection == false) {
        FL_bck(moveValue);
        FR_fwd(moveValue);
        RL_bck(moveValue);
        RR_fwd(moveValue);
      } else if (errorDirection == true) {
        FL_fwd(moveValue);
        FR_bck(moveValue);
        RL_fwd(moveValue);
        RR_bck(moveValue);
      }
      }
    } else if (steeringState == false) {
      if (errorDirection == false) {
        FL_fwd(100);
        FR_bck(100);
        RL_fwd(100);
        RR_bck(100); 
      } else if (errorDirection == true) {
        FL_bck(100);
        FR_fwd(100);
        RL_bck(100);
        RR_fwd(100);
      }
    }

  }
