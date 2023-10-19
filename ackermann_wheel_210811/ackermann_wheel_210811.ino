#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>

ros::NodeHandle  nh;

/*핀*/
#define Right_Encoder_PinA 2
#define Right_Encoder_PinB 9
#define Left_Encoder_PinA  3
#define Left_Encoder_PinB  10
#define rwheel_PWM         A3
#define rwheel_DIR_1       6
#define rwheel_DIR_2       7   
#define lwheel_PWM         A4
#define lwheel_DIR_1       8
#define lwheel_DIR_2       13


/*변수*/
float wheel_vel_tar[2]={0,0}; 
float wheel_vel_cur[2]={0,0};
long encoder_tick_cur[2]={0,0};
long encoder_tick_pre[2]={0,0}; 
unsigned long time_pre = 0;
unsigned long time_cur = 0;
double Kp = 10;

/*함수*/
float steering_angle = 0.0;
float wheel_speed = 0.0;
float wheelbase =0.18;

void messageCb( const geometry_msgs::Twist& msg){
  float x = msg.linear.x;
  float z = msg.angular.z;
  float radius = 0;
  if(z !=0){
    float radius = x / z;
    steering_angle = atan(wheelbase / radius);
  }
  else steering_angle =0;
  wheel_speed = x;
  }



ros::Subscriber<geometry_msgs::Twist> Twist_sb("chefbot_teleop/cmd_vel", &messageCb);



std_msgs::Float32 steering;
std_msgs::Float32 rwheel_vel_cur;
std_msgs::Float32 lwheel_vel_cur;
ros::Publisher Steering_angle("cur_steering_angle", &steering);
ros::Publisher Rwheel_vel("rwheel_cur_vel", &rwheel_vel_cur);
ros::Publisher Lwheel_vel("lwheel_cur_vel", &lwheel_vel_cur);


void setup() {
  SetupEncoders();
  SetupMotors();
  nh.initNode();
  nh.advertise(Steering_angle);
  nh.advertise(Rwheel_vel);
  nh.advertise(Lwheel_vel);
  nh.subscribe(Twist_sb);
}

void loop() {
  
  GetCurrentWheelVel();
  Vel_PID();
  steering.data = steering_angle;
  Steering_angle.publish( &steering );
  Rwheel_vel.publish( &rwheel_vel_cur );
  Lwheel_vel.publish( &lwheel_vel_cur );
  nh.spinOnce();
  delay(1);
}



void GetCurrentWheelVel(){
    time_cur = millis();
    unsigned long  time_diff = time_cur - time_pre;
    time_pre = time_cur;
    for(int i = 0;i < 2;i++){
      long tick_diff = encoder_tick_cur[i] - encoder_tick_pre[i];
      encoder_tick_pre[i] = encoder_tick_cur[i];
      wheel_vel_cur[i] = float(tick_diff)/float(time_diff)/1.7*250;
    }
    rwheel_vel_cur.data =  wheel_vel_cur[0];
    lwheel_vel_cur.data =  wheel_vel_cur[1];
}

double lasterror = 0.0;
int Rvel = 0;
int Lvel = 0;
void Vel_PID(){

    double error = (wheel_speed)*250 - (wheel_vel_cur[0]);
    //Rvel += int(0.02 * error);
    int Control = constrain(abs(error), 0, 255);

      if((wheel_speed > 0)){
        digitalWrite(rwheel_DIR_1, LOW );
        digitalWrite(rwheel_DIR_2, HIGH );
      }
     else{
        digitalWrite(rwheel_DIR_1, HIGH );
        digitalWrite(rwheel_DIR_2, LOW );
      }
      
      analogWrite(rwheel_PWM, Control);
      if(wheel_speed == 0){
        digitalWrite(rwheel_DIR_1, LOW );
        digitalWrite(rwheel_DIR_2, LOW );
        analogWrite(rwheel_PWM, 0);
      } 

    error = (wheel_speed)*250 - (wheel_vel_cur[1]);
    //Lvel += int(0.02 * error);
    Control = constrain(abs(error), 0, 255);
      if((wheel_speed < 0)){
        digitalWrite(lwheel_DIR_1, LOW );
        digitalWrite(lwheel_DIR_2, HIGH );
      }
     else{
        digitalWrite(lwheel_DIR_1, HIGH );
        digitalWrite(lwheel_DIR_2, LOW );
      }
      
      analogWrite(lwheel_PWM, Control);
      if(wheel_speed == 0){
        digitalWrite(lwheel_DIR_1, LOW );
        digitalWrite(rwheel_DIR_2, LOW );
        analogWrite(lwheel_PWM, 0);
      } 
}


void SetupEncoders()
{
  // Right encoder
  pinMode(Right_Encoder_PinA, INPUT_PULLUP);      // sets pin A as input
  pinMode(Right_Encoder_PinB, INPUT_PULLUP);      // sets pin B as input
  //Attaching interrupt in Right_Enc_PinA.
  attachInterrupt(digitalPinToInterrupt(Right_Encoder_PinA), do_Right_Encoder, RISING); 
 
  // Left encoder
  pinMode(Left_Encoder_PinA, INPUT_PULLUP);      // sets pin A as input  
  pinMode(Left_Encoder_PinB, INPUT_PULLUP);      // sets pin B as input
  //Attaching interrupt in Left_Enc_PinA.
  attachInterrupt(digitalPinToInterrupt(Left_Encoder_PinA), do_Left_Encoder, RISING);
}

void SetupMotors()
{
 //Right Motor
 pinMode(rwheel_DIR_1,OUTPUT);
 pinMode(rwheel_DIR_2,OUTPUT);
 pinMode(rwheel_PWM,OUTPUT);

 //Left motor
 pinMode(lwheel_DIR_1,OUTPUT);
 pinMode(lwheel_DIR_2,OUTPUT);
 pinMode(lwheel_PWM,OUTPUT); 
}

void do_Right_Encoder()
{
  int RightEncoderBSet = digitalRead(Right_Encoder_PinB);   // read the input pin
  encoder_tick_cur[0] -= RightEncoderBSet ? -1 : +1;
  
}
void do_Left_Encoder()
{
  int LeftEncoderBSet = digitalRead(Left_Encoder_PinB);   // read the input pin
  encoder_tick_cur[1] -= LeftEncoderBSet ? -1 : +1;
}
