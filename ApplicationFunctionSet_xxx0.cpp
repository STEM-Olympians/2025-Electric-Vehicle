/*
 * @Author: ELEGOO
 * @Date: 2019-10-22 11:59:09
 * @LastEditTime: 2020-06-28 14:10:45
 * @LastEditors: Changhua
 * @Description: SmartRobot robot tank
 * @FilePath: 
 */
#include <hardwareSerial.h>
#include <stdio.h>
#include "ApplicationFunctionSet_xxx0.h"
#include "DeviceDriverSet_xxx0.h"

#include <PID_v1.h>

#include "ArduinoJson-v6.11.1.h" //ArduinoJson
#include "MPU6050_getdata.h"

#define _is_print 1
#define _Test_print 0

//Define Variables we'll be connecting to
double Setpoint, Input, Output;
//Define the aggressive and conservative Tuning Parameters
double aggKp=4, aggKi=0.2, aggKd=1;
double consKp=17, consKi=5, consKd=0.75;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);

ApplicationFunctionSet Application_FunctionSet;

/*硬件设备成员对象序列*/
MPU6050_getdata AppMPU6050getdata;
DeviceDriverSet_Motor AppMotor;
DeviceDriverSet_ULTRASONIC AppULTRASONIC;
DeviceDriverSet_Servo AppServo;


/*f(x) int */
static boolean
function_xxx(long x, long s, long e) //f(x)
{
  if (s <= x && x <= e)
    return true;
  else
    return false;
}

/*运动方向控制序列*/
enum SmartRobotCarMotionControl
{
  Forward,       //(1)
  Backward,      //(2)
  Left,          //(3)
  Right,         //(4)
  LeftForward,   //(5)
  LeftBackward,  //(6)
  RightForward,  //(7)
  RightBackward, //(8)
  stop_it        //(9)
};               //direction方向:（1）、（2）、 （3）、（4）、（5）、（6）

/*模式控制序列*/
enum SmartRobotCarFunctionalModel
{
  Standby_mode,           /*空闲模式*/
  TraceBased_mode,        /*循迹模式*/
  ObstacleAvoidance_mode, /*避障模式*/
  Follow_mode,            /*跟随模式*/
  Rocker_mode,            /*摇杆模式*/
};

/*控制管理成员*/
struct Application_xxx
{
  SmartRobotCarMotionControl Motion_Control;
  SmartRobotCarFunctionalModel Functional_Mode;
  unsigned long CMD_CarControl_Millis;
  unsigned long CMD_LightingControl_Millis;
};
Application_xxx Application_SmartRobotCarxxx0;;

bool ApplicationFunctionSet_SmartRobotCarLeaveTheGround(void);
void ApplicationFunctionSet_SmartRobotCarLinearMotionControl(SmartRobotCarMotionControl direction, uint8_t directionRecord, uint8_t speed, uint8_t Kp, uint8_t UpperLimit);
void ApplicationFunctionSet_SmartRobotCarMotionControl(SmartRobotCarMotionControl direction, uint8_t is_speed);



struct Obstruction { 
  int length;

   // 0 (false) is horizontal
   // 1 (true) is vertical
   boolean orientation;
   
   int col;
    int row;
};

struct MoveAction {
  SmartRobotCarMotionControl direction;
  Obstruction target;
  int time; // ms
  boolean moveOn;
};


// CHANGE THESE!!!
const int numOfObstructionsx = 1;
const int maxMovementsx = 1;

class Path {

  public:
 
 

    int robotLength = 24;
    int robotWidth = 16;
    int robotFrontX = 0;
    int robotFrontY = 0;



    int numOfObstructions = numOfObstructionsx;
    Obstruction obstructionMap[numOfObstructionsx]; 
 


    int maxMovements = maxMovementsx;

    MoveAction movements[maxMovementsx];

    int currentMove;



    int timeMoveStarted;
    boolean newMove;
 
    Path(){

     

      Obstruction obstruction1 =  { .length = 32, .orientation = false, .col = 1, .row = 2045 };

      obstructionMap[0] = obstruction1;


      MoveAction movement1 = { .direction = Forward, .target = obstructionMap[1], .time = 10000, .moveOn = false};


      movements[0] = movement1;


       currentMove = 0;

       timeMoveStarted = millis();
       newMove = true;

      
    }

   


    void updatePosition (){
      MoveAction currentMovement = movements[currentMove];
      if(currentMovement.target.length > 0) {
        
      }
    }


    boolean shouldEnd(){
      if(currentMove >= maxMovements ){
        return true;
      }
    }
    
    void followPath(){
     MoveAction currentMovement = movements[currentMove];

      if(newMove) {
        timeMoveStarted = millis();
      }

      if(millis - timeMoveStarted > currentMovement.time){
        ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
        currentMove++;
      }


      
      Serial.println(currentMove);
      
      updatePosition();


    }

};


Path robotPath; 
void ApplicationFunctionSet::ApplicationFunctionSet_Init(void)
{
  bool res_error = true;
  Serial.begin(9600);
  AppMotor.DeviceDriverSet_Motor_Init();
  // AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Init();

  // PUT THIS BACK
  // res_error = AppMPU6050getdata.MPU6050_dveInit();

  // Serial.println(res_error);

  // AppMPU6050getdata.MPU6050_calibration();

  while (Serial.read() >= 0)
  {
    Serial.println("save me");
    /*清空串口缓存...*/
  }

  Serial.println("hi?");
  delay(2000);
  Application_SmartRobotCarxxx0.Functional_Mode = ObstacleAvoidance_mode;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-180,180);

  // robotPath = Path();
}


/*
  直线运动控制：
  direction：方向选择 前/后
  directionRecord：方向记录（作用于首次进入该函数时更新方向位置数据，即:yaw偏航）
  speed：输入速度 （0--255）
  Kp：位置误差放大比例常数项（提高位置回复状态的反映，输入时根据不同的运动工作模式进行修改）
  UpperLimit：最大输出控制量上限
*/
static void ApplicationFunctionSet_SmartRobotCarLinearMotionControl(SmartRobotCarMotionControl direction, uint8_t directionRecord, uint8_t speed, uint8_t Kp, uint8_t UpperLimit)
{
  static float Yaw; //偏航
  static float yaw_So = 0;
  static uint8_t en = 110;
  static unsigned long is_time;
  if (en != directionRecord || millis() - is_time > 10)
  {
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_void, /*speed_A*/ 0,
                                           /*direction_B*/ direction_void, /*speed_B*/ 0, /*controlED*/ control_enable); //Motor control
    AppMPU6050getdata.MPU6050_dveGetEulerAngles(&Yaw);
    is_time = millis();
  }
  //if (en != directionRecord)
  if (en != directionRecord || Application_FunctionSet.Car_LeaveTheGround == false)
  {
    en = directionRecord;
    yaw_So = Yaw;
  }
  //加入比例常数Kp
  int R = (Yaw - yaw_So) * Kp + speed;
  if (R > UpperLimit)
  {
    R = UpperLimit;
  }
  else if (R < 10)
  {
    R = 10;
  }
  int L = (yaw_So - Yaw) * Kp + speed;
  if (L > UpperLimit)
  {
    L = UpperLimit;
  }
  else if (L < 10)
  {
    L = 10;
  }
  if (direction == Forward) //前进
  {
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ R,
                                           /*direction_B*/ direction_just, /*speed_B*/ L, /*controlED*/ control_enable);
  }
  else if (direction == Backward) //后退
  {
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ L,
                                           /*direction_B*/ direction_back, /*speed_B*/ R, /*controlED*/ control_enable);
  }
}
/*
  运动控制:
  1# direction方向:前行（1）、后退（2）、 左前（3）、右前（4）、后左（5）、后右（6）
  2# speed速度(0--255)
*/
static void ApplicationFunctionSet_SmartRobotCarMotionControl(SmartRobotCarMotionControl direction, uint8_t is_speed)
{
  ApplicationFunctionSet Application_FunctionSet;
  static uint8_t directionRecord = 0;
  uint8_t Kp, UpperLimit;
  uint8_t speed = is_speed;


    Kp = 0.1;
    UpperLimit = 180;

  switch (direction)
  {
  case /* constant-expression */
      Forward:
    /* code */
    if (Application_SmartRobotCarxxx0.Functional_Mode == TraceBased_mode)
    {
      AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ speed,
                                             /*direction_B*/ direction_just, /*speed_B*/ speed, /*controlED*/ control_enable); //Motor control
    }
    else
    { //前进时进入方向位置逼近控制环处理
      ApplicationFunctionSet_SmartRobotCarLinearMotionControl(Forward, directionRecord, speed, Kp, UpperLimit);
      directionRecord = 1;
    }

    break;
  case /* constant-expression */ Backward:
    /* code */
    if (Application_SmartRobotCarxxx0.Functional_Mode == TraceBased_mode)
    {
      AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ speed,
                                             /*direction_B*/ direction_back, /*speed_B*/ speed, /*controlED*/ control_enable); //Motor control
    }
    else
    { //后退时进入方向位置逼近控制环处理
      ApplicationFunctionSet_SmartRobotCarLinearMotionControl(Backward, directionRecord, speed, Kp, UpperLimit);
      directionRecord = 2;
    }

    break;
  case /* constant-expression */ Left:
    /* code */
    directionRecord = 3;
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ speed,
                                           /*direction_B*/ direction_back, /*speed_B*/ speed, /*controlED*/ control_enable); //Motor control
    break;
  case /* constant-expression */ Right:
    /* code */
    directionRecord = 4;
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ speed,
                                           /*direction_B*/ direction_just, /*speed_B*/ speed, /*controlED*/ control_enable); //Motor control
    break;
  case /* constant-expression */ LeftForward:
    /* code */
    directionRecord = 5;
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ speed,
                                           /*direction_B*/ direction_just, /*speed_B*/ speed / 2, /*controlED*/ control_enable); //Motor control
    break;
  case /* constant-expression */ LeftBackward:
    /* code */
    directionRecord = 6;
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ speed,
                                           /*direction_B*/ direction_back, /*speed_B*/ speed / 2, /*controlED*/ control_enable); //Motor control
    break;
  case /* constant-expression */ RightForward:
    /* code */
    directionRecord = 7;
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ speed / 2,
                                           /*direction_B*/ direction_just, /*speed_B*/ speed, /*controlED*/ control_enable); //Motor control
    break;
  case /* constant-expression */ RightBackward:
    /* code */
    directionRecord = 8;
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ speed / 2,
                                           /*direction_B*/ direction_back, /*speed_B*/ speed, /*controlED*/ control_enable); //Motor control
    break;
  case /* constant-expression */ stop_it:
    /* code */
    directionRecord = 9;
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_void, /*speed_A*/ 0,
                                           /*direction_B*/ direction_void, /*speed_B*/ 0, /*controlED*/ control_enable); //Motor control
    break;
  default:
    directionRecord = 10;
    break;
  }
}

static void delay_xxx(uint16_t _ms)
{
  for (unsigned long i = 0; i < _ms; i++)
  {
    delay(1);
  }
}

static boolean timedMovement( SmartRobotCarMotionControl direction, int duration, int initialTime ){

  Serial.println("help");
  Serial.println(millis());

  Serial.println("initialTime");
  Serial.println(initialTime);

  Serial.println("duration");
  Serial.println(duration);

  Serial.println("combined");
  int thing = initialTime + duration;
  
  Serial.println(thing);
  Serial.println(millis() > (initialTime + duration));
  if(millis() > (initialTime + duration)){
    return true;
    Serial.println("WHAT");
  }

  ApplicationFunctionSet_SmartRobotCarMotionControl(direction, 180);

  return false;
}


static int normalizeDegrees(float angle){
  return (((int)(angle + 0.5)) % 360 + 360) % 360;
}

static int relativeAngularDifference(double currentAngle, double newAngle) {
    double a = normalizeDegrees(currentAngle - newAngle);
    double b = normalizeDegrees(newAngle - currentAngle);
    return a < b ? a : -b;
}



static boolean gyroPID(int targetAngle){

  static float Yaw; //偏航

  AppMPU6050getdata.MPU6050_dveGetEulerAngles(&Yaw);

  if((Yaw - 0.5 <= targetAngle) && ( Yaw + 0.5 >= targetAngle)){
    return true;
  }

  Setpoint = targetAngle;
  
  Input = Yaw;
  myPID.Compute();

  if(Output > 0){
    ApplicationFunctionSet_SmartRobotCarMotionControl(Right, Output);
  } else {
     ApplicationFunctionSet_SmartRobotCarMotionControl(Left, -Output);
  }

  Serial.println("currentAngle: ");

  Serial.println(Yaw);


  Serial.println("Output");
  Serial.println(Output);
  return false;
}

void ApplicationFunctionSet::Test(void){
  Serial.println("plz ");
  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 180);

}

void ApplicationFunctionSet::BruteForce(void){
   

  static boolean first_is = true;
  
  static int startTime = 0;
  static boolean getTime = true;

  static int numMove = 0;

  if (Application_SmartRobotCarxxx0.Functional_Mode == ObstacleAvoidance_mode)
  {
    // uint8_t switc_ctrl = 0;
    // uint16_t get_Distance;
    // if (Car_LeaveTheGround == false)
    // {
    //   ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
    //   return;
    // }
    // if (first_is == true) //Enter the mode for the first time, and modulate the steering gear to 90 degrees
    // {
    //   AppServo.DeviceDriverSet_Servo_control(90 /*Position_angle*/);
    //   first_is = false;
    // }

    // AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Get(&get_Distance /*out*/);
    //Serial.println(get_Distance);

    if(getTime){
      startTime = millis();
      getTime = false;
    }
    Serial.println("start");
  Serial.println(startTime);
  Serial.println("numMove");
  Serial.println(numMove);
    

     switch (numMove)
          {
          case 0:
            if(timedMovement(Forward, 10000, startTime)){
              getTime = true;
              numMove++;
              Serial.println("NOOOOO");
            }
            break;
          case 1:
             if(gyroPID(90)){
              getTime = true;
              numMove++;
              }
            break;
          case 2:
            if(timedMovement(Forward, 5000, startTime)){
              getTime = true;
              numMove++;
              Serial.println("NOOOOO");
            }
            break;
          case 3:
             if(gyroPID(180)){
              getTime = true;
              numMove++;
              }
            break;
          default:
            ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
          }

    
  } 
  else
  {
    first_is = true;
  }



}


void ApplicationFunctionSet::GyroTest(void){
   static float Yaw; //偏航

  AppMPU6050getdata.MPU6050_dveGetEulerAngles(&Yaw);
Serial.println(Yaw);

boolean f = false;
Setpoint = 90;
//ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 100);
  gyroPID(90);
}


/*
  避障功能
*/
void ApplicationFunctionSet::ApplicationFunctionSet_Obstacle(void)
{

  // also in cm
  static int robotFrontX = 0;
  static int robotFrontY = 0;

  // cm
  static int robotLength = 24;
  static int robotWidth = 16;

  
  static boolean first_is = true;
  if (Application_SmartRobotCarxxx0.Functional_Mode == ObstacleAvoidance_mode)
  {
    uint8_t switc_ctrl = 0;
    uint16_t get_Distance;
    if (Car_LeaveTheGround == false)
    {
      ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
      return;
    }
    if (first_is == true) //Enter the mode for the first time, and modulate the steering gear to 90 degrees
    {
      AppServo.DeviceDriverSet_Servo_control(90 /*Position_angle*/);
      first_is = false;
    }

    AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Get(&get_Distance /*out*/);
    Serial.println(get_Distance);


  //ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 180);


    if(robotPath.shouldEnd() ){
      ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 180);
    } else {
      robotPath.followPath();
    }
  

    // if (function_xxx(get_Distance, 0, 20)){
    //    ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 180);
    // } else{
    //   ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 180);
    // }



    // if (function_xxx(get_Distance, 0, 20))
    // {
     
    //   for (int i = 1; i < 6; i += 2) //1、3、5 Omnidirectional detection of obstacle avoidance status
    //   {
    //     AppServo.DeviceDriverSet_Servo_control(30 * i /*Position_angle*/);
    //     delay_xxx(1);
    //     AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Get(&get_Distance /*out*/);

    //     // If obstacle immediately in front of us
    //     if (function_xxx(get_Distance, 0, 20))
    //     {
    //       ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
    //       if (5 == i)
    //       {
    //         ApplicationFunctionSet_SmartRobotCarMotionControl(Backward, 150);
    //         delay_xxx(500);
    //         ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 150);
    //         delay_xxx(50);

    //         // reset servo?
    //         first_is = true;
    //         break;
    //       }
    //     }
    //     else
    //     {
    //       switc_ctrl = 0;
    //       switch (i)
    //       {
    //       case 1:
    //         ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 150);
    //         break;
    //       case 3:
    //         ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 150);
    //         break;
    //       case 5:
    //         ApplicationFunctionSet_SmartRobotCarMotionControl(Left, 150);
    //         break;
    //       }
    //       delay_xxx(50);
    //       first_is = true;
    //       break;
    //     }
    //   }
    // }
    // else 
    // {
    //   ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 150);
    // }
  }
  else
  {
    first_is = true;
  }
}
