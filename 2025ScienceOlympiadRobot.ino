/*
 * @Author: ELEGOO
 * @Date: 2019-10-22 11:59:09
 * @LastEditTime: 2020-06-28 14:55:26
 * @LastEditors: Changhua
 * @Description: SmartRobot robot tank
 * @FilePath: 
 */
#include <avr/wdt.h>
#include "DeviceDriverSet_xxx0.h"
#include "ApplicationFunctionSet_xxx0.h"

// Add robot dimensions!!!
// Also good luck you guys, you got this! :)


void setup()
{
  Serial.begin(9600);
  // Serial.println("I am alive");
  // Serial.println("uh oh");
  // put your setup code here, to run once:
  Application_FunctionSet.ApplicationFunctionSet_Init();
  if(true){
    Serial.println("hi");
  } else{
    Serial.println("foo");
  }
  // delay(2000);
    Serial.println("I am alive");
  Serial.println("uh oh");
}

void loop()
{
  Serial.println("something");
  //Application_FunctionSet.BruteForce();
  Application_FunctionSet.Test();

  //Application_FunctionSet.ApplicationFunctionSet_Obstacle();

  //Application_FunctionSet.GyroTest();
}
