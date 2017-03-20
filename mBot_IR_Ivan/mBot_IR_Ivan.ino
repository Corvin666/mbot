/*************************************************************************
* File Name          : Makeblock IR_Controle.ino
* Author             : Jasen
* Updated            : Jasen
* Version            : V1.1.0
* Date               : 5/22/2014
* Description        : Demo code for Makeblock Starter Robot kit,two motors
                       connect on the M1 and M2 port of baseshield, The IR receiver module
                       connect on port 6.
                       The four black button on the IR controller is used to control the direction 
                       of robot, the number button on the IR controller is for changing the speed of the robot.
                       button 1 is for setting the speed to the slowest,button 9 is for setting the speed to fastest.
* License            : CC-BY-SA 3.0
* Copyright (C) 2013 Maker Works Technology Co., Ltd. All right reserved.
* http://www.makeblock.cc/
**************************************************************************/
#include <Makeblock.h>
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Wire.h>

MeDCMotor MotorL(M1);  
MeDCMotor MotorR(M2);
MeInfraredReceiver infraredReceiverDecode(PORT_6);
MeUltrasonicSensor UltrasonicSensor(PORT_3);
int moveSpeed = 190;
boolean leftflag,rightflag;
int minSpeed = 55;
int factor = 23;
int distance = 0;

// Forward declarations
void Forward(bool safetyCheck = true);
void Stop();
void TurnRight();
void TurnLeft();

enum RobotStates_t
{
  ROBOT_STATE_IR,
  ROBOT_STATE_PRG_VASYA,
  ROBOT_STATE_PRG_RUN10,
  ROBOT_STATE_PRG_PATHFINDING
};
RobotStates_t currentState;
void switchState(enum RobotStates_t state);

enum VasyaStates_t
{
   VS_RUNNING,
   VS_RETREATING
};
 
struct VasyaState_t
{
   VasyaStates_t curState;
   unsigned long msCounter;
} vasyaState;

struct run10State_t
{
   unsigned long msCounter;
} run10State;

struct pathfindingState_t
{
   unsigned long msCounter;
} pathfindingState;

void setup()
{
    currentState = ROBOT_STATE_IR;
    infraredReceiverDecode.begin();
    Serial.begin(9600);
    randomSeed(analogRead(0));
}

// This will clear IR buffer and in switch to IR mode if needed.
void clearIRBufferAndCheckCancelProgram()
{
    if(infraredReceiverDecode.available()||infraredReceiverDecode.buttonState())
    {
        switch(infraredReceiverDecode.read())
        {
          case IR_BUTTON_D: 
               Serial.println("Aborting current program and switching to IR control");
               Stop();
               switchState(ROBOT_STATE_IR);
               break;
        }
    }
}

void loop()
{
  switch (currentState)
  {
     case ROBOT_STATE_IR:
       robotIRLoop();
       break;
     case ROBOT_STATE_PRG_VASYA:
       robotVasyaLoop();
       break;
     case ROBOT_STATE_PRG_RUN10:
       robotRun5Loop();
       break;
     case ROBOT_STATE_PRG_PATHFINDING:
       // Defined in pathfinding.ino
       robotRunPathFindingLoop();
       break;
  }

}

void switchState(enum RobotStates_t state)
{
  currentState = state;

  switch (currentState)
  {
    case ROBOT_STATE_PRG_VASYA:
      // Initialize Vasya program. Run both motors, start Running state
      Serial.println("Vasya PRG init");
      vasyaState.msCounter = 0;
      vasyaState.curState = VS_RUNNING;
      MotorL.run(moveSpeed);
      MotorR.run(moveSpeed);
      break;
    case ROBOT_STATE_PRG_RUN10:
      // Start runing forward for 10 seconds
      run10State.msCounter = millis();
      MotorL.run(moveSpeed);
      MotorR.run(moveSpeed);
      break;
  case ROBOT_STATE_PRG_PATHFINDING:
      // Run the pathfinding logic
      pathfindingState.msCounter = millis();
      MotorL.run(moveSpeed);
      MotorR.run(moveSpeed);
      break;
  } 
}

void robotRun5Loop()
{
  // Clear IR buffer
  clearIRBufferAndCheckCancelProgram();
  
  unsigned long timeelapsed = millis()-run10State.msCounter;
  
  if (timeelapsed > 10000)
  {
    Serial.println("Switching to IR control");
    Stop();
    switchState(ROBOT_STATE_IR);
  }else
  if ((timeelapsed & 1023) < 10) // Blink the serial LED when close to a second mark. Calc us a optimization for (elapsed % 1000)
  {
    Serial.println("Second mark");
  }
}

void robotVasyaLoop()
{
  // Clear IR buffer
  clearIRBufferAndCheckCancelProgram();
  
  switch(vasyaState.curState)
  {
    case VS_RUNNING:
        Serial.println("Running");
        distance = UltrasonicSensor.distanceCm();
        Serial.println(distance);
        
        if ((distance != 0) && (distance < 10))
        {
          // Have to retreat!
          MotorL.run(-moveSpeed);
          MotorR.run(-moveSpeed);
          vasyaState.curState = VS_RETREATING;
          vasyaState.msCounter = millis();
        }
        break;
      
    case VS_RETREATING:
        unsigned long ms = millis();
        Serial.println(ms - vasyaState.msCounter);
        
        if ((ms-vasyaState.msCounter) > 3000)
        {
          Serial.println("Switching to IR control");
          Stop();
          switchState(ROBOT_STATE_IR);
        }
      break;
  }
}

void robotIRLoop()
{
   if(infraredReceiverDecode.available()||infraredReceiverDecode.buttonState())
    {
        switch(infraredReceiverDecode.read())
        {
          case IR_BUTTON_PLUS: 
               Forward();
               break;
          case IR_BUTTON_MINUS:
               Backward();
               break;
          case IR_BUTTON_NEXT:
               TurnLeft();
               break;
          case IR_BUTTON_PREVIOUS:
               TurnRight();
               break;
          case IR_BUTTON_9:
               ChangeSpeed(factor*9+minSpeed);
               break;
          case IR_BUTTON_8:
               ChangeSpeed(factor*8+minSpeed);
               break;
          case IR_BUTTON_7:
               ChangeSpeed(factor*7+minSpeed);
               break;
          case IR_BUTTON_6:
               ChangeSpeed(factor*6+minSpeed);
               break;
          case IR_BUTTON_5:
               ChangeSpeed(factor*5+minSpeed);
               break;
          case IR_BUTTON_4:
               ChangeSpeed(factor*4+minSpeed);
               break;
         case IR_BUTTON_3:
               ChangeSpeed(factor*3+minSpeed);
               break;
         case IR_BUTTON_2:
               ChangeSpeed(factor*2+minSpeed);
               break;
         case IR_BUTTON_1:
               ChangeSpeed(factor*1+minSpeed);
               break;
         case IR_BUTTON_A:
               Serial.println("Running program Vasya!");
               switchState(ROBOT_STATE_PRG_VASYA);
               break;
         case IR_BUTTON_B:
               Serial.println("Running program 10secAhead!");
               switchState(ROBOT_STATE_PRG_RUN10);
               break;
         case IR_BUTTON_C:
               Serial.println("Running program pathfinder");
               switchState(ROBOT_STATE_PRG_PATHFINDING);
               break;
         case IR_BUTTON_0:
               buzzerOn();
               delay(50);
               buzzerOff();
               distance = UltrasonicSensor.distanceCm();
               Serial.println(distance);
               break;          
        }
    }
    else
    {
      Stop();
    }
}

void Forward(bool safetyCheck)
{
  Serial.println("Forward");
  distance = UltrasonicSensor.distanceCm();
  Serial.println(distance);
  
  if (((distance == 0) || (distance > 10)) || !safetyCheck)
  {
    MotorL.run(moveSpeed);
    MotorR.run(moveSpeed);
  }
  else
  {
    Stop();
  }
}
void Backward()
{
  Serial.println("Backward");
  MotorL.run(-moveSpeed);
  MotorR.run(-moveSpeed);
}
void TurnLeft()
{
  Serial.println("Left");
  MotorL.run(-moveSpeed);
  MotorR.run(moveSpeed);
}
void TurnRight()
{
  Serial.println("Right");
  
  MotorL.run(moveSpeed);
  MotorR.run(-moveSpeed);
}
void Stop()
{
//  Serial.println("Stop!");
  MotorL.run(0);
  MotorR.run(0);
}
void ChangeSpeed(int spd)
{
  moveSpeed = spd;
}
