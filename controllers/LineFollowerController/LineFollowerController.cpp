// File:          LineFollowerController.cpp
// Date: 1400/08/20
// Description:
// Author: HJB
// Modifications:

#include <limits>

// You may need to add webots include files such as
#include <webots/Robot.hpp>
#include <webots/Device.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/LED.hpp>
#include <webots/Motor.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Gyro.hpp>

// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace std;

/* Device stuff */
#define TIME_STEP 64
#define LEFT 0
#define RIGHT 1
#define MAX_SPEED 6.28*4
#define DISTANCE_SENSORS_NUMBER 8
// 5 IR ground color sensors
#define GROUND_SENSORS_NUMBER 5

#define GS_WHITE 900
#define GS_LEFT 0
#define GS_CENTER 1
#define GS_RIGHT 2
// 8 LEDs
#define LEDS_NUMBER 8
#define LFM_FORWARD_SPEED 200
#define LFM_K_GS_SPEED 0.4

class LineRobot : public Robot
{
  private:
      DistanceSensor *FieldDistanceSensor[DISTANCE_SENSORS_NUMBER];
      double FieldDistanceSensor_Values[DISTANCE_SENSORS_NUMBER];
      DistanceSensor *GroundIRSensor[GROUND_SENSORS_NUMBER];
      double GroundIRSensor_Values[GROUND_SENSORS_NUMBER];
      LED *leds[LEDS_NUMBER];
      bool leds_values[LEDS_NUMBER];
      Motor *left_motor, *right_motor;
      double MotorSpeeds[2];
      Accelerometer *EpuckAccel;
      const double *AccelValues;
      Gyro *EpuckGyro;
      const double *GyroValues;


      void PassiveWait(double sec)
      {
        double start_time = getTime();
        do {
          Step();
        } while (start_time + sec > getTime());
      }

      int GetTimeStep()
      {
         static int time_step = -1;
         if (time_step == -1)
           time_step = (int)getBasicTimeStep();
         return time_step;
       }

  public:
      LineRobot()
      {
        //char distance_sensors_names[DISTANCE_SENSORS_NUMBER][4] =
        //                 {"ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"};
        char ground_sensors_names[GROUND_SENSORS_NUMBER][4] =
                         {"gs0", "gs1", "gs2", "gs3", "gs4"};
        char leds_names[LEDS_NUMBER][5] =
             {"led0", "led1", "led2", "led3", "led4", "led5", "led6", "led7"};

        int i;
        //for (i = 0; i < DISTANCE_SENSORS_NUMBER; i++)
        //{
        //  FieldDistanceSensor[i] = getDistanceSensor(distance_sensors_names[i]);
        //  FieldDistanceSensor[i]->enable(TIME_STEP);
        //}

        for (i = 0; i < GROUND_SENSORS_NUMBER; i++)
        {
          GroundIRSensor[i] = getDistanceSensor(ground_sensors_names[i]);
          GroundIRSensor[i]->enable(TIME_STEP);
        }

        for (i = 0; i < LEDS_NUMBER; i++)
        {
          leds[i] = getLED(leds_names[i]);
        }

       // get a handler to the motors and set target position to infinity (speed control).
        left_motor = getMotor("left wheel motor");
        right_motor = getMotor("right wheel motor");
        left_motor-> setPosition(std::numeric_limits<double>::infinity());
        right_motor->setPosition(std::numeric_limits<double>::infinity());
        left_motor->setVelocity(0.0);
        right_motor->setVelocity(0.0);

        EpuckAccel=getAccelerometer("accelerometer");
        EpuckAccel->enable(TIME_STEP);

        EpuckGyro=getGyro("gyro");
        EpuckGyro->enable(TIME_STEP);

      }

      void Step()
      {
        if (step(GetTimeStep()) == -1)
        {
           exit(EXIT_SUCCESS);
        }
      }

      void ResetActuatorValues()
      {
        int i;
        for (i = 0; i < 2; i++)
          MotorSpeeds[i] = MAX_SPEED*((rand() % 20)/10.0-1.0);

        for (i = 0; i < LEDS_NUMBER; i++)
          leds_values[i] = false;
      }

      void GetSensorInputs()
      {
        int i;
        for (i = 0; i < DISTANCE_SENSORS_NUMBER; i++)
        {
          FieldDistanceSensor_Values[i] = FieldDistanceSensor[i]->getValue();
        }

      }

      void GetGroundSensorInputs(bool ShouldPrintValues)
      {
        int i;
        for (i = 0; i < GROUND_SENSORS_NUMBER; i++)
        {
          GroundIRSensor_Values[i] = GroundIRSensor[i]->getValue();
        }
        if(ShouldPrintValues)
        {
            cout<<"Ground IR Sensor values:  S0 (LL)="<<GroundIRSensor_Values[0];
            cout<<",  S1 (L)="<<GroundIRSensor_Values[1];
            cout<<",  S2 (M)="<<GroundIRSensor_Values[2];
            cout<<",  S3 (R)="<<GroundIRSensor_Values[3];
            cout<<",  S4 (RR)="<<GroundIRSensor_Values[4]<<endl;
        }

      }

      void GetAccInputs(bool ShouldPrintValues)
      {
        AccelValues=EpuckAccel->getValues();

        if(ShouldPrintValues)
        {
            cout<<"Accelerometer:  x ="<<AccelValues[0];
            cout<<",  y ="<<AccelValues[1];
            cout<<",  z ="<<AccelValues[2]<<endl;
        }
      }

      void GetGyroInputs(bool ShouldPrintValues)
      {
        GyroValues=EpuckGyro->getValues();

        if(ShouldPrintValues)
        {
            cout<<"Gyro:  x ="<<GyroValues[0];
            cout<<",  y ="<<GyroValues[1];
            cout<<",  z ="<<GyroValues[2]<<endl;
        }
      }

      void SetActuators()
      {
        int i;
        for (i = 0; i < LEDS_NUMBER; i++)
          leds[i]->set(leds_values[i]);

        left_motor->setVelocity(MotorSpeeds[LEFT]);
        right_motor->setVelocity(MotorSpeeds[RIGHT]);
      }

      void BlinkLeds()
      {
        static int counter = 0;
        counter++;
        for (int ii=0;ii<LEDS_NUMBER;ii++)
            leds_values[ii] = counter%20;
      }

      void GoBackwards()
      {
        left_motor->setVelocity(-MAX_SPEED);
        right_motor->setVelocity(-MAX_SPEED);
        PassiveWait(0.2);
      }

      void TurnLeft()
      {
        left_motor->setVelocity(-MAX_SPEED);
        right_motor->setVelocity(MAX_SPEED);
        PassiveWait(0.2);
      }

      void LineFollowingFunc()
      {
        int DeltaS = 0;
        DeltaS = GroundIRSensor_Values[GS_RIGHT] - GroundIRSensor_Values[GS_LEFT];

        MotorSpeeds[LEFT]  = (LFM_FORWARD_SPEED - LFM_K_GS_SPEED * DeltaS)*0.00628;
        MotorSpeeds[RIGHT] = (LFM_FORWARD_SPEED + LFM_K_GS_SPEED * DeltaS)*0.00628;
      }

};


// This is the main program of your controller.
int main(int argc, char **argv)
{

  // create the Robot instance.
  LineRobot *LineFollowerRobot = new LineRobot();

  // get the time step of the current world.
  int timeStep = (int)LineFollowerRobot->getBasicTimeStep();

  // Initialization
  LineFollowerRobot->ResetActuatorValues();
  LineFollowerRobot->GetGroundSensorInputs(false);
  LineFollowerRobot->GetAccInputs(false);
  LineFollowerRobot->GetGyroInputs(false);

  LineFollowerRobot->Step();

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  cout<<"Here"<<endl;
  while (LineFollowerRobot->step(timeStep) != -1)
  {

      // Read the sensors:
      //LineFollowerRobot->GetSensorInputs();
      LineFollowerRobot->GetGroundSensorInputs(true);
      LineFollowerRobot->GetAccInputs(true);
      LineFollowerRobot->GetGyroInputs(true);

      LineFollowerRobot->BlinkLeds();

    // Process sensor data here.
      LineFollowerRobot->LineFollowingFunc();

    // Enter here functions to send actuator commands, like:
      LineFollowerRobot->SetActuators();
      LineFollowerRobot->Step();
  };

  // Enter here exit cleanup code.

  delete LineFollowerRobot;
  return 0;
}
