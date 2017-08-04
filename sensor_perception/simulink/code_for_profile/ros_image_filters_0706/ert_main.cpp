//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: ert_main.cpp
//
// Code generated for Simulink model 'ros_image_filters_0706'.
//
// Model version                  : 1.446
// Simulink Coder version         : 8.12 (R2017a) 16-Feb-2017
// C/C++ source code generated on : Mon Jul 10 14:54:12 2017
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include <stdio.h>
#include <stdlib.h>
#include "ros_image_filters_0706.h"
#include "ros_image_filters_0706_private.h"
#include "rtwtypes.h"
#include "limits.h"
#include "rt_nonfinite.h"
#include "linuxinitialize.h"
#define UNUSED(x)                      x = x

// Function prototype declaration

int stepCounter = 20;

void exitFcn(int sig);
void *terminateTask(void *arg);
void *baseRateTask(void *arg);
void *subrateTask(void *arg);
volatile boolean_T runModel = true;
sem_t stopSem;
sem_t baserateTaskSem;
pthread_t schedulerThread;
pthread_t baseRateThread;
unsigned long threadJoinStatus[8];
int terminatingmodel = 0;
void *baseRateTask(void *arg)
{
  runModel = (rtmGetErrorStatus(rtM) == (NULL));
  while (runModel) {
    sem_wait(&baserateTaskSem);
    ros_image_filters_0706_step();
    // Get model outputs here
    runModel = (rtmGetErrorStatus(rtM) == (NULL));
    
    
    std::cout << stepCounter << std::endl;
    
    if ( 0 == --stepCounter )
    {
        std::cout << "exiting" << std::endl;
        SLROSNodePtr->shutdown();
        break;
    }
  }
  runModel = 0;
  terminateTask(arg);
  pthread_exit((void *)0);
  return NULL;
}

void exitFcn(int sig)
{
  UNUSED(sig);
  rtmSetErrorStatus(rtM, "stopping the model");
}

void *terminateTask(void *arg)
{
  UNUSED(arg);
  terminatingmodel = 1;

  {
    runModel = 0;
  }

  // Disable rt_OneStep() here

  // Terminate model
  ros_image_filters_0706_terminate();
  sem_post(&stopSem);
  return NULL;
}

int main(int argc, char **argv)
{
  UNUSED(argc);
  UNUSED(argv);
  void slros_node_init(int argc, char** argv);
  slros_node_init(argc, argv);
  rtmSetErrorStatus(rtM, 0);
  std::cout << "000" << std::endl;
  // Initialize model
  ros_image_filters_0706_initialize();
  std::cout << "111" << std::endl;
  // Call RTOS Initialization function
  myRTOSInit(0.01, 0);
  
  
  // Wait for stop semaphore
  sem_wait(&stopSem);
  
  std::cout << "222" << std::endl;
  return 0;
}

//
// File trailer for generated code.
//
// [EOF]
//
