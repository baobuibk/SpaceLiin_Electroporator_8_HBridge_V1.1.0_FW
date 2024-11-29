// APP HEADER //
#include "app.h"

static void Status_Led(void*);

#define         SCHEDULER_TASK_COUNT  6
uint32_t 	g_ui32SchedulerNumTasks = SCHEDULER_TASK_COUNT;
tSchedulerTask 	g_psSchedulerTable[SCHEDULER_TASK_COUNT] =
                {
                    {
                            &H_Bridge_Task,
                            (void *) 0,
                            2,                          //call every 500us
                            0,                          //count from start
                            true                        //is active
                    },
                    {
                            &CMD_Line_Task,
                            (void *) 0,
                            10,                         //call every 1ms
                            0,                          //count from start
                            true                        //is active
                    },
                    {
                            &FSP_Line_Task,
                            (void *) 0,
                            20,                         //call every 1ms
                            0,                          //count from start
                            true                        //is active
                    },
                    {
                            &Current_Sense_Task,
                            (void *) 0,
                            10,                         //call every 1ms
                            0,                          //count from start
                            false                       //is active
                    },
                    {
                            &Sensor_Read_Task,
                            (void *) 0,
                            10,                         //call every 1ms
                            0,                          //count from start
                            true                        //is active
                    },
                    {
                            &Status_Led,
                            (void *) 0,
                            10000,                      //call every 1ms
                            0,                          //count from start
                            true                        //is active
                    },
                };

void App_Main(void)
{   
    // STM32F030CCT6 @ 36MHz, 
    // can run scheduler tick max @ 100us.
    SchedulerInit(10000);

    CMD_Line_Task_Init();
    H_Bridge_Driver_Init();
    V_Switch_Driver_Init();
    FSP_Line_Task_Init();
    Current_Sense_Task_Init(LL_ADC_SAMPLINGTIME_15CYCLES);
    Sensor_Task_Init();
    
    /*
    BMP390_init();
    */

    while (1)
    {
        SchedulerRun();
    }
}

static void Status_Led(void*)
{
    LL_GPIO_TogglePin(DEBUG_LED_PORT, DEBUG_LED_PIN);
}
