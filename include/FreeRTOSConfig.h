#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

void vAssertCalled(const char* file, int line);

#define configUSE_PREEMPTION                                    1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION                 0
#define configUSE_TICKLESS_IDLE                                 0
#define configUSE_IDLE_HOOK                                     0
#define configUSE_MALLOC_FAILED_HOOK                            1
#define configUSE_DAEMON_TASK_STARTUP_HOOK                      0
#define configUSE_SB_COMPLETED_CALLBACK                         0
#define configUSE_TICK_HOOK                                     0
#define configCPU_CLOCK_HZ                                      48000000
#define configTICK_RATE_HZ                                      200
#define configMAX_PRIORITIES                                    4
#define configMINIMAL_STACK_SIZE                                64
#define configMAX_TASK_NAME_LEN                                 4
#define configUSE_TRACE_FACILITY                                0
#define configUSE_STATS_FORMATTING_FUNCTIONS                    0
#define configUSE_16_BIT_TICKS                                  0
#define configIDLE_SHOULD_YIELD                                 1
#define configUSE_TASK_NOTIFICATIONS                            0
#define configTASK_NOTIFICATION_ARRAY_ENTRIES                   1
#define configUSE_MUTEXES                                       0
#define configUSE_RECURSIVE_MUTEXES                             0
#define configUSE_COUNTING_SEMAPHORES                           0
#define configUSE_ALTERNATIVE_API                               0
#define configCHECK_FOR_STACK_OVERFLOW                          2
#define configQUEUE_REGISTRY_SIZE                               0
#define configUSE_QUEUE_SETS                                    0
#define configUSE_TIME_SLICING                                  1
#define configUSE_NEWLIB_REENTRANT                              0
#define configENABLE_BACKWARD_COMPATIBILITY                     0
#define configNUM_THREAD_LOCAL_STORAGE_POINTERS                 1
#define configUSE_MINI_LIST_ITEM                                0
#define configSTACK_DEPTH_TYPE                                  uint32_t
#define configMESSAGE_BUFFER_LENGTH_TYPE                        uint8_t
#define configSUPPORT_STATIC_ALLOCATION                         0
#define configSUPPORT_DYNAMIC_ALLOCATION                        1
#define configTOTAL_HEAP_SIZE                                   2048
#define configAPPLICATION_ALLOCATED_HEAP                        0
#define configSTACK_ALLOCATION_FROM_SEPARATE_HEAP               0
#define configGENERATE_RUN_TIME_STATS                           0
#define configUSE_CO_ROUTINES                                   0
#define configMAX_CO_ROUTINE_PRIORITIES                         3
#define configUSE_TIMERS                                        0
#define configTIMER_TASK_PRIORITY                               0
#define configTIMER_QUEUE_LENGTH                                0
#define configTIMER_TASK_STACK_DEPTH                            configMINIMAL_STACK_SIZE
#define configMAX_SYSCALL_INTERRUPT_PRIORITY                    3
#define configASSERT(x)                                         if ((x) == 0) vAssertCalled(__FILE__, __LINE__)
#define configENABLE_MPU                                        0
#define configENABLE_FPU                                        0
#define configENABLE_MVE                                        0
#define configENFORCE_SYSTEM_CALLS_FROM_KERNEL_ONLY             0
#define configHEAP_CLEAR_MEMORY_ON_FREE                         1
#define configUSE_STREAM_BUFFERS                                0

#define INCLUDE_vTaskPrioritySet                                1
#define INCLUDE_uxTaskPriorityGet                               1
#define INCLUDE_vTaskDelete                                     1
#define INCLUDE_vTaskSuspend                                    1
#define INCLUDE_vTaskDelayUntil                                 1
#define INCLUDE_vTaskDelay                                      1
#define INCLUDE_xTaskGetSchedulerState                          1
#define INCLUDE_xTaskGetCurrentTaskHandle                       1
#define INCLUDE_uxTaskGetStackHighWaterMark                     0
#define INCLUDE_uxTaskGetStackHighWaterMark2                    0
#define INCLUDE_xTaskGetIdleTaskHandle                          0
#define INCLUDE_eTaskGetState                                   0
#define INCLUDE_xTimerPendFunctionCall                          0
#define INCLUDE_xTaskAbortDelay                                 0
#define INCLUDE_xTaskGetHandle                                  0
#define INCLUDE_xTaskResumeFromISR                              1

#endif // FREERTOS_CONFIG_H
