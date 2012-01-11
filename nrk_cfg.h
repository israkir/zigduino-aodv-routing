/***************************************************************
*                            NanoRK CONFIG                     *
***************************************************************/

#ifndef __nrk_cfg_h	
#define __nrk_cfg_h

// This is a special kernel flag that records various timer
// related functions.  This is not necessary for normal operation
// #define NRK_KERNEL_TEST


// NRK_REPORT_ERRORS will cause the kernel to print out information about
// missed deadlines or reserve violations
//#define NRK_REPORT_ERRORS
// NRK_HALT_ON_ERRORS will cause the kernel to freeze on errors so that
// it is easier to see debugging messages.
//#define NRK_HALT_ON_ERROR
//#define NRK_HALT_AND_LOOP_ON_ERROR

// NRK_STACK_CHECK adds a little check to see if the bottom of the stack
// has been over written on all suspend calls
#define NRK_STACK_CHECK

// Leave NRK_NO_POWER_DOWN define in if the target can not wake up from sleep 
// because it has no asynchronously clocked
#define NRK_NO_POWER_DOWN

// This protects radio access to allow for multiple devices accessing
// the radio
#define RADIO_PRIORITY_CEILING		10

// Enable buffered and signal controlled serial RX
#define NRK_UART_BUF   1
#define MAX_RX_UART_BUF 16

// Max number of tasks in your application
// Be sure to include the idle task
// Making this the correct size will save on BSS memory which
// is both RAM and ROM...
#define NRK_MAX_TASKS       		5   
#define	NRK_N_RES			1	
                           
#define NRK_TASK_IDLE_STK_SIZE         512   // Idle task stack size min=32 
#define NRK_APP_STACKSIZE              512 
#define NRK_KERNEL_STACKSIZE           512 
#define NRK_MAX_RESOURCE_CNT           2

// Define ALL signals below
#define NRK_UART_RX_EVENT   (NRK_MAX_RESOURCE_CNT+3)
#define NRK_MAX_DRIVER_CNT		1


#endif
