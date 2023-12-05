#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/adc.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"
#include "driverlib/rom.h"
#include "grlib/grlib.h"
#include "drivers/cfal96x64x16.h"

bool heartbeatOn = true;
bool waveOn = true;
bool fullOn = false;
bool motorOn = false;
bool reverseOn = false;
bool changeRPM = true;
bool followOn = false;
uint32_t pui32ADC0Value[1]; //ADC ARRAY
int32_t new_char = 'a';
char newBuff[20]; //sprintf buffer
bool splashOn = true;
int rpm = 60;
int div = 0;
int i = 3;
int j = 3;
int count = 0;
int delta = 0;
int next = 0;
int step = 0;
int current = 0;
const uint8_t full_step_array[4] = {0x0C, 0x06, 0x03, 0x09};
const uint8_t wave_drive_array[4] = {0x08, 0x04, 0x02, 0x01};
#define MOTOR_OFF (0x00)
#define STEPS_PER_REV (200)
#define INIT_RPM (60)
#define RPM_MIN (1)
#define RPM_MAX (180)

tRectangle sRect;
tContext sContext;
//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>Timer (timers)</h1>
//!
//! This example application demonstrates the use of the timers to generate
//! periodic interrupts.  One timer is set up to interrupt once per second and
//! the other to interrupt twice per second; each interrupt handler will toggle
//! its own indicator on the display.
//
//*****************************************************************************

//*****************************************************************************
//
// Flags that contain the current value of the interrupt indicator as displayed
// on the CSTN display.
//
//*****************************************************************************
uint32_t g_ui32Flags;



//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

void print_menu();

void menu_process(int32_t local_char);

void splashScreen(tRectangle sRect, tContext sContext);

//*****************************************************************************
//
// Send a string to the UART.
//
//****************************************************************************
void
UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count)
{
    //
    // Loop while there are more characters to send.
    //
    while(ui32Count--)
    {
        //
        // Write the next character to the UART.
        //
        UARTCharPut(UART0_BASE, *pui8Buffer++);
    }
}

//*****************************************************************************
//
// The UART interrupt handler.
//
//*****************************************************************************
void
UARTIntHandler(void)
{
    uint32_t ui32Status;

    //
    // Get the interrrupt status.
    //
    ui32Status = UARTIntStatus(UART0_BASE, true);
   
    //
    // Clear the asserted interrupts.
    //
    UARTIntClear(UART0_BASE, ui32Status);

    int32_t local_char;
        
    if(UARTCharsAvail(UART0_BASE)){
        local_char = UARTCharGetNonBlocking(UART0_BASE);
        menu_process(local_char);
    }
    
}

//*****************************************************************************
//
// The interrupt handler for the first timer interrupt.
//
//*****************************************************************************
void
Timer0IntHandler()
{
    // Clear the timer interrupt.
    //
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    
    if(followOn == true){
      div = (rpm*200)/(60);
      TimerLoadSet(TIMER0_BASE, TIMER_A, (16000000/div));
      
      
      ADCProcessorTrigger(ADC0_BASE, 3);
      ADCIntClear(ADC0_BASE, 3);
      ADCSequenceDataGet(ADC0_BASE, 3, pui32ADC0Value);
      
      current = pui32ADC0Value[0]*(4095/200);
      delta = next - current;
      
      if(count != delta){
        if(delta > 0){
          count++;
          step++;                                                                                                               
        }else if(delta < 0){
          count--;
          step--;
        }
       GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2
                         | GPIO_PIN_3, wave_drive_array[count % 4]);
      }
      next = current;
    }
    
    //WAVE DRIVE
    if(waveOn == true){
      div = (rpm*200)/(60);
      TimerLoadSet(TIMER0_BASE, TIMER_A, (16000000/div));
      
      if(new_char == 'x'){
        
         GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2
                         | GPIO_PIN_3, wave_drive_array[i]);
         if(reverseOn == true){
           i--;
           if(i < 0){
             i = 3;
           }
         }else if(reverseOn == false){
             i = (i+1) % 4;
         }
        }
      }
    
    //FULL STEP
    if(fullOn == true){
      div = (rpm*200)/(60);
      TimerLoadSet(TIMER0_BASE, TIMER_A, (SysCtlClockGet()/div));
      
      if(new_char == 'x'){
        
        GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2
                        | GPIO_PIN_3, full_step_array[i]);
         if(reverseOn == true){
           i--;
           if(i < 0){
             i = 3;
           }
         }else if(reverseOn == false){
             i = (i+1) % 4;
         }
      }
    }
  
   
}



//*****************************************************************************
//
// This example application demonstrates the use of the timers to generate
// periodic interrupts.
//
//*****************************************************************************
int
main(void)
{
    tRectangle sRect;

    //
    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    //
    FPULazyStackingEnable();

    //
    // Set the clocking to run directly from the crystal.
    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
    
    //
    // Initialize the display driver.
    //
    CFAL96x64x16Init();

    //
    // Initialize the graphics context and find the middle X coordinate.
    //
    GrContextInit(&sContext, &g_sCFAL96x64x16);

    //
    // Fill the top part of the screen with blue to create the banner.
    //
    sRect.i16XMin = 0;
    sRect.i16YMin = 0;
    sRect.i16XMax = GrContextDpyWidthGet(&sContext) - 1;
    sRect.i16YMax = 9;
    GrContextForegroundSet(&sContext, ClrDarkBlue);
    GrRectFill(&sContext, &sRect);
    
    
   
    //UART
    //
    // Enable the peripherals used by this example.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    //LED
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOG))
    {
    }

    //
    // Set GPIO A0 and A1 as UART pins.
    //
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Configure the UART for 115,200, 8-N-1 operation.
    //
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));
    
    
    //
    // Enable the UART interrupt.
    //
    IntEnable(INT_UART0);
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
    //
    // Change foreground for white text.
    //
    GrContextForegroundSet(&sContext, ClrWhite);

    //
    // Put the application name in the middle of the banner.
    //
    GrContextFontSet(&sContext, g_psFontFixed6x8);
    GrStringDrawCentered(&sContext, "timers", -1,
                         GrContextDpyWidthGet(&sContext) / 2, 4, 0);

    //
    // Initialize timer status display.
    //
    GrContextFontSet(&sContext, g_psFontFixed6x8);
    
    splashScreen(sRect, sContext);
    
    //
    // Enable the peripherals used by this example.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

    //
    // Enable processor interrupts.
    //

    //
    // Configure the two 32-bit periodic timers.
    //
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    div = (60*200)/(60);
    TimerLoadSet(TIMER0_BASE, TIMER_A, (SysCtlClockGet()/div));

    //
    // Setup the interrupts for the timer timeouts.
    //
    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    //
    // Enable the timers.
    //
    TimerEnable(TIMER0_BASE, TIMER_A);
    

    
    GPIOPinTypeGPIOOutput(GPIO_PORTG_BASE, GPIO_PIN_2);
    
    //ADC
     SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
     SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
     GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_6);
     ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
     ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH21|
                             ADC_CTL_END);
     ADCSequenceEnable(ADC0_BASE, 3);
     
     //GENERAL PURPOSE GPIO PORT L
      SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
      while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOL))
    {
    }
    GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2
                         | GPIO_PIN_3);
    
     print_menu();
     IntMasterEnable();
     
    //
    // Loop forever while the timers run.
    //
    
    while(1)
    {
        // Turn on the LED.
        if(heartbeatOn == true){
          GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_2, GPIO_PIN_2);
          SysCtlDelay(115200);
        
          GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_2, 0);
          SysCtlDelay(115200);  
          }
        
       
      
    }
}

//print menu
void print_menu(){
  UARTSend((uint8_t *)"Menu Selection:\r\n", 17);
  UARTSend((uint8_t *)"M - Print this menu\r\n", 21);
  UARTSend((uint8_t *)"L - Toggle flashing LED\r\n", 25);
  UARTSend((uint8_t *)"C - Clear terminal window\r\n", 28);
  UARTSend((uint8_t *)"S - Print splash screen\r\n", 28);
  UARTSend((uint8_t *)"F - Full Step\r\n", 15);
  UARTSend((uint8_t *)"W - Wave Drive\r\n", 16);
  UARTSend((uint8_t *)"A - Add RPM\r\n", 13);
  UARTSend((uint8_t *)"D - Subtract RPM\r\n", 18);
  UARTSend((uint8_t *)"X - Turn On/Off\r\n", 17);
  UARTSend((uint8_t *)"R - Reverse\r\n", 13);
  UARTSend((uint8_t *)"H - Follow Mode\r\n", 17);
  
}

//menu controls
void menu_process(int32_t local_char)
{
 
  
  //Print menu
  if(local_char == 'm'){
    print_menu();
  }
  
  //Clear terminal
  if(local_char == 'c'){
    UARTCharPut(UART0_BASE,(char)12);
  }
  
  //LED Toggle
  if(local_char == 'd'){
    heartbeatOn = !heartbeatOn;
  }
  
  //Motor On
  if(local_char == 'x'){
    motorOn = !motorOn;
  }
  
  //RPM
    if(local_char == 'a'){
      sprintf(newBuff,"%d\r\n",rpm);
      UARTSend((uint8_t *)newBuff, 6);
      GrStringDrawCentered(&sContext, newBuff, -1,
                         GrContextDpyWidthGet(&sContext) / 2, 40, true);
      rpm++;
      if(rpm > 179){
        rpm = 180;
        sprintf(newBuff,"%d\r\n",rpm);
        UARTSend((uint8_t *)newBuff, 6);
      }
    }else if(local_char == 'd'){
      sprintf(newBuff,"%d\r\n",rpm);
      UARTSend((uint8_t *)newBuff, 6);
      GrStringDrawCentered(&sContext, newBuff, -1,
                         GrContextDpyWidthGet(&sContext) / 2, 40, true);
      rpm--;
      if(rpm <= 2){
        rpm = 1;
 
      }
    }
  
  //ON AND OFF
  if(local_char == 'x'){
    if(new_char == 'x'){
        new_char = 'a';
    }else{
      new_char = 'x';
    }
  }
  
  //MODE SWITCHING
  if(local_char == 'w'){
    waveOn = !waveOn;
    fullOn = false;
    followOn = false;
  }
  
  if(local_char == 'f'){
    fullOn = !fullOn;
    waveOn = false;
    followOn = false;
  }
  
  if(local_char == 'r'){
    reverseOn = !reverseOn;
  }
  
  if(local_char == 'h'){
    followOn = !followOn;
    waveOn = false;
    fullOn = false;
  }
}

void splashScreen(tRectangle sRect, tContext sContext){
  IntMasterDisable();
   sRect.i16XMin = 0;
    sRect.i16YMin = 0;
    sRect.i16XMax = GrContextDpyWidthGet(&sContext) - 1;
    sRect.i16YMax = GrContextDpyHeightGet(&sContext) -1;
    GrContextForegroundSet(&sContext, ClrBlack);
    GrRectFill(&sContext, &sRect);
  
  GrContextForegroundSet(&sContext, ClrWhite);
  
   GrStringDrawCentered(&sContext, "I am", -1,
                         GrContextDpyWidthGet(&sContext) / 2, 20, false);
   GrStringDrawCentered(&sContext, "really glad", -1,
                         GrContextDpyWidthGet(&sContext) / 2, 30, false);
   GrStringDrawCentered(&sContext, "this is", -1,
                         GrContextDpyWidthGet(&sContext) / 2, 40, false);
   GrStringDrawCentered(&sContext, "working", -1,
                         GrContextDpyWidthGet(&sContext) / 2, 50, false);
          
    SysCtlDelay(1*1000*10000);
           
    sRect.i16XMin = 0;
    sRect.i16YMin = 0;
    sRect.i16XMax = GrContextDpyWidthGet(&sContext) - 1;
    sRect.i16YMax = GrContextDpyHeightGet(&sContext) -1;
    GrContextForegroundSet(&sContext, ClrBlack);
    GrRectFill(&sContext, &sRect);
           
    GrContextForegroundSet(&sContext, ClrWhite);
    IntMasterEnable();
}
