
extern "C"
{
#include "can.h"
}

//#define PART_TM4C1294NCPDT
#include <tm4c1294ncpdt.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <driverlib/sysctl.h>
#include <driverlib/gpio.h>
#include <driverlib/pin_map.h>
#include <driverlib/pwm.h>
#include <driverlib/fpu.h>
#include <driverlib/interrupt.h>
#include <driverlib/timer.h>
#include <driverlib/i2c.h>
#include <driverlib/watchdog.h>
#include "driverlib/uart.h"
#include <inc/hw_ints.h>
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include <string>
#include <stdlib.h>

#define GPIO_PORTA_BASE         0x40004000  // GPIO Port A


const float max_duty_cycle = 0.64;
uint32_t g_ui32SysClock;

void int_Init()
{
	IntPrioritySet(INT_GPIOJ_TM4C129,0x80);
	IntPrioritySet(INT_UART2_TM4C129,0x80);
	IntPrioritySet(INT_CAN0_TM4C129,0x00);
	IntPrioritySet(INT_CAN1_TM4C129,0x00);
}

void led_Init()
{
  SYSCTL->RCGCGPIO |= 0x20; // GPIOF
  // Pins [4:0] digital enable
  GPIOF_AHB->DEN |= 0x11;
  // Pins [4:0] output
  GPIOF_AHB->DIR |= 0x11;
}

void led_Set(bool a, bool b)
{
  uint32_t dval = 0;
  if(a) dval |= 0x01;
  if(b) dval |= 0x10;
  GPIOF_AHB->DATA &= ~(0x11);
  GPIOF_AHB->DATA |= (dval & 0x11);
}

void HardFault_Handler(void)
{
  led_Set(true, true);
  while(1) {}
}


namespace steering
{
  void SetPWM(float duty_cycle)
  {
    const uint32_t pwm_clk_period = 13914;
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, pwm_clk_period);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, pwm_clk_period*(1-duty_cycle)); // Yellow wire (F2)
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, pwm_clk_period*duty_cycle); // Green wire (F3)
  }
  void SetNormalized(float val)
  {
		float duty_cycle;
		duty_cycle = val; //changed, may effect RC
		
		if(duty_cycle > max_duty_cycle)
			duty_cycle = max_duty_cycle;
		else if(duty_cycle < 1 - max_duty_cycle)
			duty_cycle = 1 - max_duty_cycle;
		
    SetPWM(duty_cycle);
  }
  void Init()
  {
    PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMClockSet(PWM0_BASE,PWM_SYSCLK_DIV_4);
		SetPWM(0.5f);
    PWMGenEnable(PWM0_BASE, PWM_GEN_1);
    PWMOutputState(PWM0_BASE, (PWM_OUT_2_BIT | PWM_OUT_3_BIT), true);
    PWMOutputInvert(PWM0_BASE, (PWM_OUT_2_BIT | PWM_OUT_3_BIT), true);
  }
}

namespace accelerator
{
  void SetVoltage(float v)
  {
    uint16_t bitval = 0xFFFF * (v / 3.3f);
    I2CMasterSlaveAddrSet(I2C4_BASE, 0x62, false);
    I2CMasterDataPut(I2C4_BASE, 0x40);
    I2CMasterControl(I2C4_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    while(I2CMasterBusy(I2C4_BASE)) {}
    I2CMasterDataPut(I2C4_BASE, (bitval >> 8));
    I2CMasterControl(I2C4_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
    while(I2CMasterBusy(I2C4_BASE)) {}
    I2CMasterDataPut(I2C4_BASE, bitval & 0xFF);
    I2CMasterControl(I2C4_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    while(I2CMasterBusy(I2C4_BASE)) {}
  }
  
  void SetNormalized(float val)
  {
		// Protection
		if(val < 0)
			val = 0;
		else if (val > 1)
			val = 1;
    SetVoltage(0.39f + val * (1.80f - 0.39f));
  }
	
  void Init()
  {
    // I2C Stuff on K6(SCL) and K7(SDA)
    I2CMasterInitExpClk(I2C4_BASE, SysCtlClockGet(), true);  //hard code clock val?
    GPIOPinTypeI2CSCL(GPIOK_BASE, GPIO_PIN_6);
    GPIOPinTypeI2C(GPIOK_BASE, GPIO_PIN_7);
    GPIOPinConfigure(GPIO_PK6_I2C4SCL);
    GPIOPinConfigure(GPIO_PK7_I2C4SDA);
    
		//Set initial voltage
    SetVoltage(0.39f);
  }
}

namespace brakes
{
  void SetPWM(float duty_cycle1, float duty_cycle2)
  {
		const uint32_t pwm0_clk_period = 56285;
		const uint32_t pwm2_clk_period = 61728;
		PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, pwm0_clk_period);
		PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, pwm2_clk_period);
		if(duty_cycle1 >= 0.89f)
		{
			duty_cycle1 = 0.89f;
		}
		if(duty_cycle1 <= 0.61f)
		{
			duty_cycle1 = 0.61f;
		}
		if(duty_cycle2 >= 0.39f)
		{
			duty_cycle2 = 0.39f;
		}
		if(duty_cycle2 <= 0.11f)
		{
			duty_cycle2 = 0.11f;
		}
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, pwm0_clk_period*duty_cycle1);
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, pwm2_clk_period*duty_cycle2);	
  }
  volatile float brake_value1 = 0.0f; // For debugging
	volatile float brake_value2 = 0.0f;
	
  void SetNormalized(float val)
  {
    brake_value1 = (1.0f-val)*(.89f-.61f)+.61f;
		brake_value2 = (val)*(.39f-.11f)+.11f;
		SetPWM(brake_value1,brake_value2);
  }
  void Init()
  {
		// Brake pinout F1 and G1
		PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
		PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
		PWMClockSet(PWM0_BASE,PWM_SYSCLK_DIV_4);
		brakes::SetPWM(0.89f, 0.11f); // Resting duty % for brakes
		PWMGenEnable(PWM0_BASE, PWM_GEN_0);
		PWMGenEnable(PWM0_BASE, PWM_GEN_2);
		PWMOutputState(PWM0_BASE, (PWM_OUT_1_BIT | PWM_OUT_5_BIT), true);
  }
}

float accel_pulse_width = 0.0f;
float steer_pulse_width = 0.0f;

void UpdateAcceleration(float pulse_width)
{
  if(pulse_width > 0.0008f && pulse_width < 0.0022f)
  {
    if(pulse_width < 0.00145f)
    {
      accelerator::SetNormalized((float) (0.00145f - pulse_width)/(0.00145f - 0.001f));
      brakes::SetNormalized(0.0f);
    }
    else if(pulse_width > 0.00155f)
    {
      accelerator::SetNormalized(0.0f);
      brakes::SetNormalized((float) (pulse_width - 0.00155f)/(0.002f - 0.00155f));
    }
    else
    {
      accelerator::SetNormalized(0.0f);
      brakes::SetNormalized(0.0f);
    }
  }
  else
  {
    accelerator::SetNormalized(0.0f);
    brakes::SetNormalized(0.0f);
  }
}

void UpdateSteering(float pulse_width)
{
  if(pulse_width > 0.0008f && pulse_width < 0.0022f)
  {
    if(pulse_width < 0.00145f || pulse_width > 0.00155f)
    {
      steering::SetNormalized(0.1f*((pulse_width - 0.0015f)/(0.002f - 0.0015f))+0.5f);  //changed, may effect RC
    }
    else
    {
      steering::SetNormalized(0.5f); //changed, may effect RC
    }
  }
  else
  {
    steering::SetNormalized(0.5f); //changed, may effect RC
  }
}

void PulseHandler(void)
{
  if(GPIOIntStatus(GPIOM_BASE, true) & GPIO_INT_PIN_6)
  {
    GPIOIntClear(GPIOM_BASE, GPIO_INT_PIN_6);
    static uint32_t lastval = TIMER5->TAV;
    
    if(GPIOPinRead(GPIOM_BASE, GPIO_INT_PIN_6))
    {
      lastval = TIMER5->TAV;
    }
    else
    {
      accel_pulse_width = (float) (lastval - TIMER5->TAV) / 120000000;
    }
  }
  else if(GPIOIntStatus(GPIOM_BASE, true) & GPIO_INT_PIN_7)
  {
    GPIOIntClear(GPIOM_BASE, GPIO_INT_PIN_7);
    static uint32_t lastval = TIMER5->TAV;
    
    if(GPIOPinRead(GPIOM_BASE, GPIO_INT_PIN_7))
    {
      lastval = TIMER5->TAV;
    }
    else
    {
      steer_pulse_width = (float) (lastval - TIMER5->TAV) / 120000000;
    }
  }
}
void SteerPulseHandler(void)
{
  GPIOIntClear(GPIOM_BASE, GPIO_INT_PIN_7);
}

void modeSelectHandler(void)
{
	GPIOIntClear(GPIOC_AHB_BASE, GPIO_INT_PIN_4);
	
	if(GPIOPinRead(GPIOC_AHB_BASE, GPIO_INT_PIN_4))
	{
		GPIOPinWrite(GPIOC_AHB_BASE,GPIO_INT_PIN_5,0x00);
	}
	else
	{
		GPIOPinWrite(GPIOC_AHB_BASE,GPIO_INT_PIN_5,0xFF);
	}
}

void modeSelectSetup(void)
{
	//Mode select interrupt/switch
	GPIOPinTypeGPIOInput(GPIOC_AHB_BASE, GPIO_PIN_4);
	GPIOPinTypeGPIOOutput(GPIOC_AHB_BASE, GPIO_PIN_5);
	GPIOPadConfigSet(GPIOC_AHB_BASE,GPIO_PIN_4,GPIO_STRENGTH_12MA,GPIO_PIN_TYPE_STD_WPU);
	GPIOPadConfigSet(GPIOC_AHB_BASE,GPIO_PIN_5,GPIO_STRENGTH_12MA,GPIO_PIN_TYPE_STD);
  GPIOIntTypeSet(GPIOC_AHB_BASE, GPIO_PIN_4, GPIO_BOTH_EDGES);
  GPIOIntRegister(GPIOC_AHB_BASE, modeSelectHandler);
	GPIOPinWrite(GPIOC_AHB_BASE,GPIO_INT_PIN_5,0xFF);	
  GPIOIntEnable(GPIOC_AHB_BASE, GPIO_PIN_4);
}

void watchdogSetup(void)
{
	WatchdogStallEnable(WATCHDOG0_BASE); //for debugging only
	WatchdogReloadSet(WATCHDOG0_BASE,	0x1D4C0);

	WatchdogResetDisable(WATCHDOG0_BASE);
	WatchdogIntTypeSet(WATCHDOG0_BASE,WATCHDOG_INT_TYPE_NMI);
	WatchdogIntEnable(WATCHDOG0_BASE);
	WatchdogEnable(WATCHDOG0_BASE);
}

void NMI_Handler(void)
{
	GPIOPinWrite(GPIOC_AHB_BASE,GPIO_INT_PIN_5,0xFF);	
}

void button_int(void){
	short i;

	//Delay
	IntMasterDisable();
	for(i=0;i<4000;i++){
		i++;
		i--;
	}
	IntMasterEnable();
	uint32_t iStatus=GPIOIntStatus(GPIOJ_AHB_BASE,0);	
	//Acceleration step input
	if((iStatus&0x1)==0x1){
		GPIOIntClear(GPIOJ_AHB_BASE,GPIO_INT_PIN_0); //clear interrupt
		
	}
	
	//Braking step input
	if(iStatus==0x2){
		GPIOIntClear(GPIOJ_AHB_BASE,GPIO_INT_PIN_1);	//clear interrupt
		//led_Set(0,1);
	}
	
}

float serial_app = 0.0f; 
float serial_steer = 0.0f;
float serial_brake = 0.0f;
unsigned char input_buffer[16];
int buffer_iter = 0;
bool buffer_ready = false;
bool receiving_packet = false;
	


// Send a string to the UART.
void UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count)
{
    // Loop while there are more characters to send.
    while(ui32Count--)
    {
        // Write the next character to the UART.
        UARTCharPutNonBlocking(UART2_BASE, *pui8Buffer++);
    }
}

// UART interrupt handler.
extern "C"{
	void UART2_Handler(void){
    uint32_t ui32Status;
		int32_t rxChar;
    // Get the interrrupt status.
    ui32Status = UARTIntStatus(UART2_BASE, true);

    // Clear the asserted interrupts.
    UARTIntClear(UART2_BASE, ui32Status);
		
    // Loop while there are characters in the receive FIFO.
    while(UARTCharsAvail(UART2_BASE))
    {
      // Read the next character from the UART and write it back to the UART.
			rxChar = UARTCharGetNonBlocking(UART2_BASE);
      UARTCharPutNonBlocking(UART2_BASE, rxChar);
			
			if(rxChar == 0xFA || receiving_packet == true){
				receiving_packet = true;
				input_buffer[buffer_iter] = (char)rxChar;
				buffer_iter++;
				
				if (buffer_iter >= 16){
					buffer_ready = true;
					receiving_packet = false;
					buffer_iter = 0;
				}
				
			}
    }
	}
}

void packet_parse(){
	memcpy(&serial_app,&input_buffer[2],4);
	memcpy(&serial_brake,&input_buffer[6],4);
	memcpy(&serial_steer,&input_buffer[10],4);
		
}


void peripheral_Init(){
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER5);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C4);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_WDOG0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2);
}

// Steering PWM
void steering_Init(){
	GPIOPinTypePWM(GPIOF_AHB_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
  GPIOPinConfigure(GPIO_PF2_M0PWM2);
  GPIOPinConfigure(GPIO_PF3_M0PWM3);
}

// Braking PWM
void braking_Init(){
	GPIOPinTypePWM(GPIOG_AHB_BASE, GPIO_PIN_1);
	GPIOPinConfigure(GPIO_PF1_M0PWM1);
	GPIOPinConfigure(GPIO_PG1_M0PWM5);
}

// Buttons
void buttons_Init(){
	GPIOPinTypeGPIOInput(GPIOJ_AHB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	GPIOPadConfigSet(GPIOJ_AHB_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
	GPIOIntRegister(GPIOJ_AHB_BASE, button_int);
	GPIOIntTypeSet(GPIOJ_AHB_BASE, GPIO_INT_PIN_0 | GPIO_INT_PIN_1, GPIO_RISING_EDGE);
	GPIOIntEnable(GPIOJ_AHB_BASE,GPIO_INT_PIN_0 | GPIO_INT_PIN_1);
}

// Pulse Timer
void pulseTimer_Init(){
	TimerConfigure(TIMER5_BASE, TIMER_CFG_PERIODIC);
  TimerLoadSet(TIMER5_BASE, TIMER_A, 0x7FFFFFF);
  TimerControlStall(TIMER5_BASE, TIMER_A, true);
  TimerEnable(TIMER5_BASE, TIMER_A);
}

  // GPIOM6 is accel pulse, GPIOM7 is steer pulse
void RCpulse_Init(){
	GPIOPinTypeGPIOInput(GPIOM_BASE, GPIO_PIN_6 | GPIO_PIN_7);
  GPIOIntTypeSet(GPIOM_BASE, GPIO_PIN_6 | GPIO_PIN_7, GPIO_BOTH_EDGES);
  GPIOIntRegister(GPIOM_BASE, PulseHandler);
  GPIOIntEnable(GPIOM_BASE, GPIO_INT_PIN_6 | GPIO_INT_PIN_7);
}

// UART
void uart_Init(){
	// Set GPIO A6 (RX) and A7 (TX) as UART pins.
	 GPIOPinConfigure(GPIO_PA6_U2RX);
   GPIOPinConfigure(GPIO_PA7_U2TX);
   GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_6 | GPIO_PIN_7);
	
	// Configure the UART for 115,200, 8-N-1 operation.
   UARTConfigSetExpClk(UART2_BASE, g_ui32SysClock, 115200,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));
	UARTFIFOEnable(UART2_BASE);
	
	UARTFIFOLevelSet(UART2_BASE,UART_FIFO_TX1_8,UART_FIFO_RX1_8);
	
	
	// Enable the UART interrupt.
   IntEnable(INT_UART2);
   UARTIntEnable(UART2_BASE, UART_INT_RX | UART_INT_RT);
}


unsigned short calculateCRC(unsigned char data[], unsigned int length){
	//Calculate CRC and returns
	unsigned int i;
	unsigned short crc = 0;
	
	for(i=0; i<length; i++){
		crc = (unsigned char)(crc >> 8) | (crc << 8);
		crc ^= data[i];
		crc ^= (unsigned char)(crc & 0xff) >> 4;
		crc ^= crc << 12;
		crc ^= (crc & 0x00ff) << 5;
	}

	return crc;	
}


int main(void)
{
  FPUEnable();
	
	//Set system clock
	g_ui32SysClock = SysCtlClockFreqSet(SYSCTL_XTAL_25MHZ |
																				SYSCTL_OSC_MAIN | 
																				 SYSCTL_USE_PLL |
													SYSCTL_CFG_VCO_480, 120000000);
 
	// Initialize peripherals
	peripheral_Init();
	steering_Init();
	braking_Init();
	buttons_Init();
	pulseTimer_Init();
	RCpulse_Init();
	int_Init();
	led_Init();
	uart_Init();
	can_Init();
	
	modeSelectSetup();
	watchdogSetup(); 
	
  steering::Init();
  accelerator::Init();
	brakes::Init();
  
  IntMasterEnable();

	UARTSend((uint8_t *)"\nEnter text: ", 12);
  while(true)
  {
		if(buffer_ready){
			
			if(calculateCRC(&input_buffer[2],14) == 0){
				packet_parse();
			}

		 buffer_ready = false;
		}
		accelerator::SetNormalized(serial_app);
		brakes::SetNormalized(serial_brake);
		steering::SetNormalized(serial_steer);

		WatchdogReloadSet(WATCHDOG0_BASE,0x1D4C0);  }
}
