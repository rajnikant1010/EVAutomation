
#include "can.h"
#include <TM4C1294NCPDT.h>
#include <string.h>
#include <stdbool.h>
#include <inc\hw_gpio.h>
#include <inc\hw_can.h>
#include <inc\hw_ints.h>
#include <inc\hw_nvic.h>
#include <inc\hw_sysctl.h>
#include <inc\hw_types.h>
#include <driverlib\can.h>
#include <driverlib\gpio.h>
#include <driverlib\interrupt.h>
#include "driverlib\pin_map.h"
#include "driverlib\sysctl.h"

//#pragma import(__use_realtime_heap)  			DELETE THIS I THINK


unsigned int throttleAdd = 0;
unsigned const int restingThrottle = 0x5F5A;
unsigned int throttleVal = 0;

static int can0_txid = 24;
static int can1_txid = 24;

static tCANBitClkParms CANBitClk = {
  .ui32SyncPropPhase1Seg = 13,
  .ui32Phase2Seg = 2,
  .ui32SJW = 16,
  .ui32QuantumPrescaler = 15
};


void CAN0_Handler(void)
{
  uint32_t objid = CANIntStatus(CAN0_BASE, CAN_INT_STS_CAUSE);

  // Read the message out of the message object, and
  // clear the interrupt.
  uint8_t data[8]={0};
  tCANMsgObject sMsgObjectRx;
  sMsgObjectRx.pui8MsgData = data;
  sMsgObjectRx.ui32MsgLen = 8;
  CANMessageGet(CAN0_BASE, objid, &sMsgObjectRx, true);
	
	CANMessageSet(CAN1_BASE, can1_txid, &sMsgObjectRx, MSG_OBJ_TYPE_TX);
  
  can1_txid ++;
  if(can1_txid > 32)
    can1_txid = 24;
	
}

void CAN1_Handler(void)
{
  uint32_t objid = CANIntStatus(CAN1_BASE, CAN_INT_STS_CAUSE);
  
  // Read the message out of the message object, and
  // clear the interrupt.
  uint8_t data[8]={0};
  tCANMsgObject sMsgObjectRx;
  // This isn't very well documented in the reference, but this field must be set
  sMsgObjectRx.pui8MsgData = data;
  sMsgObjectRx.ui32MsgLen = 8;
  CANMessageGet(CAN1_BASE, objid, &sMsgObjectRx, true);
	//led_Set(1,0);
	
	CANMessageSet(CAN0_BASE, can0_txid, &sMsgObjectRx, MSG_OBJ_TYPE_TX);
  
  can0_txid ++;
  if(can0_txid > 32)
    can0_txid = 24;
 
}

void SetReceiveAll(uint32_t canbase, size_t fifolen, void (*interrupt)(void))
{

  CANInit(canbase);
  CANBitTimingSet(canbase, &CANBitClk);
  CANEnable(canbase);
  
  tCANMsgObject sMsgObjectRx;
  // Configure a receive object.
  sMsgObjectRx.ui32MsgID = 0x000;
  sMsgObjectRx.ui32MsgIDMask = 0x000;
  sMsgObjectRx.ui32Flags = MSG_OBJ_USE_ID_FILTER | MSG_OBJ_FIFO | MSG_OBJ_RX_INT_ENABLE;
  sMsgObjectRx.ui32MsgLen = 8;
  sMsgObjectRx.pui8MsgData = 0;
  //
  // The first fifolen - 1 message objects have the MSG_OBJ_FIFO set to indicate
  // that they are part of a FIFO.
  //
  for(int i = 1 ; i < fifolen ; i ++) 
    CANMessageSet(canbase, i, &sMsgObjectRx, MSG_OBJ_TYPE_RX);
  //
  // Last message object does not have the MSG_OBJ_FIFO set to indicate that
  // this is the last message.
  //
  sMsgObjectRx.ui32Flags = MSG_OBJ_USE_ID_FILTER | MSG_OBJ_RX_INT_ENABLE;
  CANMessageSet(canbase, fifolen, &sMsgObjectRx, MSG_OBJ_TYPE_RX);
  
  CANIntRegister(canbase, interrupt);
  CANIntEnable(canbase, CAN_INT_MASTER);
}

/*
static void SuperLoopback(void)
{
  //Disable auto retransmit
  CAN0->CTL |= 0x20;
  CAN1->CTL |= 0x20;
  
//  CAN0->CTL |= 0x80;
//  CAN0->TST |= 0x10; // Enable loopback
//  CAN1->CTL |= 0x80;
//  CAN1->TST |= 0x10; // Enable loopback
}
*/


//void button_Init()
//{
//	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
//	
//	GPIOPinTypeGPIOInput(GPIOJ_AHB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
//	GPIOPadConfigSet(GPIOJ_AHB_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
//	GPIOIntRegister(GPIOJ_AHB_BASE, button_int);
//	GPIOIntTypeSet(GPIOJ_AHB_BASE, GPIO_INT_PIN_0 | GPIO_INT_PIN_1, GPIO_FALLING_EDGE);
//	GPIOIntEnable(GPIOJ_AHB_BASE, GPIO_INT_PIN_0 | GPIO_INT_PIN_1);
//	startRecord = false;
//	startPlayback = false;
//}

//void button_int(void)
//{
//	uint8_t data = GPIOPinRead(GPIOJ_AHB_BASE, GPIO_INT_PIN_0 |  GPIO_INT_PIN_1);
//	
//	if((data & 0x2) == 0)
//	{
//		if((startRecord == false) && (startPlayback == false))
//		{
//			startRecord = true;
//			led_Set(1,0);
//		}
//		else if((startRecord == true) && (startPlayback == false))
//		{
//			startRecord = false;
//			led_Set(0,0);
//		}
//	}
//	else if((data & 0x1) == 0)
//	{
//		if((startPlayback == false) && (startRecord == false))
//		{
//			startPlayback = true;
//			led_Set(0,1);
//		}
//		else if((startPlayback == true) && (startRecord == false))
//		{
//			startPlayback = false;
//			led_Set(0,0);
//		}
//	}
//	for(int i = 0; i < 1000000; )
//	{
//		++i;
//	}
//	GPIOIntClear(GPIOJ_AHB_BASE, GPIO_INT_PIN_0 | GPIO_INT_PIN_1);
//}


void can_Init(void)
{
  SYSCTL->RCGCGPIO |= 0x03; // GPIOA and GPIOB
  SYSCTL->RCGCCAN |= 0x03; // CAN1 and CAN0
  
  // Incoming can on GPIOA, CAN0
  // Pin 0,1 digital enable
  GPIOA_AHB->DEN |= 0x03; // B0000.0011
  // Pin 0 input, pin 1 output
  GPIOA_AHB->DIR |= 0x02; // B0000.0010
  // Pin 0,1 low
  GPIOA_AHB->AFSEL |= 0x03; // B0000.0011
  // Clear and set port mux for GPIOA 0,1
  GPIOA_AHB->PCTL &= ~0x000000FF;
  GPIOA_AHB->PCTL |= 0x00000077;
  
  // Incoming can on GPIOB, CAN1
  // Pin 0,1 digital enable
  GPIOB_AHB->DEN |= 0x03; // B0000.0011
  // Pin 0 input, pin 1 output
  GPIOB_AHB->DIR |= 0x02; // B0000.0010
  // Pin 0,1 low
  GPIOB_AHB->AFSEL |= 0x03; // B0000.0011
  // Clear and set port mux for GPIOB 4,5
  GPIOB_AHB->PCTL &= ~0x000000FF;
  GPIOB_AHB->PCTL |= 0x00000077;
  
  //IntEnable(INT_CAN0);
  //IntEnable(INT_CAN1);
  IntMasterEnable();
  SetReceiveAll(CAN0_BASE, 8, CAN0_Handler);
  SetReceiveAll(CAN1_BASE, 8, CAN1_Handler);
  
}
