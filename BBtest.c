
/***************************** Include Files ********************************/
#include "stdio.h"
#include "stdlib.h"
#include "xparameters.h"
#include "xiic.h"
#include "xintc.h"
#include "xil_exception.h"
#include "xil_printf.h"
#include "xil_types.h"
#include "xstatus.h"
#include "sleep.h"
#include "xuartlite.h"
#include "xgpio.h"
#include "gpio_header.h"


/************************** Constant Definitions *****************************/
#define MCU_UART_DEVICE_ID 		XPAR_AXI_UARTLITE_0_DEVICE_ID
#define MCU_UART_BASEADDR	   XPAR_UARTLITE_0_BASEADDR
#define DBG_UART_DEVICE_ID 		XPAR_AXI_UARTLITE_1_DEVICE_ID
#define DBG_UART_BASEADDR	   XPAR_UARTLITE_1_BASEADDR
#define IIC_DEVICE_ID			XPAR_IIC_0_DEVICE_ID
#define INTC_DEVICE_ID			XPAR_INTC_0_DEVICE_ID
#define INTC_IIC_INTERRUPT_ID	XPAR_INTC_0_IIC_0_VEC_ID
#define TEMP_SENSOR_ADDRESS		0x49
#define reg_addr 				0x00

/* UART Additions */
#define BUFF_SIZE 				16

volatile u8 TransmitComplete;
volatile u8 ReceiveComplete;
volatile u8 IicDevStatus;

s32 InitGPIO(void);
s32 InitIIC(void);
s32 InitUART(void);
s32 FetchTemp(u8 value[]);
s32 SendString(char String[]);
float TempDecode(u8 value[]);
static int SetupInterruptSystem(void);
static void SendHandler(XIic *InstancePtr);
static void ReceiveHandler(XIic *InstancePtr);
static void StatusHandler(XIic *InstancePtr, int Event);

/**************************** Variable Definitions ***************************/

XIic Iic;		  					/*     The instance of the IIC device       */
XIntc Intc;  						/* The instance of the Interrupt controller */

XUartLite UartLiteDBG;				/* 		Debug // STDOUT STDIN Instance		*/
XUartLite UartLiteMCU;				/* 			  MCU UART instance 			*/

XGpio GPIO;

char SendBuffer[BUFF_SIZE];			/*       Buffer for Transmitting Data       */
char DataBuffer[BUFF_SIZE];			/*       Buffer for Receiving Data          */
char Clear[BUFF_SIZE];
char STUCK[BUFF_SIZE] = "STUCK";

u8 val[2] = {0}; 					/*        Int array for Temp Readings       */
float temperature; 		  			/*           float value for temp           */
int hund_thousandths;				/*  Used as xil_printf doesn't take floats  */

/********************************* MAIN LOOP *******************************/

int main(void)
{
	/*	  Initialise GPIO functionality     */
	InitGPIO();
	xil_printf("\n\n\r GPIO Initialisation done");
	/* 	  Initialise IIC and INTC function	*/
	InitIIC();
	xil_printf("\n\r IIC Initialisation done");
	/*		Initialise UART function		*/
	InitUART();
	xil_printf("\n\r UART Initialisation done\n");

	//clearMCU = "0";
	//XUartLite_Send(&UartLiteMCU,(u8 *)clearMCU, sizeof(clearMCU));

	/* 	   Read temp and send to MCU loop   */
	//XUartLite_ResetFifos(&UartLiteMCU);
	//XUartLite_Recv( &UartLiteMCU, (u8 *)Clear, sizeof(Clear));



	/* ******************************************** TEST PT 1 **************************************************/
	unsigned int TimeOn_Secs = 0;
//
int SentBytes0;
int RecvBytes0;
//
//	memset(SendBuffer,0,BUFF_SIZE);
//	strcpy(SendBuffer,"CLM");
//	while((SentBytes0 = XUartLite_Send(&UartLiteMCU, (u8 *)SendBuffer, sizeof(SendBuffer))) != 16) {xil_printf("\n\r send not work");}
//	xil_printf("\n\r Sent String: %s sent in %d bytes", SendBuffer, SentBytes0);
//
//	sleep(1);
//
//	memset(DataBuffer,0,BUFF_SIZE);
//	while((RecvBytes0 = XUartLite_Recv( &UartLiteMCU, (u8 *)DataBuffer, sizeof(DataBuffer))) != 16) {usleep(100000);xil_printf("\n\r recv not work");}
//	xil_printf("\n\r Recv String: %s in %d bytes", DataBuffer, RecvBytes0);
//	sleep(3);

	for(int i = 0; i < 21; i++) {

		memset(SendBuffer,0,BUFF_SIZE);
		strcpy(SendBuffer,"SOT");
		itoa(TimeOn_Secs,&SendBuffer[3],10);
		while((SentBytes0 = XUartLite_Send(&UartLiteMCU, (u8 *)SendBuffer, sizeof(SendBuffer))) != 16) {xil_printf("\n\r send not work");}
		xil_printf("\n\r Sent String: %s sent in %d bytes", SendBuffer, SentBytes0);

		sleep(1);
		TimeOn_Secs++;


//		XGpio_DiscreteWrite(&GPIO, 0x1, 0x0);
//		XGpio_DiscreteWrite(&GPIO, 0x1, 0x1);
//		usleep(200000);

		memset(DataBuffer,0,BUFF_SIZE);
		while((RecvBytes0 = XUartLite_Recv( &UartLiteMCU, (u8 *)DataBuffer, sizeof(DataBuffer))) != 16) {usleep(100000); xil_printf("\n\r recv not work");}
		xil_printf("\n\r Recv String: %s in %d bytes", DataBuffer, RecvBytes0);

	}


	memset(SendBuffer,0,BUFF_SIZE);
	strcpy(SendBuffer,"ROT");
	SendString(SendBuffer);

	sleep(3);

	memset(DataBuffer,0,BUFF_SIZE);
	int RecvBytes1;
	while((RecvBytes1 = XUartLite_Recv( &UartLiteMCU, (u8 *)DataBuffer, sizeof(DataBuffer))) != 16) {usleep(100000); xil_printf("\n\r recv not work");}
	char codestring1[8];
	unsigned long long int dataint1;
	memcpy(&codestring1,&DataBuffer,8);
	memcpy(&dataint1,&DataBuffer[8],8);
	xil_printf("\n\r Recv String: %s = %u", codestring1, dataint1);
	xil_printf(" received in %d bytes", RecvBytes1);

	sleep(3);

	memset(SendBuffer,0,BUFF_SIZE);
	strcpy(SendBuffer,"RTD");
	SendString(SendBuffer);

	sleep(3);

	memset(DataBuffer,0,BUFF_SIZE);
	int RecvBytes2;
	while((RecvBytes2 = XUartLite_Recv( &UartLiteMCU, (u8 *)DataBuffer, sizeof(DataBuffer))) != 16) {usleep(100000);xil_printf("\n\r recv not work");}
	char codestring2[8];
	unsigned long long int dataint2;
	memcpy(&codestring2,&DataBuffer,8);
	memcpy(&dataint2,&DataBuffer[8],8);
	xil_printf("\n\r Recv String: %s = %u",codestring2, dataint2);
	xil_printf(" received in %d bytes", RecvBytes2);


	/* ******************************************** TEST PT 2 **************************************************/
//	sleep(1);
//	int SentBytes0;
//	memset(SendBuffer,0,BUFF_SIZE);
//	strcpy(SendBuffer,"ROT");
//	while((SentBytes0 = XUartLite_Send(&UartLiteMCU, (u8 *)SendBuffer, sizeof(SendBuffer))) != 16) {xil_printf("\n\r send not work");}
//	xil_printf("\n\r Sent String: %s sent in %d bytes", SendBuffer, SentBytes0);
//
//	sleep(1);
//
//	int RecvBytes0;
//	memset(DataBuffer,0,BUFF_SIZE);
//	while((RecvBytes0 = XUartLite_Recv( &UartLiteMCU, (u8 *)DataBuffer, sizeof(DataBuffer))) != 16) {usleep(100000);xil_printf("\n\r recv not work");}
//	char codestring1[8];
//	unsigned long long int dataint1;
//	memcpy(&codestring1,&DataBuffer,8);
//	memcpy(&dataint1,&DataBuffer[8],8);
//	xil_printf("\n\r Recv String: %s = %u", codestring1, dataint1);
//	xil_printf(" received in %d bytes", RecvBytes0);
//
//	sleep(3);

//
//	XIic_Stop(&Iic);
}
/***************** Fetch Temperature function Implementation ***************/

s32 FetchTemp(u8 value[])
{
	u8 *valptr = value;
	int FetchStatus;
	/* Receives data from TargetAddress (and default reg add = 0x00, therefore temp reg */
	if ((FetchStatus = XIic_MasterRecv(&Iic,valptr,sizeof(val))) != XST_SUCCESS) {
		xil_printf("\n\r failed at RECV TEMP");
		return XST_FAILURE;
	}

	/* Move on to temp decode only when IIC bus is done transmitting */
	while((XIic_IsIicBusy(&Iic)) == TRUE) {
		xil_printf("\n\r stuck in busy loop");
	}

	return XST_SUCCESS;
}

/**************** IIC Initialisation function Implementation ***************/

s32 InitIIC(void)
{
	int Status = 0;
	XIic_Config *ConfigPtr;

	if ((ConfigPtr = XIic_LookupConfig(IIC_DEVICE_ID)) == NULL) {
		return XST_FAILURE;
	}

    if ((Status = XIic_CfgInitialize(&Iic, ConfigPtr, ConfigPtr->BaseAddress)) != XST_SUCCESS) {
        return XST_FAILURE;
    }

	XIic_SetSendHandler(&Iic, &Iic, (XIic_Handler) SendHandler);
	XIic_SetRecvHandler(&Iic, &Iic, (XIic_Handler) ReceiveHandler);
	XIic_SetStatusHandler(&Iic, &Iic, (XIic_StatusHandler) StatusHandler);

	if (SetupInterruptSystem() != XST_SUCCESS) {
		return XST_FAILURE;
	}

	XIic_Start(&Iic);
	XIic_SetAddress(&Iic,XII_ADDR_TO_SEND_TYPE, TEMP_SENSOR_ADDRESS);

	return XST_SUCCESS;
}

/*************** UART Initialisation function Implementation ***************/

s32 InitUART(void)
{
	int statusinit1;
	int statusinit0;

	if ((statusinit1 = XUartLite_Initialize(&UartLiteDBG , DBG_UART_DEVICE_ID)) != XST_SUCCESS) {
		xil_printf("\n\r failed at initialise DBG UART");
		return XST_FAILURE;
	}

	if ((statusinit0 = XUartLite_Initialize(&UartLiteMCU , MCU_UART_DEVICE_ID)) != XST_SUCCESS) {
		xil_printf("\n\r failed at initialise MCU UART");
		return XST_FAILURE;
	}

	return XST_SUCCESS;
}

/*************** GPIO Initialisation function Implementation ***************/

s32 InitGPIO(void)

{
   int Status;
   Status = XGpio_Initialize(&GPIO, XPAR_AXI_GPIO_0_DEVICE_ID);
   if (Status != XST_SUCCESS)  {
	  return XST_FAILURE;
   }
   XGpio_SetDataDirection(&GPIO, 0x1, 0x0);

   XGpio_DiscreteWrite(&GPIO, 0x1, 0x1);
   xil_printf("\n\n\rGPIO PIN HELD AT 1");
   return XST_SUCCESS;
}

/******************** Send String function Implementation ******************/

s32 SendString(char String[])
{
	int SentBytes;
	while((SentBytes = XUartLite_Send(&UartLiteMCU, (u8 *)String, BUFF_SIZE)) != 16) {xil_printf("\n\r send not work");}
	xil_printf("\n\r Sent String: %s sent in %d bytes", String, SentBytes);
	return XST_SUCCESS;
}

/************** SetupInterruptSystem function Implementation ***************/

static int SetupInterruptSystem(void)
{
	int Status;
	if(Intc.IsStarted == XIL_COMPONENT_IS_STARTED) {
		return XST_FAILURE;
	}

	if(XIntc_Initialize(&Intc,INTC_DEVICE_ID) != XST_SUCCESS) {
		return XST_FAILURE;
	}

	if ((Status = XIntc_Connect(&Intc , INTC_IIC_INTERRUPT_ID , (XInterruptHandler) XIic_InterruptHandler , &Iic)) != XST_SUCCESS) {
		return XST_FAILURE;
	}

	if(XIntc_Start(&Intc , XIN_REAL_MODE) != XST_SUCCESS) {
		return XST_FAILURE;
	}

	XIntc_Enable(&Intc, INTC_IIC_INTERRUPT_ID);

	//Exceptions from https://github.com/Xilinx/embeddedsw/blob/master/XilinxProcessorIPLib/drivers/iic/examples/xiic_tempsensor_example.c
	Xil_ExceptionInit();

	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT, (Xil_ExceptionHandler) XIntc_InterruptHandler, &Intc);

	Xil_ExceptionEnable();

	return XST_SUCCESS;
}

/**************** Temperature Decode function Implementation ***************/

float TempDecode(u8 value[])

{
	float temperature;
	short int code = (((short int)value[0])<<4) + (((short int)value[1])>>4);

	if (code >= 0) {
		temperature = (float)code/16.0;
	} else {
		temperature = ((float)code-4096)/16.0;
	}

	return temperature;
}

/************************* Handler Implementations *************************/

static void SendHandler(XIic *InstancePtr)
{
	TransmitComplete = 0;
	//TransmitComplete[1] = 0;
	//TransmitComplete[2] = 0;
	//xil_printf("Send handler[0] called\r\n");
}

static void ReceiveHandler(XIic *InstancePtr)
{
	ReceiveComplete = 0;
	//xil_printf("recv handler[0] called\r\n");
}

static void StatusHandler(XIic *InstancePtr, int Event)
{
	IicDevStatus = Event;
}
