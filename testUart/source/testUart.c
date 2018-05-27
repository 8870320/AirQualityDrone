/*
* Licensee agrees that the example code provided to Licensee has been developed and released by Bosch solely as an example to be used as a potential reference for Licensee�s application development. 
* Fitness and suitability of the example code for any use within Licensee�s applications need to be verified by Licensee on its own authority by taking appropriate state of the art actions and measures (e.g. by means of quality assurance measures).
* Licensee shall be responsible for conducting the development of its applications as well as integration of parts of the example code into such applications, taking into account the state of the art of technology and any statutory regulations and provisions applicable for such applications. Compliance with the functional system requirements and testing there of (including validation of information/data security aspects and functional safety) and release shall be solely incumbent upon Licensee. 
* For the avoidance of doubt, Licensee shall be responsible and fully liable for the applications and any distribution of such applications into the market.
* 
* 
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions are 
* met:
* 
*     (1) Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer. 
* 
*     (2) Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in
*     the documentation and/or other materials provided with the
*     distribution.  
*     
*     (3)The name of the author may not be used to
*     endorse or promote products derived from this software without
*     specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR 
*  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
*  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
*  DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
*  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
*  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
*  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
*  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
*  IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
*  POSSIBILITY OF SUCH DAMAGE.
*/
/*----------------------------------------------------------------------------*/
/**
* @ingroup APPS_LIST
*
* @defgroup XDK_APPLICATION_TEMPLATE XDK Application Template
* @{
*
* @brief XDK Application Template
*
* @details Empty XDK Application Template without any functionality. Should be used as a template to start new projects.
*
* @file
**/
/* module includes ********************************************************** */

/* system header files */
#include <stdio.h>

#include "testUart.h"
#include "XdkSensorHandle.h"
/* additional interface header files */
#include "BCDS_HAL.h"
#include "FreeRTOS.h"
#include "timers.h"
//#include "BSP_UART.h"
#include "BCDS_MCU_UART.h" /*Header file containing HAL interfaces*/
#include "BSP_ExtensionPort.h"
#include "BCDS_CmdProcessor.h"
#include "BCDS_Assert.h"
#include "BSP_BoardShared.h"

#include "simplelink.h"
#include "BCDS_Basics.h"
#include "BCDS_Assert.h"
#include "BCDS_CmdProcessor.h"
#include "BCDS_WlanConnect.h"
#include "BCDS_NetworkConfig.h"
#include "Serval_Types.h"
#include "Serval_Ip.h"
#include "FreeRTOS.h"
#include "timers.h"

#include "em_cmu.h"
#include "em_adc.h"

#include "BCDS_MCU_GPIO.h"
#include "BCDS_MCU_GPIO_Handle.h"
#include "BSP_BoardShared.h"

#define TIMER_AUTORELOAD_ON pdTRUE

#define ONESECONDDELAY      UINT32_C(1000)       /* one second is represented by this macro */
#define TIMERBLOCKTIME      UINT32_C(0xffff)     /* Macro used to define blocktime of a timer */
#define TIMEBASE            UINT32_C(0)          /* Macro used to define the Timebase for the ADC */
#define ADCFREQUENCY        UINT32_C(9000000)    /* Macro Used to define the frequency of the ADC Clock */
#define NUMBER_OF_CHANNELS  UINT32_C(2)


CmdProcessor_T *AppCmdProcessorHandle;

UART_T uartHandle;
xTimerHandle wifiSendTimerHandle = NULL;
xTimerHandle scanAdcTimerHandle=NULL;
xTimerHandle gpioTimerHandle=NULL;

_i16 socketHandle=NULL;
Ip_Address_T destAddr = SERVER_IP;
SlSockAddrIn_t addr;
int sensorNo=0;

struct messagePayload
{
  float temperature;
  float pressure;
  float humidity;
  float pm25; 				//in ug/m3
  float pm10; 				//in ug/m3
  float co;				//in ppm
  float co2; 				//in ppm
  float o3sensitive; 		//in ppm
  float o3lessSensitive;	//in ppm
  float hazardousGas;		//in ppm
} payload;


/* local variables ********************************************************** */


void initUART(){
	int status;
	printf("Startup...\r\n");
	vTaskDelay (8000);
	printf("Startup... done\r\n");
	status= BSP_ExtensionPort_ConnectUart();
	int x =RETCODE_BSP_EXTENSIONPORT_GPIO_IN_USE;
	if(status!=RETCODE_OK){
		printf("Error at BSP_ExtensionPort_ConnectUart: %d\r\n",status);
	}
	else{
		printf("Uart connected successfully\r\n");
	}

	Retcode_T retVal = RETCODE_OK;
	retVal = BSP_ExtensionPort_SetUartConfig(BSP_EXTENSIONPORT_UART_BAUDRATE,9600, NULL);
	if(RETCODE_OK == retVal )
	{
	retVal = BSP_ExtensionPort_SetUartConfig(BSP_EXTENSIONPORT_UART_PARITY, BSP_EXTENSIONPORT_UART_NO_PARITY, NULL);
	}
	if(RETCODE_OK == retVal )
	{
	retVal = BSP_ExtensionPort_SetUartConfig(BSP_EXTENSIONPORT_UART_STOPBITS,BSP_EXTENSIONPORT_UART_STOPBITS_ONE,NULL);
	}
	if(RETCODE_OK == retVal )
	{
	// Take further action
	}

	uartHandle = BSP_ExtensionPort_GetUartHandle();
	if(uartHandle==NULL){
		printf("UART Handle initialized wrong\r\n");
	}
	else{
		printf("UART Handle initialized correctly\r\n");
	}


	status = MCU_UART_Initialize(uartHandle,uartCallback);
	if(status!=RETCODE_OK){
		printf("Error at MCU_UART_Initialize: %d\r\n",status);
	}
	else{
		printf("MCU_UART_Initialize initialized correctly\r\n");
	}

	status= BSP_ExtensionPort_EnableUart();
	if(status!=RETCODE_OK){
		printf("Error at BSP_ExtensionPort_EnableUart: %d\r\n",status);
	}
	else{
		printf("UART enabled correctly\r\n");
	}
}

static void initUDP(void)
{
	printf("UDP init started  correctly\r\n");
    Retcode_T retval = RETCODE_OK;
    static_assert((portTICK_RATE_MS != 0), "Tick rate MS is zero");



    /* create timer task to get and transmit accel data via BLE for every one second automatically*/

    wifiSendTimerHandle = xTimerCreate((char * const ) "wifiUdpSend", 1000, TIMER_AUTORELOAD_ON, NULL, EnqueueDatatoWifi);
    if (NULL != wifiSendTimerHandle)
    {
        retval = wifiConnect();

        if(retval==RETCODE_OK){
        	printf("wifi connect successful\r\n");
        }
        else{
        	printf("wifi connect not successful\r\n");
        }
    }
//    wifiTcpSend(null, 6666);
    if ((RETCODE_OK != retval|| (NULL == wifiSendTimerHandle))) //
    {
        printf("Failed to initialize because of Wifi/Command Processor/Timer Create fail \r\n");
        assert(false);
    }
    printf("UDP init done  correctly\r\n");
}

static void scanAdcInit(void)
{
    /* Initialize Configuration ADC Structures */
    ADC_Init_TypeDef     adc0_init_conf     = ADC_INIT_DEFAULT;
    ADC_InitScan_TypeDef adc0_scaninit_conf = ADC_INITSCAN_DEFAULT;

    /* Enable Clocks for the ADC */
    CMU_ClockEnable(cmuClock_HFPER, true);
    CMU_ClockEnable(cmuClock_ADC0, true);

    /* Configure the ADC Initialization Structure */
    adc0_init_conf.timebase = ADC_TimebaseCalc(TIMEBASE);
    adc0_init_conf.prescale = ADC_PrescaleCalc(ADCFREQUENCY, TIMEBASE);
    ADC_Init(ADC0, &adc0_init_conf);

    /* Configure the ADC Scan Structure */
    adc0_scaninit_conf.reference = adcRef2V5;
    adc0_scaninit_conf.input     = ADC_SCANCTRL_INPUTMASK_CH5 |
                                   ADC_SCANCTRL_INPUTMASK_CH6;

    ADC_InitScan(ADC0, &adc0_scaninit_conf);

    /* Setup and Start the timer to scan the ADC Channels */
    scanAdcTimerHandle = xTimerCreate(
        (const char *) "ADC read", ONESECONDDELAY/10,
        TIMER_AUTORELOAD_ON, NULL, scanAdc);

    xTimerStart(scanAdcTimerHandle, TIMERBLOCKTIME);
}

void gpioInit(){
	gpioTimerHandle = xTimerCreate(
	        (const char *) "ADC read", ONESECONDDELAY/10,
	        TIMER_AUTORELOAD_ON, NULL, gpioTask);

	    xTimerStart(gpioTimerHandle, TIMERBLOCKTIME);
}

void createNewUARTTask(void)
{
  xTaskHandle taskHandle = NULL;
  xTaskCreate(
    uartTask,                 // function that implements the task
    (const char * const) "My Task", // a name for the task
    configMINIMAL_STACK_SIZE,       // depth of the task stack
    NULL,                           // parameters passed to the function
    tskIDLE_PRIORITY,               // task priority
    taskHandle                      // pointer to a task handle for late reference
  );
}

//---------------------------------------------- Readings ---------------------------------------------

static void gpioTask(xTimerHandle pxTimer2)
{
	MCU_GPIO_Handle_T GPIOA;
	MCU_GPIO_Handle_T GPIOB;
	MCU_GPIO_Handle_T GPIOC;
    /* initialize local variables */
    GPIOA.Port = gpioPortA;
    GPIOA.Pin = 1;
    GPIOA.Mode = gpioModePushPull;
    GPIOA.InitialState = MCU_GPIO_PIN_STATE_LOW;

    GPIOB.Port = gpioPortE;
	GPIOB.Pin = 2;
	GPIOB.Mode = gpioModePushPull;
	GPIOB.InitialState = MCU_GPIO_PIN_STATE_LOW;

	GPIOC.Port = gpioPortC;
	GPIOC.Pin = 8;
	GPIOC.Mode = gpioModePushPull;
	GPIOC.InitialState = MCU_GPIO_PIN_STATE_LOW;

    /* Initialization activities for PTD driver */
    MCU_GPIO_Initialize(&GPIOA);
    MCU_GPIO_Initialize(&GPIOB);
    MCU_GPIO_Initialize(&GPIOC);
    /* blinking functionality */
    	int retcode;

	switch(sensorNo){
		case 0: retcode = MCU_GPIO_WritePin(&GPIOA,MCU_GPIO_PIN_STATE_LOW);
				retcode = MCU_GPIO_WritePin(&GPIOB,MCU_GPIO_PIN_STATE_LOW);
				retcode = MCU_GPIO_WritePin(&GPIOC,MCU_GPIO_PIN_STATE_LOW);
				printf("0 set.\n\r");
				break;
		case 1: retcode = MCU_GPIO_WritePin(&GPIOA,MCU_GPIO_PIN_STATE_HIGH);
				retcode = MCU_GPIO_WritePin(&GPIOB,MCU_GPIO_PIN_STATE_LOW);
				retcode = MCU_GPIO_WritePin(&GPIOC,MCU_GPIO_PIN_STATE_LOW);
				printf("1 est.\n\r");
				break;
		case 2: retcode = MCU_GPIO_WritePin(&GPIOA,MCU_GPIO_PIN_STATE_LOW);
				retcode = MCU_GPIO_WritePin(&GPIOB,MCU_GPIO_PIN_STATE_HIGH);
				retcode = MCU_GPIO_WritePin(&GPIOC,MCU_GPIO_PIN_STATE_LOW);
				printf("2 set.\n\r");
				break;
		case 3: retcode = MCU_GPIO_WritePin(&GPIOA,MCU_GPIO_PIN_STATE_HIGH);
				retcode = MCU_GPIO_WritePin(&GPIOB,MCU_GPIO_PIN_STATE_HIGH);
				retcode = MCU_GPIO_WritePin(&GPIOC,MCU_GPIO_PIN_STATE_LOW);
				printf("3 set.\n\r");
				break;
		case 4: retcode = MCU_GPIO_WritePin(&GPIOA,MCU_GPIO_PIN_STATE_LOW);
				retcode = MCU_GPIO_WritePin(&GPIOB,MCU_GPIO_PIN_STATE_LOW);
				retcode = MCU_GPIO_WritePin(&GPIOC,MCU_GPIO_PIN_STATE_HIGH);
				printf("4 set.\n\r");
				break;
		default:
				break;
    }
	if(retcode!= RETCODE_OK){
		printf("Write low failed!\n\r\n\r");
	}
	sensorNo+=1;
	if(sensorNo>=10){
		sensorNo=0;
	}
}

static void scanAdc(xTimerHandle pxTimer)
{
    /* Initialize the Variables */
    (void) (pxTimer);
    uint32_t _adc0ChData = 0;
    uint8_t _channelsScanned = 0;

    /* Start the ADC Scan */
    ADC_Start(ADC0, adcStartScan);

    for (_channelsScanned = 0; _channelsScanned < NUMBER_OF_CHANNELS-1; _channelsScanned++) {
        /* Wait for Valid Data */
        while (!(ADC0->STATUS & ADC_STATUS_SCANDV));

        /* Read the Scanned data */
        _adc0ChData = 0xFFF & ADC_DataScanGet(ADC0);
        switch(sensorNo){
        		case 0: payload.co=(float)_adc0ChData  /4095 * 2000;
        				printf("%f 0.\n\r", payload.co);
        				break;
        		case 1: payload.o3sensitive=(float)_adc0ChData  /4095 * 2;
        			    printf("%f 1.\n\r", payload.o3sensitive);
        				break;
        		case 2:	payload.o3lessSensitive=(float)(1.0-((float)_adc0ChData  /4095 ))* 1000;
			    		printf("%f 2.\n\r", payload.o3lessSensitive);
        				break;
        		case 3:	payload.co2=(float)_adc0ChData  /4095 * 10000;
	    				printf("%f 3.\n\r", payload.co2);
        				break;
        		case 4:	payload.hazardousGas=(float)_adc0ChData  /4095 ;
						printf("%f 4.\n\r", payload.hazardousGas);
        				break;
        		default:
        				break;
            }
//        printf("ADC %d %ld = %ld\r\n",sensorNo, ((ADC0->STATUS & _ADC_STATUS_SCANDATASRC_MASK) >> _ADC_STATUS_SCANDATASRC_SHIFT), _adc0ChData);
    }
}

static void wifiUdpSend(void * param1, uint32_t port)
{
    BCDS_UNUSED(param1);
    static uint16_t counter = UINT16_C(0);
    SlSockAddrIn_t Addr;
    uint16_t AddrSize = (uint16_t) ZERO;
    int16_t SockID = (int16_t) 6000;
    int16_t Status = (int16_t) ZERO;

    /* copies the dummy data to send array , the first array element is the running counter to track the number of packets send so far*/

    Addr.sin_family = SL_AF_INET;
    Addr.sin_port = sl_Htons((uint16_t) SERVER_PORT);
    Addr.sin_addr.s_addr = sl_Htonl(SERVER_IP);
    AddrSize = sizeof(SlSockAddrIn_t);

    SockID = sl_Socket(SL_AF_INET, SL_SOCK_DGRAM, (uint32_t) ZERO); /**<The return value is a positive number if successful; other wise negative*/
    if (SockID < (int16_t) ZERO)
    {
        printf("Error in socket opening\n");
        /* error case*/
        return;
    }
    char outBuf[1024];


	sprintf(outBuf, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",payload.temperature,payload.pressure,payload.humidity,
			payload.pm25,payload.pm10,payload.co,payload.co2,payload.o3sensitive,payload.o3lessSensitive,payload.hazardousGas);

	Status = sl_SendTo(SockID, (const void *) outBuf, strlen(outBuf), (uint32_t) ZERO, (SlSockAddr_t *) &Addr, AddrSize);/**<The return value is a number of characters sent;negative if not successful*/

//	Status = sl_SendTo(SockID, bsdBuffer_mau, BUFFER_SIZE * sizeof(uint16_t), (uint32_t) ZERO, (SlSockAddr_t *) &Addr, AddrSize);/**<The return value is a number of characters sent;negative if not successful*/
	/*Check if 0 transmitted bytes sent or error condition*/
    if (Status <= (int16_t) ZERO)
    {
        Status = sl_Close(SockID);
        if (Status < 0)
        {
            printf("Error is closing socket after failing to send the Udp data \r\n");
            return;
        }
        printf("Error in sending data \r\n");
        return;
    }
    Status = sl_Close(SockID);
    if (Status < 0)
    {
        printf("Error in closing socket after sending successfully sending the udp data \r\n");
        return;
    }
    counter++;
    printf("UDP sending successful\r\n");
    return;
}

static void uartTask(){
	int status;
	uint8_t buffer [10] ;
	buffer[0]=0;
	buffer[1]=1;
	buffer[2]=2;
	uint32_t bufLen = sizeof(buffer);
	for (;;)
	{
		for(int i=0;i<10;i++){
			buffer[i]=null;
		}
		//printf("Startup task...\r\n");
		//printf("Startup task... done\r\n");
		vTaskDelay (900);
		status = MCU_UART_Receive(uartHandle,  buffer, bufLen);
		if(status!=RETCODE_OK){
			printf("Error at MCU_UART_Receive: %d\r\n",status);
			printf("Buffer:%s\r\n",buffer);
		}
		else{
			printf("MCU_UART_Receive worked correctly\r\n");
			for(int i=0;i<10;i++){
				if(buffer[i]!=null){
					printf("Bit %d aus Buffer: %d\r\n",i,buffer[i]);
				}
			}
			payload.pm25=((float)buffer[3]*256+buffer[2])/10;
			payload.pm10=((float)buffer[5]*256+buffer[4])/10;
		}
	}
}






MCU_UART_Callback_T uartCallback(UART_T uart, struct MCU_UART_Event_S event){
	//printf("UartCallback\r\n");
	return null;
}

static void WlanEventCallback(WlanConnect_Status_T Event)
{
    switch (Event)
    {
    case WLAN_CONNECTED:
        /* start accelerometer data transmission timer */
        if (pdTRUE != xTimerStart(wifiSendTimerHandle, (WIFI_TX_FREQ/portTICK_RATE_MS)))
        {
            /* Assertion Reason : Failed to start software timer. Check command queue size of software timer service*/
            assert(false);
        }
        printf("XDK Device Connected over WIFI \r\n");
        PrintDeviceIP();
        break;
    case WLAN_DISCONNECTED:
        /* stop Ble timer accelerometer data transmission timer */
        if (pdTRUE != xTimerStop(wifiSendTimerHandle, UINT8_C(0)))
        {
            /* Assertion Reason: Failed to start software timer. Check command queue size of software timer service. */
            assert(false);
        }
        printf("XDK Device disconnected form WIFI n/w \r\n");
        break;
    case WLAN_CONNECTION_ERROR:
        printf("XDK Device WIFI Connection error \r\n");
        break;
    case WLAN_CONNECTION_PWD_ERROR:
        printf("XDK Device WIFI connection error due to wrong password \r\n");
        break;
    case WLAN_DISCONNECT_ERROR:
        printf("XDK Device WIFI Disconnect error \r\n");
        break;
    default:
        printf("XDK Device unknown WIFI event \r\n");
        break;
    }
}

static void PrintDeviceIP(void)
{
    NetworkConfig_IpSettings_T myIpSettings;
    Ip_Address_T* IpaddressHex = Ip_getMyIpAddr();
    int32_t Result = INT32_C(-1);
    char ipAddress[SERVAL_IP_ADDR_LEN] = { 0 };

    memset(&myIpSettings, (uint32_t) ZERO, sizeof(myIpSettings));

    Retcode_T ReturnValue = NetworkConfig_GetIpSettings(&myIpSettings);
    if (ReturnValue == RETCODE_OK)
    {
        *IpaddressHex = Basics_htonl(myIpSettings.ipV4);
        Result = Ip_convertAddrToString(IpaddressHex, ipAddress);
        if (Result < INT32_C(0))
        {
            printf("Couldn't convert the IP address to string format \r\n ");
        }
        printf(" Ip address of the device %s \r\n ", ipAddress);
    }
    else
    {
        printf("Error in getting IP settings\n\r");
    }
}


static void EnqueueDatatoWifi(void *pvParameters)
{
    BCDS_UNUSED(pvParameters);

    Retcode_T retVal = CmdProcessor_Enqueue(AppCmdProcessorHandle, wifiUdpSend, NULL, SERVER_PORT);
    if (RETCODE_OK != retVal)
    {
    	if(RETCODE_FAILURE ==retVal)
    		printf("Queue is full \r\n");
        printf("Failed to Enqueue SendAccelDataoverWifi to Application Command Processor \r\n");
    }
}


Retcode_T wifiConnect(void)
{
    WlanConnect_SSID_T connectSSID;
    WlanConnect_PassPhrase_T connectPassPhrase;
    Retcode_T ReturnValue = (Retcode_T) RETCODE_FAILURE;

    ReturnValue = WlanConnect_Init();

    if (RETCODE_OK != ReturnValue)
    {
        printf("Error occurred initializing WLAN \r\n ");
        return ReturnValue;
    }
    printf("Connecting to %s \r\n ", WLAN_CONNECT_WPA_SSID);

    connectSSID = (WlanConnect_SSID_T) WLAN_CONNECT_WPA_SSID;
    connectPassPhrase = (WlanConnect_PassPhrase_T) WLAN_CONNECT_WPA_PASS;
    ReturnValue = NetworkConfig_SetIpDhcp(NULL);
    if (RETCODE_OK != ReturnValue)
    {
        printf("Error in setting IP to DHCP\n\r");
        return ReturnValue;
    }
    ReturnValue = WlanConnect_WPA(connectSSID, connectPassPhrase, WlanEventCallback);
    if (RETCODE_OK != ReturnValue)
    {
        printf("Error occurred while connecting Wlan %s \r\n ", WLAN_CONNECT_WPA_SSID);
    }
    return ReturnValue;
}








void appInitSystem(void * CmdProcessorHandle, uint32_t param2)
{
    if (CmdProcessorHandle == NULL)
    {
        printf("Command processor handle is null \r\n");
        assert(false);
    }
    AppCmdProcessorHandle = (CmdProcessor_T *) CmdProcessorHandle;
    Board_EnablePowerSupply3V3(EXTENSION_BOARD);
    BCDS_UNUSED(param2);
    Retcode_T returnVal = RETCODE_OK;

    struct messagePayload payload = {-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0};

    initUART();

       //-------- Init Tasks -------------

    gpioInit();
    scanAdcInit();
    //UARTInit);
    initUDP();
    createNewUARTTask();
//    createNewUARTTask();


}




/**@} */
/** ************************************************************************* */
