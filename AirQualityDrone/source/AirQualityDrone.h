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
* @file
* @brief  This Module is Configuration header for XDK Application Template configurations
*
**/
#include "BCDS_Basics.h"
#include "BCDS_WlanConnect.h"
#include "FreeRTOS.h"
#include "timers.h"
#include "BCDS_MCU_UART.h"
/* header definition ******************************************************** */
#ifndef XDK110_XDKAPPLICATIONTEMPLATE_H_
#define XDK110_XDKAPPLICATIONTEMPLATE_H_

/* local interface declaration ********************************************** */
 /* Priorities */
#define TASK_PRIO_MAIN_CMD_PROCESSOR                (UINT32_C(1))
#define TASK_STACK_SIZE_MAIN_CMD_PROCESSOR          (UINT16_C(700))
#define TASK_Q_LEN_MAIN_CMD_PROCESSOR                (UINT32_C(10))
/* local type and macro definitions */
#define UDPTXDELAY          UINT32_C(1000) 	            /**< Macro used for UDP packet transmit frequency in msec */

#define ZERO    UINT8_C(0)  			            /**< Macro to define value zero*/
#define TIMER_AUTORELOAD_ON             UINT32_C(1)             /**< Auto reload of timer is enabled*/
#define TIMER_AUTORELOAD_OFF            UINT32_C(0)             /**< Auto reload of timer is disabled*/
#define WIFI_TX_FREQ                    UINT32_C(1000)             /**< Macro to represent One second time unit*/

#warning Please provide WLAN related configurations, with valid SSID & WPA key and server ip address where packets are to be sent in the below macros.
/** Network configurations */
#define WLAN_CONNECT_WPA_SSID                "UPC4FB1123"         /**< Macros to define WPA/WPA2 network settings Repeated_UPC4FB1123UPC4FB1123 */
#define WLAN_CONNECT_WPA_PASS                "rhj2ktjNtrmm"      /**< Macros to define WPA/WPA2 network settings rhj2ktjNtrmm*/
#define BUFFER_SIZE        UINT8_C(4)
/** IP addressed of server side socket.Should be in long format, E.g: 0xc0a82177 == 192.168.33.119 */
#define SERVER_IP         UINT32_C(0xC0A80065)  // 192.168.43.57 192.168.0.171
#define SERVER_PORT        UINT16_C(9876)           /**< Port number on which server will listen */
/* local function prototype declarations */



/* functions */
void appInitSystem(void * CmdProcessorHandle, uint32_t param2);
void initUART();
static void initUDP(void);
static void PrintDeviceIP(void);
static void wifiUdpSend(void * param1, uint32_t port);
static void EnqueueDatatoWifi(void *pvParameters);
static void WlanEventCallback(WlanConnect_Status_T Event);
MCU_UART_Callback_T uartCallback(UART_T uart, struct MCU_UART_Event_S event);
static Retcode_T wifiConnect(void);
void startUart();
static void initEnvironmental(void);

//----------------------------- Readings -------------------------------------------

static void uartTask();
static void scanAdcInit(void);
static void gpioTask(xTimerHandle pxTimer2);
void createNewGPIOTask(void);
static void scanAdc(xTimerHandle pxTimer);
void gpioInit();
static void readEnvironmental(xTimerHandle xTimer);
#endif /* XDK110_XDKAPPLICATIONTEMPLATE_H_ */

/** ************************************************************************* */
