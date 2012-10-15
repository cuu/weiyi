// protocol.h
/*

*/
#include <ioCC2530.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ZComDef.h"
#include "OSAL.h"
#include "sapi.h"
#include "OSAL_Nv.h"

#include "AF.h"
#include "AssocList.h"
#include "AddrMgr.h"

#include "ZDApp.h"
#include "ZDObject.h"
#include "ZDProfile.h"

#include "NLMEDE.h"
#include "OnBoard.h"
#include "nwk_util.h"

#include "GenericApp.h"
#include "DebugTrace.h"

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_uart.h"

#include "nwk_util.h"

#include "mac_low_level.h"
 

#define SER_PORT 0

#define TEST_BUTTON P0_1

// Stack Profile
#define ZIGBEE_2007                         0x0040
#define ZIGBEE_PRO_2007                     0x0041

#ifdef ZIGBEEPRO
#define STACK_PROFILE                       ZIGBEE_PRO_2007             
#else 
#define STACK_PROFILE                       ZIGBEE_2007
#endif

#define CPT_SOP                             0xFE
#define SYS_PING_REQUEST                    0x0021
#define SYS_PING_RESPONSE                   0x0161

#define led1    P1_0         
#define led2    P1_1         
#define led5    P1_2

/*
0x0201 start_flag

*/

void myDelay(unsigned char );
uint16 UART_Send_String( unsigned char *,int );
uint16 uprint( uint8*);

void init_cc2530(void);
void serial_init(void );

void set_panid( uint16 );
uint8 cb_ReadConfiguration( uint8 , uint8 , void * );
uint8 cb_WriteConfiguration( uint8 , uint8 , void *);

void ser_process(uint8*, uint8); // for serial port
void wir_process(uint8*, uint8); // for zigbee wireless data

uint8 calcFCS(uint8 *, uint8 );
void sysPingRsp(void);
void sysPingReqRcvd(void);

void delete_asso( uint8);

void set_coordi(void);
void set_router(void);

void send_dev_info(uint8); // 0x05 +mac+short address+device type
void get_dev_info( uint8 * );

void gateway_to_pda(uint16 , uint8*, uint8);
