
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

#if !defined( WIN32 )
  #include "OnBoard.h"
#endif

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_uart.h"


/* RTOS */
#if defined( IAR_ARMCM3_LM )
#include "RTOS_App.h"
#endif  

#include "protocol.h"


#define SAPICB_DATA_CNF   0xE0
#define SAPICB_BIND_CNF   0xE1
#define SAPICB_START_CNF  0xE2


extern uint8 ZDApp_RestoreNetworkState(void);
extern uint8 ZDApp_ReadNetworkRestoreState(void);
void init_cc2530(void); // for init cc2530 code, be careful with zstack hardware


// This list should be filled with Application specific Cluster IDs.
const cId_t GenericApp_ClusterList[GENERICAPP_MAX_CLUSTERS] =
{
  GENERICAPP_CLUSTERID
};

const SimpleDescriptionFormat_t GenericApp_SimpleDesc =
{
  GENERICAPP_ENDPOINT,              //  int Endpoint;
  GENERICAPP_PROFID,                //  uint16 AppProfId[2];
  GENERICAPP_DEVICEID,              //  uint16 AppDeviceId[2];
  GENERICAPP_DEVICE_VERSION,        //  int   AppDevVer:4;
  GENERICAPP_FLAGS,                 //  int   AppFlags:4;
  GENERICAPP_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)GenericApp_ClusterList,  //  byte *pAppInClusterList;
  GENERICAPP_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)GenericApp_ClusterList   //  byte *pAppInClusterList;
};

// This is the Endpoint/Interface description.  It is defined here, but
// filled-in in GenericApp_Init().  Another way to go would be to fill
// in the structure here and make it a "const" (in code space).  The
// way it's defined in this sample app it is define in RAM.
endPointDesc_t GenericApp_epDesc;


byte GenericApp_TaskID;   // Task ID for internal task/event processing
                          // This variable will be received when
                          // GenericApp_Init() is called.
devStates_t GenericApp_NwkState;


byte GenericApp_TransID;  // This is the unique message ID (counter)

afAddrType_t GenericApp_DstAddr;

static uint16 parentShortAddr;

extern uint8 start_flag;

// Device Info Constants
#define ZB_INFO_DEV_STATE                 0
#define ZB_INFO_IEEE_ADDR                 1
#define ZB_INFO_SHORT_ADDR                2
#define ZB_INFO_PARENT_SHORT_ADDR         3
#define ZB_INFO_PARENT_IEEE_ADDR          4
#define ZB_INFO_CHANNEL                   5
#define ZB_INFO_PAN_ID                    6
#define ZB_INFO_EXT_PAN_ID                7

void cb_GetDeviceInfo ( uint8 , void *);
void cb_SendDataRequest ( uint16 , uint16 , uint8 ,uint8 *, uint8 , uint8 , uint8  );
void CAPI_SendCback( uint8 , uint8 , uint16 );


void cb_GetDeviceInfo ( uint8 param, void *pValue )
{
  switch(param)
  {
    case ZB_INFO_DEV_STATE:
      osal_memcpy(pValue, &devState, sizeof(uint8));
      break;
    case ZB_INFO_IEEE_ADDR:
      osal_memcpy(pValue, &aExtendedAddress, Z_EXTADDR_LEN);
      break;
    case ZB_INFO_SHORT_ADDR:
      osal_memcpy(pValue, &_NIB.nwkDevAddress, sizeof(uint16));
      break;
    case ZB_INFO_PARENT_SHORT_ADDR:
      osal_memcpy(pValue, &_NIB.nwkCoordAddress, sizeof(uint16));
      break;
    case ZB_INFO_PARENT_IEEE_ADDR:
      osal_memcpy(pValue, &_NIB.nwkCoordExtAddress, Z_EXTADDR_LEN);
      break;
    case ZB_INFO_CHANNEL:
      osal_memcpy(pValue, &_NIB.nwkLogicalChannel, sizeof(uint8));
      break;
    case ZB_INFO_PAN_ID:
      osal_memcpy(pValue, &_NIB.nwkPanId, sizeof(uint16));
      break;
    case ZB_INFO_EXT_PAN_ID:
      osal_memcpy(pValue, &_NIB.extendedPANID, Z_EXTADDR_LEN);
      break;
  }
}


void CAPI_SendCback( uint8 event, uint8 status, uint16 data )
{
  sapi_CbackEvent_t *pMsg;

  pMsg = (sapi_CbackEvent_t *)osal_msg_allocate( sizeof(sapi_CbackEvent_t) );
  if( pMsg )
  {
    pMsg->hdr.event = event;
    pMsg->hdr.status = status;
    pMsg->data = data;

    osal_msg_send( GenericApp_TaskID, (uint8 *)pMsg );
  }

}

void cb_SendDataRequest ( uint16 destination, uint16 commandId, uint8 len,
                          uint8 *pData, uint8 handle, uint8 txOptions, uint8 radius )
{
  afStatus_t status;
  afAddrType_t dstAddr;

  txOptions |= AF_DISCV_ROUTE;

  // Set the destination address
  if (destination == ZB_BINDING_ADDR)
  {
    // Binding
    dstAddr.addrMode = afAddrNotPresent;
  }
  else
  {
    // Use short address
    dstAddr.addr.shortAddr = destination;
    dstAddr.addrMode = afAddr16Bit;

    if ( ADDR_NOT_BCAST != NLME_IsAddressBroadcast( destination ) )
    {
      txOptions &= ~AF_ACK_REQUEST;
    }
  }

  dstAddr.panId = 0;                                    // Not an inter-pan message.
  dstAddr.endPoint = GENERICAPP_ENDPOINT;  // Set the endpoint.

  // Send the message
  status = AF_DataRequest(&dstAddr, &GenericApp_epDesc, commandId, len,
                          pData, &handle, txOptions, radius);

  if (status != afStatus_SUCCESS)
  {
    CAPI_SendCback( SAPICB_DATA_CNF, status, handle );
  }
}


void Serial_callBack(uint8 port, uint8 event)
{
  char theMessageData[] = "Hello";
  zAddrType_t dstAddr;
  zAddrType_t ZAddr;
  
  afAddrType_t myaddr; // use for p2p
  
  char pbuf[3];
  char pbuf1[3];
  uint16 cmd;
  
  uint8 buff[128];
  uint8 readBytes = 0;
  
  uint16 short_ddr;
  uint16 panid;
  uint8 *ieeeAddr;
  
  uint16 *p1;
  
  byte cnt = 0;
  uint8 yy1;
  uint8 yy2;
  
  uint8 i;
  byte nr;
  uint16 devlist[ NWK_MAX_DEVICES + 1];
  associated_devices_t *adp; // delete devices
  
  //short_ddr = GenericApp_DstAddr.addr.shortAddr;
  uint8 startOptions;    
  uint8 logicalType;
  
  logicalType = (uint8)ZDO_Config_Node_Descriptor.LogicalType;
  
            
  readBytes = HalUARTRead(SER_PORT, buff, 127);
  if (readBytes > 0)
  {
    //HalUARTWrite( SER_PORT, "DataRead: ",10);
    // HalUARTWrite( SER_PORT, buff, readBytes);
    if(readBytes == 4)
    {
      if(buff[0] == 'p' && buff[1] == 'i' && buff[2]=='n' && buff[3] == 'g')
      {
          UART_Send_String( "pong", 4 );
      }
    }
    
    if( readBytes == 1)
    {
        yy1 = 1;
        if(buff[0]== 's')
        {
          /// short address
          short_ddr = _NIB.nwkDevAddress;
          
          UART_Send_String( (uint8*)&short_ddr, 2);
          
        }
        if(buff[0] == 'p')
        {
          /// pan id
          panid = _NIB.nwkPanId;
          UART_Send_String( (uint8*)&panid, 2);          
        }
        if(buff[0] == 'c')/// channel
        {
            yy2 = _NIB.nwkLogicalChannel;
            UART_Send_String( (uint8*)&yy2, 1);
            
        }
        if(buff[0] =='m') // mac address
        {
            ieeeAddr = NLME_GetExtAddr();
            UART_Send_String( ieeeAddr, 8);
            
        }

        if( buff[0] ==0xc0) // coordinator 
        {

          set_coordi();       
          return;
        }

        if( buff[0] ==0xe0) // router
        {
          set_router();
          
          return;
        }
        
        if(buff[0] == 0xCA)
        {
          // 使命的招唤
          // read self AssociatedDevList
          for(i=0;i< NWK_MAX_DEVICES; i++)
          {
             nr = AssociatedDevList[ i ].nodeRelation;
             if(nr > 0 && nr < 5) //CHILD_RFD CHILD_RFD_RX_IDLE CHILD_FFD CHILD_FFD_RX_IDLE
             //if( nr != 0XFF)
             {
               //   myaddr.addrMode = (afAddrMode_t)Addr16Bit;
               //   myaddr.endPoint = GENERICAPP_ENDPOINT;
             //  if( AssociatedDevList[ i ].shortAddr != 0x0000)
             //  {
                  //if( AssocIsChild( AssociatedDevList[ i ].shortAddr ) != 1 || AssociatedDevList[ i ].age > NWK_ROUTE_AGE_LIMIT)
                  //if( AssocIsChild( AssociatedDevList[ i ].shortAddr ) == 1 && AssociatedDevList[ i ].age > NWK_ROUTE_AGE_LIMIT )
                 // {
                    //  myaddr.addr.shortAddr = AssociatedDevList[ i ].shortAddr;
                      /*
                      if ( AF_DataRequest( &myaddr, &GenericApp_epDesc, GENERICAPP_CLUSTERID,(byte)osal_strlen( theMessageData ) + 1,
                              (byte *)&theMessageData,
                              &GenericApp_TransID,
                              AF_ACK_REQUEST, AF_DEFAULT_RADIUS ) != afStatus_SUCCESS )
                        {
                          uprint("delete asso");
                        */
                   //       delete_asso( AssociatedDevList[ i ]. addrIdx);
                          
                  // }
                    //    else
                     //   {
                          devlist[yy1] = AssociatedDevList[ i ].shortAddr;
                          yy1++;
                     //   }
                  
                  // }
             }//else {break;}
          }
          devlist[0] = BUILD_UINT16(0xce,  AssocCount(1, 4) );
          UART_Send_String( (uint8*)&devlist[0], yy1*2);
            //p1 = AssocMakeList( &cnt );
            //UART_Send_String( (uint8*)p1,  AssocCount(1, 4)*2);
            //osal_mem_free(p1);
          return;
        }
    }
    
#if defined( ZDO_COORDINATOR )
    // only coordinator can have this function
    if(readBytes == 3)
    {
      
      if( buff[0] == 0xCA || buff[0] == 0x6d)
      {
        // CA xx xx ,send CA to a node ,with it's short address
        ///uprint("it's CA ");
        short_ddr = BUILD_UINT16( buff[1], buff[2] );
        myaddr.addrMode = (afAddrMode_t)Addr16Bit;
        myaddr.endPoint = GENERICAPP_ENDPOINT;
        myaddr.addr.shortAddr = short_ddr;
        
        if ( AF_DataRequest( &myaddr, &GenericApp_epDesc, GENERICAPP_CLUSTERID, 1,
                          (byte *)&buff,
                          &GenericApp_TransID,
                          AF_DISCV_ROUTE, AF_DEFAULT_RADIUS ) != afStatus_SUCCESS )
        {
             AF_DataRequest( &myaddr, &GenericApp_epDesc, GENERICAPP_CLUSTERID, 1,
                          (byte *)&buff,
                          &GenericApp_TransID,
                          AF_DISCV_ROUTE, AF_DEFAULT_RADIUS );
             // send twice only, if it is still not ok, fuck it
          
        }
       return;
      }
    }
          
#endif 
          
    if( readBytes >= 2)
    {
     
      if( buff[0] == 'e' && buff[1] == '#')
      {
          uprint("EndDevice match");
          HalLedSet ( HAL_LED_1, HAL_LED_MODE_OFF );

          dstAddr.addrMode = Addr16Bit;
          dstAddr.addr.shortAddr = 0x0000; // Coordinator
          ZDP_EndDeviceBindReq( &dstAddr, NLME_GetShortAddr(), GenericApp_epDesc.endPoint,
                            GENERICAPP_PROFID,
                            GENERICAPP_MAX_CLUSTERS, (cId_t *)GenericApp_ClusterList,
                            GENERICAPP_MAX_CLUSTERS, (cId_t *)GenericApp_ClusterList,
                            FALSE );
      
      }
 
      if( buff[0] == 'r' && buff[1] == '#')
      {
          uprint("Router Device match");
          
        HalLedSet ( HAL_LED_1, HAL_LED_MODE_FLASH );

      dstAddr.addrMode = AddrBroadcast;
      dstAddr.addr.shortAddr = NWK_BROADCAST_SHORTADDR;
      ZDP_MatchDescReq( &dstAddr, NWK_BROADCAST_SHORTADDR,
                        GENERICAPP_PROFID,
                        GENERICAPP_MAX_CLUSTERS, (cId_t *)GenericApp_ClusterList,
                        GENERICAPP_MAX_CLUSTERS, (cId_t *)GenericApp_ClusterList,
                        FALSE );
      
      }
      if(readBytes == 6)
      {
        if(buff[0] == 'p' && buff[1]==':') // pan id
        {
            strncpy(pbuf, &buff[2],2); pbuf[2] = '\0';
            strncpy(pbuf1, &buff[4],2); pbuf1[2] = '\0';
            
            set_panid( BUILD_UINT16( strtol(pbuf1,NULL,16),strtol(pbuf,NULL,16) ));
            if(_NIB.nwkPanId == 0xffff)
            {
              zgWriteStartupOptions (ZG_STARTUP_SET, ZCD_STARTOPT_DEFAULT_NETWORK_STATE);           
              SystemReset();
            }            
            //SystemResetSoft();
        }
        
        if(buff[0] == 's' && buff[1]==':') // short address
        {
            /*
            strncpy(pbuf, &buff[2],2); pbuf[2] = '\0';
            strncpy(pbuf1, &buff[4],2); pbuf1[2] = '\0';
            _NIB.nwkDevAddress = BUILD_UINT16( strtol(pbuf1,NULL,16),strtol(pbuf,NULL,16));
            */
            
        }
      }
      
      
      cmd = BUILD_UINT16(buff[ 1 + 1], buff[1]);
      if( ( buff[ 0 ] == CPT_SOP) && (cmd == SYS_PING_REQUEST) ) 
      {
        sysPingReqRcvd();
        return;
      }

      if( readBytes == 2)
      {        
        if( buff[0] == 0xcc )
        {
          if( buff[1] > 0x0a && buff[1] < 0x1b )
          {

            _NIB.nwkLogicalChannel = buff[1];
            NLME_UpdateNV(0x01);
            ZMacSetReq( ZMacChannel, &buff[1]);
            
            osal_nv_item_init( ZCD_NV_CHANLIST, sizeof(zgDefaultChannelList), &zgDefaultChannelList);
            
            if( buff[1] == 0x0b)
            {
              zgDefaultChannelList = 0x00000800;
              
            } 
            if (buff[1] == 0x0c )
            {
              zgDefaultChannelList = 0x00001000;
            }            
            if (buff[1] == 0x0d )
            {
              zgDefaultChannelList = 0x00002000;
            }
            if (buff[1] == 0x0e )
            {
              zgDefaultChannelList = 0x00004000;
            }
            if (buff[1] == 0x0f )
            {
              zgDefaultChannelList = 0x00008000;
            }
            if (buff[1] == 0x10 )
            {
              zgDefaultChannelList = 0x00010000;
            }
            if (buff[1] == 0x11 )
            {
              zgDefaultChannelList = 0x00020000;
            }
            if (buff[1] == 0x12 )
            {
              zgDefaultChannelList = 0x00040000;
            }            
            if (buff[1] == 0x13 )
            {
              zgDefaultChannelList = 0x00080000;
            }
            if (buff[1] == 0x14 )
            {
              zgDefaultChannelList = 0x00100000;
            }
            if (buff[1] == 0x15 )
            {
              zgDefaultChannelList = 0x00200000;
            }            
            if (buff[1] == 0x16 )
            {
              zgDefaultChannelList = 0x00400000;
            }
            if (buff[1] == 0x17 )
            {
              zgDefaultChannelList = 0x00800000;
            }
            if (buff[1] == 0x18 )
            {
              zgDefaultChannelList = 0x01000000;
            }

            if (buff[1] == 0x19 )
            {
              zgDefaultChannelList = 0x02000000;
            }
            if (buff[1] == 0x1a )
            {
              zgDefaultChannelList = 0x04000000;
            }
            
            osal_nv_write(  ZCD_NV_CHANLIST, 0 ,sizeof(zgDefaultChannelList), &zgDefaultChannelList);
            
            UART_Send_String( (uint8*)&zgDefaultChannelList, sizeof(zgDefaultChannelList) );

            /*
            _NIB.nwkLogicalChannel = buff[1];
            NLME_UpdateNV(0x01);
            if( osal_nv_write(ZCD_NV_CHANLIST, 0, osal_nv_item_len( ZCD_NV_CHANLIST ), &tmp32) !=  ZSUCCESS)
            {
              uprint("change channel to nv failed");
            }
            */
            /*
            _NIB.nwkLogicalChannel = buff[1];
            ZMacSetReq( ZMacChannel, &buff[1]);
            ZDApp_NwkStateUpdateCB();
            _NIB.nwkTotalTransmissions = 0;
            nwkTransmissionFailures( TRUE );
            */
          }
        }
        
      }// readBytes==2


      ser_process( buff,readBytes );// 中讯威易的协议最小也是2 字节
    }//if( readBytes >= 2)


    AF_DataRequest( &GenericApp_DstAddr, &GenericApp_epDesc,
                       GENERICAPP_CLUSTERID,
                       readBytes,
                       (byte *)&buff,
                       &GenericApp_TransID,
                       AF_DISCV_ROUTE, AF_DEFAULT_RADIUS );
    
      
  } //if (readBytes > 0) 
}

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void GenericApp_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg );
static void GenericApp_HandleKeys( byte shift, byte keys );
static void GenericApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );
static void GenericApp_SendTheMessage( void );

#if defined( IAR_ARMCM3_LM )
static void GenericApp_ProcessRtosMessage( void );
#endif


void GenericApp_Init( uint8 task_id )
{
  uint16 size;
  
  GenericApp_TaskID = task_id;
  GenericApp_NwkState = DEV_INIT;
  GenericApp_TransID = 0;
  
  uint8 networkStateNV = ZDO_INITDEV_NEW_NETWORK_STATE;
  devStartModes_t devStartMode = MODE_RESUME; 
  

  //GenericApp_DstAddr.addrMode = (afAddrMode_t)AddrNotPresent; //(afAddrMode_t)Addr16Bit;
  GenericApp_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  GenericApp_DstAddr.endPoint = GENERICAPP_ENDPOINT;
  GenericApp_DstAddr.addr.shortAddr = 0xFFFF;

  // Fill out the endpoint description.
  GenericApp_epDesc.endPoint = GENERICAPP_ENDPOINT;
  GenericApp_epDesc.task_id = &GenericApp_TaskID;
  GenericApp_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&GenericApp_SimpleDesc;
  GenericApp_epDesc.latencyReq = noLatencyReqs;

  // Register the endpoint description with the AF
  afRegister( &GenericApp_epDesc );

  // Register for all key events - This app will handle all key events
  RegisterForKeys( GenericApp_TaskID );

  // Update the display
#if defined ( LCD_SUPPORTED )
  HalLcdWriteString( "GenericApp", HAL_LCD_LINE_1 );
#endif

  
#if !defined( ZDO_COORDINATOR )
  ZDO_RegisterForZDOMsg( GenericApp_TaskID, End_Device_Bind_rsp );
  ZDO_RegisterForZDOMsg( GenericApp_TaskID, Match_Desc_rsp );
  
#endif
  
#if defined ( BUILD_ALL_DEVICES )
  size = osal_nv_item_len( ZCD_NV_LOGICAL_TYPE );
  if( size  <= sizeof(zgDeviceLogicalType) && size != 0 )
  {
    cb_ReadConfiguration( ZCD_NV_LOGICAL_TYPE, sizeof(zgDeviceLogicalType), &zgDeviceLogicalType);
  }
  
#endif

#if defined ( HOLD_AUTO_START )
  if(start_flag == 1)
  {
      ZDOInitDevice(0);
  }
#endif
  serial_init();
  
  init_cc2530();
  
  HalLedSet( HAL_LED_2, HAL_LED_MODE_ON );
  
}

/*********************************************************************
 * @fn      GenericApp_ProcessEvent
 *
 * @brief   Generic Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  none
 */
uint16 GenericApp_ProcessEvent( uint8 task_id, uint16 events )
{
  afIncomingMSGPacket_t *MSGpkt;
  afDataConfirm_t *afDataConfirm;

  // Data Confirmation message fields
  byte sentEP;
  ZStatus_t sentStatus;
  byte sentTransID;       // This should match the value sent
  (void)task_id;  // Intentionally unreferenced parameter

  if ( events & SYS_EVENT_MSG )
  {
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( GenericApp_TaskID );
    while ( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )
      {
        case ZDO_CB_MSG:
          GenericApp_ProcessZDOMsgs( (zdoIncomingMsg_t *)MSGpkt );
          break;

        case KEY_CHANGE:
          GenericApp_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
          break;

        case AF_DATA_CONFIRM_CMD:
          afDataConfirm = (afDataConfirm_t *)MSGpkt;
          sentEP = afDataConfirm->endpoint;
          sentStatus = afDataConfirm->hdr.status;
          sentTransID = afDataConfirm->transID;
          (void)sentEP;
          (void)sentTransID;

          // Action taken when confirmation is received.
          if ( sentStatus != ZSuccess )
          {
            
          }
          break;

        case AF_INCOMING_MSG_CMD:
          GenericApp_MessageMSGCB( MSGpkt );
          
          
          break;

        case ZDO_STATE_CHANGE:
          GenericApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
          if ( (GenericApp_NwkState == DEV_ZB_COORD)
              || (GenericApp_NwkState == DEV_ROUTER)
              || (GenericApp_NwkState == DEV_END_DEVICE) )
          {
            
            /*
            osal_start_timerEx( GenericApp_TaskID,
                                GENERICAPP_SEND_MSG_EVT,
                                GENERICAPP_SEND_MSG_TIMEOUT );
            */
            cb_GetDeviceInfo(ZB_INFO_PARENT_SHORT_ADDR, &parentShortAddr);
          }
          break;

        default:
          break;
      }

      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );

      // Next
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( GenericApp_TaskID );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  // Send a message out - This event is generated by a timer
  //  (setup in GenericApp_Init()).
  
  if ( events & GENERICAPP_SEND_MSG_EVT )
  {
    
    GenericApp_SendTheMessage();

    osal_start_timerEx( GenericApp_TaskID,
                        GENERICAPP_SEND_MSG_EVT,
                        GENERICAPP_SEND_MSG_TIMEOUT );

    
    return (events ^ GENERICAPP_SEND_MSG_EVT);
  }

  
#if defined( IAR_ARMCM3_LM )
  // Receive a message from the RTOS queue
  if ( events & GENERICAPP_RTOS_MSG_EVT )
  {
    // Process message from RTOS queue
    GenericApp_ProcessRtosMessage();

    // return unprocessed events
    return (events ^ GENERICAPP_RTOS_MSG_EVT);
  }
#endif

  // Discard unknown events
  return 0;
}

/*********************************************************************
 * Event Generation Functions
 */

/*********************************************************************
 * @fn      GenericApp_ProcessZDOMsgs()
 *
 * @brief   Process response messages
 *
 * @param   none
 *
 * @return  none
 */
static void GenericApp_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg )
{
  switch ( inMsg->clusterID )
  {
    case End_Device_Bind_rsp:
      if ( ZDO_ParseBindRsp( inMsg ) == ZSuccess )
      {
        // Light LED
        HalLedSet( HAL_LED_2, HAL_LED_MODE_ON );
      }
#if defined( BLINK_LEDS )
      else
      {
        // Flash LED to show failure
        HalLedSet ( HAL_LED_2, HAL_LED_MODE_FLASH );
      }
#endif
      break;

    case Match_Desc_rsp:
      {
        uprint("Match_Decs_rsp");
        ZDO_ActiveEndpointRsp_t *pRsp = ZDO_ParseEPListRsp( inMsg );
        if ( pRsp )
        {
          if ( pRsp->status == ZSuccess && pRsp->cnt )
          {
            GenericApp_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
            //GenericApp_DstAddr.addr.shortAddr = pRsp->nwkAddr;
            GenericApp_DstAddr.addr.shortAddr = 0xFFFF;
            
            // Take the first endpoint, Can be changed to search through endpoints
            GenericApp_DstAddr.endPoint = pRsp->epList[0];

            // Light LED
            //HalLedSet( HAL_LED_2, HAL_LED_MODE_ON );
          }
          osal_mem_free( pRsp );
        }
      }
      break;
  }
}

/*********************************************************************
 * @fn      GenericApp_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_4
 *                 HAL_KEY_SW_3
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
static void GenericApp_HandleKeys( uint8 shift, uint8 keys )
{
  zAddrType_t dstAddr;

  
  /*
  // Shift is used to make each button/switch dual purpose.
  if ( shift )
  {
    if ( keys & HAL_KEY_SW_1 )
    {
    }
    if ( keys & HAL_KEY_SW_2 )
    {
    }
    if ( keys & HAL_KEY_SW_3 )
    {
    }
    if ( keys & HAL_KEY_SW_4 )
    {
    }
  }
  else
  {
    if ( keys & HAL_KEY_SW_1 )
    {

#if defined( SWITCH1_BIND )
      // we can use SW1 to simulate SW2 for devices that only have one switch,
      keys |= HAL_KEY_SW_2;
#elif defined( SWITCH1_MATCH )
      // or use SW1 to simulate SW4 for devices that only have one switch
      keys |= HAL_KEY_SW_4;
#endif

    }

    if ( keys & HAL_KEY_SW_2 )
    {
      HalLedSet ( HAL_LED_4, HAL_LED_MODE_OFF );

      // Initiate an End Device Bind Request for the mandatory endpoint
      dstAddr.addrMode = Addr16Bit;
      dstAddr.addr.shortAddr = 0x0000; // Coordinator
      ZDP_EndDeviceBindReq( &dstAddr, NLME_GetShortAddr(),
                            GenericApp_epDesc.endPoint,
                            GENERICAPP_PROFID,
                            GENERICAPP_MAX_CLUSTERS, (cId_t *)GenericApp_ClusterList,
                            GENERICAPP_MAX_CLUSTERS, (cId_t *)GenericApp_ClusterList,
                            FALSE );
    }

    if ( keys & HAL_KEY_SW_3 )
    {
    }

    if ( keys & HAL_KEY_SW_4 )
    {
      HalLedSet ( HAL_LED_4, HAL_LED_MODE_OFF );
      // Initiate a Match Description Request (Service Discovery)
      dstAddr.addrMode = AddrBroadcast;
      dstAddr.addr.shortAddr = NWK_BROADCAST_SHORTADDR;
      ZDP_MatchDescReq( &dstAddr, NWK_BROADCAST_SHORTADDR,
                        GENERICAPP_PROFID,
                        GENERICAPP_MAX_CLUSTERS, (cId_t *)GenericApp_ClusterList,
                        GENERICAPP_MAX_CLUSTERS, (cId_t *)GenericApp_ClusterList,
                        FALSE );
    }    
  }
  */
  
}

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      GenericApp_MessageMSGCB
 *
 * @brief   Data message processor callback.  This function processes
 *          any incoming data - probably from other devices.  So, based
 *          on cluster ID, perform the intended action.
 *
 * @param   none
 *
 * @return  none
 */
static void GenericApp_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{
  unsigned char buff[12];
  char*p;
  byte nr;
  uint8 i;
  uint8 ass_num;
  uint8* ieeeaddr;
  
  
  uint8 yy1;
  uint16 devlist[ NWK_MAX_DEVICES + 1];
  
  afAddrType_t myaddr; // use for p2p
  
  ass_num = 0;
  
  switch ( pkt->clusterId )
  {
    case GENERICAPP_CLUSTERID:
      //sprintf(buff,"%hu", pkt->cmd.DataLength);
     ///if( p[4] =='l' && p[5] =='e' && p[6] =='d' && p[7] =='1')
     // {
//       uprint( (uint8*)pkt->srcAddr );
//        uprint( (uint8*)pkt->endPoint);
       // UART_Send_String( &pkt->LinkQuality, 1);
        //uprint( pkt->cmd.Data);
     // }
     //UART_Send_String( (uint8*)&pkt->cmd.DataLength, 1);
     //UART_Send_String( &pkt->cmd.TransSeqNumber,1 );

    if( pkt->cmd.DataLength == 1)
    {      
        yy1 = 1;
#if !defined( ZDO_COORDINATOR ) 
        if(pkt->cmd.Data[0] == 0x6d)
        {
          /// return mac address : ac xx xx ieee addr
               myaddr.addrMode = (afAddrMode_t)Addr16Bit;
               myaddr.endPoint = GENERICAPP_ENDPOINT;          
               myaddr.addr.shortAddr = 0x0000;
               
            ieeeaddr = NLME_GetExtAddr();
            osal_cpyExtAddr(buff+3, ieeeaddr);
            buff[0] = 0xac;
            buff[1] = LO_UINT16(_NIB.nwkDevAddress);
            buff[2] = HI_UINT16(_NIB.nwkDevAddress);
            if ( AF_DataRequest( &myaddr, &GenericApp_epDesc, GENERICAPP_CLUSTERID,  11,
                          &buff[0],
                          &GenericApp_TransID,
                          AF_DISCV_ROUTE, AF_DEFAULT_RADIUS ) != afStatus_SUCCESS )
            {
                 AF_DataRequest( &myaddr, &GenericApp_epDesc, GENERICAPP_CLUSTERID,  11,
                          &buff[0],
                          &GenericApp_TransID,
                          AF_DISCV_ROUTE, AF_DEFAULT_RADIUS );
              // send twice only, if it is still not ok, fuck it
            }            
            
        }
        // only routers return devlist
        if( pkt->cmd.Data[0] == 0xCA )
        {
               myaddr.addrMode = (afAddrMode_t)Addr16Bit;
               myaddr.endPoint = GENERICAPP_ENDPOINT;          
               myaddr.addr.shortAddr = 0x0000;
          
        ass_num = AssocCount(1, 4);
      //  if( ass_num > 0)
        //{
          for(i=0;i< NWK_MAX_DEVICES; i++)
          {
             nr = AssociatedDevList[ i ].nodeRelation;
              if(nr > 0 && nr < 5) //CHILD_RFD CHILD_RFD_RX_IDLE CHILD_FFD CHILD_FFD_RX_IDLE
             //if( nr != 0XFF)
             {
             //  if( AssociatedDevList[ i ].shortAddr != 0x0000)
             //  {
                  //if( AssocIsChild( AssociatedDevList[ i ].shortAddr ) != 1 || AssociatedDevList[ i ].age > NWK_ROUTE_AGE_LIMIT)
                  //if( AssocIsChild( AssociatedDevList[ i ].shortAddr ) == 1 && AssociatedDevList[ i ].age > NWK_ROUTE_AGE_LIMIT )
                 // {
                    //  myaddr.addr.shortAddr = AssociatedDevList[ i ].shortAddr;
                      /*
                      if ( AF_DataRequest( &myaddr, &GenericApp_epDesc, GENERICAPP_CLUSTERID,(byte)osal_strlen( theMessageData ) + 1,
                              (byte *)&theMessageData,
                              &GenericApp_TransID,
                              AF_ACK_REQUEST, AF_DEFAULT_RADIUS ) != afStatus_SUCCESS )
                        {
                          uprint("delete asso");
                        */
                   //       delete_asso( AssociatedDevList[ i ]. addrIdx);
                          
                  // }
                    //    else
                     //   {
                          devlist[yy1] = AssociatedDevList[ i ].shortAddr;
                          yy1++;
                     //   }
                  
                  // }
             }
              else { break; }
          }
       // }else
        //{
          //devlist[yy1] = 0;
        //}
          devlist[0] = BUILD_UINT16(0xce,  ass_num );
          if ( AF_DataRequest( &myaddr, &GenericApp_epDesc, GENERICAPP_CLUSTERID, yy1*2,
                          (byte *)&devlist,
                          &GenericApp_TransID,
                          AF_DISCV_ROUTE, AF_DEFAULT_RADIUS ) != afStatus_SUCCESS )
          {
              AF_DataRequest( &myaddr, &GenericApp_epDesc, GENERICAPP_CLUSTERID, yy1*2,
                          (byte *)&devlist,
                          &GenericApp_TransID,
                          AF_DISCV_ROUTE, AF_DEFAULT_RADIUS );
             // send twice only, if it is still not ok, fuck it
          }
        
          
          //cb_SendDataRequest( 0x0000, GENERICAPP_CLUSTERID, yy1*2+1, (uint8*)&devlist[0], 0, AF_DISCV_ROUTE, 0 );
          
          //UART_Send_String( (uint8*)&devlist[0], yy1*2);
          
            //p1 = AssocMakeList( &cnt );
            //UART_Send_String( (uint8*)p1,  AssocCount(1, 4)*2);
            //osal_mem_free(p1);
        }
#endif  
    } //if( pkt->cmd.DataLength == 1)
    
    if( pkt->cmd.DataLength == 3)
    { 
        //ed fa 01 ,turn on led 1
        //ed ff 01  turn off led 1
        if( pkt->cmd.Data[0] == 0xed  && pkt->cmd.Data[1] == 0xfa )
        {
            switch(pkt->cmd.Data[2])
            {
              case 0x01:
                led1 = 1;
                break;
              case 0x05:
                // led5 = 1;
                break;
              default:break;
            }
                   
        }
        if( pkt->cmd.Data[0] == 0xed  && pkt->cmd.Data[1] == 0xff )
        {
            switch(pkt->cmd.Data[2])
            {
              case 0x01:
                led1 = 0;
                break;
              case 0x05:
               //  led5 = 0;
                break;
              default:break;
            }          
        }
        
    } // if( pkt->cmd.DataLength == 3)
    
     // led1 = !led1;
     // led5 = !led5;
      break;
  }
  
#if defined( LCD_SUPPORTED )
      UART_Send_String( &pkt->cmd.Data[0], pkt->cmd.DataLength);
#endif

}


static void GenericApp_SendTheMessage( void )
{
  char theMessageData[] = "Hello World";

  if ( AF_DataRequest( &GenericApp_DstAddr, &GenericApp_epDesc,
                       GENERICAPP_CLUSTERID,
                       (byte)osal_strlen( theMessageData ) + 1,
                       (byte *)&theMessageData,
                       &GenericApp_TransID,
                       AF_DISCV_ROUTE, AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
    // Successfully requested to be sent.
  }
  else
  {
    // Error occurred in request to send.
  }
}

#if defined( IAR_ARMCM3_LM )
/*********************************************************************
 * @fn      GenericApp_ProcessRtosMessage
 *
 * @brief   Receive message from RTOS queue, send response back.
 *
 * @param   none
 *
 * @return  none
 */
static void GenericApp_ProcessRtosMessage( void )
{
  osalQueue_t inMsg;

  if ( osal_queue_receive( OsalQueue, &inMsg, 0 ) == pdPASS )
  {
    uint8 cmndId = inMsg.cmnd;
    uint32 counter = osal_build_uint32( inMsg.cbuf, 4 );

    switch ( cmndId )
    {
      case CMD_INCR:
        counter += 1;  /* Increment the incoming counter */
                       /* Intentionally fall through next case */

      case CMD_ECHO:
      {
        userQueue_t outMsg;

        outMsg.resp = RSP_CODE | cmndId;  /* Response ID */
        osal_buffer_uint32( outMsg.rbuf, counter );    /* Increment counter */
        osal_queue_send( UserQueue1, &outMsg, 0 );  /* Send back to UserTask */
        break;
      }
      
      default:
        break;  /* Ignore unknown command */    
    }
  }
}
#endif

/*********************************************************************
 */
