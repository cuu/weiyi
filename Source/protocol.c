#include "protocol.h"

extern uint8 start_flag;

void myDelay(unsigned char n)  {
  unsigned char i;
  unsigned int j;
  for(i=0;i<n;i++)
    for(j=1;j;j++)
    ;
  
}

uint16 UART_Send_String( unsigned char *Data,int len)
{

  return HalUARTWrite( SER_PORT,  Data, len);
}

uint16 uprint( uint8 * Data)
{
  UART_Send_String( Data, strlen(Data) );
}


void set_panid( uint16 u16NewPanid)
{
  uint8 u8BackCode;
  _NIB.nwkPanId = u16NewPanid;
  macRadioSetPanID ( _NIB.nwkPanId);
  ZMacSetReq( ZMacPanId, (byte *)& _NIB.nwkPanId);
  
  u8BackCode = osal_nv_write(ZCD_NV_PANID, 0, 2, &u16NewPanid);
  if( u8BackCode == ZSUCCESS)
  {
    NLME_UpdateNV(0x01);
//    HAL_SYSTEM_RESET();
  }
  else
  {
    uprint("set_panid failed");
  }
  
}

uint8 cb_ReadConfiguration( uint8 configId, uint8 len, void *pValue ) // it's just check the configId if existed,wont create one,unlikely osal_nv_item_init
{
  uint8 size;

  size = (uint8)osal_nv_item_len( configId );
  if ( size > len )
  {
    return ZFailure;
  }
  else
  {
    return( osal_nv_read(configId, 0, size, pValue) );
  }
}

uint8 cb_WriteConfiguration( uint8 configId, uint8 len, void *pValue )
{
  return( osal_nv_write(configId, 0, len, pValue) );
}

/************************************
 cc 2530 **/

void init_cc2530(void)
{
    //P0SEL = 0x00;
 //   P1SEL = 0x00;
   //P0DIR |= 0x01;
   //P1DIR |= 0x01;
  
//  P0SEL &= ~0x01; // Set P0_0 as GPIO
//  P1DIR |= 0x01; // Set P0_0 as output
  //P0_0 = 1; // Set P0_0 High
//  P0_0 = 0; // P0_0 is Low 
  //P0INP &= ~0X00;

 
  led1 = 0;   
}

void serial_init(void )
{
  halUARTCfg_t uartConfig;

 
  uartConfig.configured           = TRUE;                                                   // 2x30 don't care - see uart driver.
  uartConfig.baudRate             = HAL_UART_BR_38400;                       //38400
  uartConfig.flowControl          = FALSE;
  uartConfig.flowControlThreshold = 64;        // 64.
  uartConfig.rx.maxBufSize        = 1024;                  // 128
  uartConfig.tx.maxBufSize        = 1024;                  // 128
  uartConfig.idleTimeout          = 6;                      // 6
  uartConfig.intEnable            = TRUE;                                                 
  uartConfig.callBackFunc         = Serial_callBack;                   // Call back function
  
  HalUARTOpen (SER_PORT, &uartConfig);

}

static void all_dev_opera(uint8 * buf) // 所有设备开关控制 0x02
{
  uint8 opera_bit;
  uint16 dev_addr;
  uint8 tmp[3];
  dev_addr = 0xffff; 
  opera_bit = buf[1]; // could be 1 2 3 4 5 6 ,回应信息和 0x01 一样，0x81打头
  // 比如全部打开 0x02 0x02 0x01 ,全部关闭是0x02 0x02 0x02
  tmp[0] = 0x02;
  tmp[1] = 0x02;
  tmp[2] = opera_bit;
  
  // wireless broadcast this tmp[3]
  
  
}

static void dev_opera(uint8*buf)
{
  uint8 dev_cnt; //device count
  uint16 dev_addr; // device 地址
  uint8 opera_bit;
  uint8 stop_bit;
  uint8 i,j;
  
  i = 0;
  j = 0;
  //0x01 0x?? ff ff xx ff ff xx ff ff xx
  dev_cnt = buf[1];
  for(i=0;i<dev_cnt;i++)
  {
    stop_bit = 2+i*3;
    
    dev_addr = BUILD_UINT16(buf[stop_bit], buf[ stop_bit + 1]);
    opera_bit = stop_bit+2;
  }
  
}

static void dim_return(uint8*buf) // 0x82 
{
    uint16 short_addr;
    uint8 dims[4];
    
    short_addr = BUILD_UINT16(buf[1],buf[2]);
    dims[0] = buf[3];
    dims[1] = buf[4];
    dims[2] = buf[5];
    dims[3] = buf[6];
    
}

static void opera_return(uint8*buf) // 0x81
{
  uint16 short_addr;
  uint8 return_val;
  
  short_addr = BUILD_UINT16(buf[1], buf[2]);
  return_val = buf[3];
  
}

static void plcbus(uint8*buf) // 0x06
{
  uint8 i;
  uint8 pc_bus[9];
  for(i=0;i<9;i++)
  {
    pc_bus[i] = buf[i+1];
  }
  
}

static void busplc(uint8*buf) // 0x06
{
  uint8 pc_bus[9];
  uint8 i;
  for(i=0;i<9;i++)
  {
    pc_bus[i] = buf[i+1];
  }
}

static void plcbus_return(uint8*buf) // 0x86,eg:0x0a 0x86 + 9bytes
{
  uint8 pc_bus[9];
  uint8 i;
  for(i=0;i<9;i++)
  {
    pc_bus[i] = buf[i+1];
  }
  
}

static void start_warning(uint8*buf) // 0x04 ,example 0x03 0x04 ff ff
{
  uint16 dev_addr;
  dev_addr = BUILD_UINT16(buf[1],buf[2]);
  
}

static void start_warning_return(uint8*buf) // 0x84, eg: 0x4 0x84 ff ff 0x??
{
  uint16 dev_addr;
  uint8 opera_return;
  
  dev_addr = BUILD_UINT16(buf[1],buf[2]);
  opera_return = buf[3];
  
}

static void stop_warning(uint8*buf) //0x03, example 0x03 0x03 ff ff 
{
  uint16 dev_addr;
  dev_addr = BUILD_UINT16(buf[1], buf[2]);
  
}

static void stop_warning_return(uint8*buf) // 0x83 0x??
{
    uint8 opera_return;
    opera_return = buf[1];
    // 3 is ok, 4 is failed
    
}

void send_dev_info(uint8 type)// when get command 0xAD
{
  uint16 short_addr;
  uint8 *ieeeAddr;
  uint8 i;
  uint8 arr[12];
  
  ieeeAddr = NLME_GetExtAddr();
  
  short_addr = _NIB.nwkDevAddress;
  
  arr[0] = 0x0b;
  arr[1] = 0x05;
  for( i = 0; i < 8; i++)
  {
    arr[i+2] = ieeeAddr[i];
  }
  arr[11] = type;
  
  /// now Send to gateway
  
}

void get_dev_info( uint8 *buf )
{
  // 0x05 mac short_addr type
  // 1 8 2 1 == 12 bytes
  // 0 1 2 3 4 5 6 7 8 9 10 11
  uint16 short_addr;
  uint8 type;
  uint8 ieeeAddr[9];
  uint8 i;
  
  type = buf[11];
  short_addr = BUILD_UINT16( buf[9], buf[10]);
  
  for(i=0;i<8;i++)
  {
    ieeeAddr[i] = buf[i+1];
  }

}

static void ir_cmd()
{
  
}

static void reboot_zigbee(uint8*buf)
{
  uint8 opera_bit;
  uint8 tmp[4];
  
  opera_bit = buf[1];
  
  switch(opera_bit)
  {
  case 0x01://表示清空 NV_RESTORE 保存的内容，然后系统重启
    
    SystemReset();
    break;
  case 0x02://表示清空 NV_RESTORE 保存的内容，将网关设置为“协调器”，然后系统重启
      
      set_coordi();
      SystemReset();
    break;
  case 0x03://表示清空 NV_RESTORE 保存的内容，将网关设置为“路由”，然后系统重启
      set_router();
      SystemReset();
    break;
  case 0x04://表示写模块panid；后面跟panid参数；
            //04     A8    04      yy          xx 
            // 长度  命令  写指令 panid高字节 panid低字节
      
    break;
  case 0x05:
        //03    95       yy    xx
        tmp[0] = 0x03;
        tmp[1] = 0x95;
        tmp[2] = HI_UINT16(_NIB.nwkPanId);
        tmp[3] = LO_UINT16(_NIB.nwkPanId);
        
        HalUARTWrite(SER_PORT,tmp, 4);
    break;
  default:break;
  
  }
}

void gateway_to_pda(uint16 shortaddr, uint8*buf, uint8 len)// 这儿的buf也 是一完整的协议data
{
   uint8*mbuf;
   uint8 i;
   mbuf = osal_mem_alloc( (len+4)*sizeof(uint8));
   
   mbuf[0] = len+3;
   mbuf[1] = 0x08;
   
   mbuf[2] = HI_UINT16(shortaddr);
   mbuf[3] = LO_UINT16(shortaddr);
   
   for( i = 0; i < len; i++)
   {
     mbuf[i+4] = buf[i];
   }
   
   // uart to send
   if( HalUARTWrite(0,mbuf, len+4) > 0)
   {
     osal_mem_free(mbuf);
   }else
   {
     HalUARTWrite(SER_PORT,mbuf, len+4);
     osal_mem_free(mbuf);
   }
   
   return;
}

static void pda_to_gateway(uint8*buf)  // 0x08, 这儿要全部的数据，开头的len+3可以段定后面数据的长度
{
  uint8 data_len;
  uint8*data;
  data = &buf[4];
  //分析这个data 
  data_len = buf[0] - 3;
  
  ser_process(data, data_len);
  
}

static void start_dev(void)
{
  
 #if defined ( HOLD_AUTO_START )
    ZDOInitDevice(0);
    osal_nv_item_init( 0x0201, sizeof(start_flag), &start_flag);
    start_flag = 1;
    osal_nv_write(0x0201,0,  sizeof(start_flag), &start_flag);
    
 #endif     

}

static void hd_alarm(uint8*buf)
{
  //流程是 上位机应用程序串口发过来  0x04 0xA9 XX XX 0x01/0x02，
  //zigbee收到，转发 给 XX XX 数据 0x02 0xA9 0x01/0x02
  //报警器XX XX收到后判断，0x01：开启报警，0x02：关闭报警
  uint16 short_addr;
  uint8 tmp[3];
  short_addr = BUILD_UINT16(buf[1],buf[2]);
  tmp[0] = 0x02;
  tmp[1] = 0xa9;
  tmp[2] = buf[3];
  
  // wireless send data to XX XX 
}

static void check_alive(void)
{
  uint8 tmp[2];
  tmp[0] = 0x01;
  tmp[1] = 0x90;
  
  /// serial port send back ,tmp[2];
  // or wireless
  HalUARTWrite(SER_PORT,tmp, 2);
  
}

static void profile_panel_number_set(uint8*buf)
{
  uint16 short_addr;
  short_addr = BUILD_UINT16(buf[2],buf[3]);
  
  //wireless send buf to short_addr 
}

void ser_process(uint8*buf,uint8 read_len) // read_len is the length of buf, keep in range,max 255
{
  
        if(buf[0] == 0x01 && buf[1] == 0xa7) // 中讯威易
        {
            start_dev();
        }
        
        if(buf[1] == 0x01) // 对单个或多个设备（灯光、电器、红外、烟感、门磁等）进行开关或状态查询。
        {
          dev_opera( &buf[1]);
        }
        
        if(buf[1] == 0x81) // 上面指令的返回
        {
            opera_return( &buf[1]);          
        }
        
        if(buf[1] == 0x82) // 调光设备的返回  07 82 XX XX   dim1    dim2   dim3   dim4
        {
          dim_return( &buf[1]);
        }
        
        if(buf[1] == 0x06) // pcbus
        {
          //plcbus(&buf[1]);
        }
        
        if(buf[1] == 0xa8) // 重启 zigbee 模块  0x02 0xa8 0x??
        {
          reboot_zigbee(&buf[1]);
        }
        if(buf[1] == 0x08)
        {
          pda_to_gateway(buf);
        }
        
        if(buf[1] == 0xa9) // 高分贝报警器报警
        {
          hd_alarm(&buf[1]);
        }
        
        if(buf[1] == 0x89) // check it node is alive
        {
          check_alive();
        }
        
        if(buf[1] == 0xb0) //场景面板的编号设置
        {
          profile_panel_number_set(buf);
        }
}


void wir_process(uint8*buf ,uint8 read_len)
{
  
}

uint8 calcFCS(uint8 *pBuf, uint8 len)
{
  uint8 rtrn = 0;

  while (len--)
  {
    rtrn ^= *pBuf++;
  }

  return rtrn;
}

void sysPingRsp(void)
{
  uint8 pBuf[7];
  
  // Start of Frame Delimiter
  pBuf[0] = CPT_SOP;
  
  // Length
  pBuf[1] = 2; 
  
  // Command type
  pBuf[2] = LO_UINT16(SYS_PING_RESPONSE); 
  pBuf[3] = HI_UINT16(SYS_PING_RESPONSE);
  
  // Stack profile
  pBuf[4] = LO_UINT16(STACK_PROFILE);
  pBuf[4+ 1] = HI_UINT16(STACK_PROFILE);
  
  // Frame Check Sequence
  pBuf[7 - 1] = calcFCS(&pBuf[1], (7 - 2));
  
  // Write frame to UART
  UART_Send_String (pBuf, 7);
}

void sysPingReqRcvd(void)
{
   sysPingRsp();
}

void delete_asso( uint8 index)
{
    uint8 i;
    AddrMgrEntry_t addrEntry;
    NLME_LeaveReq_t req;
  
        addrEntry.user = ADDRMGR_USER_DEFAULT;
        addrEntry.index = index;
        
        if (AddrMgrEntryGet( &addrEntry ))
        {
          for (i = 0; i < Z_EXTADDR_LEN; i++ )
          {
             if ( addrEntry.extAddr != 0 ){ break;}
          }
        }
        if(i < Z_EXTADDR_LEN){
          // Remove device
          req.extAddr = addrEntry.extAddr;
          req.removeChildren = TRUE;
          req.rejoin = TRUE;
          req.silent = FALSE;

          NLME_LeaveReq( &req );
        }else{
          AssocRemove(addrEntry.extAddr);
          ZDApp_NVUpdate();
        } 
}


void set_coordi(void)
{
    uint8 logicalType;

        osal_nv_item_init( ZCD_NV_LOGICAL_TYPE, sizeof(logicalType), &logicalType );
        logicalType = ZG_DEVICETYPE_COORDINATOR;
        if( osal_nv_write( ZCD_NV_LOGICAL_TYPE, 0 ,sizeof(logicalType), &logicalType) != ZSUCCESS)
        {
          uprint("set device to coordi failed");
        }else
        {
          zgWriteStartupOptions (ZG_STARTUP_SET, ZCD_STARTOPT_DEFAULT_NETWORK_STATE);
          uprint("set device to coordi,restart it");
        }         
        return; 
}

void set_router(void)
{
    uint8 logicalType;
          osal_nv_item_init( ZCD_NV_LOGICAL_TYPE, sizeof(logicalType), &logicalType );
          logicalType = ZG_DEVICETYPE_ROUTER;
          if( osal_nv_write( ZCD_NV_LOGICAL_TYPE, 0 ,sizeof(logicalType), &logicalType) != ZSUCCESS)
          {
            uprint("set device to router failed");
          }else
          {
            zgWriteStartupOptions (ZG_STARTUP_SET, ZCD_STARTOPT_DEFAULT_NETWORK_STATE);
            uprint("set device to router,restart it");
          }
          
//          zgWriteStartupOptions (ZG_STARTUP_SET, 0x02);
//          zgInit();
//          ZDOInitDevice( 0 );
//          SystemReset();   
          return;  
}




