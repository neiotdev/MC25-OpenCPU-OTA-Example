/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of Quectel Co., Ltd. 2019
*
*****************************************************************************/
/*****************************************************************************
 *
 * Filename:
 * ---------
 *   example_location2.c
 *
 * Project:
 * --------
 *   OpenCPU
 *
 * Description:
 * ------------
 *   This example demonstrates how to program module location with APIs in OpenCPU.
 *   
 * Usage:
 * ------
 *   1. Redefine the APN according to your network in the codes below.
 *
 *   2. Compile & Run:
 *
 *     Set "C_PREDEF=-D __EXAMPLE_QLBS__" in gcc_makefile file. And compile the 
 *     app using "make clean/new".
 *     Download image bin to module to run.
 * 
 * Author:
 * -------
 * -------
 *
 *============================================================================
 *             HISTORY
 *----------------------------------------------------------------------------
 * 
 ****************************************************************************/
#ifdef __EXAMPLE_QLBS__

#include "ql_type.h"
#include "ql_trace.h"
#include "ql_uart.h"
#include "ril.h"
#include "ril_network.h"
#include "ril_location2.h"
#include "ql_gprs.h"
#include "ql_stdlib.h"
#include "ql_error.h"
#ifdef __OCPU_RIL_QLBS_SUPPORT__
/****************************************************************************
* Definition for APN
****************************************************************************/
#define APN      "CMNET\0"
#define USERID   ""
#define PASSWD   ""


#define DEBUG_ENABLE 1
#if DEBUG_ENABLE > 0
#define DEBUG_PORT  UART_PORT1
#define DBG_BUF_LEN   512
static char DBG_BUFFER[DBG_BUF_LEN];
#define APP_DEBUG(FORMAT,...) {\
    Ql_memset(DBG_BUFFER, 0, DBG_BUF_LEN);\
    Ql_sprintf(DBG_BUFFER,FORMAT,##__VA_ARGS__); \
    if (UART_PORT2 == (DEBUG_PORT)) \
    {\
        Ql_Debug_Trace(DBG_BUFFER);\
    } else {\
        Ql_UART_Write((Enum_SerialPort)(DEBUG_PORT), (u8*)(DBG_BUFFER), Ql_strlen((const char *)(DBG_BUFFER)));\
    }\
}
#else
#define APP_DEBUG(FORMAT,...) 
#endif

static void CallBack_UART_Hdlr(Enum_SerialPort port, Enum_UARTEventType msg, bool level, void* customizedPara)
{
     
}


ST_Qlbs_Cfg_Para qlbs_cfg_value = {0};
static void Location_Program(void);
static void Callback_Location(s32 result,ST_Lbs_LocInfo* loc_info);

/************************************************************************/
/*                                                                      */
/* The entrance to application, which is called by bottom system.       */
/*                                                                      */
/************************************************************************/
void proc_main_task(s32 taskId)
{
    s32 ret;
    ST_MSG msg;

    // Register & open UART port
    ret = Ql_UART_Register(UART_PORT1, CallBack_UART_Hdlr, NULL);
    if (ret < QL_RET_OK)
    {
        Ql_Debug_Trace("Fail to register serial port[%d], ret=%d\r\n", UART_PORT1, ret);
    }
    ret = Ql_UART_Open(UART_PORT1, 115200, FC_NONE);
    if (ret < QL_RET_OK)
    {
        Ql_Debug_Trace("Fail to open serial port[%d], ret=%d\r\n", UART_PORT1, ret);
    }
    

    APP_DEBUG("<-- OpenCPU: Demo for QuecLocator 2.0 -->\r\n");  
    while (TRUE)
    {
         Ql_OS_GetMessage(&msg);
        switch(msg.message)
        {
            case MSG_ID_RIL_READY:
                APP_DEBUG("<-- RIL is ready -->\r\n");
                Ql_RIL_Initialize();
                break;

                // Handle URC messages.
                // URC messages include "module init state", "CFUN state", "SIM card state(change)",
                // "GSM network state(change)", "GPRS network state(change)" and other user customized URC.
            case MSG_ID_URC_INDICATION:
                switch (msg.param1)
                {
                    // URC for module initialization state
                case URC_SYS_INIT_STATE_IND:
                    //APP_DEBUG("<-- Sys Init Status %d -->\r\n", msg.param2);
                    if (SYS_STATE_SMSOK == msg.param2)
                    {
                        // SMS option has been initialized, and application can program SMS
                        //APP_DEBUG("<-- Application can program SMS -->\r\n");
                    }
                    break;

                    // URC for SIM card state(change)
                case URC_SIM_CARD_STATE_IND:
                    if (SIM_STAT_READY == msg.param2)
                    {
                        APP_DEBUG("<-- SIM card is ready -->\r\n");
                    }else{
                        APP_DEBUG("<-- SIM card is not available, cause:%d -->\r\n", msg.param2);
                        /* cause: 0 = SIM card not inserted
                        *        2 = Need to input PIN code
                        *        3 = Need to input PUK code
                        *        9 = SIM card is not recognized
                        */
                    }
                    break;

                    // URC for GSM network state(change).
                    // Application receives this URC message when GSM network state changes, such as register to 
                    // GSM network during booting, GSM drops down.
                case URC_GSM_NW_STATE_IND:
                    if (NW_STAT_REGISTERED == msg.param2 || NW_STAT_REGISTERED_ROAMING == msg.param2)
                    {
                        APP_DEBUG("<-- Module has registered to GSM network -->\r\n");
                    }else{
                        APP_DEBUG("<-- GSM network status:%d -->\r\n", msg.param2);
                        /* status: 0 = Not registered, module not currently search a new operator
                        *         2 = Not registered, but module is currently searching a new operator
                        *         3 = Registration denied 
                        */
                    }
                    break;

                    // URC for GPRS network state(change).
                    // Application receives this URC message when GPRS network state changes, such as register to 
                    // GPRS network during booting, GSM drops down.
                case URC_GPRS_NW_STATE_IND:
                    if (NW_STAT_REGISTERED == msg.param2 || NW_STAT_REGISTERED_ROAMING == msg.param2)
                    {
                        APP_DEBUG("<-- Module has registered to GPRS network -->\r\n");

                        // Module has registered to GPRS network, and app may start to program with QuecLocator
                        Location_Program();
                    }else{
                        APP_DEBUG("<-- GPRS network status:%d -->\r\n", msg.param2);
                        /* status: 0 = Not registered, module not currently search a new operator
                        *         2 = Not registered, but module is currently searching a new operator
                        *         3 = Registration denied 
                        */
                    }
                    break;
                default:
                    //APP_DEBUG("<-- Other URC: type=%d\r\n", msg.param1);
                    break;
                }
        default:
            break;
        }
    }
}
ST_Lbs_LocInfo locinfo;
static void Location_Program(void)
{
	s32 ret;
	u8  pdpCntxtId;
    s32 i;
	u8 location_mode = 0;

    Ql_memset(qlbs_cfg_value.token_value,0,MAX_TOKEN_LENGTH);
    Ql_memcpy(qlbs_cfg_value.token_value,"yourtoken",Ql_strlen("yourtoken"));

    qlbs_cfg_value.asynch_mode = SYNC_MODE;
    qlbs_cfg_value.time_mode = OUTPUT_TIME;
    qlbs_cfg_value.update_mode = UPDATE_TIME2RTC;
	// Set PDP context
	ret = Ql_GPRS_GetPDPContextId();
	APP_DEBUG("<-- The PDP context id available is: %d (can be 0 or 1)-->\r\n", ret);
	if (ret >= 0)
	{
	    pdpCntxtId = (u8)ret;
	} else {
    	APP_DEBUG("<-- Fail to get PDP context id, try to use PDP id(0) -->\r\n");
	    pdpCntxtId = 0;
	}

	ret = RIL_NW_SetGPRSContext(pdpCntxtId);
	APP_DEBUG("<-- Set PDP context id to %d -->\r\n", pdpCntxtId);
	if (ret != RIL_AT_SUCCESS)
	{
	    APP_DEBUG("<-- Ql_RIL_SendATCmd error  ret=%d-->\r\n",ret );
	}

	// Set APN
	ret = RIL_NW_SetAPN(1, APN, USERID, PASSWD);
	APP_DEBUG("<-- Set APN -->\r\n");

	//PDP activated
    ret = RIL_NW_OpenPDPContext();
    if (ret == RIL_AT_SUCCESS)
	{
	    APP_DEBUG("<-- RIL_NW_OpenPDPContext  ret=%d-->\r\n",ret );
	}

    ret = RIL_QLBS_Cfg(TOKEN,&qlbs_cfg_value);
    if(ret == RIL_AT_SUCCESS)
	{
		APP_DEBUG("<-- RIL_QLBS_Cfg TOKEN  ret=%d-->\r\n",ret );
	}

    ret = RIL_QLBS_Cfg(WITHTIME,&qlbs_cfg_value);
    if(ret == RIL_AT_SUCCESS)
	{
		APP_DEBUG("<-- RIL_QLBS_Cfg WITHTIME  ret=%d-->\r\n",ret );
	}

    ret = RIL_QLBS_Cfg(TIMEUPDATE,&qlbs_cfg_value);
    if(ret == RIL_AT_SUCCESS)
	{
		APP_DEBUG("<-- RIL_QLBS_Cfg TIMEUPDATE  ret=%d-->\r\n",ret );
	}

    ret = RIL_QLBS_Cfg(ASYNCH_MODE,&qlbs_cfg_value);
    if(ret == RIL_AT_SUCCESS)
	{
		APP_DEBUG("<-- RIL_QLBS_Cfg ASYNCH_MODE ret=%d-->\r\n",ret );
	}
    if(ASYNC_MODE == qlbs_cfg_value.asynch_mode)
    {
        ret = RIL_QLBS_Loc(Callback_Location);
        if(ret == RIL_AT_SUCCESS)
    	{
    		APP_DEBUG("<-- RIL_QLBS_Loc,ret=%d-->\r\n",ret );
    	}
    }
    else
    {
        ret = RIL_QLBS_Loc_Ex(&locinfo);
    	if(ret == RIL_AT_SUCCESS)
    	{
    		APP_DEBUG("<-- RIL_QLBS_Loc_Ex succeed  ret=%d-->\r\n",ret );
            APP_DEBUG("<--Module location: latitude=%f, longitude=%f time:%s\r\n", locinfo.latitude, locinfo.longitude,locinfo.time);
    	}
        else
        {
            APP_DEBUG("<-- RIL_QLBS_Loc_Ex failed  ret=%d-->\r\n",ret );
        }
    }
    
}

void Callback_Location(s32 result, ST_Lbs_LocInfo* loc_info)
{
    APP_DEBUG("\r\n<-- Module location: latitude=%f, longitude=%f result:%d-->\r\n", loc_info->latitude, loc_info->longitude,result);
    APP_DEBUG("time:%s\r\n",loc_info->time);
}

#endif
#endif // __EXAMPLE_LOCATION__
