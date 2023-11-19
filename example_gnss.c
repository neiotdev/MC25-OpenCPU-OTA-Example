
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
 *   example_gnss.c
 *
 * Project:
 * --------
 *   OpenCPU
 *
 * Description:
 * ------------
 *   This example demonstrates how to use GNSS function with RIL APIs in OpenCPU.
 *
 * Usage:
 * ------
 *   Compile & Run:
 *
 *     Set "C_PREDEF=-D __EXAMPLE_GNSS__" in gcc_makefile file. And compile the 
 *     app using "make clean/new".
 *     Download image bin to module to run.
 * 
 *   Operation:
 *            
 *     command: 
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

#ifdef __EXAMPLE_GNSS__ 

#include "ril.h"
#include "ril_util.h"
#include "ql_type.h"
#include "ql_trace.h"
#include "ql_system.h"
#include "ql_uart.h"
#include "ql_stdlib.h"
#include "ql_error.h"
#include "ril_gnss.h"
#include "ril_sim.h"
#include "ril_network.h"
#include "ql_timer.h"

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

/*****************************************************************
* define process state
******************************************************************/
typedef enum{
	STATE_GNSS_POWERON,
    STATE_GNSS_QUERY_STATE,
    STATE_GNSS_APN_CONFIG,
    STATE_GNSS_PDP_Context,
	STATE_GNSS_AGPS_START,
	STATE_GNSS_AGPS_AID,
	STATE_GNSS_CHECK_FIX,
	STATE_GNSS_READ_ALL_NMEA,
    STATE_GNSS_TOTAL_NUM
}Enum_GNSS_STATE;

static u8 m_gnss_state = STATE_GNSS_POWERON;

/*****************************************************************
* UART Param
******************************************************************/
Enum_SerialPort m_out_port = UART_PORT1;
#define SERIAL_RX_BUFFER_LEN  2048
#define SERIAL_TX_BUFFER_LEN  2048
static u8 m_RxBuf_Uart[SERIAL_RX_BUFFER_LEN];
static u8 m_TxBuf_Uart[SERIAL_TX_BUFFER_LEN];
static u8 pre_NMEA_buf[SERIAL_TX_BUFFER_LEN];
u16 remainLen = 0;


/*****************************************************************
* timer param
******************************************************************/
#define GNSS_TIMER_ID         			TIMER_ID_USER_START

#define GNSS_TIMER_PERIOD		1000
#define NMEA_TIMER_PERIOD		500


/*****************************************************************
* APN Param
******************************************************************/
static u8 m_apn[50] 	= "www";
static u8 m_userid[50] 	= "";
static u8 m_passwd[50] 	= "";


/*****************************************************************
* Other global variable
******************************************************************/
static s32 	ret;

/*****************************************************************
* uart callback function
******************************************************************/
static void Callback_UART_Hdlr(Enum_SerialPort port, Enum_UARTEventType msg, bool level, void* param);

/*****************************************************************
* timer callback function
******************************************************************/
static void Callback_Timer(u32 timerId, void* param);

/************************************************************************/
/* The entrance for this example application                            */
/************************************************************************/
void proc_main_task(s32 taskId)
{
    s32 ret;
    ST_MSG msg;

    // Register & open UART port
    ret = Ql_UART_Register(UART_PORT1, Callback_UART_Hdlr, NULL);
    if (ret < QL_RET_OK)
    {
        Ql_Debug_Trace("Fail to register serial port[%d], ret=%d\r\n", UART_PORT1, ret);
    }
    ret = Ql_UART_Open(UART_PORT1, 115200, FC_NONE);

    if (ret < QL_RET_OK)
    {
        Ql_Debug_Trace("Fail to open serial port[%d], ret=%d\r\n", UART_PORT1, ret);
    }
    
    APP_DEBUG("\r\n<-- OpenCPU: GNSS demo -->\r\n");

	//register & start timer 
    Ql_Timer_Register(GNSS_TIMER_ID, Callback_Timer, NULL);
	Ql_Timer_Start(GNSS_TIMER_ID, GNSS_TIMER_PERIOD, TRUE);

    // Start message loop of this task
    while (TRUE)
    {
        Ql_OS_GetMessage(&msg);

        switch(msg.message)
        {
            case MSG_ID_RIL_READY:
                Ql_RIL_Initialize();
                APP_DEBUG("<-- RIL is ready -->\r\n");
            break;

            case MSG_ID_USER_START:
                break;

            default:
                break;
        }
    }
}

static void Callback_UART_Hdlr(Enum_SerialPort port, Enum_UARTEventType msg, bool level, void* param)
{
    s32 iRet = 0;
    switch (msg)
    {
        case EVENT_UART_READY_TO_READ:
        {
            char* p = NULL;
            s32 totalBytes = ReadSerialPort(port, m_RxBuf_Uart, sizeof(m_RxBuf_Uart));
            if (totalBytes <= 0)
            {
                break;
            }

        }break;

        case EVENT_UART_READY_TO_WRITE:
        {
            if(m_out_port == port)
            {
				if(remainLen > 0)
				{
					s32 retLen = Ql_UART_Write(m_out_port, m_TxBuf_Uart, remainLen);
					if(retLen < remainLen)
					{
						remainLen -= ret;
						Ql_memmove(m_TxBuf_Uart, m_TxBuf_Uart+retLen, remainLen);
					}
				}
            }
        }break;

        default:
        break;
    }
}

bool GNSS_Get_FixStatus_from_RMC(u8 *NMEA_Str)
{
	char *p1 = NMEA_Str;
	bool fixed = FALSE;
	u8 comma = 0;
	u16 strLen = 0;

	if(NMEA_Str == NULL)
	{
		return FALSE;
	}
	strLen = Ql_strlen(NMEA_Str);
	for(u8 i = 0 ; i < strLen; i++)
	{
		if(*p1 == ',')
		{
			comma++;
			if(comma == 2)
			{
				break;
			}
		}
		p1++;
	}
	p1++;
	return (*p1 == 'A')?TRUE:FALSE;
}

void Callback_GNSS_APGS_Hdlr(char *str_URC)
{
		char* p1 = NULL;
        char* p2 = NULL;
        char strTmp[20];
        s32 len = Ql_strlen("\r\n+QGAGPS: ");
        APP_DEBUG("\r\n<- URC-->"); // log
        APP_DEBUG(str_URC); // added amit


        if (Ql_StrPrefixMatch(str_URC, "\r\n+QGAGPS:"))
        {
            p1 = Ql_strstr(str_URC, "\r\n+QGAGPS: ");
            p1 += len;
            p2 = Ql_strstr(p1, "\r\n");
            if (p1 && p2)
            {
				Ql_memset(strTmp, 0x0, sizeof(strTmp));
				Ql_memcpy(strTmp, p1, p2 - p1);
				if(0 == Ql_atoi(strTmp))
				{
					m_gnss_state = STATE_GNSS_AGPS_AID;
					APP_DEBUG("\r\n<--Download AGPS data successful-->\r\n");
				}
				else
				{
					APP_DEBUG("\r\n<--Download AGPS data failed-->\r\n");
				}
            }
        }
}

static s32 ATResponse_custom(char* line, u32 len, void* userdata)
{
 APP_DEBUG("<-print AT resp");
 APP_DEBUG(line);
}

static void Callback_Timer(u32 timerId, void* param)
{
    if (GNSS_TIMER_ID == timerId)
    {
        switch (m_gnss_state)
        {
			case STATE_GNSS_POWERON:
			{
				ret = RIL_GNSS_Open(1);
				if(ret == RIL_AT_SUCCESS)
				{
					APP_DEBUG("\r\n<-- Open GNSS OK-->\r\n");
					m_gnss_state = STATE_GNSS_QUERY_STATE;
				}
				else
				{
					APP_DEBUG("\r\n<-- Open GNSS fail -->\r\n");
				}
				break;
			}
			case STATE_GNSS_QUERY_STATE:
            {
                s32 creg = 0;
                s32 cgreg = 0;
                ret = RIL_NW_GetGSMState(&creg);
                ret = RIL_NW_GetGPRSState(&cgreg);
                APP_DEBUG("<--Network State:creg=%d,cgreg=%d-->\r\n",creg,cgreg);
                if((cgreg == NW_STAT_REGISTERED)||(cgreg == NW_STAT_REGISTERED_ROAMING))
                {
                    m_gnss_state = STATE_GNSS_APN_CONFIG;
                }
                break;
            }
			case STATE_GNSS_APN_CONFIG:
            {
                ret = RIL_GNSS_EPO_Config_APN(0,m_apn, m_userid,m_passwd);
                if (RIL_ATRSP_SUCCESS == ret)
                {
                    APP_DEBUG("<--configure APN of GNSS context OK.-->\r\n");
					m_gnss_state = STATE_GNSS_PDP_Context;
                }
				else
                {
                    APP_DEBUG("<--configure APN of GNSS context fail,ret=%d.-->\r\n",ret);
                }
                break;
            }
			case STATE_GNSS_PDP_Context:
			{
				ret = RIL_NW_OpenPDPContext();
                if (RIL_ATRSP_SUCCESS == ret)
                {
                    APP_DEBUG("<--GNSS PDP active sucessful.-->\r\n");
					m_gnss_state =STATE_GNSS_CHECK_FIX;// STATE_GNSS_AGPS_START;
                }
				else
                {
                    APP_DEBUG("<--GNSS PDP active fail,ret=%d.-->\r\n",ret);
                }
                break;
			}
            case STATE_GNSS_AGPS_START:
            {
                
	                char strAT[200]; 
                	Ql_memset( strAT, 0, sizeof(strAT) );
	                Ql_sprintf( strAT, "AT+GMR\r\n");
	
	                ret = Ql_RIL_SendATCmd( strAT, Ql_strlen(strAT), ATResponse_custom, NULL, 0 ) ;
    
               
				ret = RIL_GNSS_AGPS(Callback_GNSS_APGS_Hdlr);
				if(ret == RIL_ATRSP_SUCCESS)
				{
					m_gnss_state = STATE_GNSS_TOTAL_NUM;
                    APP_DEBUG("Start Download AGPS data, iRet = %d.\r\n", ret);
				}
                else
                {
					APP_DEBUG("<--Enable EPO download fail.-->\r\n");
                }
                break;
            }
			case STATE_GNSS_AGPS_AID:
            {
				ret = RIL_GNSS_AGPSAID();
                if(RIL_AT_SUCCESS != ret) 
                {
                    APP_DEBUG("AGPS aiding fail, iRet = %d.\r\n", ret);
                    break;
                }
				m_gnss_state = STATE_GNSS_CHECK_FIX;
                APP_DEBUG("AGPS aiding successful, iRet = %d.\r\n", ret);
                break;
            }
			case STATE_GNSS_CHECK_FIX:
            {
				u8 rd_buf[1024] = {0};
				s32 ret = RIL_GNSS_Read("RMC", rd_buf);
				if(ret == RIL_ATRSP_SUCCESS)
				{
					//APP_DEBUG("%s\r\n", rd_buf);
					if(GNSS_Get_FixStatus_from_RMC(rd_buf) == TRUE)
					{
					    APP_DEBUG("\r\n<--GNSS Successful position sucessful-->\r\n");
						m_gnss_state = STATE_GNSS_READ_ALL_NMEA;
					}
					else
					{
						APP_DEBUG("<--GPS not fixed.-->\r\n");
					}
				}
                else
                {
					APP_DEBUG("<--Read RMC fail.-->\r\n");
                }
                break;
            }
			case STATE_GNSS_READ_ALL_NMEA:
			{
				Ql_memset(m_TxBuf_Uart,0, sizeof(m_TxBuf_Uart));
				ret = RIL_GNSS_Read("ALL", m_TxBuf_Uart);
				if(Ql_strcmp(pre_NMEA_buf, m_TxBuf_Uart))
				{
					Ql_memset(pre_NMEA_buf,0, sizeof(pre_NMEA_buf));
					Ql_strcpy(pre_NMEA_buf, m_TxBuf_Uart);
					remainLen = Ql_strlen(m_TxBuf_Uart);
					if(RIL_ATRSP_SUCCESS == ret)
					{
						if(remainLen > 0)
						{
							s32 retLen = Ql_UART_Write(m_out_port, m_TxBuf_Uart, remainLen);
							if(retLen < remainLen)
							{
								remainLen -= ret;
								Ql_memmove(m_TxBuf_Uart, m_TxBuf_Uart+retLen, remainLen);
							}
						}
					}
				}
				break;
			}		
            default:
                break;
        }    
    }
}

#endif	//__EXAMPLE_GNSS__

