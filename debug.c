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
 *   debug.c 
 *
 * Project:
 * --------
 *   OpenCPU
 *
 * Description:
 * ------------
 *   The APIs are used to debug.
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
#include "ql_stdlib.h"
#include "ql_uart.h"

/*****************************************************************************
* Function:     ReadSerialPort
*
* Description:
*               Loop read uart data.
*
* Parameters:
*               port:
*                    [in] please see the definition of Enum_SerialPort.
*               pBuffer:
*                    [out]read buffer.
*               bufLen:
*                    [in] read buffer length.
* Return:
*                   return the length of the read data.
*****************************************************************************/
s32 ReadSerialPort(Enum_SerialPort port,u8* pBuffer, u32 bufLen)
{
    s32 rdLen = 0;
    s32 rdTotalLen = 0;
    if (NULL == pBuffer || 0 == bufLen)
    {
        return -1;
    }
    Ql_memset(pBuffer, 0x0, bufLen);
    while (1)
    {
        rdLen = Ql_UART_Read(port, pBuffer + rdTotalLen, bufLen - rdTotalLen);
        if (rdLen <= 0)  // All data is read out, or Serial Port Error!
        {
            break;
        }
		// Continue to read...
        rdTotalLen += rdLen;
		Ql_Sleep(20);//Avoid serial data is divided into multiple packages.
    }
    if (rdLen < 0) // Serial Port Error!
    {
        return -99;
    }
    return rdTotalLen;
}


/***************************************************************************************
*Analyse_Command
*********************/
//check Symbol completeness
static bool Check_Separator(char* pCfgStr)
{
    u32 i=0,j=0,k=0;
    char* pChar1;
    char* pChar2;
    char* pConnCfg = pCfgStr;
    
    while (pChar1 = Ql_strstr(pConnCfg, "<"))
    {
        i++;
        pConnCfg = pChar1 + 1;
    }

    pConnCfg = pCfgStr;
    while (pChar1 = Ql_strstr(pConnCfg, ">"))
    {
        j++;
        pConnCfg = pChar1 + 1;
    }
    
    pConnCfg = pCfgStr;
    while (pChar1 = Ql_strstr(pConnCfg, ">,<"))
    {
        k++;
        pConnCfg = pChar1 + 1;
    }
    
    if (!((i == j)&&(i == k + 1)))
    {
        return FALSE;
    }
    return TRUE;
}

/*****************************************************************************
* Function:     Analyse_Command
*
* Description:
*               Analyse command string
*
* Parameters:
*               src_str:
*                    [in]point to string which need to analyse.
*               symbol_num:
*                    [in]symbol number, the data which want to get in the front
*                        of the symbol.
*               symbol:
*                    [in]symbol ">"
*               dest_buf:
*                    [out]Point to the buffer that save the analysed data .
* Return:
*               None
*****************************************************************************/
s32 Analyse_Command(u8* src_str,s32 symbol_num,u8 symbol, u8* dest_buf)
{
    s32 i = 0;
    u8 *p[30];
    u8 *q;
    s32 result = -1;

    if (!Check_Separator((char*)src_str))
    {
        return result;
    }
    
    if (q = (u8*)Ql_strstr((char*)src_str,"\r\n"))//remove\r\n
    {
        *q = '\0';
    }

    if (!(q = (u8*)Ql_strstr((char*)src_str,"<")))//find first'<'
    {
        return result;
    }
    p[0] = q + 1;//remove first'<'
    
    switch(symbol)
    {
        case '>':
            
            for(i=0;i<symbol_num;i++)
            {
                if (p[i+1] = (u8*)Ql_strstr((char*)p[i],">"))
                {
                    p[i+1] += 3;
                    if (i == symbol_num - 1)
                    {
                        result = 0;
                    }
                }else
                {
                    break;
                }      
            }

            if (!result)
            {
                Ql_strncpy((char*)dest_buf,(const char *)(p[i-1]),p[i]-p[i-1]-3);
            }
            break;

        default:
            result = -1;
            break;
    }

    return result;
}


