/***********************************************************************************************************************
* DISCLAIMER
* This software is supplied by Renesas Electronics Corporation and is only intended for use with Renesas products.
* No other uses are authorized. This software is owned by Renesas Electronics Corporation and is protected under all
* applicable laws, including copyright laws. 
* THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING THIS SOFTWARE, WHETHER EXPRESS, IMPLIED
* OR STATUTORY, INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NON-INFRINGEMENT.  ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED.TO THE MAXIMUM EXTENT PERMITTED NOT PROHIBITED BY
* LAW, NEITHER RENESAS ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES SHALL BE LIABLE FOR ANY DIRECT,
* INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON RELATED TO THIS SOFTWARE, EVEN IF RENESAS OR
* ITS AFFILIATES HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* Renesas reserves the right, without notice, to make changes to this software and to discontinue the availability 
* of this software. By using this software, you agree to the additional terms and conditions found by accessing the 
* following link:
* http://www.renesas.com/disclaimer
*
* Copyright (C) 2015, 2021 Renesas Electronics Corporation. All rights reserved.
***********************************************************************************************************************/

/***********************************************************************************************************************
* File Name    : r_cg_rtc.c
* Version      : Code Generator for RL78/I1C V1.01.07.02 [08 Nov 2021]
* Device(s)    : R5F10NLE
* Tool-Chain   : CCRL
* Description  : This file implements device driver for RTC module.
* Creation Date: 05-12-2024
***********************************************************************************************************************/

/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "r_cg_macrodriver.h"
#include "r_cg_rtc.h"
/* Start user code for include. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
#include "r_cg_userdefine.h"

/***********************************************************************************************************************
Pragma directive
***********************************************************************************************************************/
/* Start user code for pragma. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
extern volatile rtc_capture_value_t g_rtc_calendar_rtcic0;
extern volatile rtc_capture_value_t g_rtc_calendar_rtcic1;
extern volatile rtc_capture_value_t g_rtc_calendar_rtcic2;
extern volatile uint32_t g_rtc_binary_rtcic0;
extern volatile uint32_t g_rtc_binary_rtcic1;
extern volatile uint32_t g_rtc_binary_rtcic2;
extern volatile uint8_t g_lvd_vrtc_ready_flag;    /* VRTC pin voltage ready flag */
/* Start user code for global. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
* Function Name: R_RTC_Create
* Description  : This function initializes the real-time clock module.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_RTC_Create(void)
{
    volatile uint8_t tmp;
    volatile uint16_t w_count;

    VRTCEN = 1U;    /* enables input clock supply */

    /* Change the waiting time according to the system */
    for (w_count = 0U; w_count <= RTC_STARTWAITTIME; w_count++)
    {
        NOP();
    }

    if ((_00_RTC_POWERON_RESET_OCCUR == RTCPORSR) && (1U == g_lvd_vrtc_ready_flag))
    {
        RTCPORSR = _01_RTC_POWERON_RESET_CHECK;
        g_lvd_vrtc_ready_flag = 0U;
        tmp = RCR2;
        tmp &= (uint8_t)~_01_RTC_COUNTER_NORMAL;
        RCR2 = tmp;
        RTCRMK = 1U;    /* disable INTRTCPRD interrupt */
        RTCRIF = 0U;    /* clear INTRTCPRD interrupt flag */
        RTCAMK = 1U;    /* disable INTRTCALM interrupt */
        RTCAIF = 0U;    /* clear INTRTCALM interrupt flag */
        tmp = RCR1;
        tmp &= (uint8_t)~_01_RTC_ALARM_ENABLE;
        RCR1 = tmp;
        tmp = RCR1;
        tmp &= (uint8_t)~_04_RTC_PERIODIC_ENABLE;
        RCR1 = tmp;
        RCR5GD = 0x00U;
        RCR5GD = 0x72U;
        RCR5GD = 0x64U;
        RCR5 = 0x00U;
        RCR5GD = 0x00U;
        RCR4 = _00_RTC_SELECT_FSUB;
        tmp = RCR3;
        tmp |= _01_RTC_INPUT_ENABLE;
        RCR3 = tmp;

        /* Change the waiting time according to the system */
        for (w_count = 0U; w_count <= RTC_6CLOCKWAITTIME; w_count++)
        {
            NOP();
        }

        tmp = RCR2;
        tmp &= (uint8_t)~_80_RTC_BINARY_MODE;
        RCR2 = tmp;

        /* Change the waiting time according to the system */
        for (w_count = 0U; w_count <= RTC_STARTWAITTIME; w_count++)
        {
            NOP();
        }

        tmp = RCR2;
        tmp |= _02_RTC_RESET_WRITER_INITIALIZED;
        RCR2 = tmp;

        /* Change the waiting time according to the system */
        for (w_count = 0U; w_count <= RTC_RESETWAITTIME; w_count++)
        {
            NOP();
        }

        RSECCNT = _10_RTC_CALENDER_COUNTER_10SEC | _00_RTC_CALENDER_COUNTER_1SEC;
        RMINCNT = _30_RTC_CALENDER_COUNTER_10MIN | _01_RTC_CALENDER_COUNTER_1MIN;
        RHRCNT = _40_RTC_CALENDER_COUNTER_PMHOUR | _10_RTC_CALENDER_COUNTER_10HOUR | _04_RTC_CALENDER_COUNTER_1HOUR;
        RWKCNT = _04_RTC_CALENDER_COUNTER_THURWEEK;
        RDAYCNT = _00_RTC_CALENDER_COUNTER_10DAY | _05_RTC_CALENDER_COUNTER_1DAY;
        RMONCNT = _10_RTC_CALENDER_COUNTER_10MONTH | _02_RTC_CALENDER_COUNTER_1MONTH;
        RYRCNT = _0020_RTC_CALENDER_COUNTER_10YEAR | _0004_RTC_CALENDER_COUNTER_1YEAR;
        tmp = RCR1;
        tmp |= _00_RTC_PERIODIC_DISABLE | _00_RTC_ALARM_DISABLE;
        RCR1 = tmp;
        tmp = RCR2;
        tmp |= _00_RTC_CALENDER_MODE | _40_RTC_24HOUR_MODE | _00_RTC_RTCOUT_DISABLE;
        RCR2 = tmp;
 }
}

/***********************************************************************************************************************
* Function Name: R_RTC_Start
* Description  : This function enables the real-time clock.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_RTC_Start(void)
{
    volatile uint8_t tmp;

    tmp = RCR2;
    tmp |= _01_RTC_COUNTER_NORMAL;
    RCR2 = tmp;
}

/***********************************************************************************************************************
* Function Name: R_RTC_Stop
* Description  : This function disables the real-time clock.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_RTC_Stop(void)
{
    volatile uint8_t tmp;
    volatile uint16_t w_count;

    tmp = RCR2;
    tmp &= (uint8_t)~_01_RTC_COUNTER_NORMAL;
    RCR2 = tmp;

    /* Change the waiting time according to the system */
    for (w_count = 0U; w_count <= RTC_STARTWAITTIME; w_count++)
    {
        NOP();
    }

    tmp = RCR2;
    tmp |= _02_RTC_RESET_WRITER_INITIALIZED;
    RCR2 = tmp;
	
    /* Change the waiting time according to the system */
    for (w_count = 0U; w_count <= RTC_RESETWAITTIME; w_count++)
    {
        NOP();
    }

}


/***********************************************************************************************************************
* Function Name: R_RTC_Set_CalendarCounterValue
* Description  : This function changes the calendar real-time clock value.
* Arguments    : counter_write_val -
*                    the expected real-time clock value(BCD code)
* Return Value : status -
*                    MD_OK or MD_BUSY1
***********************************************************************************************************************/
MD_STATUS R_RTC_Set_CalendarCounterValue(rtc_counter_value_t counter_write_val)
{
    MD_STATUS status = MD_OK;
    volatile uint16_t  w_count;
    volatile uint8_t tmp;

    tmp = RCR2;
    tmp &= (uint8_t)~_01_RTC_COUNTER_NORMAL;
    RCR2 = tmp;

    /* Change the waiting time according to the system */
    for (w_count = 0U; w_count < RTC_WAITTIME; w_count++)
    {
        NOP();
    }

    if (_01_RTC_COUNTER_NORMAL == (RCR2 & _01_RTC_COUNTER_NORMAL))
    {
        status = MD_BUSY1;
    }
    else
    {
        RSECCNT = counter_write_val.sec;
        RMINCNT = counter_write_val.min;
        RHRCNT = counter_write_val.hour;
        RWKCNT = counter_write_val.week;
        RDAYCNT = counter_write_val.day;
        RMONCNT = counter_write_val.month;
        RYRCNT = counter_write_val.year;
    }

    return (status);
}


/***********************************************************************************************************************
* Function Name: R_RTC_Get_CalendarCounterValue
* Description  : This function reads the results of real-time clock and store them in the variables.
* Arguments    : counter_read_val -
*                    the expected real-time clock value(BCD code)
* Return Value : status -
*                    MD_OK or MD_ERROR
***********************************************************************************************************************/
MD_STATUS R_RTC_Get_CalendarCounterValue(rtc_counter_value_t * const counter_read_val)
{
    MD_STATUS status = MD_OK;
    volatile uint8_t tmp;

    tmp = RSR;
    tmp &= (uint8_t)~_02_RTC_SECOND_CARRY;
    RSR = tmp;
    counter_read_val->sec = RSECCNT;
    counter_read_val->min = RMINCNT;
    counter_read_val->hour = RHRCNT;
    counter_read_val->week = RWKCNT;
    counter_read_val->day = RDAYCNT;
    counter_read_val->month = RMONCNT;
    counter_read_val->year = RYRCNT;
    if (_02_RTC_SECOND_CARRY == (RSR & _02_RTC_SECOND_CARRY))
    {
        status = MD_ERROR;
    }

    return (status);
}














/* Start user code for adding. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
