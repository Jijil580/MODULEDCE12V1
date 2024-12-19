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
* File Name    : r_cg_tau_user.c
* Version      : Code Generator for RL78/I1C V1.01.07.02 [08 Nov 2021]
* Device(s)    : R5F10NLE
* Tool-Chain   : CCRL
* Description  : This file implements device driver for TAU module.
* Creation Date: 18-10-2024
***********************************************************************************************************************/

/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "r_cg_macrodriver.h"
#include "r_cg_tau.h"
/* Start user code for include. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
#include "r_cg_userdefine.h"
#include "r_cg_wdt.h"
/***********************************************************************************************************************
Pragma directive
***********************************************************************************************************************/
#pragma interrupt r_tau0_channel0_interrupt(vect=INTTM00)
/* Start user code for pragma. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
uint8_t DATA_RECIEVED=0;
uint8_t TCP_INIT_STATUS;
int START_TIMER=0;
//uint8_t METER_DATA=0;
/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
/* Start user code for global. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
* Function Name: r_tau0_channel0_interrupt
* Description  : This function INTTM00 interrupt service routine.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
static void __near r_tau0_channel0_interrupt(void)
{
    /* Start user code. Do not edit comment generated here */
     //R_WDT_Restart();  
      TIMER_COUNT++;
      R_WDT_Restart();
      if(TIMER_COUNT>=1&&TCP_INIT_STATUS==0)///INITIALISING MODULE
	  {
           TIMER_COUNT=0;
	   DATA_RECIEVED=1;
	   R_TAU0_Channel0_Stop();
	  
	   }
       if(TIMER_COUNT>=1&&TCP_INIT_STATUS==1&&METER_DATA==0)///TCP DATA RECVD
	  {
           TCP_DATA=1;
           TIMER_COUNT=0;
	   DATA_RECIEVED=1;
	   R_TAU0_Channel0_Stop();
	   START_TIMER=0;
	   }
	  if(TIMER_COUNT>=1&&TCP_INIT_STATUS==1&&METER_DATA==1)///DATA FROM METER RECVD
	  {
           TCP_DATA=0;
           TIMER_COUNT=0;
	   DATA_RECIEVED=1;
	   R_TAU0_Channel0_Stop();
	   
	   }
	  
	  
     R_WDT_Restart();
    
    /* End user code. Do not edit comment generated here */
}

extern void delay_In_Seconds(long int seconds)
{
	
	R_TAU0_Channel0_Start();
	while(TIMER_COUNT<seconds);
	TIMER_COUNT=0;
	R_TAU0_Channel0_Stop();
}


/* Start user code for adding. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
