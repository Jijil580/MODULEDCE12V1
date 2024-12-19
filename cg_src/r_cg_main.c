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
* File Name    : r_cg_main.c
* Version      : Code Generator for RL78/I1C V1.01.07.02 [08 Nov 2021]
* Device(s)    : R5F10NLE
* Tool-Chain   : CCRL
* Description  : This file implements main function.
* Creation Date: 11-10-2024
***********************************************************************************************************************/

/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "r_cg_macrodriver.h"
#include "r_cg_cgc.h"
#include "r_cg_port.h"
#include "r_cg_tau.h"
#include "r_cg_wdt.h"
#include <stdio.h>
#include <stdlib.h>

#include "r_cg_sau.h"
#include "string.h"
//#include "Quecktel.h"
/* Start user code for include. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
#include "r_cg_userdefine.h"
 const char *at_commands[] = {
    "ATI\r\n",
    "ATE0\r\n",
    "AT+QSCLKEX=0\r\n",
    "AT+CEDRXS=0,4\r\n",
    "AT+CEDRXS?\r",
    "AT+QCFG=\"NWSCANMODE\",0\r",
    "AT+QICSGP=1,3,\"tpplus.smwbiot\",\"\",\"\"\r",
    "AT+CPIN?\r",
    "AT+EGMR=0,7\r",
    "AT+CCID\r",
    "AT+CREG?\r",
    "AT+CREG=0\r",
    "AT+CGREG=0\r",
    "AT+COPS?\r",
    "AT+CSQ\r",
    "AT+QICFG=\"passiveclosed\",1\r",
    "AT+QICFG=\"tcp/keepalive\"\r",
    "AT+QIACT=1\r",
    "AT+QIACT?\r",
    "AT+QLTS=2\r",
    "AT+QLTS=2\r",
    "AT+QIOPEN=1,1,\"TCP LISTENER\",\"::1\",0,4059,1,1\r"
    //"AT+QIOPEN=1,1,\"TCP LISTENER\",\"2401:4900:9831:FD7B::2\",0,4059,1,1\r"
    //"AT+QIOPEN=1,1,\"TCP LISTENER\",\"::1\",0,4059,1,1\r",
   // "AT+QICLOSE=1\r",
    //"AT+QIACT?\r",
    //"AT+QLTS=2\r",
    //"AT+QLTS=2\r",
   // "AT+QIOPEN=1,1,\"TCP LISTENER\",\"2401:4900:9831:FD7B::2\",0,8086,1,1\r"
};




char at_command[50];
uint8_t TCP_SUCCESS[]="tcplisten mode\r\n";
uint8_t TCP_REPLY[]="HELLO IAM METER\r\n";
uint8_t TCP_CLOSE[]="AT+QICLOSE=1\r";
uint8_t TCP_OPEN[]="AT+QIOPEN=1,1,\"TCP LISTENER\",\"2401:4900:9831:FD7B::2\",0,4059,1,1\r";
uint8_t TEST_TCP_DATA[]="my name is jijil";


uint8_t UART1_RECIEVED_DATA[512];
uint8_t UART0_RECIEVED_DATA[512];
uint8_t RX1_BUFFER_COPY[512];
uint8_t TEMP_BUFFER[512];
 //uint8_t result[][512] = {0};  // Array to hold extracted data


uint8_t RECIEVED=0;
uint8_t RECIEVED_TCP=0;
long int TIMER_COUNT;

extern uint8_t DATA_RECIEVED;
extern uint8_t COMPARE_MATCH1=0;
extern uint8_t TCP_INIT_STATUS;
extern int  RX0_BUFFER_COUNT;

int FEEDBACK_LENGTHG;
uint8_t count=0;
uint8_t BUF_FLAG=0;
uint8_t TCP_INT_FLAG=0;
uint8_t MODULE_MODE=0;
uint8_t TCP_DATA_BUFFER[3][512];
uint8_t TCP_DATA_PROCESSED=0;
int connection_id=11;
uint8_t frame_size=sizeof(TCP_REPLY);
uint8_t framsize=0;
uint8_t tcp_count=0;
uint8_t TCP_DATA=0;
uint8_t TCP_FETCH[30];
//uint8_t **TCP_DATA_BUFFER;  // 2D array to hold up to 10 strings
/***********************************************************************************************************************
Pragma directive
***********************************************************************************************************************/
/* Start user code for pragma. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
/* Start user code for global. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
void Initialize_Module(void);
void PROCESS_TCP_DATA(void);
static void R_MAIN_UserInit(void);
void FETCH_TCPDATA_AND_SEND(void);
extern void __delay_ms(unsigned int milliseconds);
uint8_t INIT_MODULE_TO_LISTEN_TCP(void);
uint8_t CHECK_MODULE_RESPONSE(uint8_t *RESPONSE);
void SPLIT_TCP_DATA(uint8_t *BUFFER);
void generate_at_command(uint8_t connection_id, uint8_t frame_size);
static void r_uart0_callback_receiveend(void);
//void split_and_store(const uint8_t *input, uint8_t *output, size_t max_output_size);
uint8_t SEND_TCP_REPLY(void);
void split_and_store(void) ;
//void allocate_buffer();
int m;

 void __delay_ms(unsigned int milliseconds) {
    volatile unsigned int i, j;
    for (i = 0; i < milliseconds; i++) {
        for (j = 0; j < 1000; j++) {
          //  __asm("nop");  // No operation, just waste some cycles
        }
    }
}

/***********************************************************************************************************************
* Function Name: main
* Description  : This function implements main function.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void main(void)
{
    R_MAIN_UserInit();
    R_UART0_Start();
    R_UART1_Start();
    //R_TAU0_Channel0_Start();
    P7&=~(1<<2);
    P7|=(1<<1);
    P7 |=(1<<3);
    __delay_ms(1000);
    __delay_ms(1000);
     P7&=~(1<<3);
    R_UART0_Receive(UART0_RECIEVED_DATA, sizeof(UART0_RECIEVED_DATA));//gprs
    R_UART1_Receive(UART1_RECIEVED_DATA, sizeof(UART1_RECIEVED_DATA));//module mcu
    __delay_ms(500); 
    
    
   // allocate_buffer();
    while (1U)
    {
    	R_WDT_Restart(); 
	
	//if(TCP_INIT_STATUS==0)
	if(MODULE_MODE==INIT_MODE)
	{
		
		TCP_INIT_STATUS= INIT_MODULE_TO_LISTEN_TCP();
	}
	   
      //  else if(TCP_INIT_STATUS==1)
        if(MODULE_MODE==TCP_MODE)
	{	
		if(DATA_RECIEVED == 1) //ENSURE DATA IS RECIVED
                {
			DATA_RECIEVED=0;
			
			///CONDITION FOR HANDLING DATAS FROM METYER // 
			
               		if(METER_DATA==1&&TCP_DATA==0)//CHECK THE DATA RECIVED IS FROM METER AND NALSO NOT FROM THE TCP DATA
	      	        {
				FETCH_TCPDATA_AND_SEND();
				METER_DATA=4;
				//DATA_RECIEVED=0;
	       		}
			
	       		//CONDITION FOR HANDLING TCP DATA/////
			
	       		else if(METER_DATA==0&&TCP_DATA==1)//data recieved from TCP NOT FROM METER
	       		{
	        		memset(TCP_DATA_BUFFER, 0, sizeof(TCP_DATA_BUFFER));
                		DATA_RECIEVED=0;
				RX0_BUFFER[RX0_BUFFER_COUNT]='\0';
				
				METER_DATA=3;
				TCP_DATA=3;
				split_and_store(); 
	       			// R_UART1_Send(RX0_BUFFER, strlen(RX0_BUFFER));
	       			if(RECIEVED_TCP==1)
	       			{
		    			RECIEVED_TCP=0;
	             			R_UART1_Send(TEMP_BUFFER, strlen(TEMP_BUFFER));
					//MODULE_MODE=TCP_RESTART_MODE;
	      			}
		
		
	       		}
	 
	      }
		
//	if(MODULE_MODE==TCP_RESTART_MODE)
//	{
//	 R_UART0_Send(TCP_CLOSE, sizeof(TCP_CLOSE));	
		
//	}

	      
	   TCP_INIT_STATUS=1;
	   
	   
	}
	
        ;
	//delay_In_Seconds(1000);
      }
      
      
      
      
    /* End user code. Do not edit comment generated here */
}
/***********************************************************************************************************************
* Function Name: R_MAIN_UserInit
* Description  : This function adds user code before implementing main function.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
static void R_MAIN_UserInit(void)
{
    /* Start user code. Do not edit comment generated here */
    EI();
    /* End user code. Do not edit comment generated here */
}

/* Start user code for adding. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
uint8_t INIT_MODULE_TO_LISTEN_TCP(void) // Fix: Function name formatting (spaces not allowed)
{
    if (DATA_RECIEVED == 1) // Ensure proper spacing for readability
    {
	    
	
        COMPARE_MATCH1=CHECK_MODULE_RESPONSE(RX0_BUFFER);
	if(COMPARE_MATCH1==1)
		{
        	TCP_INT_FLAG++; // Increment `TCP_INT_FLAG`
		COMPARE_MATCH1=0;
		}
        DATA_RECIEVED = 0; // Reset flag
        memset(RX0_BUFFER, 0, 200); // Clear buffer
        RX0_BUFFER_COUNT = 0; // Reset buffer count

        if (TCP_INT_FLAG<=22) // Ensure `TCP_INT_FLAG` is within valid range
       		 {
         	 R_UART0_Send(at_commands[TCP_INT_FLAG], strlen(at_commands[TCP_INT_FLAG])); // Send AT command
		 TCP_INIT_STATUS=0;
		 MODULE_MODE=0;
       		 }
	if(TCP_INT_FLAG>=22)
	{
	  TCP_INIT_STATUS=TCP_INITIALISED;
	  TCP_INT_FLAG=0;
	  MODULE_MODE=1;
	}
    }
   return TCP_INIT_STATUS;
}
uint8_t CHECK_MODULE_RESPONSE(uint8_t *RESPONSE)
{
	RX0_BUFFER[RX0_BUFFER_COUNT]='\0';
	for (BUF_FLAG = 0;BUF_FLAG < RX0_BUFFER_COUNT;BUF_FLAG++)
        {
            // Check if the current sequence matches "OK\r\n"
            if(RESPONSE[BUF_FLAG] == 'O' && RESPONSE[BUF_FLAG+ 1] == 'K' && RESPONSE[BUF_FLAG+ 2] == '\r' && RESPONSE[BUF_FLAG + 3] == '\n')
            {
                return 1; // Match found, return 1
            }
	}

    return 0; // No match found

}

void PROCESS_TCP_DATA(void)
{
 if (DATA_RECIEVED == 1) // Ensure proper spacing for readability
    {	RX0_BUFFER[RX0_BUFFER_COUNT]='\0';
	DATA_RECIEVED = 0; // Reset flag
	 
       // memset(RX0_BUFFER, 0, 200); // Clear buffer
        //RX0_BUFFER_COUNT = 0; // Reset buffer count
	memset(TCP_DATA_BUFFER,0,sizeof(TCP_DATA_BUFFER));
    }
}


void SPLIT_TCP_DATA(uint8_t *BUFFER)
{
    int token_index = 0; // Current token index
    int char_index = 0;  // Current character index in a token
    int buffer_i = 0;

    for (buffer_i = 0; buffer_i<=RX0_BUFFER_COUNT; buffer_i++)
    {
        if (BUFFER[buffer_i] == ',') // If the delimiter is found
        {
          //  TCP_DATA_BUFFER[token_index][char_index] = '\0'; // Null-terminate the current token
            token_index++; // Move to the next token
            char_index = 0; // Reset character index for the next token

            // Prevent overflow of tokens
            if (token_index >= 3) 
                break;
        }
        else
        {
            // Add character to the current token
            if (char_index < sizeof(TCP_DATA_BUFFER[token_index]) - 1)  // Prevent overflow
            {
                TCP_DATA_BUFFER[token_index][char_index++] = BUFFER[buffer_i];
            }
        }
    }

    // Null-terminate the last token if it's not already null-terminated
    if (token_index < 3 && char_index < sizeof(TCP_DATA_BUFFER[token_index])) 
    {
        TCP_DATA_BUFFER[token_index][char_index] = '\0';
    }

    TCP_DATA_PROCESSED = 1;
   // memset(RX0_BUFFER,0,512);
}

uint8_t SEND_TCP_REPLY(void)
{
 R_UART0_Send(at_command, strlen(at_command));
 __delay_ms(700);
 R_UART0_Send(TCP_DATA_BUFFER[2], strlen((char*)TCP_DATA_BUFFER[2]));
//  __delay_ms(500);
// R_UART0_Send(TCP_CLOSE, sizeof(TCP_CLOSE));
// __delay_ms(500);
//  R_UART0_Send(TCP_CLOSE, sizeof(TCP_OPEN));
 
 return 1;
}

void generate_at_command(uint8_t connection_id, uint8_t frame_size)
{
	int ti;
	TCP_DATA_BUFFER[1][2] = '\0';
     //connection_id = atoi((char *)&TCP_DATA_BUFFER[1][0]);
	for(ti=0;TCP_DATA_BUFFER[2][ti]!='\n';ti++)
	{
	framsize++;
	}
    // Buffer to store the final AT command string

    // Use sprintf to format the string with connection_id and frame_size
    sprintf(at_command, "AT+QISEND=%u,%u\r\n", 11, strlen(RX1_BUFFER_COPY));

   
}
void FETCH_TCPDATA_AND_SEND(void)
{
	 memset(RX1_BUFFER_COPY,0,sizeof(RX1_BUFFER_COPY));
		 memset(TCP_FETCH,0,sizeof(TCP_FETCH));
		 DATA_RECIEVED=0;
		 METER_DATA=0;
		
		 RX1_BUFFER[RX1_BUFFER_COUNT]='\0';
		 strcpy((char *)RX1_BUFFER_COPY, (char *)(RX1_BUFFER + 2));
		  for(m=0;RX1_BUFFER_COPY[m]!='\r';m++)
		 {
		 TCP_FETCH[m]=RX1_BUFFER_COPY[m];	 
		 }
		 TCP_FETCH[m++]='\r';
		 TCP_FETCH[m++]='\0';
		 m=0;
		 sprintf(at_command, "AT+QISEND=%u,%u\r\n", 11, (strlen(RX1_BUFFER_COPY)-1));//CREATE AT COMMAND FOR SENDING

		//R_UART0_Send(TCP_FETCH, strlen(TCP_FETCH));
		
		  R_UART0_Send(at_command, strlen(at_command));
		  __delay_ms(400);
		  R_UART0_Send(RX1_BUFFER_COPY, (strlen(RX1_BUFFER_COPY)));
		 
		// R_UART0_Send(TEST_TCP_DATA, 17);
		
	
}
void split_and_store(void)
{
      // Temporary buffer to work with the input
    int i;  // Declare all variables at the top
    int temp_index=0;
    int data_intex=0;
    //size_t index = 0;  // Declare index at the top
    memset(TEMP_BUFFER,0,sizeof(TEMP_BUFFER));
    for(i=0;i<500;i++)
    {
	if(RX0_BUFFER[i]=='r'&&RX0_BUFFER[i+1]=='e'&&
	RX0_BUFFER[i+2]=='c'&&RX0_BUFFER[i+3]=='v')  
	{
          RECIEVED_TCP=1;
	  RECIEVED=1;
	  data_intex=i;
	  break;
	}
    }
    if(RECIEVED==1)
    {
	 for(i=data_intex;i<=512;i++)
	  {
		if((RX0_BUFFER[i]=='\n')&&(RECIEVED!=2))
		{
	         	RECIEVED=2;
			i++;
		}
		if(RECIEVED==2)
		{
	  		TEMP_BUFFER[temp_index]=RX0_BUFFER[i];
	  		temp_index++;
		}
	  }
           TEMP_BUFFER[temp_index]='\0';
           RECIEVED=0;
	   memset(RX0_BUFFER,0,sizeof(RX0_BUFFER));
	   data_intex=0;
	   temp_index=0;
     }
	
}




