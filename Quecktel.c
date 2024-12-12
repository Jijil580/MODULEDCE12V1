
//*******************quectel EC200U C SOURCE FILE FOR METER MODULE*****************************//

#include "Quecktel.h"
#include "r_cg_macrodriver.h"
#include "r_cg_userdefine.h"
#include "r_cg_tau.h"
#include "r_cg_sau.h"
#include "string.h"
#include "r_cg_wdt.h"

void __delay_ms(unsigned int milliseconds);
void Compare_response(void);
uint8_t COMPARE_MATCH=1;
uint16_t COMPARE_LENGTH;
uint8_t ci;
uint8_t COMPARE_BUFF[200];
  

 void Initialize_Module(void)
 {
	 uint8_t wait_flag=0;
	 
   while(TIMER_COUNT>10&&TIMER_COUNT<=30)
      {
       if(TIMER_COUNT>=11&&wait_flag==0)
       {
         R_UART0_Send((uint8_t *)GET_PRODUCT_INFO , sizeof(GET_PRODUCT_INFO));
         wait_flag=1;
	 R_WDT_Restart();
       }
       else if(TIMER_COUNT>=13&&wait_flag==1)
       {
         R_UART0_Send((uint8_t *)REQ_OPERATOR , sizeof(REQ_OPERATOR));
         wait_flag=2;
	  R_WDT_Restart();
       }
       else if(TIMER_COUNT>=15&&wait_flag==2)
       {
         R_UART0_Send((uint8_t *)NETWORK_REG_STATUS , sizeof(NETWORK_REG_STATUS));
         wait_flag=3;
	 R_WDT_Restart();
       }
       else if(TIMER_COUNT>=17&&wait_flag==3)
       {
         R_UART0_Send((uint8_t *)CHECK_SIG_QUALITY , sizeof(CHECK_SIG_QUALITY));
	 wait_flag=4;
	  R_WDT_Restart();
       }
      else if(TIMER_COUNT>=19&&wait_flag==4)
       {
         R_UART0_Send((uint8_t *)QRY_SERVIECE_PROVIDER , sizeof(QRY_SERVIECE_PROVIDER));
         wait_flag=5;
	  R_WDT_Restart();
	
       }
       else if(TIMER_COUNT>=21&&wait_flag==5)
       {
         R_UART0_Send((uint8_t *)QRY_PDP_CONTEXT , sizeof(QRY_PDP_CONTEXT));
         wait_flag=6;
	R_WDT_Restart();
       }
       else if(TIMER_COUNT>=23&&wait_flag==6)
       {
         R_UART0_Send((uint8_t *)ACTIVATE_PDP_CONTEXT , sizeof(ACTIVATE_PDP_CONTEXT));
         wait_flag=7;
	 R_WDT_Restart();
	
       }
       else if(TIMER_COUNT>=26&&wait_flag==7)
       {
         R_UART0_Send((uint8_t *)QRY_PDP_CONTEXT , sizeof(QRY_PDP_CONTEXT));
         wait_flag=8;
	  R_WDT_Restart();
	 
	
       }
       else if(TIMER_COUNT>=28&&wait_flag==8)
       {
         R_UART0_Send((uint8_t *)OPEN_TCP_SERVER , sizeof(OPEN_TCP_SERVER));
         wait_flag=9;
	
	  R_WDT_Restart();
       }
       else if(TIMER_COUNT>=30&&wait_flag==9)
       {
         R_UART0_Send((uint8_t *)QRY_SOCKET_CONNECTION , sizeof(QRY_SOCKET_CONNECTION));
         wait_flag=9;
	 TIMER_COUNT=32;
	 R_WDT_Restart();
       }
       
       
      }
       
     	   
   	 
  	 
	 
 }
 
 
 
 void Querry_Module_status(void)
 {
     R_UART0_Send((uint8_t *)GET_PRODUCT_INFO , sizeof(GET_PRODUCT_INFO ));	 
  	 
 }
 
 
 void Start_client_mode(void)
 {
     R_UART0_Send((uint8_t *)OPEN_TCP_CLIENT_SERVIECE , sizeof(OPEN_TCP_CLIENT_SERVIECE));	 
 }
 
 void Start_server_mode(void)
 {
	 
     R_UART0_Send((uint8_t *)OPEN_TCP_SERVER  ,sizeof(OPEN_TCP_SERVER));	 
	 
 }
 
 
 void Compare_response(void)
 {
	  COMPARE_MATCH=0; 
	  
	 COMPARE_LENGTH=sizeof(COMPARE_BUFF);
	 if(memcmp(RX0_BUFFER,ATI_RESPONSE, COMPARE_LENGTH)==0)
	 {
		  COMPARE_MATCH=0; 
		  //memset(RX0_BUFFER,0,200);  
	 }
	 else
	 {
		 COMPARE_MATCH =1; 
		// memset(RX0_BUFFER,0,200);  
	 }
		
	//memset(RX0_BUFFER,0,200);    
 }
 