#ifndef QUECKTEL_H
#define QUECKTEL_H

/////////////////////////////////general flags////
#include "r_cg_macrodriver.h"

extern uint8_t COMPARE_MATCH;

//*******************************GENERAL PURPUSE COMMANDS ************************************************************************//


 #define GET_PRODUCT_INFO                                  "ATI\r\n"     // Display Product Identification Information 
 #define REQ_MANUFACTURER_ID                               "AT+GMI\r\n"  // Request Manufacturer Identification 
 #define REQ_TA_MODEL                                     "AT+GMM\r\n"  // Request TA Model Identification 
 #define REQ_TA_FW_REV                                     "AT+GMR\r\n"  // Request TA Firmware Revision Identification 
 

 #define REQ_IMEI_NUM                                      "AT+GSN=1\r\n"  // Request International Mobile Equipment Identity and Serial Number 
 #define REQ_IMEI_SERIAL                                   "AT+CGSN\r\n" // Request International Mobile Equipment Identity and Serial Number 
 #define RESTOR_ATCOMMAND_FACTORY                          "AT&F\r\n"    // Restore AT Command Settings to Factory Defaults      
 #define REQ_CURRENT_CONFIG                           "AT&V\r\n"    // Display Current Configuration
 #define STORE_CURRENT_SETTINGS                            "AT&W\r\n"    // Store Current Settings to User-defined Profile
 #define RESTORE_ATCOMMAND_SETTINGS                        "ATZ\r\n"	 // Restore AT Command Setting from User-defined Profile 
 #define SET_RESULT_CODE_ECHO_MODE                         "ATQ\r\n"     // Set Result Code Echo Mode 
 #define TA_RESPONSE_FORMAT                                "ATV\r\n"     // TA Response Format
 #define SET_COMMAND_ECHOMODE                              "ATE1\r\n"    // Set Command Echo Mode
 #define DISABLE_COMMAND_ECHOMODE                          "ATE0\r\n"    // Set Command Echo Mode
 #define REPEAT_PREV_COMMAND                               "A/"          // Repeat Previous Command Line 
// #define SET_COMMAND_LINE_TERMINATION                    "ATS3=<N>\r\n"//Set Command Line Termination Character WHERE N=0-127
 //#define SET_RESPONSE_FORMATING_CHAR                     "ATS4=<N>\r\n"// Set Response Formatting Character WHERE N=0-127
 #define SET_COMMAND_LINE_EDITING_CHAR                      "ATS5=<N>"   // Set Command Line Editing Character WHERE N=0-127
 #define MONITOR_CALL_PROGRESS                              "ATX         // Set CONNECT Result Code Format and Monitor Call Progress 
 //AT+CFUN Set UE Functionality 
 //AT+CMEE Error Message Format 
 //AT+CSCS Select TE Character Set 
 //#define CONFIG_OUTPUT_PORT                                  "AT+QURCCFG Configure Output Port of URC 
 
 
 //*************************************STATUS CONTROL COMMANDS******************************************************************//
 
 
 #define MOBILE_ACTIVITY_STATUS                               "AT+CPAS\r\n"    // Mobile Equipment Activity Status
 #define ERROR_REPORT                                         "AT+CEER\r\n"    // Extended Error Report     
 #define URC_INDICATION                                       "AT+QINDCFG\r\n" // URC Indication Configuration

 #define SCAN_NETWORK                                         "AT+QCFG=\"nwscanmode",3\r\n"  //Configure Network Search Mode 3 means LTE ONLY 
 
 /*AT+QCFG="band" Configure Bands to Be Searched
 AT+QCFG="airplanecontrol" Enable/Disable Airplane Mode Control via
W_DISABLE# 
 AT+QCFG="usbnet" Configure USB Network Port Protocol 
 AT+QCFG="nat/cid" Enable NAT of PDP Context Identifier 
 AT+QCFG="qoos" Configure Network Search Timer Information in OOS State
 AT+QCFG="urc/ri/other" Specify General URC RI Behavior
 AT+QCFG="urc/ri/smsincoming" Specify SMS URC RI Behavior 
 AT+QCFG="urc/ri/ring" Specify Call Incoming URC RI Behavior
 AT+QCFG="urc/delay" Specify Delay of URC Indication 
 AT+QCFG="urc/cache" Enable/Disable URC Cache 
 AT+QCFG="risignaltype" RI Signal Output Carrier 
 AT+QCFG="cmux/urcport" Set CMUX Port to Output URC 
 AT+QCFG="ledmode" Configure Output Mode of Network Light 
 AT+QCFG="fota/cid" Configure PDP Context ID in FOTA Download
 AT+QCFG="fota/times" Configure Connection Expiration Time in HTTP FOTA
Download
 AT+QCFG="fota/path" Configure FOTA Firmware Package Storage Space */
 
 
 //***************************************NETWORK RELATED COMMANDS**********************************//
 
 #define REQ_OPERATOR                                           "AT+COPS?\r\n"   //returns current operator
 
 #define NETWORK_REG_STATUS                                     "AT+CREG?\r\n"   // returns network registration status
 #define CHECK_SIG_QUALITY                                      "AT+CSQ\r\n"    //return signal quality 
 #define QRY_NETWORK_INFO                                       "AT+QNWINFO\r\n" //query network info
 #define QRY_SERVIECE_PROVIDER                                  "AT+QSPN?\r\n"   //query serviecwe provider
 #define QRY_PDP_CONTEXT                                        "AT+CGDCONT?\r\n"//query pdp context
 #define ACTIVATE_PDP_CONTEXT                                   "AT+CGACT=1,1\r\n" //activate pdp context  
 #define OPEN_TCP_CLIENT_SERVIECE                               "AT+QIOPEN=1,0,\"TCP\",\"2401:4900:9831:fd7a:bc13:1790:33e1:8b68\",8086\r\n"   ///CLIENT MODE(change IP AND PORT AS PER REQUIREMENT)
 #define OPEN_TCP_SERVER                                        "AT+QIOPEN=1,0,\"TCP LISTENER\",\"2401:4900:9831:FD7B::2\",8086,5050,1\r\n" //server mode   
 #define CLOSE_SOCKET_CONNECTION                                "AT+QICLOSE=1\r\n"         //close socket
 #define QRY_SOCKET_CONNECTION                                  "AT+QISTATE\r\n"           //querry status of socket connections
 #define SEND_DATA_FRAME                                        "AT+QISEND=0,FRAME_SIZE\r\n"    ///SEND DAta (FRAME_SIZE is the data frame size)
 #define READ_TCP_RECIEVED_DATA                                 "AT+QIRD=0,READ_SIZE\r\n"       /////READ DATA(READ_SIZE size of the data recieved)
 
 
 #define SET_URC_NOTIFICATION                                    "AT+QIURC=1\r\n"//this command is used to set URC notification for data recieved



 
 //***********************************SIM RELATED COMMANDS*****************************************//
 
 
 
 #define REQ_IMSI                                                "AT+CIMI\r\n"   // Request International Mobile Subscriber Identity (IMSI) 
 #define LOCK_MT                                                 "AT+CLCK\r\n"// Facility Lock
 //AT+CPIN Enter PIN 
 //AT+CPWD Change Password 
 //AT+CSIM Generic (U)SIM Access 
 //AT+CRSM Restricted (U)SIM Access
// AT+QCCID Show ICCID 
 #define QRY_SIM_INITIAL_STATUS                                   "AT+QINISTAT\r\n" // Query Initialization Status of (U)SIM Card
 #define DETECT_SIM_CARD                                          "AT+QSIMDET?\r\n" // (U)SIM Card Detection 
 
 #define SIM_INSERTION_STATUS                                     "AT+QSIMSTAT?\r\n"  // (U)SIM Card Insertion Status Report 
 //AT+CCHO Open Logical Channel
 //AT+CGLA UICC Logical Channel Access 
 //AT+CCHC Close Logical Channel 
 
 //////////////////******************expected response***************************************************//
 #define ATI_RESPONSE                                             "\r\nQuectel\r\nEC200U\r\nRevision: EC200UCNAAR03A09M08\r\nOK\r\n"
 
 /////////////////////////////basic command struct////////////////////
typedef struct
{
    char* req_Operator;
    char* check_Signal;
    char* scan_Network;
    char* check_network_reg_Status;
    char* check_network_Info;
    char* check_pdp_Context;
    char* activate_pdp_Context;
    char* start_TCP_Server;
    char* start_TCP_Client;
    char* stop_TCP_Socket;
    char* set_urc_notification;
} AT_COMMANDS;


AT_COMMANDS at_cmd = { 
    "AT+COPS?\r\n",
    "AT+CSQ?\r\n",
    "AT+QCFG=\"nwscanmode\",3\r\n",
    "AT+CREG?\r\n",
    "AT+QNWINFO\r\n",
    "AT+CGDCONT?\r\n",
    "AT+CGACT=1,1\r\n",
    "AT+QIOPEN=1,0,\"TCP\",\"2401:4900:9831:fd7a:bc13:1790:33e1:8b68\",8086\r\n",
    "AT+QIOPEN=1,0,\"TCP LISTENER\",\"2401:4900:9831:FD7B::2\",8086,5050\r\n",
    "AT+QICLOSE=1\r\n",
    "AT+QIURC=1\r\n"
};



//extern const char *at_commands[] = {
//    "ATI\r",
//    "ATE0\r",
//    "AT+QSCLKEX=0\r",
//    "AT+CEDRXS=0,4\r",
//    "AT+CEDRXS?\r",
//    "AT+QCFG=\"NWSCANMODE\",0\r",
//    "AT+QICSGP=1,3,\"tpplus.smwbiot\",\"\",\"\"\r",
//    "AT+CPIN?\r",
//    "AT+EGMR=0,7\r",
//    "AT+CCID\r",
//    "AT+CREG?\r",
//    "AT+CREG=0\r",
//    "AT+CGREG=0\r",
//    "AT+COPS?\r",
//    "AT+CSQ\r",
//    "AT+QICFG=\"passiveclosed\",1\r",
//    "AT+QICFG=\"tcp/keepalive\"\r",
//    "AT+QIACT=1\r",
//    "AT+QIACT?\r",
//    "AT+QLTS=2\r",
//    "AT+QLTS=2\r",
//    "AT+QIOPEN=1,1,\"TCP LISTENER\",\"::1\",0,4059,1,1\r",
//    "AT+QICLOSE=1\r",
//    "AT+QIACT?\r",
//    "AT+QLTS=2\r",
//    "AT+QLTS=2\r",
//    "AT+QIOPEN=1,1,\"TCP LISTENER\",\"::1\",0,4059,1,1\r"
//};

#define NUM_AT_COMMANDS (sizeof(at_commands) / sizeof(at_commands[0]))

//// Array of pointers to strings, initialized after struct setup
// char*at_commands[] = {
//    at_cmd.req_Operator,
//    at_cmd.check_Signal,
//    at_cmd.scan_Network,
//    at_cmd.check_network_reg_Status,
//    at_cmd.check_network_Info,
//    at_cmd.check_pdp_Context,
//    at_cmd.activate_pdp_Context,
//    at_cmd.start_TCP_Server,
//    at_cmd.start_TCP_Client,
//    at_cmd.stop_TCP_Socket,
//    at_cmd.set_urc_notification
//};
			
		  
			  
			  
                          
             
 
 
 

 
 ////////////////////////////////////////////*******************user functions*************************//
 
 
 void Initialize_Module(void);
 void Querry_Module_status(void);
 extern void Compare_response(void);
 void Start_client_mode(void);
 void Start_server_mode(void);
 
 
#endif