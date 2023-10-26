/*******************************************************************************
* Copyright © 2021 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2023 Analog Devices Inc. All Rights Reserved. This software is
* proprietary & confidential to Analog Devices, Inc. and its licensors.
*******************************************************************************/

/** \file TMCL.h ***********************************************************
 *
 *             Project: Homebus Reference Design
 *            Filename: TMCL.h
 *         Description: TMCL command and status code definitions
 *
 *
 *    Revision History:
 *                    2021_01_26    Rev 1.00    Olav Kahlbaum   File created
 *
 *  -------------------------------------------------------------------- */

//States of the command interpreter
#define TM_IDLE 0              //!< idle mode (no stand-alone program running)
#define TM_RUN 1               //!< run mode (stand-alone program running)
#define TM_STEP 2              //!< step mode (stepping through a stand-alone program)
#define TM_RESET 3             //!< not used
#define TM_DOWNLOAD 4          //!< download mode
#define TM_DEBUG 5             //!< debug mode

#define TCS_IDLE  0            //!< TMCL interpreter is in idle mode (no command to process)
#define TCS_UART  1            //!< processing a command from RS485
#define TCS_UART_ERROR 2       //!< last command from RS485 had bad check sum
#define TCS_USB   3            //!< processing a command from USB
#define TCS_USB_ERROR  4       //!< last command from USB had bad check sum
#define TCS_CAN7  5            //!< processing a command from CAN (7 bytes)
#define TCS_CAN8  6            //!< processing a command from CAN (8 bytes)
#define TCS_MEM   7            //!< processing a command from memory

//TMCL commands
#define TMCL_ROR 1
#define TMCL_ROL 2
#define TMCL_MST 3
#define TMCL_MVP 4
#define TMCL_SAP 5
#define TMCL_GAP 6
#define TMCL_STAP 7
#define TMCL_RSAP 8
#define TMCL_SGP 9
#define TMCL_GGP 10
#define TMCL_STGP 11
#define TMCL_RSGP 12
#define TMCL_RFS 13
#define TMCL_SIO 14
#define TMCL_GIO 15
#define TMCL_CALC 19
#define TMCL_COMP 20
#define TMCL_JC   21
#define TMCL_JA   22
#define TMCL_CSUB 23
#define TMCL_RSUB 24
#define TMCL_EI 25
#define TMCL_DI 26
#define TMCL_WAIT 27
#define TMCL_STOP 28
#define TMCL_SAC 29
#define TMCL_SCO 30
#define TMCL_GCO 31
#define TMCL_CCO 32
#define TMCL_CALCX 33
#define TMCL_AAP 34
#define TMCL_AGP 35
#define TMCL_CLE 36
#define TMCL_VECT 37
#define TMCL_RETI 38
#define TMCL_ACO 39
#define TMCL_UF0 64
#define TMCL_UF1 65
#define TMCL_UF2 66
#define TMCL_UF3 67
#define TMCL_UF4 68
#define TMCL_UF5 69
#define TMCL_UF6 70
#define TMCL_UF7 71

#define TMCL_SAPX   16
#define TMCL_GAPX   17
#define TMCL_AAPX   18
#define TMCL_CALCVV 40
#define TMCL_CALCVA 41
#define TMCL_CALCAV 42
#define TMCL_CALCVX 43
#define TMCL_CALCXV 44
#define TMCL_CALCV  45
#define TMCL_MVPA   46
#define TMCL_MVPXA  47
#define TMCL_RST    48
#define TMCL_DJNZ   49
#define TMCL_ROLA   50
#define TMCL_RORA   51
#define TMCL_ROLXA  52
#define TMCL_RORXA  53
#define TMCL_MSTX   54
#define TMCL_SIV    55
#define TMCL_GIV    56
#define TMCL_AIV    57
#define TMCL_PUSHA  58
#define TMCL_PUSHX  59
#define TMCL_PUSHV  60
#define TMCL_POPA   61
#define TMCL_POPX   62
#define TMCL_POPV   63
//64..79 => UFx
#define TMCL_CALL   80

#define TMCL_ApplStop 128
#define TMCL_ApplRun 129
#define TMCL_ApplStep 130
#define TMCL_ApplReset 131
#define TMCL_DownloadStart 132
#define TMCL_DownloadEnd 133
#define TMCL_ReadMem 134
#define TMCL_GetStatus 135
#define TMCL_GetVersion 136
#define TMCL_FactoryDefault 137
#define TMCL_SetEvent 138
#define TMCL_SetASCII 139
#define TMCL_SecurityCode 140
#define TMCL_Breakpoint 141

#define TMCL_DriverCalibration 154

#define TMCL_Boot 0xf2
#define TMCL_SoftwareReset 0xff


//Type codes of the MVP command
#define MVP_ABS   0            //!< absolute movement (with MVP command)
#define MVP_REL   1            //!< relative movement (with MVP command)
#define MVP_COORD 2            //!< coordinate movement (with MVO command)

//Optionen für relative Positionierung
#define RMO_TARGET 0    //letzte Zielposition
#define RMO_ACTINT 1    //aktuelle Rampengeneratorposition
#define RMO_ACTENC 2    //aktuelle Encoderposition

//TMCL status codes
#define REPLY_OK 100                //!< command successfully executed
#define REPLY_CMD_LOADED 101        //!< command succsssfully stored in EEPROM
#define REPLY_DELAYED 128           //!< delayed reply
#define REPLY_CHKERR 1              //!< checksum error
#define REPLY_INVALID_CMD 2         //!< command not supported
#define REPLY_WRONG_TYPE 3          //!< wrong type code
#define REPLY_INVALID_VALUE 4       //!< wrong value
#define REPLY_EEPROM_LOCKED 5       //!< EEPROM is locked
#define REPLY_CMD_NOT_AVAILABLE 6   //!< command not available due to current state
#define REPLY_CMD_LOAD_ERROR 7      //!< error when storing command to EEPROM
#define REPLY_WRITE_PROTECTED 8     //!< EEPROM is write protected
#define REPLY_MAX_EXCEEDED 9        //!< maximum number of commands in EEPROM exceeded

//Reply format
#define RF_STANDARD 0               //!< use standard TMCL reply
#define RF_SPECIAL 1                //!< use special reply

//Optionscodes
#define RFS_START 0
#define RFS_STOP 1
#define RFS_STATUS 2

#define WAIT_TICKS 0
#define WAIT_POS 1
#define WAIT_REFSW 2
#define WAIT_LIMSW 3
#define WAIT_RFS 4

#define JC_ZE 0
#define JC_NZ 1
#define JC_EQ 2
#define JC_NE 3
#define JC_GT 4
#define JC_GE 5
#define JC_LT 6
#define JC_LE 7
#define JC_ETO 8
#define JC_EAL 9
#define JC_EDV 10
#define JC_EPO 11
#define JC_ESD 12

#define CALC_ADD 0
#define CALC_SUB 1
#define CALC_MUL 2
#define CALC_DIV 3
#define CALC_MOD 4
#define CALC_AND 5
#define CALC_OR 6
#define CALC_XOR 7
#define CALC_NOT 8
#define CALC_LOAD 9
#define CALC_SWAP 10
#define CALC_COMP 11

#define CLE_ALL 0
#define CLE_ETO 1
#define CLE_EAL 2
#define CLE_EDV 3
#define CLE_EPO 4
#define CLE_ESD 5

//"CPU-Flags"
#define FLAG_EQUAL           0x00000001
#define FLAG_LOWER           0x00000002
#define FLAG_GREATER         0x00000004
#define FLAG_GREATER_EQUAL   0x00000005
#define FLAG_LOWER_EQUAL     0x00000003
#define FLAG_ZERO            0x00000008
#define FLAG_ERROR_TIMEOUT   0x00000010
#define FLAG_ERROR_EXT_ALARM 0x00000020
#define FLAG_ERROR_DEVIATION 0x00000040
#define FLAG_ERROR_POSITION  0x00000080
#define FLAG_ERROR_SHUTDOWN  0x00000100

//Schreib-/Leseschutzbits
#define PB_READ  0x01
#define PB_WRITE 0x02

//Motor-Fehler-Flags
#define ME_STALLGUARD 0x01
#define ME_DEVIATION  0x02

#if !defined(MKL03Z32)
  #define TMCL_MEM_SIZE 2048
  #define TMCL_STACK_DEPTH 8
  #define TMCL_RAM_USER_VARS 256     //Es gibt 256 User-Variablen (mehr als 256 geht nicht)...
  #define TMCL_EEPROM_USER_VARS 56   //...von denen die ersten 56 auch im EEPROM gespeichert werden können.
  #define TMCL_COORDINATES 21
  #define TMCL_BREAKPOINTS 10
#else
  #define TMCL_MEM_SIZE 876
  #define TMCL_STACK_DEPTH 8
  #define TMCL_RAM_USER_VARS 20     //Es gibt 20 User-Variablen
  #define TMCL_EEPROM_USER_VARS 20
  #define TMCL_COORDINATES 10
  #define TMCL_BREAKPOINTS 5
#endif

//Data structures needed by the TMCL interpreter
//! TMCL command
typedef struct __attribute__ ((packed))     //darf im EEPROM nur 7 Bytes belegen
{
  uint8_t Opcode;      //!< command opcode
  uint8_t Type;        //!< type parameter
  uint8_t Motor;       //!< motor/bank parameter
  union
  {
    int Int32;      //!< value parameter as 32 bit integer
    uint8_t Byte[4];   //!< value parameter as 4 bytes
  } Value;           //!< value parameter
} TTMCLCommand;

//! TMCL reply
typedef struct
{
  uint8_t Status;      //!< status code
  uint8_t Opcode;      //!< opcode of executed command
  union
  {
    int Int32;         //!< reply value as 32 bit integer
    uint8_t Byte[4];   //!< reply value as 4 bytes
  } Value;             //!< value parameter
} TTMCLReply;


//Prototypes of exported functions
void InitTMCL(void);
void ProcessTMCL(void);
