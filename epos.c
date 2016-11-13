/*! \file epos.c

\brief libEPOS - a library to control an EPOS 24/5

*/

/*! \mainpage libEPOS - a library to control an EPOS motor control unit

\b Because MAXON does not provide an EPOS driver for STM32, so I wrote this
driver. This driver is modified form Marcus Hauser's libEPOS and I ported it
to STM32. This driver thinks you are using CAN to connect the EPOS driver and
you uses the HAL version of the STM32 peripheral driver.

It based on the following maxon motor documents:
  - EPOS 2 Positioning Controller - Firmware specification (Edition October 2014)
  - EPOS 2 Positioning Controller - Communication Guide (Edition April 2013)

The only fully implemented and tested "Operation Mode" are the SDO
control method of "Profile Position Mode" and "Profile Velocity Mode", 
but adding support for other OpModes should be fairly easy, since 
the main work was implementing the data exchange with EPOS. The PDO 
control mode is highly configurable, so you have to configure the PDO 
in the driver first to use the PDO functions

I have only checked the library to work with an EPOS 2 24/5 (firmware
v2126h). Since I have no access to other hardware, I have no chance to
check other EPOS versions. But there is no hint at all that it should
NOT work with other EPOS variants.

\author Liyu Wang, SJTU
\date April 2016

*/

#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <errno.h>   /* Error number definitions */
#include <stdlib.h>
#include <stdint.h>  /* int types with given size */
#include <math.h>
#include "SEGGER_RTT.h"
#include "epos.h"
#include "main.h"



/* ********************************************* */
/*    definitions used only internal in epos.c   */
/* ********************************************* */


/*! starting point for (slow!) homing movement. If the zero point is
  not off too far, this will speed things up enormously!
*/
#define E_STARTPOS_HOMING -200000


//#define DEBUG

/* EPOS codes */

#define E_OK      0x4f  ///< EPOS answer code for <em>all fine</em>
#define E_FAIL    0x46  ///< EPOS answer code to indicate a <em>failure</em>
#define E_ANS     0x00  ///< EPOS code to indicate an answer <em>frame</em>


/* EPOS error codes (Communication Guide, 6.4)  */

/* CANopen defined error codes */
#define E_NOERR         0x00000000   ///< Error code: no error
#define E_TOGGLE        0x05030000   ///< Error code: Toggle bit not alternated
#define E_SDOTOUT       0x05040000   ///< Error code: SDO protocol timed out
#define E_CMDUKNOWN     0x05040001   ///< Error code: Client/server command specifier not valid or unknown
#define E_INVBLKSIZE    0x05040002   ///< Error code: Invalid block size (block mode only)
#define E_INVSEQ        0x05040003   ///< Error code: Invalid sequence number (block mode only)
#define E_CRCERR        0x05040004   ///< Error code: CRC error (block mode only)
#define E_OUTMEM        0x05040005   ///< Error code: out of memory
#define E_NOACCES       0x06010000   ///< Error code: Unsupported access to an object
#define E_WRITEONLY     0x06010001   ///< Error code: Attempt to read a write-only object
#define E_READONLY      0x06010002   ///< Error code: Attempt to write a read-only object
#define E_ONOTEX        0x06020000   ///< Error code: object does not exist
#define E_PDOMAP        0x06040041   ///< Error code: The object cannot be mapped to the PDO
#define E_PDOLEN        0x06040042   ///< Error code: The number and length of the objects to be would exceed PDO length
#define E_PARAMINCOMP   0x06040043   ///< Error code: general parameter incompatibility
#define E_INTINCOMP     0x06040047   ///< Error code: general internal incompatibility in the device
#define E_HWERR         0x06060000   ///< Error code: access failed due to an hardware error
#define E_SVCPAR        0x06070010   ///< Error code: Data type does not match, length or service does not match
#define E_SVCPARHI      0x06070012   ///< Error code: Data type does not match, length or service too high
#define E_SVCPARLO      0x06070013   ///< Error code: Data type does not match, length or service too low
#define E_SUBINEX       0x06090011   ///< Error code: Last read or write command had wrong object SubIndex
#define E_PRAGNEX       0x06090030   ///< Error code: value range of parameter exeeded
#define E_PARHIGH       0x06090031   ///< Error code: value of parameter written is too high
#define E_PARLOW        0x06090032   ///< Error code: value of parameter written is too low
#define E_PARREL        0x06090036   ///< Error code: maximum value is less than minimum value
#define E_GENERR        0x08000000   ///< Error code: General error
#define E_TFERSTORE     0x08000020   ///< Error code: Data cannot be transferred or stored
#define E_LOCALCTL      0x08000021   ///< Error code: Data cannot be transferred or stored to because of local control
#define E_DEVSTAT       0x08000022   ///< Error code: Data cannot be transferred or stored to because of the present device state

/* maxon specific error codes */
#define E_NMTSTATE      0x0F00FFC0   ///< Error code: wrong NMT state
#define E_RS232         0x0F00FFBF   ///< Error code: rs232 command illegeal
#define E_PASSWD        0x0F00FFBE   ///< Error code: password incorrect
#define E_NSERV         0x0F00FFBC   ///< Error code: device not in service mode
#define E_NODEID        0x0F00FFB9   ///< Error code: error in Node-ID

/* EPOS Device Error */
#define EP_NOERR 0x0000
#define EP_GENERR 0x1000
#define EP_OCERR 0x2310
#define EP_OVERR 0x3210
#define EP_UVERR 0x3220
#define EP_OTERR 0x4210
#define EP_SUPVOLLOW 0x5113
#define EP_OUTVOLLOW 0x5114
#define EP_INTSOFT 0x6100
#define EP_SOFTPAR 0x6320
#define EP_POSSENS 0x7320
#define EP_OBJLOST 0x8110
#define EP_CANOVRUN 0x8111
#define EP_CANPASS 0x8120
#define EP_HEARTBEAT 0x8130

/* EPOS Statusword -- singe bits, see firmware spec 14.1.58 */
#define E_BIT15        0x8000      ///< bit code: position referenced to home position
#define E_BIT14        0x4000      ///< bit code: refresh cycle of power stage
#define E_BIT13        0x2000      ///< bit code: OpMode specific, some error
#define E_BIT12        0x1000      ///< bit code: OpMode specific
#define E_BIT11        0x0800      ///< bit code: NOT USED
#define E_BIT10        0x0400      ///< bit code: Target reached
#define E_BIT09        0x0200      ///< bit code: Remote (?)
#define E_BIT08        0x0100      ///< bit code: offset current measured (?)
#define E_BIT07        0x0080      ///< bit code: WARNING
#define E_BIT06        0x0040      ///< bit code: switch on disable
#define E_BIT05        0x0020      ///< bit code: quick stop
#define E_BIT04        0x0010      ///< bit code: voltage enabled
#define E_BIT03        0x0008      ///< bit code: FAULT
#define E_BIT02        0x0004      ///< bit code: operation enable
#define E_BIT01        0x0002      ///< bit code: switched on
#define E_BIT00        0x0001      ///< bit code: ready to switch on

/* EPOS modes of operation, firmware spec 14.1.59 (p.133, tbl. 72) */
#define E_HOMING      6 ///< EPOS operation mode: homing
#define E_PROFVEL     3 ///< EPOS operation mode: profile velocity mode
#define E_PROFPOS     1 ///< EPOS operation mode: profile position mode
// the modes below should not be used by user, defined here only for
// completeness
#define E_POSMOD     -1 ///< EPOS operation mode: position mode
#define E_VELMOD     -2 ///< EPOS operation mode: velocity mode
#define E_CURRMOD    -3 ///< EPOS operation mode: current mode
#define E_DIAGMOD    -4 ///< EPOS operation mode: diagnostics mode
#define E_MASTERENCMOD -5 ///< EPOS operation mode:internal
#define E_STEPDIRECMOD -6 ///< EPOS operation mode:internal

/* Implement read functions defined in EPOS Communication Guide, 6.3.1 */

/*! \brief Read Object from EPOS memory, firmware definition 6.3.1.1*/
static int ReadObject(epos_t *epos, WORD index, BYTE subindex, DWORD *answer);


/* 6.3.2:  write functions */

/*! 6.3.2.1 WriteObject() 

   WORD *data is a pointer to a 2 WORDs array (== 4 BYTES) 
   holding data to transmit  
*/
static int WriteObject(epos_t *epos, WORD index, BYTE subindex, WORD param[]);


/* helper functions below */


/*! \brief  send command to EPOS, taking care of all neccessary 'ack' and
   checksum tests*/
static int sendCom(epos_t *epos);

/*! \brief  int readAnswer(WORD **ptr) - read an answer frame from EPOS */
static int readAnswer(epos_t *epos);

/*! \brief compare two 16bit bitmasks, return 1 (true) or 0 (false) */
static int bitcmp(WORD a, WORD b);

/* Global Varibles */
bool SDOBusy = false;
bool CAN_RxReady = false;
bool CAN_TxReady = false;
static bool isPDO = false;

CanRxMsgTypeDef CANMsgBuf[16];
uint8_t pCANMsg = 0;

/************************************************************/
/*           implementation of functions are following      */
/************************************************************/



/************************************************************/
/*            open/close device                             */
/************************************************************/



/*! establish the connection to EPOS

\param dev the handle to the CAN device
to, e.g. hcan1
\param ID the CAN ID of the EPOS device.

\retval 0 success 
\retval -1 failure

*/
epos_t* openEPOS(CAN_HandleTypeDef *dev, uint8_t ID) {
    epos_t *epos = NULL;

#ifndef __cplusplus
    if ((epos = malloc(sizeof(epos_t)))) {
#else
    if ((epos = (epos_t *)malloc(sizeof(epos_t)))) {
#endif
        epos->dev = dev;
        epos->dev->pTxMsg = &epos->TxMessage;
        epos->dev->pRxMsg = &epos->RxMessage;
        epos->Node_ID = ID;
        epos->PDO1RcvFlag = false;
        epos->PDO2RcvFlag = false;
        epos->PDO3RcvFlag = false;
        epos->PDO4RcvFlag = false;
        epos->SDORcvFlag = false;
        if(HAL_CAN_Receive_IT(epos->dev, CAN_FIFO0) != HAL_OK)
        {
          //Error Handler
        }
    }

    return epos;
}



int checkEPOS(epos_t *epos) {
    if (epos->dev < 0) {
        SEGGER_RTT_printf(0, "ERROR: EPOS device not open!");
        return (-1);
    }
    return (0);
}

/************************************************************/
/*          high-level read functions */
/************************************************************/


/*! read EPOS status word 

\param epos pointer on the EPOS object.
\param status pointer to WORD, the content of EPOS statusword will be placed there

\retval 0 success 
\retval -1 failure

*/
int readStatusword(epos_t *epos, WORD *status) {

    DWORD answer;
    int n = 0;

    if (!epos) return -1;

    if (checkEPOS(epos) != 0)
        return (-1);

    if ((n =  ReadObject(epos, 0x6041, 0x00, &answer)) < 0) {

        SEGGER_RTT_printf(0, " *** %s: ReadObject() returned %d **\n",
                __func__, n);
        return (-1);
    }

    // check error code
    checkEPOSerror(epos);

#ifdef DEBUG
    SEGGER_RTT_printf(0, "==> EPOS status word: %#06x\n", answer);
#endif
    *status = answer & 0xFFFF;

    return (0);
}

int read_DevErr(epos_t *epos, BYTE idx, WORD *err)
{

    DWORD answer;
    int n = 0;

    if (!epos) return -1;

    if (checkEPOS(epos) != 0)
        return (-1);

    if ((n =  ReadObject(epos, 0x1003, idx, &answer)) < 0) {

        SEGGER_RTT_printf(0, " *** %s: ReadObject() returned %d **\n",
                __func__, n);
        return (-1);
    }

    // check error code
    checkEPOSerror(epos);

#ifdef DEBUG
    SEGGER_RTT_printf(0, "==> EPOS error word: %#06x\n", answer);
#endif
    *err = answer & 0xFFFF;

    return (0);
}

/*! pretty-print Statusword to stdout

\param s WORD variable holding the statusword

*/
int printEPOSstatusword(WORD s) {

    SEGGER_RTT_printf(0, "\nmeaning of EPOS statusword %#06x is:\n", s);


    SEGGER_RTT_printf(0, "15: position referenced to home position: ");
    if ((s & E_BIT15) == E_BIT15) SEGGER_RTT_printf(0, "true\n");
    else SEGGER_RTT_printf(0, "false\n");

    SEGGER_RTT_printf(0, "14: refresh cycle of power stage:         ");
    if ((s & E_BIT14) == E_BIT14) SEGGER_RTT_printf(0, "true\n");
    else SEGGER_RTT_printf(0, "false\n");

    SEGGER_RTT_printf(0, "13: OpMode specific, some error:          ");
    if ((s & E_BIT13) == E_BIT13) SEGGER_RTT_printf(0, "true\n");
    else SEGGER_RTT_printf(0, "false\n");

    SEGGER_RTT_printf(0, "12: OpMode specific:                      ");
    if ((s & E_BIT12) == E_BIT12) SEGGER_RTT_printf(0, "true\n");
    else SEGGER_RTT_printf(0, "false\n");

    SEGGER_RTT_printf(0, "11: NOT USED                              ");
    if ((s & E_BIT11) == E_BIT11) SEGGER_RTT_printf(0, "true\n");
    else SEGGER_RTT_printf(0, "false\n");

    SEGGER_RTT_printf(0, "10: Target reached:                       ");
    if ((s & E_BIT10) == E_BIT10) SEGGER_RTT_printf(0, "true\n");
    else SEGGER_RTT_printf(0, "false\n");

    SEGGER_RTT_printf(0, "09: Remote (?)                            ");
    if ((s & E_BIT09) == E_BIT09) SEGGER_RTT_printf(0, "true\n");
    else SEGGER_RTT_printf(0, "false\n");

    SEGGER_RTT_printf(0, "08: offset current measured (?)           ");
    if ((s & E_BIT08) == E_BIT08) SEGGER_RTT_printf(0, "true\n");
    else SEGGER_RTT_printf(0, "false\n");

    SEGGER_RTT_printf(0, "07: WARNING                               ");
    if ((s & E_BIT07) == E_BIT07) SEGGER_RTT_printf(0, "true\n");
    else SEGGER_RTT_printf(0, "false\n");

    SEGGER_RTT_printf(0, "06: switch on disable                     ");
    if ((s & E_BIT06) == E_BIT06) SEGGER_RTT_printf(0, "true\n");
    else SEGGER_RTT_printf(0, "false\n");

    SEGGER_RTT_printf(0, "05: quick stop                            ");
    if ((s & E_BIT05) == E_BIT05) SEGGER_RTT_printf(0, "true\n");
    else SEGGER_RTT_printf(0, "false\n");

    SEGGER_RTT_printf(0, "04: voltage enabled                       ");
    if ((s & E_BIT04) == E_BIT04) SEGGER_RTT_printf(0, "true\n");
    else SEGGER_RTT_printf(0, "false\n");

    SEGGER_RTT_printf(0, "03: FAULT                                 ");
    if ((s & E_BIT03) == E_BIT03) SEGGER_RTT_printf(0, "true\n");
    else SEGGER_RTT_printf(0, "false\n");

    SEGGER_RTT_printf(0, "02: operation enable                      ");
    if ((s & E_BIT02) == E_BIT02) SEGGER_RTT_printf(0, "true\n");
    else SEGGER_RTT_printf(0, "false\n");

    SEGGER_RTT_printf(0, "01: switched on                           ");
    if ((s & E_BIT01) == E_BIT01) SEGGER_RTT_printf(0, "true\n");
    else SEGGER_RTT_printf(0, "false\n");

    SEGGER_RTT_printf(0, "00: ready to switch on                    ");
    if ((s & E_BIT00) == E_BIT00) SEGGER_RTT_printf(0, "true\n");
    else SEGGER_RTT_printf(0, "false\n");

    return (0);
}


/*! check EPOS state, firmware spec 8.1.1 

\param epos pointer on the EPOS object.
\return EPOS status as defined in firmware specification 8.1.1

*/
int checkEPOSstate(epos_t *epos) {

    WORD w = 0x0;
    int n;

    if (!epos) return -1;

    if ((n = readStatusword(epos, &w)) < 0) {
        SEGGER_RTT_printf(0, " *** %s: readStatusword() returned %d **\n",
                __func__, n);
        return (-1);
    }

    /* state 'start' (0)
         fedc ba98  7654 3210
    w == x0xx xxx0  x000 0000 */
    if (!bitcmp(w, E_BIT00) && !bitcmp(w, E_BIT01)
        && !bitcmp(w, E_BIT02) && !bitcmp(w, E_BIT03)
        && !bitcmp(w, E_BIT04) && !bitcmp(w, E_BIT05)
        && !bitcmp(w, E_BIT06) && !bitcmp(w, E_BIT08)
        && !bitcmp(w, E_BIT14)) return (0);

    /* state 'not ready to switch on' (1)
            fedc ba98  7654 3210
       w == x0xx xxx1  x000 0000 */
    if (!bitcmp(w, E_BIT00) && !bitcmp(w, E_BIT01)
        && !bitcmp(w, E_BIT02) && !bitcmp(w, E_BIT03)
        && !bitcmp(w, E_BIT04) && !bitcmp(w, E_BIT05)
        && !bitcmp(w, E_BIT06) && bitcmp(w, E_BIT08)
        && !bitcmp(w, E_BIT14)) return (1);


    /* state 'switch on disabled' (2)
            fedc ba98  7654 3210
       w == x0xx xxx1  x100 0000 */
    if (!bitcmp(w, E_BIT00) && !bitcmp(w, E_BIT01)
        && !bitcmp(w, E_BIT02) && !bitcmp(w, E_BIT03)
        && !bitcmp(w, E_BIT04) && !bitcmp(w, E_BIT05)
        && bitcmp(w, E_BIT06) && bitcmp(w, E_BIT08)
        && !bitcmp(w, E_BIT14)) return (2);

    /* state 'ready to switch on' (3)
            fedc ba98  7654 3210
       w == x0xx xxx1  x010 0001 */
    if (bitcmp(w, E_BIT00) && !bitcmp(w, E_BIT01)
        && !bitcmp(w, E_BIT02) && !bitcmp(w, E_BIT03)
        && !bitcmp(w, E_BIT04) && bitcmp(w, E_BIT05)
        && !bitcmp(w, E_BIT06) && bitcmp(w, E_BIT08)
        && !bitcmp(w, E_BIT14)) return (3);

    /* state 'switched on' (4)
            fedc ba98  7654 3210
       w == x0xx xxx1  x010 0011 */
    if (bitcmp(w, E_BIT00) && bitcmp(w, E_BIT01)
        && !bitcmp(w, E_BIT02) && !bitcmp(w, E_BIT03)
        && !bitcmp(w, E_BIT04) && bitcmp(w, E_BIT05)
        && !bitcmp(w, E_BIT06) && bitcmp(w, E_BIT08)
        && !bitcmp(w, E_BIT14)) return (4);

    /* state 'refresh' (5)
            fedc ba98  7654 3210
       w == x1xx xxx1  x010 0011 */
    if (bitcmp(w, E_BIT00) && bitcmp(w, E_BIT01)
        && !bitcmp(w, E_BIT02) && !bitcmp(w, E_BIT03)
        && !bitcmp(w, E_BIT04) && bitcmp(w, E_BIT05)
        && !bitcmp(w, E_BIT06) && bitcmp(w, E_BIT08)
        && bitcmp(w, E_BIT14)) return (5);

    /* state 'measure init' (6)
            fedc ba98  7654 3210
       w == x1xx xxx1  x011 0011 */
    if (bitcmp(w, E_BIT00) && bitcmp(w, E_BIT01)
        && !bitcmp(w, E_BIT02) && !bitcmp(w, E_BIT03)
        && bitcmp(w, E_BIT04) && bitcmp(w, E_BIT05)
        && !bitcmp(w, E_BIT06) && bitcmp(w, E_BIT08)
        && bitcmp(w, E_BIT14)) return (6);

    /* state 'operation enable' (7)
            fedc ba98  7654 3210
       w == x0xx xxx1  x011 0111 */
    if (bitcmp(w, E_BIT00) && bitcmp(w, E_BIT01)
        && bitcmp(w, E_BIT02) && !bitcmp(w, E_BIT03)
        && bitcmp(w, E_BIT04) && bitcmp(w, E_BIT05)
        && !bitcmp(w, E_BIT06) && bitcmp(w, E_BIT08)
        && !bitcmp(w, E_BIT14)) return (7);

    /* state 'quick stop active' (8)
            fedc ba98  7654 3210
       w == x0xx xxx1  x001 0111 */
    if (bitcmp(w, E_BIT00) && bitcmp(w, E_BIT01)
        && bitcmp(w, E_BIT02) && !bitcmp(w, E_BIT03)
        && bitcmp(w, E_BIT04) && !bitcmp(w, E_BIT05)
        && !bitcmp(w, E_BIT06) && bitcmp(w, E_BIT08)
        && !bitcmp(w, E_BIT14)) return (8);

    /* state 'fault reaction active (disabled)' (9)
            fedc ba98  7654 3210
       w == x0xx xxx1  x000 1111 */
    if (bitcmp(w, E_BIT00) && bitcmp(w, E_BIT01)
        && bitcmp(w, E_BIT02) && bitcmp(w, E_BIT03)
        && !bitcmp(w, E_BIT04) && !bitcmp(w, E_BIT05)
        && !bitcmp(w, E_BIT06) && bitcmp(w, E_BIT08)
        && !bitcmp(w, E_BIT14)) return (9);

    /* state 'fault reaction active (enabled)' (10)
            fedc ba98  7654 3210
       w == x0xx xxx1  x001 1111 */
    if (bitcmp(w, E_BIT00) && bitcmp(w, E_BIT01)
        && bitcmp(w, E_BIT02) && bitcmp(w, E_BIT03)
        && bitcmp(w, E_BIT04) && !bitcmp(w, E_BIT05)
        && !bitcmp(w, E_BIT06) && bitcmp(w, E_BIT08)
        && !bitcmp(w, E_BIT14)) return (10);

    /* state 'fault' (11)
            fedc ba98  7654 3210
       w == x0xx xxx1  x000 1000 */
    if (!bitcmp(w, E_BIT00) && !bitcmp(w, E_BIT01)
        && !bitcmp(w, E_BIT02) && bitcmp(w, E_BIT03)
        && !bitcmp(w, E_BIT04) && !bitcmp(w, E_BIT05)
        && !bitcmp(w, E_BIT06) && bitcmp(w, E_BIT08)
        && !bitcmp(w, E_BIT14)) return (11);


    // if we get down here, statusword has a unknown value!
    SEGGER_RTT_printf(0, "WARNING: EPOS status word %#06x is an unkown state!\n", w);
    SEGGER_RTT_printf(0, "(function %s() in file %s, line %d)\n",
            __func__, __FILE__, __LINE__);

    return (-2);
}


/* pretty-print EPOS state */
int printEPOSstate(epos_t *epos) {

    if (!epos) return -1;

    SEGGER_RTT_printf(0, "\nEPOS is in state ");

    switch (checkEPOSstate(epos)) {
    case 0:
        SEGGER_RTT_printf(0, "start\n"); break;
    case 1:
        SEGGER_RTT_printf(0, "Not ready to switch on.\n"); break;
    case 2:
        SEGGER_RTT_printf(0, "Switch on disabled.\n"); break;
    case 3:
        SEGGER_RTT_printf(0, "Ready to switch on.\n"); break;
    case 4:
        SEGGER_RTT_printf(0, "Switched on.\n"); break;
    case 5:
        SEGGER_RTT_printf(0, "Refresh.\n"); break;
    case 6:
        SEGGER_RTT_printf(0, "Measure init.\n"); break;
    case 7:
        SEGGER_RTT_printf(0, "Operation enable.\n"); break;
    case 8:
        SEGGER_RTT_printf(0, "Quick stop active\n"); break;
    case 9:
        SEGGER_RTT_printf(0, "Fault reaction active (disabled)\n"); break;
    case 10:
        SEGGER_RTT_printf(0, "Fault reaction active (enabled)\n"); break;
    case 11:
        SEGGER_RTT_printf(0, "FAULT\n"); break;

    default:
        SEGGER_RTT_printf(0, "UNKNOWN!\n");
        return (-1);
    }
    return (0);
}


/* change EPOS state according to firmware spec 8.1.3 */
int changeEPOSstate(epos_t *epos, int32_t state) {
    WORD dw[2];
    int n;

    if (!epos) return -1;

    dw[1] = 0x0000; // high WORD of DWORD is not used here

    /* ! DO NOT READ OLD CONTROLWORD BACK, JUST SET THE BITS. It works
       this way, but does NOT work otherways! -- mh, 07.07.06
    */

    dw[0] = 0x0000;

    switch (state) {
    case 0: //shutdown, controlword: 0xxx x110
        dw[0] &= ~E_BIT15;  // bit 15 ->0
        dw[0] |= E_BIT02;   // bit 02 ->1
        dw[0] |= E_BIT01;
        dw[0] &= ~E_BIT00;

        n = WriteObject(epos, 0x6040, 0x00, dw);
        if (n < 0) {
            SEGGER_RTT_printf(0, "%s: writeObject() returned %d at %s, line %d\n",
                    __func__, n, __FILE__, __LINE__);
            return (-1);
        }
        break;

    case 1: // switch on, controllword: 0xxx x111
        dw[0] &= ~E_BIT15;
        dw[0] |= E_BIT02;
        dw[0] |= E_BIT01;
        dw[0] |= E_BIT00;

        n = WriteObject(epos, 0x6040, 0x00, dw);
        if (n < 0) {
            SEGGER_RTT_printf(0, "%s: writeObject() returned %d at %s, line %d\n",
                    __func__, n, __FILE__, __LINE__);
            return (-1);
        }
        break;

    case 2: // disable voltage, controllword: 0xxx xx0x
        dw[0] &= ~E_BIT15;
        dw[0] &= ~E_BIT02;

        n = WriteObject(epos, 0x6040, 0x00, dw);
        if (n < 0) {
            SEGGER_RTT_printf(0, "%s: writeObject() returned %d at %s, line %d\n",
                    __func__, n, __FILE__, __LINE__);
            return (-1);
        }
        break;

    case 3: // quick stop, controllword: 0xxx x01x
        dw[0] &= ~E_BIT15;
        dw[0] &= ~E_BIT02;
        dw[0] |= E_BIT02;

        n = WriteObject(epos, 0x6040, 0x00, dw);
        if (n < 0) {
            SEGGER_RTT_printf(0, "%s: writeObject() returned %d at %s, line %d\n",
                    __func__, n, __FILE__, __LINE__);
            return (-1);
        }
        break;

    case 4: // disable operation, controllword: 0xxx 0111
        dw[0] &= ~E_BIT15;
        dw[0] &= ~E_BIT03;
        dw[0] |= E_BIT02;
        dw[0] |= E_BIT01;
        dw[0] |= E_BIT00;

        n = WriteObject(epos, 0x6040, 0x00, dw);
        if (n < 0) {
            SEGGER_RTT_printf(0, "%s: writeObject() returned %d at %s, line %d\n",
                    __func__, n, __FILE__, __LINE__);
            return (-1);
        }
        break;

    case 5: // enable operation, controllword: 0xxx 1111
        dw[0] &= ~E_BIT15;
        dw[0] |= E_BIT03;
        dw[0] |= E_BIT02;
        dw[0] |= E_BIT01;
        dw[0] |= E_BIT00;

        n = WriteObject(epos, 0x6040, 0x00, dw);
        if (n < 0) {
            SEGGER_RTT_printf(0, "%s: writeObject() returned %d at %s, line %d\n",
                    __func__, n, __FILE__, __LINE__);
            return (-1);
        }
        break;

    case 6: // fault reset, controllword: 1xxx xxxx

        //dw[0] |= E_BIT15; this is according to firmware spec 8.1.3,
        //but does not work!
        dw[0] |= E_BIT07; // this is according to firmware spec 14.1.57
                          // and IS working!


/*       WORD estatus = 0x0; */
/*       if ( ( n = readStatusword(&estatus) ) < 0) checkEPOSerror(); */
/*       printEPOSstatusword(estatus); */


        n = WriteObject(epos, 0x6040, 0x00, dw);
        if (n < 0) {
            SEGGER_RTT_printf(0, "%s: writeObject() returned %d at %s, line %d\n",
                    __func__, n, __FILE__, __LINE__);
            return (-1);
        }

/*       if ( ( n = readStatusword(&estatus) ) < 0) checkEPOSerror(); */
/*       printEPOSstatusword(estatus); */

        break;


    default:
        SEGGER_RTT_printf(0, "ERROR: demanded state %d is UNKNOWN!\n", state);
        return (-1);
    }
    return (0);
}

/* returns software version as HEX  --  14.1.33*/
uint16_t readSWversion(epos_t *epos) {
    DWORD answer;
    int n = 0;

    if (!epos) return -1;

    if (checkEPOS(epos) != 0) return (-1);

    if ((n =  ReadObject(epos, 0x2003, 0x01, &answer)) < 0) {

        SEGGER_RTT_printf(0, " *** %s: ReadObject() returned %d **\n",
                __func__, n);
        return (-1);
    }


    // check error code
    checkEPOSerror(epos);

    n =  answer&0xFFFF;
    return (n);
}


/* read digital input functionality polarity -- firmware spec 14.1.47 */
int readDInputPolarity(epos_t *epos, WORD *w) {

    DWORD answer;
    int n = 0;

    if (!epos) return -1;

    if (checkEPOS(epos) != 0) return (-1);

    if ((n =  ReadObject(epos, 0x2071, 0x03, &answer)) < 0) {

        SEGGER_RTT_printf(0, " *** %s: ReadObject() returned %d **\n",
                __func__, n);
        return (-1);
    }
    // check error code
    checkEPOSerror(epos);

    *w = answer&0xFFFF;
    return (0);
}





/* set home switch polarity -- firmware spec 14.1.47 */
int setHomePolarity(epos_t *epos, int32_t pol) {
    DWORD answer;
    WORD mask = 0x00;
    WORD dw[2] = { 0x0, 0x0 };
    int n = 0;

    if (!epos) return -1;

    if (pol != 0 && pol != 1) {
        SEGGER_RTT_printf(0, "ERROR: polarity must be 0 (hight active) or 1 (low active)\n");
        return (-1);
    }

    if (checkEPOS(epos) != 0) return (-1);

    // read present functionalities polarity mask
    if (readDInputPolarity(epos, &mask)) {
        SEGGER_RTT_printf(0, "\aERROR while reading digital input polarity!\n");
        return (-2);
    }


    // set bit 2 (==home switch) to 0 or 1:
    if (pol == 0)      mask &= ~E_BIT02;
    else if (pol == 1) mask |= E_BIT02;



    dw[1] = 0x0000; // high WORD of DWORD is not used here
    dw[0] = mask;

    n = WriteObject(epos, 0x2071, 0x03, dw);
    if (n < 0) {
        SEGGER_RTT_printf(0, "%s: writeObject() returned %d at %s, line %d\n",
                __func__, n, __FILE__, __LINE__);
        return (-1);
    }

    return (0);
}






/* read EPOS control word (firmware spec 14.1.57) */
int readControlword(epos_t *epos, WORD *w) {

    DWORD answer;
    int n = 0;

    if (!epos) return -1;

    if (checkEPOS(epos) != 0) return (-1);

    if ((n =  ReadObject(epos, 0x6040, 0x00, &answer)) < 0) {

        SEGGER_RTT_printf(0, " *** %s: ReadObject() returned %d **\n",
                __func__, n);
        return (-1);
    }

    // check error code
    checkEPOSerror(epos);

    *w = answer&0xFFFF;
    return (0);
}



/* pretty-print Controlword */
int printEPOScontrolword(WORD s) {
    SEGGER_RTT_printf(0, "\nmeaning of EPOS controlword %#06x is:\n", s);
    // bit 15..11 not in use
    // bit 10, 9 reserved
    SEGGER_RTT_printf(0, "  HALT:                                 ");
    if ((s & E_BIT08) == E_BIT08) SEGGER_RTT_printf(0, "true\n");
    else SEGGER_RTT_printf(0, "false\n");

    SEGGER_RTT_printf(0, "  fault reset                           ");
    if ((s & E_BIT07) == E_BIT07) SEGGER_RTT_printf(0, "true\n");
    else SEGGER_RTT_printf(0, "false\n");

    SEGGER_RTT_printf(0, "  Op mode specific                      ");
    if ((s & E_BIT06) == E_BIT06) SEGGER_RTT_printf(0, "true\n");
    else SEGGER_RTT_printf(0, "false\n");

    SEGGER_RTT_printf(0, "  Op mode specific                      ");
    if ((s & E_BIT05) == E_BIT05) SEGGER_RTT_printf(0, "true\n");
    else SEGGER_RTT_printf(0, "false\n");

    SEGGER_RTT_printf(0, "  Op mode specific                      ");
    if ((s & E_BIT04) == E_BIT04) SEGGER_RTT_printf(0, "true\n");
    else SEGGER_RTT_printf(0, "false\n");

    SEGGER_RTT_printf(0, "  enable operation                      ");
    if ((s & E_BIT03) == E_BIT03) SEGGER_RTT_printf(0, "true\n");
    else SEGGER_RTT_printf(0, "false\n");

    SEGGER_RTT_printf(0, "  quick stop                            ");
    if ((s & E_BIT02) == E_BIT02) SEGGER_RTT_printf(0, "true\n");
    else SEGGER_RTT_printf(0, "false\n");

    SEGGER_RTT_printf(0, "  enable voltage                        ");
    if ((s & E_BIT01) == E_BIT01) SEGGER_RTT_printf(0, "true\n");
    else SEGGER_RTT_printf(0, "false\n");

    SEGGER_RTT_printf(0, "  switch on                             ");
    if ((s & E_BIT00) == E_BIT00) SEGGER_RTT_printf(0, "true\n");
    else SEGGER_RTT_printf(0, "false\n");

    return (0);
}

/* set mode of operation --- 14.1.59 */
int setOpMode(epos_t *epos, int m) {

    WORD dw[2] = { 0x0, 0x0 };
    int n = 0;

    if (!epos) return -1;

    dw[1] = 0x0000; // high WORD of DWORD is not used here
    dw[0] = m;

    n = WriteObject(epos, 0x6060, 0x00, dw);
    if (n < 0) {
        SEGGER_RTT_printf(0, "%s: writeObject() returned %d at %s, line %d\n",
                __func__, n, __FILE__, __LINE__);
        return (-1);
    }


    return (0);
}

/** read mode of operation --- 14.1.60

\return RETURN(0) MEANS ERROR! -1 is a valid OpMode, but 0 is not!

 */
int readOpMode(epos_t *epos) {
    DWORD answer;
    //short int *i;
    int8_t aa;
    int n = 0;

    if (!epos) return -1;
    if ((n =  ReadObject(epos, 0x6061, 0x00, &answer)) < 0) {

        SEGGER_RTT_printf(0, " *** %s: ReadObject() returned %d **\n",
                __func__, n);
        return (0);
    }

    aa = answer&0xFF;

    // check error code
    checkEPOSerror(epos);

    // give warning, if internal mode is used
    if (aa < 0)
      SEGGER_RTT_printf(0, "WARNING: EPOS is set to internal mode of operation (%hd).\n Make sure that this was really intended!\n", aa);

    //return(*i);
    return (aa);
}




/* read demand position; 14.1.61 */
int readDemandPosition(epos_t *epos, int32_t *pos) {
    DWORD answer;
    int n = 0;

    if (!epos) return -1;

    if (checkEPOS(epos) != 0)
      return (-1);

    if ((n =  ReadObject(epos, 0x6062, 0x00, &answer)) < 0) {

        SEGGER_RTT_printf(0, " *** %s: ReadObject() returned %d **\n",
                __func__, n);
        return (-1);
    }
    // check error code
    checkEPOSerror(epos);

    // return value is a 32bit integer (==long int)
    *pos = answer;
#ifdef DEBUG
    SEGGER_RTT_printf(0, "==> EPOS actual position: %ld\n", *pos);
#endif
    return (0);
}




/*! read actual position; firmware description 14.1.62 

\retval 0 success
\retval <0 some error, check with checkEPOSerror()
*/
int readActualPosition(epos_t *epos, int32_t *pos) {
    DWORD answer;
    int n = 0;

    if (!epos) return -1;

    if (checkEPOS(epos) != 0) return (-1);

    if ((n =  ReadObject(epos, 0x6064, 0x00, &answer)) < 0) {

        SEGGER_RTT_printf(0, " *** %s: ReadObject() returned %d **\n",
                __func__, n);
        return (-1);
    }
    // check error code
    checkEPOSerror(epos);

    // return value is a 32bit integer (==long int)
    epos->RxPosition = answer;
    *pos = answer;
#ifdef DEBUG
    SEGGER_RTT_printf(0, "==> %s(): EPOS actual position: %ld\n", __func__, *pos);
#endif

    return (0);
}



/* read position window; 14.1.64 */
int readPositionWindow(epos_t *epos, uint32_t *pos) {
    DWORD answer;
    int n = 0;

    if (!epos) return -1;

    if (checkEPOS(epos) != 0) return (-1);

    if ((n =  ReadObject(epos, 0x6067, 0x00, &answer)) < 0) {

        SEGGER_RTT_printf(0, " *** %s: ReadObject() returned %d **\n",
                __func__, n);
        return (-1);
    }
    // check error code
    checkEPOSerror(epos);

    // return value is a 32bit integer (==long int)
    *pos = answer;

#ifdef DEBUG
    SEGGER_RTT_printf(0, "==> %s(): EPOS position window is %ld\n", __func__, *pos);
#endif

    return (0);
}


/* write  position window; 14.1.64 */
int writePositionWindow(epos_t *epos, uint32_t val) {

    WORD dw[2];
    int n = 0;

    if (!epos) return -1;

    // write intended position window
    dw[0] = (WORD)(val & 0x0000FFFF);
    dw[1] = (WORD)(val >> 16);

    n = WriteObject(epos, 0x6067, 0x00, dw);
    if (n < 0) {
        SEGGER_RTT_printf(0, "%s: writeObject() returned %d at %s, line %d\n",
                __func__, n, __FILE__, __LINE__);
        return (-1);
    }
    checkEPOSerror(epos);

    return (0);
}


/* read demand position; 14.1.67 */
int readDemandVelocity(epos_t *epos, int32_t *val) {
    DWORD answer;
    int n = 0;

    if (!epos) return -1;

    if (checkEPOS(epos) != 0) return (-1);

    if ((n =  ReadObject(epos, 0x606b, 0x00, &answer)) < 0) {

        SEGGER_RTT_printf(0, " *** %s: ReadObject() returned %d **\n",
                __func__, n);
        return (-1);
    }
    // check error code
    checkEPOSerror(epos);

    // return value is a 32bit integer (==long int)
    *val = answer;

#ifdef DEBUG
    SEGGER_RTT_printf(0, "==> EPOS demand velocity: %ld\n", *val);
#endif

    return (0);
}



/* read actual position; 14.1.68 */
int readActualVelocity(epos_t *epos, int32_t *val) {
    DWORD answer;
    int n = 0;

    if (!epos) return -1;

    if (checkEPOS(epos) != 0) return (-1);

    if ((n =  ReadObject(epos, 0x606c, 0x00, &answer)) < 0) {

        SEGGER_RTT_printf(0, " *** %s: ReadObject() returned %d **\n",
                __func__, n);
        return (-1);
    }
    // check error code
    checkEPOSerror(epos);

    // return value is a 32bit integer (==long int)
    epos->RxVelocity = answer;
    *val = answer;

#ifdef DEBUG
    SEGGER_RTT_printf(0, "==> EPOS actual velocity: %ld\n", *val);
#endif

    return (0);
}


/*! read actual motor current, see firmware description 14.1.69 

\param epos pointer on the EPOS object.
\param val pointer to short int where the actual motor current will be
placed.

\retval 0 success
\retval -1 error

*/
int readActualCurrent(epos_t *epos, short int *val) {
    DWORD answer;
    int n = 0;

    if (!epos) return -1;

    if (checkEPOS(epos) != 0) return (-1);

    if ((n =  ReadObject(epos, 0x6078, 0x00, &answer)) < 0) {

        SEGGER_RTT_printf(0, " *** %s: ReadObject() returned %d **\n",
                __func__, n);
        return (-1);
    }
    // check error code
    checkEPOSerror(epos);

    *val = answer&0xFF;
#ifdef DEBUG
    SEGGER_RTT_printf(0, "==> EPOS actual current: %dmA\n", *val);
#endif

    return (0);
}



/*!  read EPOS target position; firmware description 14.1.70 

\param epos pointer on the EPOS object.
\param val pointer to long int, will be filled with EPOS target position
\retval 0 success
\retval -1 error

*/

int readTargetPosition(epos_t *epos, int32_t *val) {
    DWORD answer;
    int n = 0;

    if (!epos) return -1;

    if (checkEPOS(epos) != 0) return (-1);

    if ((n =  ReadObject(epos, 0x607a, 0x00, &answer)) < 0) {

        SEGGER_RTT_printf(0, " *** %s: ReadObject() returned %d **\n",
                __func__, n);
        return (-1);
    }
    // check error code
    checkEPOSerror(epos);

    // return value is a 32bit integer (==long int)
    *val = answer;
#ifdef DEBUG
    SEGGER_RTT_printf(0, "==> EPOS target position: %ld\n", *val);
#endif

    return (0);
}



int setTargetVelocity(epos_t *epos, int32_t vel) {

    WORD dw[2] = { 0x0, 0x0 };
    int n = 0;

    dw[0] = (WORD)(vel & 0x0000FFFF);
    dw[1] = (WORD)(vel >> 16);

    n = WriteObject(epos, 0x60FF, 0x00, dw);
    if (n < 0) {
        SEGGER_RTT_printf(0, "%s: writeObject() returned %d at %s, line %d\n",
                __func__, n, __FILE__, __LINE__);
        return (-1);
    }

    return (0);
}

int setGPIOProfile(epos_t *epos, eposGPIO purpose, FlagStatus status)
{
    static WORD dw[2] = { 0x0, 0x0 };
    int n = 0;

    if(status == SET)
    {
      dw[0] |= purpose << 8;
    }
    else
    {
      dw[0] &= ~purpose << 8;
    }
    dw[1] = 0x00;

    n = WriteObject(epos, 0x2078, 0x01, dw);
    if (n < 0) {
        SEGGER_RTT_printf(0, "%s: writeObject() returned %d at %s, line %d\n",
                __func__, n, __FILE__, __LINE__);
        return (-1);
    }

    return (0);
}

int setProfileVelocity(epos_t *epos, uint32_t val)
{
    WORD dw[2] = { 0x0, 0x0 };
    int n = 0;

    dw[0] = (WORD)(val & 0x0000FFFF);
    dw[1] = (WORD)(val >> 16);

    n = WriteObject(epos, 0x6081, 0x00, dw);
    if (n < 0) {
        SEGGER_RTT_printf(0, "%s: writeObject() returned %d at %s, line %d\n",
                __func__, n, __FILE__, __LINE__);
        return (-1);
    }

    return (0);
}

int setProfileAcceleration(epos_t *epos, uint32_t val) {

    WORD dw[2] = { 0x0, 0x0 };
    int n = 0;

    dw[0] = (WORD)(val & 0x0000FFFF);
    dw[1] = (WORD)(val >> 16);

    n = WriteObject(epos, 0x6083, 0x00, dw);
    if (n < 0) {
        SEGGER_RTT_printf(0, "%s: writeObject() returned %d at %s, line %d\n",
                __func__, n, __FILE__, __LINE__);
        return (-1);
    }

    return (0);
}

int setProfileDeceleration(epos_t *epos, uint32_t val) {

    WORD dw[2] = { 0x0, 0x0 };
    int n = 0;

    dw[0] = (WORD)(val & 0x0000FFFF);
    dw[1] = (WORD)(val >> 16);

    n = WriteObject(epos, 0x6084, 0x00, dw);
    if (n < 0) {
        SEGGER_RTT_printf(0, "%s: writeObject() returned %d at %s, line %d\n",
                __func__, n, __FILE__, __LINE__);
        return (-1);
    }

    return (0);
}


int setMotionProfileType(epos_t *epos, uint16_t val) {

    WORD dw[2] = { 0x0, 0x0 };
    int n = 0;

    dw[1] = 0x0000; // high WORD of DWORD is not used here
    dw[0] = val;

    n = WriteObject(epos, 0x6086, 0x00, dw);
    if (n < 0) {
        SEGGER_RTT_printf(0, "%s: writeObject() returned %d at %s, line %d\n",
                __func__, n, __FILE__, __LINE__);
        return (-1);
    }

    return (0);
}


int setMaximalProfileVelocity(epos_t *epos, uint32_t val) {

    WORD dw[2] = { 0x0, 0x0 };
    int n = 0;

    dw[0] = (WORD)(val & 0x0000FFFF);
    dw[1] = (WORD)(val >> 16);

    n = WriteObject(epos, 0x607F, 0x00, dw);
    if (n < 0) {
        SEGGER_RTT_printf(0, "%s: writeObject() returned %d at %s, line %d\n",
                __func__, n, __FILE__, __LINE__);
        return (-1);
    }

    return (0);
}

int setQuickStopDeceleration(epos_t *epos, uint32_t val) {

    WORD dw[2] = { 0x0, 0x0 };
    int n = 0;

    dw[0] = (WORD)(val & 0x0000FFFF);
    dw[1] = (WORD)(val >> 16);

    n = WriteObject(epos, 0x6085, 0x00, dw);
    if (n < 0) {
        SEGGER_RTT_printf(0, "%s: writeObject() returned %d at %s, line %d\n",
                __func__, n, __FILE__, __LINE__);
        return (-1);
    }

    return (0);
}


int moveWithVelocity(epos_t *epos, int speed) {
    int n;
    n = setTargetVelocity(epos, speed);
    if (n != 0) return (n);
    return (startVelocityMovement(epos));
}

int startVelocityMovement(epos_t *epos) {

    WORD dw[2] = { 0x0, 0x0 };
    int n = 0;

    dw[1] = 0x0000; // high WORD of DWORD is not used here
    dw[0] = 0x000F; // see 14.59

    n = WriteObject(epos, 0x6040, 0x00, dw);
    if (n < 0) {
        SEGGER_RTT_printf(0, "%s: writeObject() returned %d at %s, line %d\n",
                __func__, n, __FILE__, __LINE__);
        return (-1);
    }

    return (0);
}


int haltVelocityMovement(epos_t *epos) {

    WORD dw[2] = { 0x0, 0x0 };
    int n = 0;

    dw[1] = 0x0000; // high WORD of DWORD is not used here
    dw[0] = 0x010F; // see 14.59

    n = WriteObject(epos, 0x6040, 0x00, dw);
    if (n < 0) {
        SEGGER_RTT_printf(0, "%s: writeObject() returned %d at %s, line %d\n",
                __func__, n, __FILE__, __LINE__);
        return (-1);
    }

    return (0);
}

/*!  read EPOS target velocity; 

\param val pointer to long int, will be filled with EPOS target velocity
\retval 0 success
\retval -1 error

*/
int readTargetVelocity(epos_t *epos, int32_t *val) {
    DWORD answer;
    int n = 0;

    if ((n = checkEPOS(epos)) < 0) return n;

    if ((n =  ReadObject(epos, 0x60FF, 0x00, &answer)) < 0) {

        SEGGER_RTT_printf(0, " *** %s: ReadObject() returned %d **\n",
                __func__, n);
        return (-1);
    }
    // check error code
    checkEPOSerror(epos);

    // return value is a 32bit integer (==long int)
    *val = answer;
#ifdef DEBUG
    SEGGER_RTT_printf(0, "==> EPOS target velocity: %ld\n", *val);
#endif

    return (0);
}




/*! readDeviceName: read manufactor device name string firmware

\param epos pointer on the EPOS object.
\param str previously allocated string, will be filled with device name
\retval 0 success
\retval -1 error


 */
int readDeviceName(epos_t *epos, char *str) {
    DWORD answer;
    int n = 0;

    if (!epos) return -1;

    if (checkEPOS(epos) != 0) return (-1);

    if ((n = ReadObject(epos, 0x1008, 0x00, &answer)) < 0) {
        SEGGER_RTT_printf(0, " *** readObject returned %d at %s, line %d ***\n",
               n, __func__, __LINE__);
    }



    str[0] = (answer & 0x00FF);
    str[1] = (answer & 0xFF00) >> 8;
    str[2] = (answer & 0xFF0000) >>16;
    str[3] = (answer & 0xFF000000) >> 24;
    str[4] = '\0';  // end of string

#ifdef DEBUG
    SEGGER_RTT_printf(0, "%s: %s \n", __func__, str);
#endif

    return (0);
}


/* firmware spec 14.1.35 */
int readRS232timeout(epos_t *epos) {

    DWORD answer;
    int n = 0;

    if (!epos) return -1;

    if (checkEPOS(epos) != 0) return (-1);

    if ((n = ReadObject(epos, 0x2005, 0x00, &answer)) < 0) {
        SEGGER_RTT_printf(0, " *** readObject returned %d at %s, line %d ***\n",
               n, __func__, __LINE__);
    }


    n =  (int)(answer&0xFF);
    return (n);
}


/* run the HomingMode, get the coordinate system zeropoint correct 

 this is done as shown in "EPOS Application Note: device Programming,
 3: Homing Mode"

*/
int doHoming(epos_t *epos, int32_t method, int32_t start) {

    WORD dw[2] = { 0x0000, 0x0000 };
    WORD w = 0x0000;
    int n, status = 0;

    if (!epos) return -1;

    //move motor to a pre-defined position before the reference
    //point. This will speed-up things if the coordinates are not too
    //wrong.

    if (moveAbsolute(epos, start)) {
        SEGGER_RTT_printf(0, "ERROR: could not move to homing starting point!\n");
        SEGGER_RTT_printf(0, "       (problem at %s; %s line %d)\n",
                __func__, __FILE__, __LINE__);
        return (-1);
    }
    // wait for positioning to finish, set timeout to approx. 30sec
    // CAUSES BIG PROBLEMS IF WE DO NOT WAIT!
    waitForTarget(epos, 30);
    //monitorStatus();


    // switch to homing mode
    if (setOpMode(epos, E_HOMING)) {
        SEGGER_RTT_printf(0, "ERROR: problem at %s; %s line %d\n",
                __func__, __FILE__, __LINE__);
        return (-1);
    }
    
    // homing speeds are left at default values.. (firmware 14.1.86)

    // set homing method
    dw[0] = method; // NO hex number here!
    dw[1] = 0x0000; // high WORD of DWORD is not used here
    n = WriteObject(epos, 0x6098, 0x00, dw);
    if (n < 0) {
        SEGGER_RTT_printf(0, "%s: writeObject() returned %d at %s, line %d\n",
                __func__, n, __FILE__, __LINE__);
        return (-1);
    }
    checkEPOSerror(epos);


    // switch on
    dw[0] = 0x000f;
    dw[1] = 0x0000; // high WORD of DWORD is not used here
    n = WriteObject(epos, 0x6040, 0x00, dw);
    if (n < 0) {
        SEGGER_RTT_printf(0, "%s: writeObject() returned %d at %s, line %d\n",
                __func__, n, __FILE__, __LINE__);
        return (-1);
    }
    // start homing mode
    dw[0] = 0x001f;
    dw[1] = 0x0000; // high WORD of DWORD is not used here
    n = WriteObject(epos, 0x6040, 0x00, dw);
    if (n < 0) {
        SEGGER_RTT_printf(0, "%s: writeObject() returned %d at %s, line %d\n",
                __func__, n, __FILE__, __LINE__);
        return (-1);
    }


    checkEPOSerror(epos);


    //read/print status
    status = monitorHomingStatus(epos);
    if (status) {
        // something was wrong during homing...
        if (status == 1) {
            SEGGER_RTT_printf(0, "We did more that 2 complete turns without finding the home switch!\n");
            SEGGER_RTT_printf(0, "\aDEVICE IS BROKEN!!!\n");
            return 2;
        } else {
            SEGGER_RTT_printf(0, "got %d as response from monitorHoming()...this is BAD!\n", status);
            SEGGER_RTT_printf(0, "[ %s: at %s, line %d ]\n",
                    __func__, __FILE__, __LINE__);
        }
    }

    readStatusword(epos, &w);
    if ((w & E_BIT13) == E_BIT13) {
        SEGGER_RTT_printf(0, "\a *** got a HomingError! ***\n");
        return (-1);
    }

    if ((w & E_BIT12) == E_BIT12) {
        SEGGER_RTT_printf(0, "homing finished!\n");
        return (0);
    } else {
        //  can this be reached? position finished, no homing error but
        //  homing NOT finished? I guess not..
        return (-5);
    }
}

int moveRelative(epos_t *epos, int32_t steps) {

    WORD dw[2];
    int n = 0;

    if (!epos) return -1;

    // check, if we are in Profile Position Mode
    if (readOpMode(epos) != E_PROFPOS) {
        if (setOpMode(epos, E_PROFPOS)) {
            SEGGER_RTT_printf(0, "ERROR: problem at %s; %s line %d\n",
                    __func__, __FILE__, __LINE__);
            return (-1);
        }
    }


    // write intended target position
    // firmware 14.1.70
    dw[0] = (WORD)(steps & 0x0000FFFF);
    dw[1] = (WORD)(steps >> 16);

    n = WriteObject(epos, 0x607A, 0x00, dw);
    if (n < 0) {
        SEGGER_RTT_printf(0, "%s: writeObject() returned %d at %s, line %d\n",
                __func__, n, __FILE__, __LINE__);
        return (-1);
    }
    checkEPOSerror(epos);

    // switch to relative positioning BY WRITING TO CONTROLWORD, finish
    // possible ongoing operation first!  ->maxon applicattion note:
    // device programming 2.1
    dw[0] = 0x005f;
    dw[1] = 0x0000; // high WORD of DWORD is not used here
    n = WriteObject(epos, 0x6040, 0x00, dw);
    if (n < 0) {
        SEGGER_RTT_printf(0, "%s: writeObject() returned %d at %s, line %d\n",
                __func__, n, __FILE__, __LINE__);
        return (-1);
    }
    checkEPOSerror(epos);
    return (0);
}



int moveAbsolute(epos_t *epos, int32_t steps) {

    WORD dw[2];
    int n = 0;

    if (!epos) return -1;

#ifdef DEBUG
    SEGGER_RTT_printf(0, "-> %s(): will move to %ld (%#010lx)\n", __func__, steps, steps);
#endif

    // check, if we are in Profile Position Mode
    if (readOpMode(epos) != E_PROFPOS) {
        if (setOpMode(epos, E_PROFPOS)) {
            SEGGER_RTT_printf(0, "ERROR: problem at %s; %s line %d\n",
                    __func__, __FILE__, __LINE__);
            return (-1);
        }
    }
#ifdef DEBUG
    SEGGER_RTT_printf(0, "-> OpMode is (now) 'Profile Position Mode'. That's OK!\n");
#endif


    // write intended target position, is signed 32bit int
    // firmware 14.1.70
    dw[0] = (WORD)(steps & 0x0000FFFF);
    dw[1] = (WORD)(steps >> 16);

#ifdef DEBUG
    SEGGER_RTT_printf(0, "-> %s(): dw[0,1] = %#06x  %#06x\n", __func__, dw[0], dw[1]);
#endif

    n = WriteObject(epos, 0x607A, 0x00, dw);
    if (n < 0) {
        SEGGER_RTT_printf(0, "%s: writeObject() returned %d at %s, line %d\n",
                __func__, n, __FILE__, __LINE__);
        return (-1);
    }
    checkEPOSerror(epos);

    // switch to absolute positioning, cancel possible ongoing operation
    // first!  ->maxon application note: device programming 2.1
    dw[0] = 0x3f;
    dw[1] = 0x0000; // high WORD of DWORD is not used here
    n = WriteObject(epos, 0x6040, 0x00, dw);
    if (n < 0) {
        SEGGER_RTT_printf(0, "%s: writeObject() returned %d at %s, line %d\n",
                __func__, n, __FILE__, __LINE__);
        return (-1);
    }
    checkEPOSerror(epos);


    return (0);
}


// monitor device status
int monitorStatus(epos_t *epos) {
    int  n;
    int32_t postarget, posactual, veldemand, velactual;
    int16_t curactual;
    WORD status;

    if (!epos) return -1;

    SEGGER_RTT_printf(0, "\nEPOS operating figures (note: update here is done AS FAST AS POSSIBLE!):\n");
    int i = 0;
    do {
        i++;
        if  ((n = readTargetPosition(epos, &postarget))) {
            SEGGER_RTT_printf(0, "ERROR while readActualPosition() [%d]\n", n);
            break;
        }
/*     if  ( (n=readDemandPosition( &posdemand ) ) ){ */
/*       SEGGER_RTT_printf(0, "ERROR while readDemandPosition() [%d]\n", n); */
/*       break; */
/*     } */
        if  ((n = readActualPosition(epos, &posactual))) {
            SEGGER_RTT_printf(0, "ERROR while readActualPosition() [%d]\n", n);
            break;
        }
        if  ((n = readDemandVelocity(epos, &veldemand))) {
            SEGGER_RTT_printf(0, "ERROR while readDemandVelocity() [%d]\n", n);
            break;
        }
        if  ((n = readActualVelocity(epos, &velactual))) {
            SEGGER_RTT_printf(0, "ERROR while readActualVelicity() [%d]\n", n);
            break;
        }
        if  ((n = readActualCurrent(epos, &curactual))) {
            SEGGER_RTT_printf(0, "ERROR while readActualCurrent() [%d]\n", n);
            break;
        }

        SEGGER_RTT_printf(0, "\rEPOS: pos=%+10ld |%+10ld (%ld to go); v= %+4ld | %+4ld[rpm]; I=%+4dmA",
               postarget, posactual, postarget - posactual,
               veldemand, velactual, curactual);
        fflush(stdout);

        readStatusword(epos, &status);
    } while ((status & E_BIT10) != E_BIT10); // bit 10 says: target reached!

    // update values a last time to get a nicer output:
    i++;
    if  ((n = readTargetPosition(epos, &postarget))) {
        SEGGER_RTT_printf(0, "ERROR while readActualPosition() [%d]\n", n);

    }
    if  ((n = readActualPosition(epos, &posactual))) {
        SEGGER_RTT_printf(0, "ERROR while readActualPosition() [%d]\n", n);
    }
    if  ((n = readDemandVelocity(epos, &veldemand))) {
        SEGGER_RTT_printf(0, "ERROR while readDemandVelocity() [%d]\n", n);
    }
    if  ((n = readActualVelocity(epos, &velactual))) {
        SEGGER_RTT_printf(0, "ERROR while readActualVelicity() [%d]\n", n);
    }
    if  ((n = readActualCurrent(epos, &curactual))) {
        SEGGER_RTT_printf(0, "ERROR while readActualCurrent() [%d]\n", n);
    }

    SEGGER_RTT_printf(0, "\r%d EPOS: pos=%+10ld |%+10ld (%ld to go); v= %+4ld | %+4ld[rpm]; I=%+4dmA\n",
           i, postarget, posactual, postarget - posactual,
           veldemand, velactual, curactual);
    SEGGER_RTT_printf(0, "target reached\n");

    return (0);
}


int monitorHomingStatus(epos_t *epos) {
    int  n;
    int32_t posactual, velactual;
    int16_t curactual;
    WORD status = 0x0;

    if (!epos) return -1;

    SEGGER_RTT_printf(0, "\nEPOS operating figures (note: update here is done AS FAST AS POSSIBLE!):\n");
    int i = 0;
    do {
        i++;
        if  ((n = readActualPosition(epos, &posactual))) {
            SEGGER_RTT_printf(0, "ERROR while readActualPosition() [%d]\n", n);
            break;
        }
        if  ((n = readActualVelocity(epos, &velactual))) {
            SEGGER_RTT_printf(0, "ERROR while readActualVelicity() [%d]\n", n);
            break;
        }
        if  ((n = readActualCurrent(epos, &curactual))) {
            SEGGER_RTT_printf(0, "ERROR while readActualCurrent() [%d]\n", n);
            break;
        }

        readStatusword(epos, &status);


        SEGGER_RTT_printf(0, "\r%d EPOS: pos=%+10ld; v =  %+4ldrpm I=%+3dmA status = %#06x ",
               i,  posactual, velactual, curactual, status);

        fflush(stdout);

        readStatusword(epos, &status);

        if ((status & E_BIT13) == E_BIT13) {
            SEGGER_RTT_printf(0, "\aHOMING ERROR!\n");
            return (-2);
        }

    } while (
             ((status & E_BIT10) != E_BIT10)
             && ((status & E_BIT12) != E_BIT12)
            );
    // bit 10 says: target reached!, bit 12: homing attained
    //printEPOSstatusword(status);

    i++;
    if  ((n = readActualPosition(epos, &posactual))) {
        SEGGER_RTT_printf(0, "ERROR while readActualPosition() [%d]\n", n);
    }
    if  ((n = readActualVelocity(epos, &velactual))) {
        SEGGER_RTT_printf(0, "ERROR while readActualVelicity() [%d]\n", n);
    }
    if  ((n = readActualCurrent(epos, &curactual))) {
        SEGGER_RTT_printf(0, "ERROR while readActualCurrent() [%d]\n", n);
    }

    readStatusword(epos, &status);


    SEGGER_RTT_printf(0, "\r%d EPOS: pos=%+10ld; v =  %+4ldrpm I=%+3dmA status = %#06x\n",
           i,  posactual, velactual, curactual, status);
    SEGGER_RTT_printf(0, "homing finished! Position should now be '0'\n");

    return (0);
}

/* waits for positoning to finish, argument is timeout in
   seconds. give timeout==0 to disable timeout */
int waitForTarget(epos_t *epos, unsigned int t) {

    WORD status;
    unsigned int i = 0;

    if (!epos) return -1;

    do {
        if (t != 0) { // use timeout?
            if (++i > t ) return (1);
        }
        HAL_Delay(50);
        readStatusword(epos, &status);
    } while ((status & E_BIT10) != E_BIT10); // bit 10 says: target reached!


    return (0);
}

int checkTarget(epos_t *epos)
{
  WORD status;
  if (!epos) return -1;
  readStatusword(epos, &status);
  if((status & E_BIT10) != E_BIT10)
  {
    return 0;
  }
  else
  {
    return 1;
  }
}


/* 
*************************************************************
            check EPOS error code
****************************************************************
*/

/* check the global variable E_error for EPOS error code */
int checkEPOSerror(epos_t *epos) {

    if (!epos) return -1;

    switch (epos->E_error) {
    case E_NOERR:
        return (0);
        break;
    case E_ONOTEX:
        SEGGER_RTT_printf(0, "EPOS responds with error: requested object does not exist!\n");
        break;
    case E_SUBINEX:
        SEGGER_RTT_printf(0, "EPOS responds with error: requested subindex does not exist!\n");
        break;
    case E_OUTMEM:
        SEGGER_RTT_printf(0, "EPOS responds with error: out of memory!\n");
        break;
    case E_NOACCES:
        SEGGER_RTT_printf(0, "EPOS responds with error: unsupported access to an object!\n");
        break;
    case E_WRITEONLY:
        SEGGER_RTT_printf(0, "EPOS responds with error: attempt to read a write-only object!\n");
        break;
    case E_READONLY:
        SEGGER_RTT_printf(0, "EPOS responds with error: attempt to write a read-only object!\n");
        break;
    case E_PARAMINCOMP:
        SEGGER_RTT_printf(0, "EPOS responds with error: general parameter incompatibility!\n");
        break;
    case E_INTINCOMP:
        SEGGER_RTT_printf(0, "EPOS responds with error: general internal incompatibility in the device!\n");
        break;
    case E_HWERR:
        SEGGER_RTT_printf(0, "EPOS responds with error: access failed due to an HARDWARE ERROR!\n");
        break;
    case E_PRAGNEX:
        SEGGER_RTT_printf(0, "EPOS responds with error: value range of parameter exeeded!\n");
        break;
    case E_PARHIGH:
        SEGGER_RTT_printf(0, "EPOS responds with error: value of parameter written is too high!\n");
        break;
    case E_PARLOW:
        SEGGER_RTT_printf(0, "EPOS responds with error: value of parameter written is too low!\n");
        break;
    case E_PARREL:
        SEGGER_RTT_printf(0, "EPOS responds with error: maximum value is less than minimum value!\n");
        break;
    case E_NMTSTATE:
        SEGGER_RTT_printf(0, "EPOS responds with error: wrong NMT state!\n");
        break;
    case E_RS232:
        SEGGER_RTT_printf(0, "EPOS responds with error: rs232 command illegeal!\n");
        break;
    case E_PASSWD:
        SEGGER_RTT_printf(0, "EPOS responds with error: password incorrect!\n");
        break;
    case E_NSERV:
        SEGGER_RTT_printf(0, "EPOS responds with error: device not in service mode!\n");
        break;
    case E_NODEID:
        SEGGER_RTT_printf(0, "EPOS responds with error: error in Node-ID!\n");
        break;
    default:
        SEGGER_RTT_printf(0, "EPOS responds with error: unknown EPOS error code: %u32\n",
                epos->E_error);
        break;
    }
    return (-1);
}



/*
****************************************************************
            basic I/O functions
****************************************************************
*/


/*  send command to EPOS, taking care of all neccessary 'ack' and
   checksum tests*/
static int sendCom(epos_t *epos) {

    if (!epos) return -1;



    /* sending to EPOS */
    if (HAL_CAN_Transmit_IT(epos->dev) != HAL_OK) {
        SEGGER_RTT_printf(0, "\nTransmit Error!\n");
        return -1;
    }
    while(CAN_TxReady != true)
    {
      HAL_Delay(5);
    }
#ifdef DEBUG
    short i;
    SEGGER_RTT_printf(0, "\n>> Sent Message ID: %04x\n", epos->dev->pTxMsg->StdId);
    SEGGER_RTT_printf(0, ">> ");
    for (i = 0; i < epos->dev->pTxMsg->DLC; ++i) {
        SEGGER_RTT_printf(0, "%02x ", epos->dev->pTxMsg->Data[i]);
    }
    SEGGER_RTT_printf(0, "\n");
#endif
    CAN_TxReady = false;
    return 1;
}



/*!  int readAnswer(WORD **ptr) - read an answer frame from EPOS

\param epos pointer on the EPOS object.
\param ptr WORD **ptr; pointer address where answer frame is placed. 
    
\retval >0 number of WORDs recieved from EPOS. ptr points now to
     answer frame.  
\retval <0 failure; ptr points to NULL. Global E_error is also set to
     returnd EPOS ErrorCode

 */
static int readAnswer(epos_t *epos) {
    if (!epos) return -1;

    epos->E_error = 0x00;

    while(epos->SDORcvFlag != true)
    {
      HAL_Delay(1);
    }
    epos->SDORcvFlag = false;
    
#ifdef DEBUG
    short i;
    SEGGER_RTT_printf(0, "\n<< Get SDO Message.\n");
    SEGGER_RTT_printf(0, "<< ");
    for (i = 0; i < epos->SDOMsg.DLC; i++) {
        SEGGER_RTT_printf(0, "%02x ", epos->SDOMsg.Data[i]);
    }
    SEGGER_RTT_printf(0, "\n");
#endif
    
    /* check for error code */
    if (epos->SDOMsg.Data[0] == 0x80) {
        epos->E_error = (((int32_t)(epos->SDOMsg.Data[7])) << 24) + (((int32_t)(epos->SDOMsg.Data[6])) << 16) + (((int32_t)(epos->SDOMsg.Data[5])) << 8) + (int32_t)(epos->SDOMsg.Data[4]);
    }
    return 1;
}


static int ReadObject(epos_t *epos, WORD Index, BYTE SubIndex, DWORD *param) {
    int n = 0;
    int ret = -1;

    if (!epos) return -1;
    SDOBusy = true;
    epos->TxMessage.StdId = 0x600 + epos->Node_ID;
    epos->TxMessage.RTR = CAN_RTR_DATA;
    epos->TxMessage.IDE = CAN_ID_STD;
    epos->TxMessage.DLC = 8;
    epos->TxMessage.Data[0] = 0x40;
    epos->TxMessage.Data[1] = Index&0xFF;
    epos->TxMessage.Data[2] = (Index&0xFF00)>>8;
    epos->TxMessage.Data[3]= SubIndex;
    epos->TxMessage.Data[4]=0x00;
    epos->TxMessage.Data[5]=0x00;
    epos->TxMessage.Data[6]=0x00;
    epos->TxMessage.Data[7]=0x00;

    if ((n = sendCom(epos)) < 0) {
        SEGGER_RTT_printf(0, " *** %s: problems with sendCom(), return value was %d ***\n ",  __func__, n);
        return (-1);
    }
    
    ret = readAnswer(epos);
    *param=(((int32_t)(epos->SDOMsg.Data[7]))<<24)+(((int32_t)(epos->SDOMsg.Data[6]))<<16)+(((int32_t)(epos->SDOMsg.Data[5]))<<8) + (int32_t)(epos->SDOMsg.Data[4]);
    SDOBusy = false;
    // read response
    return ret;
}


/*! Low-level function to write an object to EPOS memory. Is called by
 writing libEPOS functions.

\param epos pointer on the EPOS object.

\param index WORD describing EPOS memory index for writing. See
firmware documentation for valid values 

\param subindex BYTE describing EPOS memory subindex for writing. See
firmware documentation for valid values 

\param param pointer to WORD array holding the data to be written to EPOS memory

\retval 0 success
\retval -1 error
*/
int WriteObject(epos_t *epos, WORD Index, BYTE SubIndex, WORD param[]) {

    int n = 0;

    if (!epos) return -1;

    epos->TxMessage.StdId = 0x600 + epos->Node_ID;
    epos->TxMessage.RTR = CAN_RTR_DATA;
    epos->TxMessage.IDE = CAN_ID_STD;
    epos->TxMessage.DLC = 8;
    epos->TxMessage.Data[0] = 0x22;
    epos->TxMessage.Data[1] = Index&0xFF;
    epos->TxMessage.Data[2] = (Index&0xFF00)>>8;
    epos->TxMessage.Data[3]= SubIndex;
    epos->TxMessage.Data[4]=param[0]&0xFF;
    epos->TxMessage.Data[5]=(param[0]&0xFF00)>>8;
    epos->TxMessage.Data[6]=(param[1]&0xFF);
    epos->TxMessage.Data[7]=(param[1]&0xFF00)>>8;

    if ((n = sendCom(epos)) < 0) {
        SEGGER_RTT_printf(0, " *** %s: problems with sendCom(), return value was %d ***\n ",  __func__, n);
        return (-1);
    }

    if ((n = readAnswer(epos)) < 0) {
        SEGGER_RTT_printf(0, " *** %s: problems with readAnswer(), return value was %d ***\n ",  __func__, n);
        return (-1);
    }

    return (checkEPOSerror(epos));

}

/* compare WORD a with WORD b bitwise */
static int bitcmp(WORD a, WORD b) {
    if ((a & b) == b) return (1);
    else return (0);
}

int startPDO(epos_t *epos)
{
  int n = 0;
  if (!epos) return -1;
  epos->TxMessage.StdId = 0x0000;
  epos->TxMessage.RTR = CAN_RTR_DATA;
  epos->TxMessage.IDE = CAN_ID_STD;
  epos->TxMessage.DLC = 2;
  epos->TxMessage.Data[0] = 0x01;
  epos->TxMessage.Data[1] = epos->Node_ID;
  if ((n = sendCom(epos)) < 0) {
    SEGGER_RTT_printf(0, " *** %s: problems with sendCom(), return value was %d ***\n ",  __func__, n);
    return (-1);
  }
  isPDO = true;

  return 1;
}

int stopPDO(epos_t *epos)
{
  int n = 0;
  if (!epos) return -1;
  epos->TxMessage.StdId = 0x0000;
  epos->TxMessage.RTR = CAN_RTR_DATA;
  epos->TxMessage.IDE = CAN_ID_STD;
  epos->TxMessage.DLC = 2;
  epos->TxMessage.Data[0] = 0x80;
  epos->TxMessage.Data[1] = epos->Node_ID;
  if ((n = sendCom(epos)) < 0) {
    SEGGER_RTT_printf(0, " *** %s: problems with sendCom(), return value was %d ***\n ",  __func__, n);
    return (-1);
  }
  isPDO = false;
  return 1;
}

int PDOShutDown(epos_t *epos)
{
  int n = 0;
  if (!epos) return -1;
  epos->TxMessage.StdId = 0x200 + epos->Node_ID;
  epos->TxMessage.RTR = CAN_RTR_DATA;
  epos->TxMessage.IDE = CAN_ID_STD;
  epos->TxMessage.DLC = 2;
  epos->TxMessage.Data[0] = 0x06;
  epos->TxMessage.Data[1] = 0x00;
  if ((n = sendCom(epos)) < 0) {
    SEGGER_RTT_printf(0, " *** %s: problems with sendCom(), return value was %d ***\n ",  __func__, n);
    return (-1);
  }
  return 1;
}

int PDOSwitchOn(epos_t *epos)
{
  int n = 0;
  if (!epos) return -1;
  epos->TxMessage.StdId = 0x200 + epos->Node_ID;
  epos->TxMessage.RTR = CAN_RTR_DATA;
  epos->TxMessage.IDE = CAN_ID_STD;
  epos->TxMessage.DLC = 2;
  epos->TxMessage.Data[0] = 0x07;
  epos->TxMessage.Data[1] = 0x00;
  if ((n = sendCom(epos)) < 0) {
    SEGGER_RTT_printf(0, " *** %s: problems with sendCom(), return value was %d ***\n ",  __func__, n);
    return (-1);
  }
  return 1;
}

int PDOEnableOp(epos_t *epos)
{
  int n = 0;
  if (!epos) return -1;
  epos->TxMessage.StdId = 0x200 + epos->Node_ID;
  epos->TxMessage.RTR = CAN_RTR_DATA;
  epos->TxMessage.IDE = CAN_ID_STD;
  epos->TxMessage.DLC = 2;
  epos->TxMessage.Data[0] = 0x0F;
  epos->TxMessage.Data[1] = 0x00;
  if ((n = sendCom(epos)) < 0) {
    SEGGER_RTT_printf(0, " *** %s: problems with sendCom(), return value was %d ***\n ",  __func__, n);
    return (-1);
  }
  return 1;
}

int PDOSwitchProfile(epos_t *epos, Profile_t profile)
{
  int n = 0;
  if (!epos) return -1;
  epos->TxMessage.StdId = 0x300 + epos->Node_ID;
  epos->TxMessage.RTR = CAN_RTR_DATA;
  epos->TxMessage.IDE = CAN_ID_STD;
  epos->TxMessage.DLC = 3;
  epos->TxMessage.Data[0] = 0x0F;
  epos->TxMessage.Data[1] = 0x00;
  epos->TxMessage.Data[2] = profile;
  if ((n = sendCom(epos)) < 0) {
    SEGGER_RTT_printf(0, " *** %s: problems with sendCom(), return value was %d ***\n ",  __func__, n);
    return (-1);
  }
  epos->CurProfile = profile;
  return 1;
}

int PDOSetVelocity(epos_t *epos, int32_t velocity)
{
  int n = 0;
  if (!epos) return -1;
  epos->TxMessage.StdId = 0x500 + epos->Node_ID;
  epos->TxMessage.RTR = CAN_RTR_DATA;
  epos->TxMessage.IDE = CAN_ID_STD;
  epos->TxMessage.DLC = 6;
  epos->TxMessage.Data[0] = 0x0F;
  epos->TxMessage.Data[1] = 0x00;
  epos->TxMessage.Data[2] = (uint8_t)(velocity & 0xFF);
  epos->TxMessage.Data[3] = (uint8_t)((velocity>>8) & 0xFF);
  epos->TxMessage.Data[4] = (uint8_t)((velocity>>16) & 0xFF);
  epos->TxMessage.Data[5] = (uint8_t)((velocity>>24) & 0xFF);
  if ((n = sendCom(epos)) < 0) {
    SEGGER_RTT_printf(0, " *** %s: problems with sendCom(), return value was %d ***\n ",  __func__, n);
    return (-1);
  }
  epos->TxVelocity = velocity;
  return 1;
}

int PDOSetPosition(epos_t *epos, int32_t position)
{
  int n = 0;
  if (!epos) return -1;
  epos->TxMessage.StdId = 0x400 + epos->Node_ID;
  epos->TxMessage.RTR = CAN_RTR_DATA;
  epos->TxMessage.IDE = CAN_ID_STD;
  epos->TxMessage.DLC = 6;
  epos->TxMessage.Data[0] = 0x0F;
  epos->TxMessage.Data[1] = 0x00;
  epos->TxMessage.Data[2] = (uint8_t)(position & 0xFF);
  epos->TxMessage.Data[3] = (uint8_t)((position>>8) & 0xFF);
  epos->TxMessage.Data[4] = (uint8_t)((position>>16) & 0xFF);
  epos->TxMessage.Data[5] = (uint8_t)((position>>24) & 0xFF);
  if ((n = sendCom(epos)) < 0) {
    SEGGER_RTT_printf(0, " *** %s: problems with sendCom(), return value was %d ***\n ",  __func__, n);
    return (-1);
  }
  epos->TxPosition = position;
  return 1;
}

int PDOSetRelativePosition(epos_t *epos, int32_t position_r)
{
  int n = 0;
  if (!epos) return -1;
  int32_t position = epos->RxPosition + position_r;
  if(PDOSetPosition(epos, position) == 1)
    return 1;
  else
    return -1;
}

int processCANMsg(epos_t **epos, uint8_t num)
{
  while(pCANMsg > 0)
  {
    int i = 0;
    for(i = 0; i < num; i++)
    {
      if(!epos[i])
      {
        SEGGER_RTT_printf(0, "\nEPOS %d not initialized!\n", i);
        continue;
      }
      if(CANMsgBuf[pCANMsg-1].StdId == (epos[i]->Node_ID + 0x180))
      {
        epos[i]->PDO1Msg = CANMsgBuf[pCANMsg-1];
        epos[i]->PDO1RcvFlag = true;
        break;
      }
      else if(CANMsgBuf[pCANMsg-1].StdId == (epos[i]->Node_ID + 0x280))
      {
        epos[i]->PDO2Msg = CANMsgBuf[pCANMsg-1];
        epos[i]->PDO2RcvFlag = true;
        break;
      }
      else if(CANMsgBuf[pCANMsg-1].StdId == (epos[i]->Node_ID + 0x380))
      {
        epos[i]->PDO3Msg = CANMsgBuf[pCANMsg-1];
        epos[i]->PDO3RcvFlag = true;
        break;
      }
      else if(CANMsgBuf[pCANMsg-1].StdId == (epos[i]->Node_ID + 0x480))
      {
        epos[i]->PDO4Msg = CANMsgBuf[pCANMsg-1];
        epos[i]->PDO4RcvFlag = true;
        break;
      }
      else if(CANMsgBuf[pCANMsg-1].StdId == (epos[i]->Node_ID + 0x580))
      {
        epos[i]->SDOMsg = CANMsgBuf[pCANMsg-1];
        epos[i]->SDORcvFlag = true;
        break;
      }
      else if(CANMsgBuf[pCANMsg-1].StdId == (epos[i]->Node_ID + 0x80))
      {
        epos[i]->Dev_Err = (((uint16_t)(CANMsgBuf[pCANMsg-1].Data[1])) << 8) + (uint16_t)(CANMsgBuf[pCANMsg-1].Data[0]);
        epos[i]->ErrFlag = true;
        break;
      }
    }
    if( i == num)
    {
      SEGGER_RTT_printf(0, "\nMessage id: %04x cannot be process!\n", CANMsgBuf[pCANMsg-1].StdId);
    }
    pCANMsg --;
  }
  processPDOMessage(epos, num);
  return 1;
}

int processPDOMessage(epos_t **epos, uint8_t num)
{
  for(int i = 0; i < num; i++)
  {
    if(epos[i]->PDO3RcvFlag == true)
    {
      epos[i]->RxPosition = (((int32_t)(epos[i]->PDO3Msg.Data[5]))<<24)+(((int32_t)(epos[i]->PDO3Msg.Data[4]))<<16)+(((int32_t)(epos[i]->PDO3Msg.Data[3]))<<8)+ (int32_t)(epos[i]->PDO3Msg.Data[2]);
      epos[i]->PDO3RcvFlag = false;
    }
    if(epos[i]->PDO4RcvFlag == true)
    {
      epos[i]->RxVelocity = (((int32_t)(epos[i]->PDO4Msg.Data[5]))<<24)+(((int32_t)(epos[i]->PDO4Msg.Data[4]))<<16)+(((int32_t)(epos[i]->PDO4Msg.Data[3]))<<8)+ (int32_t)(epos[i]->PDO4Msg.Data[2]);
      epos[i]->PDO4RcvFlag = false;
    }
  }
  return 1;
}

/**
  * @brief  Transmission  complete callback in non blocking mode 
  * @param  hcan: pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_TxCpltCallback(CAN_HandleTypeDef* hcan)
{
  CAN_TxReady = true;
}

/**
  * @brief  Transmission  complete callback in non blocking mode 
  * @param  hcan: pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan)
{
//  CANMsgBuf[pCANMsg] = *(hcan->pRxMsg);
  memcpy(&CANMsgBuf[pCANMsg], hcan->pRxMsg, sizeof(CanRxMsgTypeDef));
  
#ifdef DEBUG
  SEGGER_RTT_printf(0, "\n<< Message id: %04x received!\n", CANMsgBuf[pCANMsg].StdId);
  short i;
  SEGGER_RTT_printf(0, "<< ");
  for (i = 0; i < CANMsgBuf[pCANMsg].DLC; ++i) {
    SEGGER_RTT_printf(0, "%02x ", CANMsgBuf[pCANMsg].Data[i]);
  }
  SEGGER_RTT_printf(0, "\n");
#endif
  pCANMsg ++;
  CAN_RxReady = true;
  processCANMsg(epos, EPOS_NUM);
  if(HAL_CAN_Receive_IT(hcan, CAN_FIFO0) != HAL_OK)
  {
    //Error Handler
  }
}
