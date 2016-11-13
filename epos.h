/*! \file epos.h

  header file for libEPOS functions

  Liyu Wang, April 2016

*/

#ifndef _EPOS_H
#define _EPOS_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"


/* all EPOS data exchange is based on 16bit words, but other types are
   also used...*/
//typedef unsigned long DWORD ; ///< \brief 32bit type for EPOS data exchange
//typedef unsigned short WORD ; ///< \brief 16bit type for EPOS data exchange
typedef uint32_t DWORD ; ///< \brief 32bit type for EPOS data exchange
typedef uint16_t WORD ; ///< \brief 16bit type for EPOS data exchange
#ifndef CPP
typedef uint8_t BYTE ; ///< \brief 8bit type for EPOS data exchange
#endif

typedef enum Profile_s{
  PPM = 0x01, //Profile Position Mode
  PVM = 0x03, //Profile Velocity Mode
  PVT = 0x07, //Interpolated Position Mode
  PM = 0xFF,  //Posotion Mode
  VM = 0xFE,  //Velocity Mode
  CM = 0xFD,  //Current Mode
  HM = 0x06  //Homing Mode
}Profile_t;


typedef struct epos_s {
  CAN_HandleTypeDef *dev;
  uint8_t Node_ID;
  CanTxMsgTypeDef TxMessage;
  CanRxMsgTypeDef RxMessage;
  CanRxMsgTypeDef SDOMsg;
  bool ErrFlag;
  uint16_t Dev_Err;
  bool SDORcvFlag;
  CanRxMsgTypeDef PDO1Msg;
  bool PDO1RcvFlag;
  CanRxMsgTypeDef PDO2Msg;
  bool PDO2RcvFlag;
  CanRxMsgTypeDef PDO3Msg;
  bool PDO3RcvFlag;
  CanRxMsgTypeDef PDO4Msg;
  bool PDO4RcvFlag;
  uint8_t CurProfile;
  int32_t TxPosition;
  int32_t RxPosition;
  int32_t TxVelocity;
  int32_t RxVelocity;
  uint32_t E_error;    ///< EPOS global error status
} epos_t;


typedef enum eposGPIO_s{
  PurposeA = 0x80,
  PurposeB = 0x40,
  PurposeC = 0x20,
  PurposeD = 0x10,
  PurposeE = 0x08,
  PurposeF = 0x04,
  PurposeG = 0x02,
  PurposeH = 0x01
}eposGPIO;

/* EPOS will reset communication after 500ms of inactivity */

/*! \brief try NTRY times to read one byte from EPOS, the give up */
#define NTRY      5
/*! \brief sleep TRYSLEEP usec between read() from EPOS, if no data available */
//#define TRYSLEEP  (unsigned int)1e5 
#define TRYSLEEP  (unsigned int)1e4 


/* all high-level functions return <0 in case of error */

/*! create new EPOS object */
epos_t *newEPOS(CAN_HandleTypeDef *device);
/*! delete EPOS object */
int deleteEPOS(epos_t *epos);


/*! open the connection to EPOS */
epos_t *openEPOS(CAN_HandleTypeDef *dev, uint8_t ID);
/*! close the connection to EPOS */
int closeEPOS(epos_t *epos);
/*! check if the connection to EPOS is alive */
int checkEPOS(epos_t *epos);

int startPDO(epos_t *epos);

int stopPDO(epos_t *epos);

int PDOShutDown(epos_t *epos);

int PDOSwitchOn(epos_t *epos);

int PDOEnableOp(epos_t *epos);

int PDOSwitchProfile(epos_t *epos, Profile_t profile);
int PDOSetVelocity(epos_t *epos, int32_t velocity);
int PDOSetPosition(epos_t *epos, int32_t position);
int PDOSetRelativePosition(epos_t *epos, int32_t position_r);

int processCANMsg(epos_t **epos, uint8_t num);
int processPDOMessage(epos_t **epos, uint8_t num);


/*! \brief check global variable E_error for EPOS error code */
int checkEPOSerror(epos_t *epos);

/*! \brief check EPOS status, return state according to firmware spec 8.1.1 */
int checkEPOSstate(epos_t *epos);
/*! \brief pretty-print EPOS state */
int printEPOSstate(epos_t *epos);
/*! \brief change EPOS state   ==> firmware spec 8.1.3 */
int changeEPOSstate(epos_t *epos, int state);

int checkTarget(epos_t *epos);

/*! \brief example from EPOS com. guide: ask EPOS for software version 

firmware spec 14.1.33
** returns software version as HEX **

*/
uint16_t readSWversion(epos_t *epos);


/*! \brief ask for device name,  
   device name is placed in 'name' (string must be big enough, NO CHECKING!!)
*/
int readDeviceName(epos_t *epos, char *name);


/*! \brief ask for RS232 timeout; firmware spec 14.1.35 */
int readRS232timeout(epos_t *epos);

/*! \brief read digital input polarity mask */
int readDInputPolarity(epos_t *epos, WORD* w);

/*! \brief set home switch polarity -- firmware spec 14.1.47 */
int setHomePolarity(epos_t *epos, int pol);



/*! \brief read Statusword; 14.1.58 */
int readStatusword(epos_t *epos, WORD *eposStatus);
/*! \brief pretty-print Statusword */
int printEPOSstatusword(WORD statusword);

int read_DevErr(epos_t *epos, BYTE idx, WORD *err);

/*! \brief read EPOS control word (firmware spec 14.1.57) */
int readControlword(epos_t *epos, WORD *w);
/*! \brief pretty-print Controlword */
int printEPOScontrolword(WORD controlword);


/*! \brief set EPOS mode of operation -- 14.1.59 */
int setOpMode(epos_t *epos, int32_t OpMode);

/*! \brief read and returns  EPOS mode of operation -- 14.1.60 
here, RETURN(0) MEANS ERROR! 
'-1' is a valid OpMode, but 0 is not!
*/
int readOpMode(epos_t *epos);


/*! \brief read actual position; 14.1.61 */
int readDemandPosition(epos_t *epos, int32_t *val);
/*! \brief read actual position; 14.1.62 */
int readActualPosition(epos_t *epos, int32_t *val);

/*! \brief read position window; 14.1.64 */
int readPositionWindow(epos_t *epos, uint32_t *value);
/*! \brief write position window; 14.1.64 */
int writePositionWindow(epos_t *epos, uint32_t value);

/*! \brief read actual position; 14.1.67 */
int readDemandVelocity(epos_t *epos, int32_t *val);
/*! \brief read actual position; 14.1.68 */
int readActualVelocity(epos_t *epos, int32_t *val);

//FIXME doc
int setTargetVelocity(epos_t *epos, int32_t m);
int readTargetVelocity(epos_t *epos, int32_t *OUTPUT);
int moveWithVelocity(epos_t *epos, int32_t m);
int haltVelocityMovement(epos_t *epos );
int startVelocityMovement(epos_t *epos );
int setProfileVelocity(epos_t *epos, uint32_t val);
int setProfileAcceleration(epos_t *epos, uint32_t val);
int setProfileDeceleration(epos_t *epos, uint32_t val);
int setMotionProfileType(epos_t *epos, uint16_t val);
int setMaximalProfileVelocity(epos_t *epos, uint32_t val);
int setQuickStopDeceleration(epos_t *epos, uint32_t val);
int setGPIOProfile(epos_t *epos, eposGPIO port, FlagStatus stat);



/*! \brief read actual current; 14.1.69 */
int readActualCurrent(epos_t *epos, int16_t *val);

/*! \brief read target position; 14.1.70 */
int readTargetPosition(epos_t *epos, int32_t *val);



/*! \brief does a homing move. Give homing mode (see firmware 9.3) and start
   position */
int doHoming(epos_t *epos, int32_t method, int32_t start);


/*! \brief set OpMode to ProfilePosition and make relative movement */
int moveRelative(epos_t *epos, int32_t steps);
/*! \brief set OpMode to ProfilePosition and make absolute movement */
int moveAbsolute(epos_t *epos, int32_t steps);

/*! \brief reads position, velocity and current and displays them in an
   endless loop. Returns after target position has been reached */
int monitorStatus(epos_t *epos);
/*! \brief as monitorStatus(), but also waits for Homing Attained' signal */
int monitorHomingStatus(epos_t *epos);

/*! \brief waits for positoning to finish, argument is timeout in
   seconds. give timeout==0 to disable timeout */
int waitForTarget(epos_t *epos, uint32_t t);


#endif
