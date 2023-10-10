
#ifndef __MAIN_H
#define __MAIN_H

#include "macrodriver.h"
#include "PublicWare.h"
#include "userdefine.h"
#include "tau.h"
#include "sau.h"
#include "comppga.h"
#include "SubTask.h"
#include "tmrd.h"  
#include "MainTask.h"
#include "PID.h"
#include "r_cg_intp.h"
#include "r_cg_it.h"

//#include "GlobalVar.h"

#define  CLRWDT    WDTE=0xAC   //Î¹¹·

void GetMotorZeroCurrent(S_FILT_DATA_T *psFdata,S_MOTOR_T *psMotor); 
//void GetBatVol(S_FILT_DATA_T *psFdata,U_ERROR_FLAG_T *psflag);
//void GetBatVol(S_FILT_DATA_T *psFdata);


#endif


























