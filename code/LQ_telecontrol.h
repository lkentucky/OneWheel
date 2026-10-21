#ifndef _LQ_TELECONTROL_H_
#define _LQ_TELECONTROL_H_

#include "zf_common_headfile.h"

extern char KEY_Value,DSWA,DSWB,DSWC,DSWD;

extern int L_Around, L_UpDown;
extern int R_Around, R_UpDown;
extern char buf1[32];
extern unsigned char ReadBuff0[512];
extern char Usart1_Rec_Fini_Flag;
void Telecontrol(void);
void Telecontrol_Test(void);
void Uart_Read(void);

#endif
