#ifndef CODE_MENU_H
#define CODE_MENU_H
#include "zf_common_headfile.h"
void menu_init(void);
void menu_hardware_init(void);
void menu_poll(void);
bool menu_imu_recovery_stopped(void);
void Menu_Create(void);
void Menu_Init(void);
void Menu_Show(void);
void Menu_Switch(void);
extern char txt[256];
#endif
