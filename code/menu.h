/* menu.h：菜单接口。创建日期：2025年1月11日；上游作者姓名已损坏。 */
#ifndef CODE_MENU_H_
#define CODE_MENU_H_
#include "zf_common_headfile.h"

#define mt9v034_test_mode 1

void monotroch_init(void);
void menu(int menu_key);
void Menu_Create(void);
void Menu_Init(void);
void Menu_Show(void);
void Menu_Switch(void);

extern char txt[256];
extern volatile uint8 wifi_spi_flag;

#endif /* CODE_MENU_H_ */
