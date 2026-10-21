#include "LQ_telecontrol.h"


char KEY_Value = 0,DSWA = 0,DSWB = 0,DSWC = 0,DSWD = 0;

int L_Around = 0, L_UpDown = 0;
int R_Around = 0, R_UpDown = 0;

char buf1[32];
char TXT[50];

unsigned char ReadBuff0[512];
char Usart1_Rec_Fini_Flag=0;
void Telecontrol(void)
{
    char *start;
    char numbers[9][10];
    uint8_t count = 0;

    if((start =strstr((const char *)ReadBuff0,"A6A8K:"))!= NULL)
    {
         start +=6;
         while (count < 9)
         {
             if (*start == '-' || (*start >= '0' && *start <= '9'))
             {
                 int i = 0;
                 while ((*start >= '0' && *start <= '9') || *start == '-')
                 {
                     numbers[count][i++] = *start++;
                 }
                 numbers[count][i] = '\0';
                 count++;
             }
             else
             {
                 start++;
             }
         }
         KEY_Value = (char)atoi(numbers[0]);
         DSWA = (char)atoi(numbers[1]);
         DSWB = (char)atoi(numbers[2]);
         DSWC = (char)atoi(numbers[3]);
         DSWD = (char)atoi(numbers[4]);
         L_Around = (int)atoi(numbers[5]);
         L_UpDown = (int)atoi(numbers[6]);
         R_Around = (int)atoi(numbers[7]);
         R_UpDown = (int)atoi(numbers[8]);

//         printf("K%d,A%d,B%d,C%d,D%d,L%d,R%d\r\n",KEY_Value,DSWA,DSWB,DSWC,DSWD,L_Around,R_Around);

    }
    else
    {
//        printf("帧头未找到！\r\n");
    }
}

void Telecontrol_Test(void)
{
//    TFTSPI_Init(0);        //LCD初始化  0:横屏  1：竖屏
//    TFTSPI_CLS(u16BLACK);   //蓝色屏幕
//    TFTSPI_Show_Logo(0,37);//显示龙邱LOGO
//    TFTSPI_P16x16Str(0,0,(unsigned char*)"北京龙邱智能科技",u16RED,u16BLUE);      //字符串显示
//    TFTSPI_P8X16Str(0,1,"Long Qiu i.s.t.",u16WHITE,u16BLACK);     //字符串显示
//    UART_InitConfig(UART0_RX_P14_1, UART0_TX_P14_0, 115200);
//    delayms(1000);         //延时等待
//    TFTSPI_CLS(u16BLACK);   //蓝色屏幕
//    while(1)
//    {
//        if(Usart1_Rec_Fini_Flag == 1)
//        {
//            Usart1_Rec_Fini_Flag = 0;
//            Telecontrol();
//        }
//        sprintf(TXT, "K:%1d", KEY_Value);
//        TFTSPI_P8X16Str(1, 1, TXT, u16RED, u16BLUE);
//
//
//        sprintf(TXT, "L:%4d R:%4d", L_UpDown,R_UpDown);
//        TFTSPI_P8X16Str(1, 3, TXT, u16RED, u16BLUE);
//        sprintf(TXT, "L:%4d R:%4d", L_Around,R_Around);
//        TFTSPI_P8X16Str(1, 5, TXT, u16RED, u16BLUE);
//
//        sprintf(TXT, "A:%1d B:%1d C:%1d D:%1d", DSWA,DSWB,DSWC,DSWD);
//        TFTSPI_P8X16Str(1, 7, TXT, u16RED, u16BLUE);
//
////        printf("K%d,A%d,B%d,C%d,D%d,L%d,R%d\r\n",KEY_Value,DSWA,DSWB,DSWC,DSWD,L_Around,R_Around);
//
//        system_delay_ms(100);  // 延时100毫秒
//    }
}

void Uart_Read(void)
{
    static int num0;
    unsigned char buf0;
    buf0 = uart_read_byte(UART_3);
    if(num0 ==0 && buf0 == 'A')
    {
        ReadBuff0[num0] = buf0;
        num0++;
    }
    else if(num0 == 1 && buf0 == '6')
    {
        ReadBuff0[num0] = buf0;
        num0++;
    }
    else if(num0 == 2 && buf0 == 'A')
    {
        ReadBuff0[num0] = buf0;
        num0++;
    }
    else if(num0 == 3 && buf0 == '8')
    {
        ReadBuff0[num0] = buf0;
        num0++;
    }
    else if(num0 > 3 && ReadBuff0[0] == 'A' && ReadBuff0[1] == '6' && ReadBuff0[2] == 'A' && ReadBuff0[3] == '8' && num0 < 70)
    {
        ReadBuff0[num0] = buf0;
        num0++;
        if(ReadBuff0[num0 -1] == '9' && ReadBuff0[num0-2] == 'C' && ReadBuff0[num0-3] == '9' && ReadBuff0[num0-4] == 'B')
        {
//            printf("%d\r\n",num0);
            num0 = 0;
            Usart1_Rec_Fini_Flag = 1;
        }
    }
    else    //不满足，舍弃一帧数据
    {
        num0 = 0;
    }

}





