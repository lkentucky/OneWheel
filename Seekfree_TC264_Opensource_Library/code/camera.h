/*
 * camera.h
 *
 *  Created on: 2025年4月19日
 *      Author: lenovo
 */

#ifndef CODE_CAMERA_H_
#define CODE_CAMERA_H_
#include "zf_common_headfile.h"
#define IMG_BLACK     0X00      //0x00是黑
#define IMG_WHITE     0Xff      //0xff为白
#define IR_THRESHOLD       200
#define MIN_BEACON_AREA    20
#define CENTER_THRESHOLD_X (MT9V03X_W / 10)
#define CENTER_THRESHOLD_Y (MT9V03X_H / 10)

#define GrayScale 256
#define grayscale 256
#define guding_erro 2000

extern float erro;
extern float erro_yawan;
extern float error_back;
extern float erro_yawan_back;
extern float last_erro;
extern uint8 img_threshold;
extern uint8    image_binary[MT9V03X_H][MT9V03X_W];
typedef enum {
    BEACON_NOT_FOUND = 0,
    BEACON_FOUND,
    BEACON_CENTERED
} BeaconStatus;

// 信标信息结构体
typedef struct {
    BeaconStatus status;
    int centerX;
    int centerY;
    int length;
    int length_last;
    uint16_t area;
    int last_centerX;
    int last_centerY;
    int sum_last;
    int sum;
    int quanzhong;
    int last_quanzhong;
} BeaconInfo;

extern BeaconInfo beacon;
uint8 Ostu(uint8 index[MT9V03X_H][MT9V03X_W]);
void detect_beacon(uint8 image[MT9V03X_H][MT9V03X_W], BeaconInfo *info) ;
void Image_Binarization(int threshold);//图像二值化;
float retarder(float input,float unit);
float en_retarder(float input,float unit);
int  speed_expect(float input,float unit);

#endif /* CODE_CAMERA_H_ */
