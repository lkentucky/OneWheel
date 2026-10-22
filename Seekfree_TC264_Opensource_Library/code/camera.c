///*
// * camera.c
// *
// *  Created on: 2025年4月19日
// *      Author: lenovo
// */
#include "camera.h"
uint8    image_binary[MT9V03X_H][MT9V03X_W];
BeaconInfo beacon;
float erro;
float error_back;
float erro_yawan;
float erro_yawan_back;
float last_erro=CAMERA_CENTER_X;
//红外检测，包括误差
void detect_beacon(uint8 image[MT9V03X_H][MT9V03X_W], BeaconInfo *info){
    int16 sumvalue=0;
    int16 value=0;
    int lingshi=0;
    // 重置信标信息
    info->status = BEACON_NOT_FOUND;
    info->centerX = 0;
    info->centerY = 0;//行120
    info->area = 0;

    // 遍历图像检测红外点
    for (int y = CAMERA_SCAN_MIN_Y; y < MT9V03X_H-CAMERA_SCAN_BOTTOM_MARGIN; y++) {
        for (int x = CAMERA_SCAN_MIN_X; x < MT9V03X_W; x++) {
            if (image[y][x] > IR_THRESHOLD ){
                for(int i=0;i<CAMERA_WINDOW_SIZE;i++)
                    for(int j=0;j<CAMERA_WINDOW_SIZE;j++)
                    {
                        value+=image[y-i][x-j];
                    }
                if(value>sumvalue)
                {
                    if(func_abs(y-info->centerY)>CAMERA_CANDIDATE_SEPARATION)
                    {
                        info->sum_last=sumvalue;
                        info->last_centerX=info->centerX;
                        info->last_centerY=info->centerY;
                    }

                    sumvalue=value;
                    info->sum=sumvalue;
                    info->centerX=x;
                    info->centerY=y;
                    value=0;
                }
                else if(value>info->sum_last && func_abs(y-info->centerY)>CAMERA_CANDIDATE_SEPARATION)
                {
                    info->sum_last=value;
                    info->last_centerX=x;
                    info->last_centerY=y;
                    value=0;
                }
                else
                {
                    value=0;
                }
            }
        }
    }

    info->length=func_abs(info->centerY-MT9V03X_H/2)+func_abs(info->centerX-MT9V03X_W/2);
    info->length_last=func_abs(info->last_centerY-MT9V03X_H/2)+func_abs(info->last_centerX-MT9V03X_W/2);
//    if(info->sum>6000 && info->centerY<40)
//    {
//        info->sum=info->sum_last;
//        info->centerX=info->last_centerX;
//        info->centerY=info->last_centerY;
//    }
    info->quanzhong=CAMERA_WEIGHT_X*func_abs(CAMERA_CENTER_X-info->centerX)+func_abs(MT9V03X_H-info->centerY)*CAMERA_WEIGHT_Y-info->sum*CAMERA_WEIGHT_AREA;
    info->last_quanzhong=CAMERA_WEIGHT_X*func_abs(MT9V03X_W/2-info->last_centerX)+func_abs(MT9V03X_H-info->last_centerY)*CAMERA_WEIGHT_Y+CAMERA_WEIGHT_AREA*info->sum_last;
    if(info->quanzhong >info->last_quanzhong )
    {
        lingshi=info->sum;
        info->sum=info->sum_last;
        info->sum_last=lingshi;
        lingshi=info->centerX;
        info->centerX=info->last_centerX;
        info->last_centerX=lingshi;
        lingshi=info->centerY;
        info->centerY=info->last_centerY;
        info->last_centerY=lingshi;
    }
    last_erro=erro;
    erro=1.0*(CAMERA_CENTER_X-info->centerX);
    if(erro>0)
    {
        erro+=0;
    }
    else if(erro<0)
    {
        erro+=0;
    }
    if(info->centerX==0)
    {
//        erro=func_abs(last_erro+1)/(last_erro+1)*CAMERA_LOST_ERROR;
        if(last_erro<CAMERA_LOST_SWITCH_ERROR)
        {
            erro=func_abs(last_erro+1)/(last_erro+1)*CAMERA_LOST_ERROR;
        }
        else
        {
            erro=last_erro;
        }

    }
    erro_yawan=erro;
//    change_the_speed();

}


uint16 hist[GrayScale]={0};
float P[GrayScale]={0};
float PK[GrayScale]={0};
float MK[GrayScale]={0};
uint8 img_threshold;
float imgsize;
uint8 Ostu(uint8 index[MT9V03X_H][MT9V03X_W])
{
    uint8 threshold;
    imgsize=MT9V03X_H*MT9V03X_W;
    uint8 image_value_temp;

    float sumPK=0;
    float sumMK=0;
    float var=0;
    float vartmp=0;

    for(uint16 i=0;i<GrayScale;i++)
    {
        hist[i]=0;
        P[i]=0;
        PK[i]=0;
        MK[i]=0;
    }
    for(uint8 i=0;i<MT9V03X_H;i++)
    {
        for(uint8 j=0;j<MT9V03X_W;j++)
        {
            image_value_temp=index[i][j];
            hist[image_value_temp]++;
        }
    }
    for(uint16 i=0;i<GrayScale;i++)
    {
        P[i]=(float)hist[i]/imgsize;
        PK[i]=sumPK+P[i];
        sumPK=PK[i];
        MK[i]=sumMK+i*P[i];
        sumMK=MK[i];
    }
    for(uint8 i=CAMERA_OTSU_START;i<CAMERA_OTSU_END;i++)
    {
        vartmp=((MK[GrayScale-1]*PK[i]-MK[i])*(MK[GrayScale-1]*PK[i]-MK[i]))/(PK[i]*(1-PK[i]));
        if(vartmp>var)
        {
            var=vartmp;
            threshold=i;
        }
    }
    return threshold;
}


//***********************二值化
void Image_Binarization(int threshold)//图像二值化
{
    uint16 i,j;
    for(i=0;i<MT9V03X_H;i++)
    {
        for(j=0;j<MT9V03X_W;j++)
        {
            if(mt9v03x_image[i][j]>=threshold)
                image_binary[i][j]=IMG_WHITE;//白
            else
                image_binary[i][j]=IMG_BLACK;//黑
        }
    }
    for(int y=0;y<CAMERA_BLANK_TOP_ROWS;y++)
        for(int x=0;x<MT9V03X_W;x++)
        {
            image_binary[y][x]=0;
            base_image[y][x]=0;
        }
}

static float retarder_output;
void retarder_reset(void) { retarder_output = 0; }
float retarder(float input,float unit)
{
//    if(func_abs(error_back-retarder_output)>30)
//    {
//        unit=2*unit;
//    }
    if(retarder_output>input)
    {
        retarder_output-=unit;
        if(retarder_output<input)
        {
            retarder_output=input;
        }
    }
    else
    {
        retarder_output+=unit;
        if(retarder_output>input)
        {
            retarder_output=input;
        }
    }
    return retarder_output;
}
float en_retarder(float input,float unit)
{
    static float en_output=0;
//    if(func_abs(erro_yawan_back-en_output)>100)
//    {
//        unit=2*unit;
//    }
     if(en_output>input)
    {
        en_output-=unit;
        if(en_output<input)
        {
            en_output=input;
        }
    }
    else
    {
        en_output+=unit;
        if(en_output>input)
        {
            en_output=input;
        }
    }
    return en_output;
}
int  speed_expect(float input,float unit)
{
    static float speed=0;

     if(speed>input)
    {
         speed-=unit;
        if(speed<input)
        {
            speed=input;
        }
    }
    else
    {
        speed+=unit;
        if(speed>input)
        {
            speed=input;
        }
    }
    return speed;
}
