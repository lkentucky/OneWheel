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
float last_erro=94;
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
    for (int y = 39; y < MT9V03X_H-10; y++) {
        for (int x = 10; x < MT9V03X_W; x++) {
            if (image[y][x] > IR_THRESHOLD ){
                for(int i=0;i<10;i++)
                    for(int j=0;j<10;j++)
                    {
                        value+=image[y-i][x-j];
                    }
                if(value>sumvalue)
                {
                    if(func_abs(y-info->centerY)>20)
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
                else if(value>info->sum_last && func_abs(y-info->centerY)>20)
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
    info->quanzhong=1.5*func_abs(MT9V03X_W/2-info->centerX)+func_abs(MT9V03X_H-info->centerY)*6.5-info->sum*2;
    info->last_quanzhong=1.5*func_abs(MT9V03X_W/2-info->last_centerX)+func_abs(MT9V03X_H-info->last_centerY)*6.5+2*info->sum_last;
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
    erro=1.0*(MT9V03X_W/2-info->centerX);
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
//        erro=func_abs(last_erro+1)/(last_erro+1)*65;
        if(last_erro<20)
        {
            erro=func_abs(last_erro+1)/(last_erro+1)*65;
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
    for(uint8 i=5;i<245;i++)
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
    for(int y=0;y<30;y++)
        for(int x=0;x<MT9V03X_W;x++)
        {
            image_binary[y][x]=0;
            base_image[y][x]=0;
        }
}

float retarder(float input,float unit)
{
    static float output=0;
//    if(func_abs(error_back-output)>30)
//    {
//        unit=2*unit;
//    }
    if(output>input)
    {
        output-=unit;
        if(output<input)
        {
            output=input;
        }
    }
    else
    {
        output+=unit;
        if(output>input)
        {
            output=input;
        }
    }
    return output;
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
