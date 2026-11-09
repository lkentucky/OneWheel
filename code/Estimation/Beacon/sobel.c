#if defined(__TASKING__)
#pragma section all "cpu1_dsram"
#pragma section code "beacon_flash"
#endif
#include "beacon_platform.h"
#include "sobel.h"
/* 四向 Sobel 卷积核 */
//static const int8_t KX[3][3]     = { { -1,  0,  1 },
//                                     { -2,  0,  2 },
//                                     { -1,  0,  1 } };
//
//static const int8_t KY[3][3]     = { {  1,  2,  1 },
//                                     {  0,  0,  0 },
//                                     { -1, -2, -1 } };
//
//static const int8_t K45[3][3]    = { {  0,  1,  2 },
//                                     { -1,  0,  1 },
//                                     { -2, -1,  0 } };
//
//static const int8_t K135[3][3]   = { {  2,  1,  0 },
//                                     {  1,  0, -1 },
//                                     {  0, -1, -2 } };



void four_sobel_binarize(const uint8_t *in, uint8_t *out, uint8_t threshold, int frame_width, int frame_height) {
    int x, y;
    //memset(out, 0, frame_width * frame_height);  // 初始化输出图像

    for (y = 1; y < frame_height - 1; ++y) {
        for (x = 1; x < frame_width - 1; ++x) {
            if(in[(y-1)*frame_width + (x-1)]< 60){
                out[y * frame_width + x]=0;
                continue;
            }
            // 1. 共享邻域像素值
            uint16_t p00 = in[(y-1)*frame_width + (x-1)];
            uint16_t p01 = in[(y-1)*frame_width + (x  )];
            uint16_t p02 = in[(y-1)*frame_width + (x+1)];
            uint16_t p10 = in[(y  )*frame_width + (x-1)];
            //int p11 = in[(y  )*frame_width + (x  )];  // 未使用，但可留作扩展
            uint16_t p12 = in[(y  )*frame_width + (x+1)];
            uint16_t p20 = in[(y+1)*frame_width + (x-1)];
            uint16_t p21 = in[(y+1)*frame_width + (x  )];
            uint16_t p22 = in[(y+1)*frame_width + (x+1)];

            // 2. 计算三方向梯度
            int32_t gx = -p00 - 2*p10 - p20 + p02 + 2*p12 + p22;
            int32_t gy =  p00 + 2*p01 + p02 - p20 - 2*p21 - p22;
            int32_t g45 = p01 + 2*p02 - p10 + p12 - 2*p20 - p21;
            int32_t g135 = p00 + 2*p10 + p21 - p01 - 2*p12 - p22;

//
            // 3. 合并梯度（可换成 sqrt() 或其他方式）
            int32_t mag = abs(gx) + abs(gy) + abs(g45)+abs(g135);

            // 4. 二值化输出
            out[y * frame_width + x] = (mag > threshold&&in[y * frame_width + x]>60) ? 255 : 0;
        }
    }
}
