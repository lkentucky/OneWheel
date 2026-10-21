#include "otus.h"

/* 统计灰度直方图：将像素值 0～255 对应的计数写入 histogram。 */
void calculateHistogram(uint8_t *image,int width,int height,Histogram histogram){
    // 清空灰度直方图。
    memset(histogram,0,sizeof(int)*256);
    
    // 发送图像数据
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            histogram[image[i*width+j]]++;
        }
    }
}

/* 判断直方图中是否存在灰度超过指定阈值的像素。 */
int hasPixelAboveThresholdInHistogram(Histogram histogram, uint8_t threshold){
     for (int i = threshold + 1; i < 256; i++) {
        if (histogram[i] > 0) {
            return 1;  // 存在超过阈值的像素，返回 1。
        }
     }
     return 0;   // 不存在超过阈值的像素，返回 0。
}

/* 使用 Otsu 类间方差法计算二值化阈值。 */
int otsuThresholding(const Histogram histogram, int width, int height, uint8_t startThreshold, OtsuResult* otsuresult) {
    if (histogram == NULL || otsuresult == NULL || width <= 0 || height <= 0) {
        return -1;  // 设置引脚
    }
    // 初始化累计变量。
    int totalPixels = width * height;
    int sum = 0, sumB = 0, wB = 0, wF = 0;
    double maxVariance = 0.0;

    // 初始化 Otsu 计算结果。
    otsuresult->startThreshold = startThreshold;
    otsuresult->optimalThreshold = 0;
    otsuresult->whiteAreaRatio = 0.0;

    // 计算像素灰度加权总和。
    for (int i = 0; i < 256; i++) {
        sum += i * histogram[i];
        if (i < startThreshold) {
            wB += histogram[i];
            sumB += i * histogram[i];
        }
    }
    
    // 遍历候选阈值并计算类间方差。
    for (uint8_t t = startThreshold; t < 256; t++) {
        wB += histogram[t];     // 累计阈值一侧的像素数量。
        if (wB == 0) continue;  // 当前类别为空，跳过该阈值。

        wF = totalPixels - wB;  // 配置整体数据
        if (wF == 0) break;     // 另一类别为空，停止遍历。

        sumB += t * histogram[t];
        double mB = (double)sumB / wB;      // 计算当前类别的平均灰度。
        double mF = (double)(sum - sumB) / wF;  // 计算另一类别的平均灰度。

        // 计算类间方差。
        double variance = (double)wB * wF * (mB - mF) * (mB - mF);
        if (variance > maxVariance) {
            maxVariance = variance;
            otsuresult->optimalThreshold = t;
        }
    }

    // 计算阈值以上白色区域的像素比例。
    int whitePixels = 0;
    for (int i = otsuresult->optimalThreshold + 1; i < 256; i++) {
        whitePixels += histogram[i];
    }
    otsuresult->whiteAreaRatio = (double)whitePixels / totalPixels;

    return 1;  // 保存最优阈值及统计结果。
}

/* 将灰度图转换为二值图。 */
void gray2binary(uint8_t* input, uint8_t* output, int width, int height, int threshold) {
    int total_pixels = width * height;

    //memset(output, 0, sizeof(uint8_t) * total_pixels);
    // 遍历图像像素并执行阈值判断。
    for (int i = 0; i < total_pixels; i++) {
        // 超过阈值的像素设为 255，其余设为 0。
        output[i] = (input[i] >= threshold) ? 255: 0;
    }
}
