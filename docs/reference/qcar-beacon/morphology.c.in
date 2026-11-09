#include "morphology.h"

// 创建椭圆形结构核，支持2x2, 3x3, 5x5大小
EllipseKernel createEllipseKernel(int size) {
    EllipseKernel kernel;
    kernel.size = size;

    // 分配内存并初始化为0
    kernel.kernel = (uint8_t*)malloc(size * size * sizeof(uint8_t));
    memset(kernel.kernel, 0, size * size * sizeof(uint8_t));

    if (size == 2) {
        // 2x2 核
        kernel.kernel[0] = 1; kernel.kernel[1] = 1;
        kernel.kernel[2] = 1; kernel.kernel[3] = 1;
    } else if (size == 3) {
        // 3x3 核
        kernel.kernel[0] = 0; kernel.kernel[1] = 1; kernel.kernel[2] = 0;
        kernel.kernel[3] = 1; kernel.kernel[4] = 1; kernel.kernel[5] = 1;
        kernel.kernel[6] = 0; kernel.kernel[7] = 1; kernel.kernel[8] = 0;
    } else if (size == 5) {
        // 5x5 核
        kernel.kernel[0] = 0; kernel.kernel[1] = 0; kernel.kernel[2] = 1; kernel.kernel[3] = 0; kernel.kernel[4] = 0;
        kernel.kernel[5] = 0; kernel.kernel[6] = 1; kernel.kernel[7] = 1; kernel.kernel[8] = 1; kernel.kernel[9] = 0;
        kernel.kernel[10] = 1; kernel.kernel[11] = 1; kernel.kernel[12] = 1; kernel.kernel[13] = 1; kernel.kernel[14] = 1;
        kernel.kernel[15] = 0; kernel.kernel[16] = 1; kernel.kernel[17] = 1; kernel.kernel[18] = 1; kernel.kernel[19] = 0;
        kernel.kernel[20] = 0; kernel.kernel[21] = 0; kernel.kernel[22] = 1; kernel.kernel[23] = 0; kernel.kernel[24] = 0;
    } else {
        // 不支持的大小，释放内存
        free(kernel.kernel);
        kernel.kernel = NULL;
        kernel.size = 0;
    }

    return kernel;
}

// 销毁椭圆形核
void destroyEllipseKernel(EllipseKernel* kernel) {
    if (kernel->kernel) {
        free(kernel->kernel);
        kernel->kernel = NULL;
        kernel->size = 0;
    }
}   

void dilate(uint8_t* input, uint8_t* output, int width, int height, const EllipseKernel* kernel) {
    int ksize = kernel->size;
    int kcenter = ksize / 2;

    // 清空输出图像
    memset(output, 0, width * height * sizeof(uint8_t));

    // 遍历图像并执行膨胀操作
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            // 如果当前像素值为0，则跳过
            if (input[y * width + x] == 0) continue;

            // 打印当前正在处理的像素位置和像素值
            //printf("Processing pixel at (%d, %d), value: %d\r\n", x, y, input[y * width + x]);

            // 遍历核
            for (int ky = 0; ky < ksize; ky++) {
                for (int kx = 0; kx < ksize; kx++) {
                    // 如果核值为0，则跳过
                    if (kernel->kernel[ky * ksize + kx] == 0) continue;

                    // 计算新的位置
                    int nx = x + kx - kcenter;
                    int ny = y + ky - kcenter;

                    // 如果新位置在图像范围内
                    if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                        // 设置输出图像的像素值
                        output[ny * width + nx] = 1;

                        // 打印更新的像素位置
                        //printf("Setting output[%d, %d] to %d\r\n", ny, nx, output[ny * width + nx]);
                    }
                }
            }
        }
    }

    //printf("Dilate operation completed.\r\n");
}

void treshold_dilate(uint8_t* input, uint8_t* output, int width, int height, const EllipseKernel* kernel, uint8_t threshold) {
    int ksize = kernel->size;
    int kcenter = ksize / 2;

    // 清空输出图像
    memset(output, 0, width * height * sizeof(uint8_t));

    // 遍历图像并执行膨胀操作
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            // 判断当前像素值是否大于阈值，若是则视为对象像素（1），否则视为背景像素（0）
            if (input[y * width + x] <= threshold) continue;

            // 遍历核
            for (int ky = 0; ky < ksize; ky++) {
                for (int kx = 0; kx < ksize; kx++) {
                    // 如果核值为0，则跳过
                    if (kernel->kernel[ky * ksize + kx] == 0) continue;

                    // 计算新的位置
                    int nx = x + kx - kcenter;
                    int ny = y + ky - kcenter;

                    // 如果新位置在图像范围内
                    if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                        // 设置输出图像的像素值
                        output[ny * width + nx] = 1;
                    }
                }
            }
        }
    }
}

