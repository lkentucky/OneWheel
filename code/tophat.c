/*
 * tophat.c
 *
 *  Created on: 2025年7月9日
 *      Author: Lenovo
 */

/* tophat.c  -------------------------------------------------------- */
#include "tophat.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#define H          120
#define W          188
#define PIX        (H*W)
#define THRESH     60


#define ROI_Y0     25          /* 处理区起始行 (含) */
#define ROI_Y1     70          /* 处理区结束行 (含) */
#define KERNEL_SZ  3
static uint8_t kernel   [KERNEL_SZ*KERNEL_SZ];
static uint8_t buf_erode[PIX]  __attribute__((section(".bss.cpu1_dsram")));  /* 开运算-腐蚀结果 */
static uint8_t buf_open [PIX]  __attribute__((section(".bss.cpu1_dsram")));  /* 开运算-膨胀结果 */
static void erode_u8(const uint8_t *src, uint8_t *dst,
                     int w, int h,
                     const uint8_t *kernel, int k, int k2)
{
    const int R = k / 2;
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            uint8_t mn = 255;
            for (int ky = -R, idx = 0; ky <= R; ++ky) {
                int yy = y + ky;
                if (yy < 0 || yy >= h) { idx += k;  continue; }
                for (int kx = -R; kx <= R; ++kx, ++idx) {
                    if (!kernel[idx]) continue;        /* 核值为 0 -> 跳过 */
                    int xx = x + kx;
                    if (xx < 0 || xx >= w) continue;   /* 越界忽略 */
                    uint8_t v = src[yy * w + xx];
                    if (v < mn) mn = v;
                }
            }
            dst[y * w + x] = mn;
        }
    }
}

static void dilate_u8(const uint8_t *src, uint8_t *dst,
                      int w, int h,
                      const uint8_t *kernel, int k, int k2)
{
    const int R = k / 2;
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            uint8_t mx = 0;
            for (int ky = -R, idx = 0; ky <= R; ++ky) {
                int yy = y + ky;
                if (yy < 0 || yy >= h) { idx += k;  continue; }
                for (int kx = -R; kx <= R; ++kx, ++idx) {
                    if (!kernel[idx]) continue;
                    int xx = x + kx;
                    if (xx < 0 || xx >= w) continue;
                    uint8_t v = src[yy * w + xx];
                    if (v > mx) mx = v;
                }
            }
            dst[y * w + x] = mx;
        }
    }
}
void make_elliptic_kernel(uint8_t *k, int sz)
{
    int R = sz / 2;
    for (int y = -R, idx = 0; y <= R; ++y) {
        for (int x = -R; x <= R; ++x, ++idx) {
            k[idx] = (x * x + y * y <= R * R) ? 1 : 0;
        }
    }
}

void tophat_u8(const uint8_t *src, uint8_t *dst,
               int w, int h,
               const uint8_t *kernel, int k)
{
    const int pix = w * h;
    static uint8_t buf1[22560];
    static uint8_t buf2[22560];


    const int k2 = k * k;
    erode_u8 (src, buf1, w, h, kernel, k, k2);
    dilate_u8(buf1, buf2, w, h, kernel, k, k2);
    for (int i = 0; i < pix; ++i) {
        int val = src[i] - buf2[i];
        dst[i] = (uint8_t)(val > 0 ? val : 0);
    }
}


void erode_roi(const uint8_t *src, uint8_t *dst,
               int w, int h, const uint8_t *k, int ks,
               int y0, int y1)
{
    int R = ks >> 1;
    for (int y = y0; y <= y1; ++y) {              /* ⬅ 只扫 ROI */
        for (int x = 0; x < w; ++x) {
            uint8_t mn = 255;
            for (int ky = -R, idx = 0; ky <= R; ++ky) {
                int yy = y + ky;
                if (yy < 0 || yy >= h) { idx += ks; continue; }
                for (int kx = -R; kx <= R; ++kx, ++idx) {
                    if (!k[idx]) continue;
                    int xx = x + kx;
                    if (xx < 0 || xx >= w) continue;
                    uint8_t v = src[yy*w + xx];
                    if (v < mn) mn = v;
                }
            }
            dst[y*w + x] = mn;                    /* 仅 ROI 行被写入 */
        }
    }
}

void dilate_roi(const uint8_t *src, uint8_t *dst,
                int w, int h, const uint8_t *k, int ks,
                int y0, int y1)
{
    int R = ks >> 1;
    for (int y = y0; y <= y1; ++y) {
        for (int x = 0; x < w; ++x) {
            uint8_t mx = 0;
            for (int ky = -R, idx = 0; ky <= R; ++ky) {
                int yy = y + ky;
                if (yy < 0 || yy >= h) { idx += ks; continue; }
                for (int kx = -R; kx <= R; ++kx, ++idx) {
                    if (!k[idx]) continue;
                    int xx = x + kx;
                    if (xx < 0 || xx >= w) continue;
                    uint8_t v = src[yy*w + xx];
                    if (v > mx) mx = v;
                }
            }
            dst[y*w + x] = mx;
        }
    }
}

void process_roi_tophat_thresh(const uint8_t *src, uint8_t *dst)
{
    make_elliptic_kernel(kernel, KERNEL_SZ);

    /* ① 只在 ROI 行段做开运算 */
    erode_roi (src, buf_erode, W, H, kernel, KERNEL_SZ, ROI_Y0, ROI_Y1);
    dilate_roi(buf_erode, buf_open, W, H, kernel, KERNEL_SZ, ROI_Y0, ROI_Y1);

    /* ② 行分区处理 */
    for (int y = 0; y < H; ++y) {
        int row = y * W;

        if (y < ROI_Y0) {                        /* 0 ≤ y < 25 → 置零 */
            memset(dst + row, 0, W);
        }
        else if (y <= ROI_Y1) {                  /* 25 ≤ y ≤ 59 → Top-Hat */
            for (int x = 0; x < W; ++x) {
                int idx   = row + x;
                int t_hat = src[idx] - buf_open[idx];
                dst[idx]  = (t_hat > THRESH) ? 255 : 0;
            }
        }
        else {                                   /* y ≥ 60 → 原图阈值 */
            for (int x = 0; x < W; ++x) {
                int idx = row + x;
                dst[idx] = (src[idx] > THRESH) ? 255 : 0;
            }
        }
    }
}

/* --------------------------------------------------------------
 * 形态学梯度：Gradient = dilate(src) − erode(src)
 * 与 process_roi_tophat_thresh 思路一致，只是把
 *     TopHat = src − open(src)
 * 换成
 *     Grad   = dilate − erode
 * --------------------------------------------------------------*/
static uint8_t buf_dilate[PIX];   /* dilation(src) 结果 */

void process_roi_gradient_thresh(const uint8_t *src, uint8_t *dst)
{
    /* ① 生成椭圆核（与 Top-Hat 共用） */
    make_elliptic_kernel(kernel, KERNEL_SZ);

    /* ② 仅在 ROI 行段做腐蚀 / 膨胀 */
    erode_roi (src, buf_erode , W, H, kernel, KERNEL_SZ, ROI_Y0, ROI_Y1);
    dilate_roi(src, buf_dilate, W, H, kernel, KERNEL_SZ, ROI_Y0, ROI_Y1);

    /* ③ 分行处理并二值化输出 */
    for (int y = 0; y < H; ++y) {
        int row = y * W;

        if (y < ROI_Y0) {                     /* 0 ≤ y < 25 → 置零 */
            memset(dst + row, 0, W);
        }
        else if (y <= ROI_Y1) {               /* 25 ≤ y ≤ 59 → Gradient */
            for (int x = 0; x < W; ++x) {
                int idx   = row + x;
                int grad  = buf_dilate[idx] - buf_erode[idx];
                dst[idx]  = (grad > THRESH) ? 255 : 0;
            }
        }
        else {                                /* y ≥ 60 → 原图阈值 */
            for (int x = 0; x < W; ++x) {
                int idx = row + x;
                dst[idx] = (src[idx] > 120) ? 255 : 0;
            }
        }
    }
}


// 新增：分段处理函数
void process_roi_segment_thresh(const uint8_t *src, uint8_t *dst) {
    make_elliptic_kernel(kernel, KERNEL_SZ);
    // 预计算 ROI 的腐蚀与膨胀
    erode_roi(src, buf_erode, W, H, kernel, KERNEL_SZ, ROI_Y0, ROI_Y1);
    dilate_roi(src, buf_dilate, W, H, kernel, KERNEL_SZ, ROI_Y0, ROI_Y1);
    for(int y=0; y<H; ++y) {
        int row = y*W;
        if(y < 25) {
            // 0-24 行置零
            memset(dst+row, 0, W);
        }
//        } else if(y < 70) {
//            // 25-54 行使用腐蚀结果阈值
//            for(int x=0; x<W; ++x) {
//                dst[row+x] = (buf_erode[row+x] > THRESH) ? 255 : 0;
//            }
//        }
       else if(y < 70) {
            // 55-69 行使用膨胀结果阈值
            for(int x=0; x<W; ++x) {
                dst[row+x] = (buf_dilate[row+x] > THRESH) ? 255 : 0;
            }
        } else {
            // 70 行及以上使用原图固定阈值
            for(int x=0; x<W; ++x) {
                dst[row+x] = (src[row+x] > THRESH) ? 255 : 0;
            }
        }
    }
}


