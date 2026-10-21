#ifndef FINDCONTOURS_H
#define FINDCONTOURS_H

#include "zf_common_headfile.h"
#define IMG_HEIGHT 120
#define IMG_WIDTH 188
#define M_PI 3.1415926f

/**
 * @brief 计算最大横纵比
 */
//double computeMinBoundingBoxAspectRatio(Contour contour);

/** 
 * @brief 计算轮廓的面积，使用鞋带公式。
 * 
 * @param contour 传入一个包含多个点的轮廓结构体。
 * @return 返回轮廓的面积（浮动类型）。
 */
double computeArea(Contour contour);

/** 
 * @brief 计算轮廓的周长，使用两点间距离的和。
 * 
 * @param contour 传入一个包含多个点的轮廓结构体。
 * @return 返回轮廓的周长（浮动类型）。
 */
double computePerimeter(Contour contour);

/** 
 * @brief 计算轮廓的圆形度，用来衡量形状是否接近圆形。
 * 
 * @param contour 传入一个包含多个点的轮廓结构体。
 * @return 返回圆形度，值越接近 1 代表形状越接近圆形。
 */
double calculate_circularity(Contour contour);


/** 
 * @brief 计算轮廓的质心（重心），使用几何矩法。
 * 
 * @param contour 传入一个包含多个点的轮廓结构体。
 * @param cx 指向存储计算出的质心 x 坐标的指针。
 * @param cy 指向存储计算出的质心 y 坐标的指针。
 */
void computeCentroid(Contour contour, double *cx, double *cy);

/** 
 * @brief 计算轮廓的质心（重心），并返回质心坐标。
 * 
 * 该函数作为对外提供的接口，判断轮廓的点数是否足够进行计算，
 * 如果点数不足（小于 3），则返回质心为 (0, 0)。
 * 否则，调用内部函数进行质心计算并通过指针返回计算结果。
 * 
 * @param contour 输入的轮廓结构体，包含多个轮廓点。
 * @param cx 输出参数，指向存储计算出的质心 x 坐标的变量。
 * @param cy 输出参数，指向存储计算出的质心 y 坐标的变量。
 */
void calculate_center(Contour contour, double *cx, double *cy);

/** 
 * @brief 使用 4 连通洪泛填充背景区域。
 * 
 * 从指定的起始点开始，按照 4 连通规则填充图像中的背景区域（背景像素值为 0）。
 * 被填充的区域会在 `ext` 数组中标记为 `true`。
 * 
 * @param ext 用于标记填充状态的二维布尔数组，标记填充过的区域。
 * @param image 输入的图像数据，0 表示背景，非 0 表示前景。
 * @param h 图像的高度（行数）。
 * @param w 图像的宽度（列数）。
 * @param startRow 起始点的行坐标。
 * @param startCol 起始点的列坐标。
 */
static void floodFillExternalBG(bool *ext, uint8_t *image, int h, int w, int startRow, int startCol);

/**
 * @brief 创建一个二维布尔数组并进行初始化，标记图像的外部背景。
 *
 * 该函数会根据输入的图像进行背景区域的填充，首先会从图像的边界开始，找到所有值为0的像素，
 * 这些像素代表背景。通过洪泛填充的方式，将与边界相连的所有背景像素标记为背景。
 *
 * @param image 输入的二值图像，0表示背景，255表示前景
 * @param h 图像的高度（行数）
 * @param w 图像的宽度（列数）
 *
 * @return 返回一个二维布尔数组 ext，其中值为true表示背景像素，false表示前景像素。
 */
//bool *buildExternalBG(uint8_t *image, int h, int w);

/**
 * @brief 标记图像中的外部轮廓
 *
 * 对图像中所有前景像素（值为 1），检查其 4 邻域：
 * - 如果邻域有越界或邻域像素为 0 且属于外部背景（由 buildExternalBG 函数确定），
 *   则将该像素标记为轮廓（值为 2）。
 *
 * 轮廓像素通过在图像上设置相应位置的值为 2 来进行标记。
 *
 * @param image 输入/输出的二值化图像，前景像素为 1，背景像素为 0。
 * @param h 图像的高度（行数）。
 * @param w 图像的宽度（列数）。
 */
void markOuterContour(uint8_t *image, int h, int w);

/**
 * @brief Moore 邻域轮廓跟踪算法
 *
 * 从给定的起始点开始，按顺时针顺序跟踪图像中的轮廓像素（值为 2），
 * 直到再次遇到起始点为止。此算法使用 Moore 邻域（8 个方向）来提取轮廓。
 * 如果轮廓没有闭合或无法继续跟踪，则会返回已经提取的轮廓点。
 * 
 * @param image 输入的二值化图像，轮廓像素值为 2。
 * @param h 图像的高度（行数）。
 * @param w 图像的宽度（列数）。
 * @param startRow 起始轮廓点的行坐标。
 * @param startCol 起始轮廓点的列坐标。
 * @param visited 用于标记像素是否已经被访问过的布尔二维数组。
 * 
 * @return 轮廓点集（Contour 结构体），包含轮廓点数组和点数。
 */
//Contour traceContourMoore(uint8_t *image, int h, int w, int startRow, int startCol, bool *visited);

/**
 * @brief 处理图像中的轮廓，计算每个轮廓的圆度、质心和面积，并存储到 `ContourInfo` 数组中。
 *
 * @param image        输入图像数据，二维数组，值为 0 表示背景，1 表示前景，2 表示轮廓。
 * @param h            图像的高度（行数）。
 * @param w            图像的宽度（列数）。
 * @param contourInfos 存储轮廓信息的数组，每个元素包含该轮廓的圆度、质心和面积。
 * @param numContours  指向轮廓数量的指针，每次调用后更新为已处理的轮廓数量。如果轮廓数量已满（最大100个），则跳过后续轮廓。
 *
 * @note 该函数会在图像上标记外部轮廓，使用 Moore 邻域轮廓跟踪算法提取每个轮廓，并计算每个轮廓的圆度、质心和面积。
 *       如果轮廓数量超过100，则不再处理后续轮廓。
 */
void processContours(uint8_t *image, int h, int w, ContourInfo *contourInfos, int *numContours,uint8_t* grayimage);



#endif
