#include "findcontours.h"

// 计算轮廓的面积
double computeArea(Contour contour) {
    double area = 0.0;
    for (int i = 0; i < contour.count; i++) {
        int j = (i + 1) % contour.count; // 检查轮廓点
        double term = (double)contour.points[i].col * contour.points[j].row -
                      (double)contour.points[j].col * contour.points[i].row;
        area += term;
    }
    double finalArea = fabs(area) / 2.0;
    return finalArea;
}

// 计算轮廓的周长
double computePerimeter(Contour contour) {
    double perimeter = 0.0;
    for (int i = 0; i < contour.count; i++) {
        int j = (i + 1) % contour.count;
        double dx = (double)contour.points[j].col - contour.points[i].col;
        double dy = (double)contour.points[j].row - contour.points[i].row;
        double dist = sqrt(dx * dx + dy * dy);
        perimeter += dist;
    }
    return perimeter;
}

// 计算轮廓的圆度
double calculate_circularity(Contour contour) {
    if (contour.count < 3)
        return 0;
    double area = computeArea(contour);
    double perimeter = computePerimeter(contour);
    if (perimeter == 0)
        return 0;
    double circ = (4 * M_PI * area) / (perimeter * perimeter);
    return circ;
}

// 计算轮廓的质心
void computeCentroid(Contour contour, double *cx, double *cy) {
    double A = 0.0;  // 计算面积
    double Cx = 0.0, Cy = 0.0;
    for (int i = 0; i < contour.count; i++) {
        int j = (i + 1) % contour.count;
        double cross = (double)contour.points[i].col * contour.points[j].row -
                       (double)contour.points[j].col * contour.points[i].row;
        A += cross;
        Cx += (contour.points[i].col + contour.points[j].col) * cross;
        Cy += (contour.points[i].row + contour.points[j].row) * cross;
    }
    A *= 0.5;
    if (fabs(A) < 1e-6) {  // 面积接近零，认为轮廓不存在
        *cx = 0;
        *cy = 0;
        return;
    }
    *cx = Cx / (6 * A);
    *cy = Cy / (6 * A);
}

// 提供计算质心的接口
void calculate_center(Contour contour, double *cx, double *cy) {
    if (contour.count < 3) {
        *cx = 0;
        *cy = 0;
        return;
    }
    computeCentroid(contour, cx, cy);
}

////计算轮廓信息
//void calculate_contourinfo(Contour contour, ContourInfo* contourinfo, uint8_t* image) {
//    if(contour.count<3){
//       contourinfo->area = 0.0;
//       contourinfo->circularity = 0.0;
//       contourinfo->cx = 0.0;
//       contourinfo->cy = 0.0;
//       contourinfo->boxratio = 0.0;
//       contourinfo->ave_gray = 0.0;
//       return;
//    }
//    float A = 0.0;       // 累加交叉乘积，用于面积和质心计算
//    float Cx = 0.0, Cy = 0.0;  // 用于质心的加权和
//    float perimeter = 0.0;     // 轮廓周长
//    int minX = contour.points[0].col, maxX = contour.points[0].col;
//    int minY = contour.points[0].row, maxY = contour.points[0].row;
//
//    int gray_sum = 0;  // 累加轮廓线上像素的灰度值
//
//    // 遍历所有轮廓点，一次计算面积、质心、周长、边界框以及累加灰度值
//    for (int i = 0; i < contour.count; i++) {
//        int j = (i + 1) % contour.count;
//        int x0 = contour.points[i].col;
//        int y0 = contour.points[i].row;
//        int x1 = contour.points[j].col;
//        int y1 = contour.points[j].row;
//
//        // 更新边界框
//        if (x0 < minX) minX = x0;
//        if (x0 > maxX) maxX = x0;
//        if (y0 < minY) minY = y0;
//        if (y0 > maxY) maxY = y0;
//
//        // 计算交叉乘积，用于面积和质心
//        float cross = (float)x0 * y1 - (float)x1 * y0;
//        A += cross;
//        Cx += (x0 + x1) * cross;
//        Cy += (y0 + y1) * cross;
//
//        // 计算当前边的长度，累加得到周长
//        float dx = (float)x1 - x0;
//        float dy = (float)y1 - y0;
//        perimeter += sqrt(dx * dx + dy * dy);
//
//        // 直接取轮廓线上当前点的灰度值累加
//        gray_sum += image[y0 * 188 + x0];
//        //printf("%d:%d\r\n",i,image[y0 * 188 + x0]);
//    }
//
//    //printf("sum:%d\r\n",gray_sum);
//    A *= 0.5;
//    float area = fabs(A);
//    float cx = 0.0, cy = 0.0;
//    if (fabs(A) > 1e-6) {  // 避免面积接近0时分母为零
//        cx = Cx / (6.0 * A);
//        cy = Cy / (6.0 * A);
//    }
//
//    float circularity = 0.0;
//    if (perimeter > 1e-6)
//        circularity = (4 * M_PI * area) / (perimeter * perimeter);
//
//    // 计算外接矩形的长宽比
//    int width = maxX - minX;
//    int height = maxY - minY;
//    float boxratio = 0.0;
//
//    boxratio=(float)width/height;
//
//    // 计算轮廓线上像素的平均灰度值
//    float ave_gray = (float)gray_sum / contour.count;
//
//    // 填充轮廓信息结构体
//    contourinfo->area = area;
//    contourinfo->cx = cx;
//    contourinfo->cy = cy;
//    contourinfo->circularity = circularity;
//    contourinfo->boxratio = boxratio;
//    contourinfo->ave_gray = ave_gray;
//}
int compare_int(const void* a, const void* b) {
    return (*(int*)a - *(int*)b);
}

void calculate_contourinfo(Contour contour, ContourInfo* contourinfo, uint8_t* image) {
//    const int image_width = 188;
//    contourinfo->num=contour.count;

//    float A = 0.0f, Cx = 0.0f, Cy = 0.0f, perimeter = 0.0f;
//    int minX = contour.points[0].col, maxX = contour.points[0].col;
//    int minY = contour.points[0].row, maxY = contour.points[0].row;
//
//    float mu20 = 0.0f, mu02 = 0.0f, mu11 = 0.0f; //mu30 = 0.0f, mu03 = 0.0f;
////  float mu21 = 0.0f, mu12 = 0.0f;
//    float cx = 0.0f, cy = 0.0f;
//
//    int gray_sum = 0;
//    float gray_square_sum = 0.0f;
//    int gray_max = 0, gray_min = 255;
//    int cornerCount = 0; // 角点数，可后续加角度检测算法
//
//    for (int i = 0; i < contour.count; i++) {
//        int curr = i;
//        int next = (i + 1) % contour.count;
//        int x0 = contour.points[curr].col, y0 = contour.points[curr].row;
//        int x1 = contour.points[next].col, y1 = contour.points[next].row;
//
//        if (x0 < minX) minX = x0;
//        if (x0 > maxX) maxX = x0;
//        if (y0 < minY) minY = y0;
//        if (y0 > maxY) maxY = y0;
//
//        float cross = (float)x0 * y1 - (float)x1 * y0;
//        A += cross;
//        Cx += (x0 + x1) * cross;
//        Cy += (y0 + y1) * cross;
//
//        float dx = (float)x1 - x0;
//        float dy = (float)y1 - y0;
//        perimeter += sqrtf(dx * dx + dy * dy);
//
//        int gray = image[y0 * image_width + x0];
//        gray_sum += gray;
//        gray_square_sum += gray * gray;
//        if (gray > gray_max) gray_max = gray;
//        if (gray < gray_min) gray_min = gray;
//
//        // 以下中心矩暂时保留原始结构（不使用预先计算的质心）
//        mu20 += (x0 - cx) * (x0 - cx) * cross;
//        mu02 += (y0 - cy) * (y0 - cy) * cross;
////        mu11 += (x0 - cx) * (y0 - cy) * cross;
////        mu21 += (x0 - cx) * (x0 - cx) * (y0 - cy) * cross;
////        mu12 += (x0 - cx) * (y0 - cy) * (y0 - cy) * cross;
////        mu30 += (x0 - cx) * (x0 - cx) * (x0 - cx) * cross;
////        mu03 += (y0 - cy) * (y0 - cy) * (y0 - cy) * cross;
//    }
//
//    A *= 0.5f;
//    if (fabsf(A) > 1e-6f) {
//        cx = Cx / (6.0f * A);
//        cy = Cy / (6.0f * A);
//    }
//
//    // Hu矩计算
//    contourinfo->Hu[0] = mu20 + mu02;
//    contourinfo->Hu[1] = (mu20 - mu02) * (mu20 - mu02) + 4 * mu11 * mu11;
//    //contourinfo->Hu[2] = (mu30 - 3 * mu12) * (mu30 - 3 * mu12) + (3 * mu21 - mu03) * (3 * mu21 - mu03);
//
//    float epsilon = 1e-10f;
//    contourinfo->Hu[0] = log2f(fabsf(contourinfo->Hu[0]) + epsilon);
//    contourinfo->Hu[1] = log2f(fabsf(contourinfo->Hu[1]) + epsilon);
//    //contourinfo->Hu[2] = log2f(fabsf(contourinfo->Hu[2]) + epsilon);
//
//    float circularity = 0.0f;
//    if (perimeter > 1e-6f)
//        circularity = (4 * M_PI * A) / (perimeter * perimeter);
//
//    float ave_gray = (float)gray_sum / contour.count;
//    float variance = (gray_square_sum / contour.count) - (ave_gray * ave_gray);
//    uint16_t range = gray_max - gray_min;
//
//    int width = maxX - minX;
//    int height = maxY - minY;
//    float boxratio = (height != 0) ? (float)width / height : 0.0f;
//
//    // 填充结构体
//    contourinfo->area = A;
//    contourinfo->cx = cx;
//    contourinfo->cy = cy;
//    contourinfo->circularity = circularity;
//    contourinfo->boxratio = boxratio;
//    contourinfo->ave_gray = ave_gray;
//    contourinfo->Varaince = variance;
//    contourinfo->range = range;
//    contourinfo->middle_range = 0;  // 未计算
//    contourinfo->conner_point_num = cornerCount;
//    location_estimate(contourinfo);
}



//计算长宽比
//double computeMinBoundingBoxAspectRatio(Contour contour) {
//    // 初始化最小值和最大值为第一个点的坐标
//    int minX = contour.points[0].col, maxX = contour.points[0].col;
//    int minY = contour.points[0].row, maxY = contour.points[0].row;
//
//    // 找到最边上的点
//    for (int i = 1; i < contour.count; i++) { // 从第二个点开始遍历
//        int x = contour.points[i].col;
//        int y = contour.points[i].row;
//
//        if (x < minX) minX = x;
//        if (x > maxX) maxX = x;
//        if (y < minY) minY = y;
//        if (y > maxY) maxY = y;
//    }
//
//    // 计算长和宽
//    int width = maxX - minX;
//    int height = maxY - minY;
//
//    // 计算长宽比，长宽比 = max(长, 宽) / min(长, 宽)
//   return (double)width/height;
//}



// 使用栈存储外部背景像素填充
//static void floodFillExternalBG(bool *ext, uint8_t *image, int h, int w, int startRow, int startCol) {
//    // 使用栈实现深度优先遍历，查找并标记外部背景区域
//    static Point stack[IMG_HEIGHT * IMG_WIDTH]
//            __attribute__((section(".bss.cpu1_dsram")));  // 未初始化数据用.bss
//    int top = 0;
//    stack[top++] = (Point){ startRow, startCol };
//    ext[startRow * w + startCol] = true;  // 标记起始点为已访问
//
//    while (top > 0) {
//        Point p = stack[--top];
//        int r = p.row, c = p.col;
//        // 定义四个邻居点
//        Point neighbors[4] = { {r - 1, c}, {r + 1, c}, {r, c - 1}, {r, c + 1} };
//        for (int i = 0; i < 4; i++) {
//            Point np = neighbors[i];
//            if (np.row >= 0 && np.row < h && np.col >= 0 && np.col < w) {
//                if (image[np.row * w + np.col] == 0 && !ext[np.row * w + np.col]) {
//                    ext[np.row * w + np.col] = true;
//                    stack[top++] = np;
//                }
//            }
//        }
//    }
//}

// 构建外部背景标记
//bool *buildExternalBG(uint8_t *image, int h, int w) {
//    // 使用栈存储并标记外部背景区域
//    static bool ext[IMG_HEIGHT * IMG_WIDTH]
//        __attribute__((section(".bss.cpu2_dsram")));  // 未初始化数据用.bss
//    // 初始化为 false
//    for (int i = 0; i < h * w; i++) {
//        ext[i] = false;
//    }
//
//    // 对上下边缘和左右边缘进行处理，填充外部背景
//    for (int c = 0; c < w; c++) {
//        if (image[c] == 0 && !ext[c]) // 处理上边缘
//            floodFillExternalBG(ext, image, h, w, 0, c);
//        if (image[(h - 1) * w + c] == 0 && !ext[(h - 1) * w + c]) // 处理下边缘
//            floodFillExternalBG(ext, image, h, w, h - 1, c);
//    }
//    for (int r = 0; r < h; r++) {
//        if (image[r * w] == 0 && !ext[r * w]) // 处理左边缘
//            floodFillExternalBG(ext, image, h, w, r, 0);
//        if (image[r * w + w - 1] == 0 && !ext[r * w + w - 1]) // 处理右边缘
//            floodFillExternalBG(ext, image, h, w, r, w - 1);
//    }
//    return ext;
//}

// 标记外部轮廓
void markOuterContour(uint8_t *image, int h, int w) {
//    bool *externalBG = buildExternalBG(image, h, w);
//
//    // 使用静态数组存储已标记的像素
//    static bool mask[IMG_HEIGHT * IMG_WIDTH]
//             __attribute__((section(".bss.cpu2_dsram")));  // 未初始化数据用.bss
//    for (int i = 0; i < h * w; i++) {
//        mask[i] = false;
//    }
//
//    int dr[4] = { 0, 1, 0, -1 };
//    int dc[4] = { -1, 0, 1, 0 };
//    for (int r = 0; r < h; r++) {
//        for (int c = 0; c < w; c++) {
//            if (image[r * w + c] == 1) {
//                bool isContour = false;
//                for (int d = 0; d < 4; d++) {
//                    int nr = r + dr[d], nc = c + dc[d];
//                    if (nr < 0 || nr >= h || nc < 0 || nc >= w) {
//                        isContour = true;
//                        break;
//                    }
//                    if (image[nr * w + nc] == 0 && externalBG[nr * w + nc]) {
//                        isContour = true;
//                        break;
//                    }
//                }
//                if (isContour)
//                    mask[r * w + c] = true;
//            }
//        }
//    }
//
//    // 根据标记的轮廓点将像素值设为 2
//    for (int r = 0; r < h; r++) {
//        for (int c = 0; c < w; c++) {
//            if (mask[r * w + c])
//                image[r * w + c] = 2;
//        }
//    }
    // 在函数结束后，externalBG 和 mask 会被清理
}


//Contour traceContourMoore(uint8_t *image, int h, int w, int startRow, int startCol, bool *visited) {
//    // 定义 8 个方向的偏移量
//    int DR[8] = {0, -1, -1, -1, 0, 1, 1, 1};
//    int DC[8] = {-1, -1, 0, 1, 1, 1, 0, -1};
//
//    Contour contour;
//    static Point contourPoints[IMG_HEIGHT * IMG_WIDTH]
//           __attribute__((section(".bss.cpu1_dsram")));
//    contour.points = contourPoints;
//    contour.count = 0;
//
//    Point b0 = { startRow, startCol };
//
//    // 标记并加入起始点
//    if (!visited[b0.row * w + b0.col]) {
//        visited[b0.row * w + b0.col] = true;
//        contour.points[contour.count++] = b0;
//    }
//
//    // 找起始点邻居
//    int firstDir = -1;
//    Point b1;
//    for (int d = 0; d < 8; d++) {
//        int nr = b0.row + DR[d];
//        int nc = b0.col + DC[d];
//        if (nr >= 0 && nr < h && nc >= 0 && nc < w && image[nr * w + nc] == 2) {
//            firstDir = d;
//            b1.row = nr;
//            b1.col = nc;
//            break;
//        }
//    }
//    if (firstDir == -1) {
//        // 孤立点，只有自己一个像素，也返回
//        return contour;
//    }
//
//    if (!visited[b1.row * w + b1.col]) {
//        visited[b1.row * w + b1.col] = true;
//        contour.points[contour.count++] = b1;
//    }
//
//    Point init_b0 = b0;
//    Point prev = b0, current = b1;
//    int currentDir = firstDir;
//
//    int max_iterations = h * w;
//    int iterations = 0;
//    bool first = true;
//
//    while (iterations < max_iterations) {
//        iterations++;
//
//        int searchDir = (currentDir + 5) % 8;
//        int foundDir = -1;
//        Point candidate;
//
//        for (int i = 0; i < 8; i++) {
//            int d = (searchDir + i) % 8;
//            int nr = current.row + DR[d];
//            int nc = current.col + DC[d];
//            if (nr < 0 || nr >= h || nc < 0 || nc >= w)
//                continue;
//            if (image[nr * w + nc] == 2) {
//                foundDir = d;
//                candidate.row = nr;
//                candidate.col = nc;
//                break;
//            }
//        }
//
//        if (foundDir == -1) {
//            break;
//        }
//
//        currentDir = foundDir;
//        prev = current;
//        current = candidate;
//
//        // 若回到起点，结束追踪
//        if (!first && current.row == init_b0.row && current.col == init_b0.col) {
//            break;
//        }
//        first = false;
//
//        if (!visited[current.row * w + current.col]) {
//            visited[current.row * w + current.col] = true;
//            contour.points[contour.count++] = current;
//        }
//    }
//
//    return contour;
//}


// 处理图像中的所有轮廓
void processContours(uint8_t *image, int h, int w, ContourInfo *contourInfos, int *numContours,uint8_t* grayimage) {
//    // 使用静态数组标记已访问的点
//    static bool visited[IMG_HEIGHT * IMG_WIDTH]
//        __attribute__((section(".bss.cpu2_dsram")));  // 未初始化数据用.bss
//    for (int i = 0; i < h * w; i++) {
//        visited[i] = false;
//    }
//
//    // 标记外部轮廓
//    markOuterContour(image, h, w);
//
//    // 遍历图像，查找所有轮廓并处理
//    for (int r = 0; r < h; r++) {
//        for (int c = 0; c < w; c++) {
//            if (image[r * w + c] == 2 && !visited[r * w + c]) {
//                Contour cnt = traceContourMoore(image, h, w, r, c, visited);
//                if (cnt.count > 0) {
//                    if (*numContours >= 100) {
////                        printf("轮廓数量超过上限，无法处理更多轮廓\n");
//                        return;
//                    }
//
////                 // 计算圆度
////                    double circularity = calculate_circularity(cnt);
////
////                    // 计算质心
////                    double cx, cy;
////                    calculate_center(cnt, &cx, &cy);
////
////                    // 计算面积
////                    double area = computeArea(cnt);
////
////                    double ratio=computeMinBoundingBoxAspectRatio(cnt);
////                    // 存储轮廓信息
////                    contourInfos[*numContours].circularity = circularity;
////                    contourInfos[*numContours].cx = cx;
////                    contourInfos[*numContours].cy = cy;
////                    contourInfos[*numContours].area = area;
////                    contourInfos[*numContours].boxratio=ratio;
//                    contourInfos[*numContours].num=cnt.count;
//                    calculate_contourinfo(cnt,&contourInfos[*numContours],grayimage);
//                    (*numContours)++;  // 增加轮廓计数
//                }
//            }
//        }
//    }
}

