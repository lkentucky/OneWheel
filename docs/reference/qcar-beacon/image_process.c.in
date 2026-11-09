#include"image_process.h"
ImageInfo imageinfo;

int cnt=0;
/* 图像处理函数 */
//int image_process(ImageInfo* imageinfo) {
// 初始化图像丢帧状态。
//    imageinfo->islostframe = 1;
//
// 清空图像直方图。
//    memset(imageinfo->histogram, 0, sizeof(int) * 256);
//
// 首帧初始化上一帧最佳点和最佳轮廓。
//    if (imageinfo->isfirstframe == 1) {
//        imageinfo->last_best_point.col = -1;
//        imageinfo->last_best_point.row = -1;
//        imageinfo->last_best_contour.cx = -1;
//        imageinfo->last_best_contour.cy = -1;
//    }
//
//    // 计算图像的直方图
//    calculateHistogram(imageinfo->image, imageinfo->width, imageinfo->height, imageinfo->histogram);
//
// 检查图像是否含有超过阈值的像素。
//    if (!hasPixelAboveThresholdInHistogram(imageinfo->histogram, imageinfo->maxPixelthreshold)) {
// 未检测到目标且尚未超过丢帧限制时，沿用上一帧最佳点。
//        if (imageinfo->lostframe < imageinfo->maxlostfram) {
//            imageinfo->best_point.col = imageinfo->last_best_point.col;
//            imageinfo->best_point.row = imageinfo->last_best_point.row;
//            imageinfo->best_contour.cx = imageinfo->last_best_contour.cx;
//            imageinfo->best_contour.cy = imageinfo->last_best_contour.cy;
//
//            imageinfo->lostframe++;
//            return 1;
//        } else {
// 超过丢帧限制后重置最佳点和轮廓，并重新进入首帧状态。
//            imageinfo->best_contour.cx = -1;
//            imageinfo->best_contour.cy = -1;
//            imageinfo->best_point.col = -1;
//            imageinfo->best_point.row = -1;
//            imageinfo->lostframe++;
//            imageinfo->isfirstframe = 1;
//            return 2;
//        }
//    }
//
// 使用 Otsu 算法计算二值化阈值。
//    otsuThresholding(imageinfo->histogram,
//                      imageinfo->width, imageinfo->height,
//                      imageinfo->startthreshold, &imageinfo->otusresult);
//
// 将灰度图转换为二值图。
//    gray2binary(imageinfo->image,
//                imageinfo->binaryimage,
//                imageinfo->width, imageinfo->height,
//                imageinfo->otusresult.optimalThreshold);
//
// 根据白色区域比例选择不同尺寸的膨胀核。
////    if (imageinfo->otusresult.whiteAreaRatio < imageinfo->whiteRatiothreshold) {
////        dilate(imageinfo->binaryimage, imageinfo->dilateimage,
////               imageinfo->width, imageinfo->height, &imageinfo->maxsizekernel);
////    } else {
////        dilate(imageinfo->binaryimage, imageinfo->dilateimage,
////               imageinfo->width, imageinfo->height, &imageinfo->minsizekernel);
////    }
//
// 初始化轮廓数量。
//    imageinfo->numContours = 0;
// 提取图像轮廓。
//    processContours(imageinfo->binaryimage, imageinfo->height,
//                    imageinfo->width, imageinfo->contourinfo, &imageinfo->numContours,imageinfo->image);
//
// 初始化临时轮廓信息。
//    ContourInfo tempcontourinfo;
//    tempcontourinfo.cx = -1;
//    tempcontourinfo.cy = -1;
//    tempcontourinfo.circularity = 0;
//
// 首帧或历史最佳点无效时，遍历轮廓选择目标。
//    if (imageinfo->isfirstframe == 1 ||
//        imageinfo->last_best_point.col == -1 ||
//        imageinfo->last_best_point.row == -1) {
//
//        for (int i = 0; i < imageinfo->numContours; i++) {
//            if (/*imageinfo->contourinfo[i].circularity > 0&&
//                imageinfo->contourinfo[i].circularity < 0.9*/
//                /*Rule_Base_Filter(&imageinfo->contourinfo[i])*/
//                SVM_Filter(&imageinfo->contourinfo[i])) {
//
//                if (fabs(imageinfo->contourinfo[i].circularity - 1) <
//                    fabs(tempcontourinfo.circularity - 1)) {
// 更新最佳轮廓信息。
//                    imageinfo->islostframe = 0;
//                    imageinfo->isfirstframe = 0;
//                    imageinfo->lostframe = 0;
//                    tempcontourinfo.cx = imageinfo->contourinfo[i].cx;
//                    tempcontourinfo.cy = imageinfo->contourinfo[i].cy;
//                    tempcontourinfo.circularity = imageinfo->contourinfo[i].circularity;
//                    tempcontourinfo.area = imageinfo->contourinfo[i].area;
//                    tempcontourinfo.boxratio=imageinfo->contourinfo[i].boxratio;
//                    tempcontourinfo.ave_gray=imageinfo->contourinfo[i].ave_gray;
//                    tempcontourinfo.conner_point_num=imageinfo->contourinfo[i].conner_point_num;
//                    tempcontourinfo.Hu[0]=imageinfo->contourinfo[i].Hu[0];
//                    tempcontourinfo.Hu[1]=imageinfo->contourinfo[i].Hu[1];
//                    tempcontourinfo.Hu[2]=imageinfo->contourinfo[i].Hu[2];
//                    tempcontourinfo.Varaince=imageinfo->contourinfo[i].Varaince;
//                    tempcontourinfo.range=imageinfo->contourinfo[i].range;
//                }
//            }
//        }
// 记录当前帧最佳点和最佳轮廓。
//        imageinfo->best_point.col = tempcontourinfo.cx;
//        imageinfo->best_point.row = tempcontourinfo.cy;
//        imageinfo->best_contour.cx = tempcontourinfo.cx;
//        imageinfo->best_contour.cy = tempcontourinfo.cy;
//        imageinfo->best_contour.area = tempcontourinfo.area;
//        imageinfo->best_contour.circularity = tempcontourinfo.circularity;
//        imageinfo->best_contour.boxratio=tempcontourinfo.boxratio;
//        imageinfo->best_contour.ave_gray=tempcontourinfo.ave_gray;
//        imageinfo->best_contour.conner_point_num=tempcontourinfo.conner_point_num;
//        imageinfo->best_contour.Hu[0]=tempcontourinfo.Hu[0];
//        imageinfo->best_contour.Hu[1]=tempcontourinfo.Hu[1];
//        imageinfo->best_contour.Hu[2]=tempcontourinfo.Hu[2];
//        imageinfo->best_contour.Varaince=tempcontourinfo.Varaince;
//        imageinfo->best_contour.range=tempcontourinfo.range;
//
//    }
// 根据与上一帧最佳点的距离选择当前目标。
//    else {
//        float tempdistance;
//        for (int i = 0; i < imageinfo->numContours; i++) {
//            if (/*imageinfo->contourinfo[i].circularity > 0&&
//                imageinfo->contourinfo[i].circularity < 0.9*/
//                /*Rule_Base_Filter(&imageinfo->contourinfo[i])*/
//                SVM_Filter(&imageinfo->contourinfo[i])) {
//
// 更新丢帧状态。
//                imageinfo->islostframe = 0;
//                imageinfo->isfirstframe = 0;
//                imageinfo->lostframe = 0;
//
// 首次取得有效候选时，初始化临时轮廓并计算距离。
//                if (tempcontourinfo.cx == -1 || tempcontourinfo.cy == -1) {
//                    tempcontourinfo.cx = imageinfo->contourinfo[i].cx;
//                    tempcontourinfo.cy = imageinfo->contourinfo[i].cy;
//                    tempdistance = (tempcontourinfo.cx - imageinfo->last_best_point.col) *
//                                   (tempcontourinfo.cx - imageinfo->last_best_point.col) +
//                                   (tempcontourinfo.cy - imageinfo->last_best_point.row) *
//                                   (tempcontourinfo.cy - imageinfo->last_best_point.row);
//                    tempcontourinfo.area = imageinfo->contourinfo[i].area;
//                    tempcontourinfo.circularity = imageinfo->contourinfo[i].circularity;
//                    tempcontourinfo.boxratio=imageinfo->contourinfo[i].boxratio;
//                    tempcontourinfo.ave_gray=imageinfo->contourinfo[i].ave_gray;
//                    tempcontourinfo.conner_point_num=imageinfo->contourinfo[i].conner_point_num;
//                    tempcontourinfo.Varaince=imageinfo->contourinfo[i].Varaince;
//                    tempcontourinfo.range=imageinfo->contourinfo[i].range;
//                    tempcontourinfo.Hu[0]=imageinfo->contourinfo[i].Hu[0];
//                    tempcontourinfo.Hu[1]=imageinfo->contourinfo[i].Hu[1];
//                    tempcontourinfo.Hu[2]=imageinfo->contourinfo[i].Hu[2];
//                    continue;
//                } else {
// 计算当前轮廓到上一帧最佳点的距离。
//                    double distance = (imageinfo->contourinfo[i].cx - imageinfo->last_best_point.col) *
//                                      (imageinfo->contourinfo[i].cx - imageinfo->last_best_point.col) +
//                                      (imageinfo->contourinfo[i].cy - imageinfo->last_best_point.row) *
//                                      (imageinfo->contourinfo[i].cy - imageinfo->last_best_point.row);
//                    if (distance < tempdistance) {
//                        tempdistance = distance;
//                        tempcontourinfo.cx = imageinfo->contourinfo[i].cx;
//                        tempcontourinfo.cy = imageinfo->contourinfo[i].cy;
//                        tempcontourinfo.area = imageinfo->contourinfo[i].area;
//                        tempcontourinfo.circularity = imageinfo->contourinfo[i].circularity;
//                        tempcontourinfo.boxratio=imageinfo->contourinfo[i].boxratio;
//                        tempcontourinfo.conner_point_num=imageinfo->contourinfo[i].conner_point_num;
//                        tempcontourinfo.Hu[0]=imageinfo->contourinfo[i].Hu[0];
//                        tempcontourinfo.Hu[1]=imageinfo->contourinfo[i].Hu[1];
//                        tempcontourinfo.Hu[2]=imageinfo->contourinfo[i].Hu[2];
//                        tempcontourinfo.ave_gray=imageinfo->contourinfo[i].ave_gray;
//                        tempcontourinfo.Varaince=imageinfo->contourinfo[i].Varaince;
//                        tempcontourinfo.range=imageinfo->contourinfo[i].range;
//                    }
//                }
//            }
//        }
// 记录当前帧最佳点和最佳轮廓。
//        imageinfo->best_point.col = tempcontourinfo.cx;
//        imageinfo->best_point.row = tempcontourinfo.cy;
//        imageinfo->best_contour.cx = tempcontourinfo.cx;
//        imageinfo->best_contour.cy = tempcontourinfo.cy;
//        imageinfo->best_contour.area = tempcontourinfo.area;
//        imageinfo->best_contour.circularity = tempcontourinfo.circularity;
//        imageinfo->best_contour.boxratio=tempcontourinfo.boxratio;
//        imageinfo->best_contour.ave_gray=tempcontourinfo.ave_gray;
//        imageinfo->best_contour.Varaince=tempcontourinfo.Varaince;
//        imageinfo->best_contour.range=tempcontourinfo.range;
//        imageinfo->best_contour.conner_point_num=tempcontourinfo.conner_point_num;
//        imageinfo->best_contour.Hu[0]=tempcontourinfo.Hu[0];
//        imageinfo->best_contour.Hu[1]=tempcontourinfo.Hu[1];
//        imageinfo->best_contour.Hu[2]=tempcontourinfo.Hu[2];
//    }
//
// 未找到有效轮廓时累计丢帧数，暂时沿用上一帧最佳点。
//    if (imageinfo->islostframe) {
//        imageinfo->lostframe++;
//        if (imageinfo->lostframe < imageinfo->maxlostfram) {
//            imageinfo->best_point.col = imageinfo->last_best_point.col;
//            imageinfo->best_point.row = imageinfo->last_best_point.row;
//            imageinfo->best_contour.cx = imageinfo->last_best_contour.cx;
//            imageinfo->best_contour.cy = imageinfo->last_best_contour.cy;
//        } else {
//            imageinfo->best_point.col = -1;
//            imageinfo->best_point.row = -1;
//            imageinfo->isfirstframe = 1;
//            imageinfo->best_contour.cx = -1;
//            imageinfo->best_contour.cy = -1;
//        }
//    }
//
// 更新历史最佳点和轮廓信息。
//    imageinfo->last_best_point.col = imageinfo->best_point.col;
//    imageinfo->last_best_point.row = imageinfo->best_point.row;
//    imageinfo->last_best_contour.cx = imageinfo->best_contour.cx;
//    imageinfo->last_best_contour.cy = imageinfo->best_contour.cy;
//
//    return 3;
//}



// 初始化图像处理状态。
void initImageInfo(ImageInfo* imageinfo, int width, int height) {
    // 设置图像宽度和高度。
    imageinfo->width  = width;
    imageinfo->height = height;
    imageinfo->minrow=19;
    // 历史动态图像缓存方案；当前工程使用静态缓存。
    // imageinfo->image        = (uint8_t*)malloc(width * height * sizeof(uint8_t));
    // imageinfo->binaryimage  = (uint8_t*)malloc(width * height * sizeof(uint8_t));
    // imageinfo->dilateimage  = (uint8_t*)malloc(width * height * sizeof(uint8_t));

    // 历史动态内存分配失败检查。
    // if(!imageinfo->image || !imageinfo->binaryimage || !imageinfo->dilateimage) {
    //     fprintf(stderr, "Failed to allocate memory for image buffers.\n");
    //     exit(EXIT_FAILURE);
    // }

    // 设置图像分割和目标判定参数。
    imageinfo->startthreshold     = 60;    //
    imageinfo->whiteRatiothreshold= 0.05;   // 白色像素比例阈值。
    imageinfo->maxPixelthreshold  = 60;    // 最大像素灰度判定阈值。
    imageinfo->islostframe        = 1;      // 当前帧是否丢失目标。
    imageinfo->lostframe          = 0;      // 连续丢失目标的帧数。
    imageinfo->maxlostfram        = 30;     // 允许连续丢失目标的最大帧数。
    imageinfo->isfirstframe       = 1;      // 是否按第一帧进行处理。
    imageinfo->numContours        = 0;      // 当前有效轮廓数量。
    imageinfo->totalContours      = 0;
    imageinfo->mincircularity     = 0.75;    // 允许的最小圆度。
    imageinfo->maxcircularity     = 1.2;    // 允许的最大圆度。

    // 初始化当前及历史最佳点。
    imageinfo->last_best_point.col = -1;
    imageinfo->last_best_point.row = -1;
    imageinfo->best_point.col      = -1;
    imageinfo->best_point.row      = -1;
    imageinfo->sobel_threshold     =80;
    // 清空图像的直方图
    memset(&imageinfo->histogram, 0, sizeof(imageinfo->histogram));

    // 创建小尺寸和大尺寸的形态学核。
    imageinfo->minsizekernel=createEllipseKernel(2);
    imageinfo->maxsizekernel=createEllipseKernel(3);

}

void init_img_process_params(void)
{
    imageinfo.fixed_threshold=60;
    imageinfo.domain_radius=3;
    imageinfo.std_var_threshold=10;
    imageinfo.boxratio_threshold=1;
    imageinfo.far_contour_max_area=10;
    imageinfo.near_contour_max_area=20;
    imageinfo.near_contour_min_area=100;
    imageinfo.near_threshold=80;
    imageinfo.far_threshold=40;
}

void init_img_process_params_2(void)
{
    img_process_params.max_std=40;      //大目标方差下限
    img_process_params.max_gray=200;  //大目标最大灰度下限
    img_process_params.iou=0.01;             //交并比要求
    img_process_params.min_std=20;          //被激活要求
    img_process_params.min_gray=100;
    img_process_params.max_miss=5;         //最大丢帧
    img_process_params.max_distance=1;    //最远距离
}

// 释放图像处理使用的形态学核。
void freeImageInfo(ImageInfo* imageinfo) {
    if (!imageinfo) return;  // 空指针直接返回。

    // 销毁已创建的椭圆形核。
    destroyEllipseKernel(&imageinfo->minsizekernel);
    destroyEllipseKernel(&imageinfo->maxsizekernel);
}

//int image_process_multi(ImageInfo* imageinfo) {
// 清空直方图数组。
//    memset(imageinfo->histogram, 0, sizeof(int) * 256);
//
// 计算图像灰度直方图。
//    calculateHistogram(imageinfo->image, imageinfo->width, imageinfo->height, imageinfo->histogram);
// 检查图像是否含有超过阈值的像素。
//    if (hasPixelAboveThresholdInHistogram(imageinfo->histogram, imageinfo->maxPixelthreshold)==0){
//           imageinfo->numContours=0;
//           return 0;
//    }
// 使用 Otsu 算法计算最佳阈值。
//    otsuThresholding(imageinfo->histogram,
//                     imageinfo->width, imageinfo->height,
//                     imageinfo->startthreshold, &imageinfo->otusresult);
// 将灰度图转换为二值图。
//    gray2binary(imageinfo->image,
//                imageinfo->binaryimage,
//                imageinfo->width, imageinfo->height,
//                imageinfo->otusresult.optimalThreshold);
//
//    int totalContours = 0;
//    processContours(imageinfo->binaryimage, imageinfo->height,
//                    imageinfo->width, imageinfo->contourinfo, &totalContours, imageinfo->image);
////
// 筛选满足 SVM 条件的轮廓并保存结果。
////    int filteredCount = 0;
////    for (int i = 0; i < totalContours; i++) {
////        if (   imageinfo->contourinfo[i].circularity > 0&&imageinfo->contourinfo[i].circularity < 0.9/*SVM_Filter(&imageinfo->contourinfo[i])*/) {
// 保留满足条件的当前轮廓。
////            if (filteredCount != i) {
////                imageinfo->contour_filter[filteredCount] = imageinfo->contourinfo[i];
////            }
////            filteredCount++;
////        }
////    }
//    imageinfo->numContours = filteredCount;
//
// 返回筛选后的轮廓数量。
//    return filteredCount;
//    //return totalContours;
//}


// 供 qsort 使用：按 distance 从小到大排序。
static int compareContoursByDistance(const void *a, const void *b) {
    const ContourInfo *contourA = *(const ContourInfo **)a;
    const ContourInfo *contourB = *(const ContourInfo **)b;
    if (contourA->distance < contourB->distance)
        return -1;
    else if (contourA->distance > contourB->distance)
        return 1;
    else
        return 0;
}

// 计算两个图像点之间的距离。
float calculateDistance(float cx1, float cy1, float cx2, float cy2) {
    return sqrt((cx2 - cx1) * (cx2 - cx1) + (cy2 - cy1) * (cy2 - cy1));
}

int image_process_distance(ImageInfo* imageinfo,Turn_info* turn_info) {
    // 将灰度图转换为二值图。
    comgray2binary((const uint8_t (*)[WIDTH])imageinfo->image,imageinfo->binaryimage,imageinfo->fixed_threshold);

    // 提取连通域并记录数量。
   imageinfo->totalContours=label_components_change(imageinfo->binaryimage,(const uint8_t (*)[WIDTH])imageinfo->image,imageinfo->contourinfo,imageinfo->domain_radius);
    // 临时保存筛选后的轮廓指针。
    ContourInfo* filteredPtrs[QCAR_MAX_CONTOURS];
    int filteredCount = 0;
    for (int i = 0; i < imageinfo->totalContours; i++)
    {
        if(imageinfo->contourinfo[i].distance==100){
            continue;
        }
//        if(imageinfo->contourinfo[i].cy<=40&&imageinfo->contourinfo[i].area>imageinfo->far_contour_max_area){
//            continue;
//        }
        if(imageinfo->contourinfo[i].cy>=80&&imageinfo->contourinfo[i].area<imageinfo->near_contour_max_area){
            continue;
        }
        if (imageinfo->contourinfo[i].cy>=25&&imageinfo->contourinfo[i].boxratio>=imageinfo->boxratio_threshold&&imageinfo->contourinfo[i].Std_Varaince>=imageinfo->std_var_threshold)
        {
            filteredPtrs[filteredCount++] = &imageinfo->contourinfo[i];
            continue;
        }
    }
    if (filteredCount == 0) {
        imageinfo->numContours = 0;
        imageinfo->last_numContours=0;
        imageinfo->filterContours=0;
        cnt++;
        if(cnt==20)
        {
            if(turn_info->cy<mono_beacon.turn_threshold)
            {
                if(turn_info->cx<mono_beacon.img_mid_line)
                {
                    turn_info->turn_flag=right;
                }
                else
                {
                    turn_info->turn_flag=left;
                }
            }
        }
        return 0;
    }

    imageinfo->filterContours = filteredCount;

    qsort(filteredPtrs, filteredCount, sizeof(ContourInfo*), compareContoursByDistance);

    for (int i = 0; i < imageinfo->filterContours; i++) {
        imageinfo->contour_filter[i] = *filteredPtrs[i];
    }

    imageinfo->numContours = imageinfo->filterContours < MAX_BEACONS ? imageinfo->filterContours : MAX_BEACONS;

//    if(turn_info->cy<mono_beacon.track_threshold)
//    {
        tragectory_planning_update(imageinfo,turn_info);
//    }
//    else
//    {
//        /* 最近邻：找离上帧 (turn_info.cx, turn_info.cy) 最近的轮廓 */
//        float min_d2 = 1e9;
//        int   best   = turn_info->next_beacon - 1;   // 默认用原编号
//        for (int i = 0; i < imageinfo->numContours; ++i) {
//            float dx = imageinfo->contour_filter[i].cx - turn_info->cx;
//            float dy = imageinfo->contour_filter[i].cy - turn_info->cy;
//            float d2 = dx*dx + dy*dy;
//            if (d2 < min_d2) { min_d2 = d2; best = i; }
//        }
//        /* 更新轨迹编号和参考坐标 */
//        turn_info->next_beacon = best + 1;                // 1-based
//        turn_info->cx = imageinfo->contour_filter[best].cx;
//        turn_info->cy = imageinfo->contour_filter[best].cy;
//    }

    imageinfo->last_numContours=imageinfo->numContours;

    return imageinfo->numContours;
}


int image_process_time(ImageInfo* imageinfo, Turn_info* turn_info) {

    // 1. 图像预处理：二值化
//    if (imageinfo->dayflag == 1) {
//        // 如果白天模式需要调用其他处理逻辑，可在此扩展
//        // process_roi_tophat_thresh(imageinfo->image, imageinfo->binaryimage);
//    } else {
        comgray2binary((const uint8_t (*)[WIDTH])imageinfo->image, imageinfo->binaryimage, imageinfo->fixed_threshold);
//    }

    // 2. 提取连通域
    imageinfo->totalContours = label_components_change(
        imageinfo->binaryimage,
        (const uint8_t (*)[WIDTH])imageinfo->image,
        imageinfo->contourinfo,
        imageinfo->domain_radius
    );

    // 3. 分类：基于时间追踪信息的多帧稳定检测
    imageinfo->filterContours = classify_targets(
        imageinfo->time_small_track_target,
        &imageinfo->time_num,                       // 当前追踪目标数(候选集)
        imageinfo->contourinfo,
        imageinfo->totalContours,
        imageinfo->contour_filter_sort,
        QCAR_MAX_CONTOURS
    );

    // 4. 如果没有有效目标 → 回退逻辑
    if (imageinfo->filterContours == 0) {
        imageinfo->numContours = 0;
        imageinfo->last_numContours=0;
        cnt++;
        if(cnt==20)
        {
            cnt=0;
            if (turn_info->turn_flag==0)
            {
                if (turn_info->cx >= mono_beacon.img_mid_line||turn_info->cx==0||turn_info->cy==0) {
                    turn_info->turn_flag = left;
                }
                else {
                    turn_info->turn_flag = right;
                }
            }
        }
        return 0;
    }

    // 5. 对分类结果排序，按距离选取最优目标
    ContourInfo* filteredPtrs[QCAR_MAX_CONTOURS];
    int filteredCount = 0;
    for (int i = 0; i < imageinfo->filterContours && i < QCAR_MAX_CONTOURS; i++) {
        filteredPtrs[filteredCount++] = &imageinfo->contour_filter_sort[i];
    }

    qsort(filteredPtrs, filteredCount, sizeof(ContourInfo*), compareContoursByDistance);

    // 最多保留前MAX_BEACONS个
    for (int i = 0; i < filteredCount; i++) {
        imageinfo->contour_filter[i] = *filteredPtrs[i];
    }

    imageinfo->numContours = filteredCount < MAX_BEACONS ? filteredCount : MAX_BEACONS;;

    // 6. **轨迹更新逻辑**（新增）
    // 目标在远距离 → 按规划更新下一目标
    tragectory_planning_update(imageinfo,turn_info);

    imageinfo->last_numContours=imageinfo->numContours;

    return imageinfo->numContours;
}

int image_process_time_pro(ImageInfo* imageinfo, Turn_info* turn_info) {

    // 1. 图像预处理：二值化
//    if (imageinfo->dayflag == 1) {
//        // 如果白天模式需要调用其他处理逻辑，可在此扩展
//        // process_roi_tophat_thresh(imageinfo->image, imageinfo->binaryimage);
//    } else {
        comgray2binary((const uint8_t (*)[WIDTH])imageinfo->image, imageinfo->binaryimage, imageinfo->fixed_threshold);
//    }

    // 2. 提取连通域
    imageinfo->totalContours = label_components_change(
        imageinfo->binaryimage,
        (const uint8_t (*)[WIDTH])imageinfo->image,
        imageinfo->contourinfo,
        imageinfo->domain_radius
    );

    // 3. 分类：基于时间追踪信息的多帧稳定检测
    imageinfo->filterContours = classify_targets_pro(
        imageinfo->time_small_track_target,
        &imageinfo->time_num,                       // 当前追踪目标数(候选集)
        imageinfo->contourinfo,
        imageinfo->totalContours,
        imageinfo->contour_filter_sort,
        QCAR_MAX_CONTOURS
    );

    // 4. 如果没有有效目标 → 回退逻辑
    if (imageinfo->filterContours == 0) {
        imageinfo->numContours = 0;
        imageinfo->last_numContours=0;
        cnt++;
        if(cnt==20)
        {
            cnt=0;
            if (turn_info->turn_flag==0)
            {
                if (turn_info->cx >= mono_beacon.img_mid_line||turn_info->cx==0||turn_info->cy==0) {
                    turn_info->turn_flag = left;
                }
                else {
                    turn_info->turn_flag = right;
                }
            }
        }
        return 0;
    }

    // 5. 对分类结果排序，按距离选取最优目标
    ContourInfo* filteredPtrs[QCAR_MAX_CONTOURS];
    int filteredCount = 0;
    for (int i = 0; i < imageinfo->filterContours && i < QCAR_MAX_CONTOURS; i++) {
        filteredPtrs[filteredCount++] = &imageinfo->contour_filter_sort[i];
    }

    qsort(filteredPtrs, filteredCount, sizeof(ContourInfo*), compareContoursByDistance);

    // 最多保留前MAX_BEACONS个
    for (int i = 0; i < filteredCount; i++) {
        imageinfo->contour_filter[i] = *filteredPtrs[i];
    }

    imageinfo->numContours = filteredCount < MAX_BEACONS ? filteredCount : MAX_BEACONS;;

    // 6. **轨迹更新逻辑**（新增）
    // 目标在远距离 → 按规划更新下一目标
    tragectory_planning_update(imageinfo,turn_info);

    imageinfo->last_numContours=imageinfo->numContours;

    return imageinfo->numContours;
}

