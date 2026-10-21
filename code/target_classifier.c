/*
 * target_classifier.c
 *
 *  Created on: 2025年7月15日
 *      Author: Lenovo
 */



#include "target_classifier.h"
#include<stdio.h>
#define Small_Area 20 //大小目标阈值区分
#define ROI_Y0     25     //感兴趣目标下界
#define MAX_ROI_Y0 40
#define MAX_MAX_STD    40   //大目标方差下限
#define MAX_MAX_GRAY   200  //大目标最大灰度下限
#define Box_Ratio  1        //长宽比阈值
#define IOU 0.01             //交并比要求
#define MIN_STD 20          //被激活要求
#define MIN_GRAY  100
#define MAX_TIME_TRACK   QCAR_MAX_CONTOURS //最大维护时间数组数目
#define MAX_MISS 20         //最大丢帧
#define MIN_DISTANCE   3    //最远距离
#define ACT_WARMUP          3      // 滑窗有效样本<3时，不激发（预热期）
#define ACT_DGRAY_MEAN_UP   25     // 当前灰度 - 窗口均值 >= 该阈值 视为“亮沿”
#define ACT_DGRAY_MEAN_DOWN 25     // 窗口均值 - 当前灰度 >= 该阈值 视为“暗沿”
#define ACT_DSTD_MEAN       6      // |当前方差 - 窗口均值方差| >= 该阈值 也可激发（可选）

Img_process_params img_process_params;

float bbox_iou(const Bbox* a, const Bbox* b) {
    int ax1 = a->x, ay1 = a->y;
    int ax2 = a->x + a->w;
    int ay2 = a->y + a->h;

    int bx1 = b->x, by1 = b->y;
    int bx2 = b->x + b->w;
    int by2 = b->y + b->h;

    // 计算交集坐标
    int ix1 = ax1 > bx1 ? ax1 : bx1;
    int iy1 = ay1 > by1 ? ay1 : by1;
    int ix2 = ax2 < bx2 ? ax2 : bx2;
    int iy2 = ay2 < by2 ? ay2 : by2;

    int iw = ix2 - ix1;
    int ih = iy2 - iy1;
    if (iw <= 0 || ih <= 0)
        return 0.0f;

    int inter = iw * ih;
    int area_a = a->w * a->h;
    int area_b = b->w * b->h;
    int union_area = area_a + area_b - inter;

    return (float)inter / (float)union_area;
}


float calc_centroid_distance(const Bbox* a, const Bbox* b) {
    float cx1 = a->x + a->w / 2.0f;
    float cy1 = a->y + a->h / 2.0f;
    float cx2 = b->x + b->w / 2.0f;
    float cy2 = b->y + b->h / 2.0f;

    return sqrtf((cx1 - cx2)*(cx1 - cx2) + (cy1 - cy2)*(cy1 - cy2));
}

//假设所有候选集都未命中，在函数开头执行一次
void SmallTracker_UpdateLife(Time_Small_TrackedTarget* tracked_list, int num_tracked) {
    for (int i = 0; i < num_tracked; i++) {
        tracked_list[i].miss += 1;
    }
}


int Small_Tracker_with_timeinfo(
    Time_Small_TrackedTarget* tracked_list, // 追踪目标数组
    int  *num_tracked,                       // 当前追踪目标数
    ContourInfo* contourinfo                // 当前单个目标
){
    //被某个区域命中
    for(int i = 0; i < *num_tracked; ++i){
        float iou_val = bbox_iou(&tracked_list[i].bbox, &contourinfo->bbox);
        float diatance = calc_centroid_distance(&tracked_list[i].bbox, &contourinfo->bbox);
//        printf("[IOU DEBUG]%d track: (%d,%d,%d,%d), contour: (%d,%d,%d,%d,%f),iou:%f\r\n",i,
//                   tracked_list[i].bbox.x, tracked_list[i].bbox.y,
//                   tracked_list[i].bbox.w, tracked_list[i].bbox.h,
//                   contourinfo->bbox.x, contourinfo->bbox.y,
//                   contourinfo->bbox.w,contourinfo->bbox.h,contourinfo->Std_Varaince,iou_val);
        //具体命中逻辑
        if(iou_val >= img_process_params.iou||diatance <= img_process_params.max_distance){
            //命中：更新 bbox 和 miss
            tracked_list[i].bbox = contourinfo->bbox;
            tracked_list[i].miss = 0;

            // 更新滑动窗口
            tracked_list[i].idx_q = (tracked_list[i].idx_q + 1) % WIN_LEN;
            tracked_list[i].std_q[tracked_list[i].idx_q]  = contourinfo->Std_Varaince;
            tracked_list[i].gray_q[tracked_list[i].idx_q] = contourinfo->max_gray;

            // 重新计算滑动窗口最大值
            uint8_t max_std = 0, max_gray = 0;
            for (int j = 0; j < WIN_LEN; ++j) {
                if (tracked_list[i].std_q[j] > max_std)
                    max_std = tracked_list[i].std_q[j];
                if (tracked_list[i].gray_q[j] > max_gray)
                    max_gray = tracked_list[i].gray_q[j];
            }
//            printf("id_x:%d\r\n",tracked_list[i].idx_q);
//            printf("value\r\n");
            for (int k = 0; k < WIN_LEN; ++k) {
//                   printf("%.1d ",tracked_list[i].gray_q[k]);
            }
//            printf("\r\n");
            tracked_list[i].max_std = max_std;
            tracked_list[i].max_gray = max_gray;
            return 1;
        }
    }
    //如果完全未命中,判断是否满足加入时间序列的条件
    if(contourinfo->Std_Varaince>img_process_params.min_std && contourinfo->max_gray>img_process_params.min_gray){
         // 判断是否超过最大追踪数
         if (*num_tracked<MAX_TIME_TRACK) {
               Time_Small_TrackedTarget *new_track = &tracked_list[*num_tracked];
               new_track->bbox      = contourinfo->bbox;
               new_track->miss      = 0;
               new_track->max_std   = contourinfo->Std_Varaince;
               new_track->max_gray  = contourinfo->max_gray;
               new_track->idx_q     = 0;

               // 初始化滑动窗口
               for (int j = 0; j < 5; ++j) {
                   new_track->std_q[j]  = (j == 0) ? contourinfo->Std_Varaince : 0;
                   new_track->gray_q[j] = (j == 0) ? contourinfo->max_gray     : 0;
               }
               (*num_tracked)++;  // 表示更新了时间序列
               return 1;          // 表示该轮廓是正确的
           }
    }
    //未被命中且不满足新建条件，错误分类
    return 0;
}

int clean_tracked_targets(
    Time_Small_TrackedTarget* tracked_list,
    int* num_tracked
) {
    int alive_idx = 0;
    for (int i = 0; i < *num_tracked; ++i) {
        Time_Small_TrackedTarget* t = &tracked_list[i];

        uint8_t max_std = 0, max_gray = 0;
        for (int j = 0; j < WIN_LEN; ++j) {
            if (t->std_q[j] > max_std)   max_std = t->std_q[j];
            if (t->gray_q[j] > max_gray) max_gray = t->gray_q[j];
        }

        int keep = 1;
        if (t->miss > img_process_params.max_miss){
//            printf("miss\r\n");
            keep = 0;
        }
//      if (t->miss == 0 && (max_std<MIN_STD)){
//            printf("miss_std\r\n");
//            keep = 0;
//        }
        if (t->miss == 0 && (max_std < img_process_params.min_std||max_gray<img_process_params.min_gray)) {
            //printf("[miss_std] 因为滑窗最大std = %.1f < MIN_STD = %d ⇒ 删除该track\n", max_std, MIN_STD);

            // 打印整个滑动窗口内容
//            printf("[miss_std] std_q: ");
//            for (int k = 0; k < WIN_LEN; ++k) {
//                printf("%.1f ", t->std_q[k]);
//            }
//            printf("miss_std\r\n");


        }

        if (keep) {
            if (alive_idx != i) {
                tracked_list[alive_idx] = tracked_list[i];  // 覆盖式保留
            }
            alive_idx++;
        }
    }

    *num_tracked = alive_idx;  // 更新追踪目标数量
    return alive_idx;
}



int classify_targets(
    Time_Small_TrackedTarget* tracked_list,
    int* num_tracked,                       // 当前追踪目标数(候选集数目)
    ContourInfo* contours,
    int numContours,
    ContourInfo* result,
    int maxResult
){
    int result_count=0;//返回值数量
    //假设所有候选集均未被命中
    SmallTracker_UpdateLife(tracked_list,*num_tracked);
    for(int i=0;i<numContours;i++){
        ContourInfo* c = &contours[i];
        //跳过位于图像最上方的目标
        if(c->cy<ROI_Y0){
            continue;
        }
        if(c->distance==100){
            continue;
        }
        if(c->cy>=80&&c->area<imageinfo.near_contour_max_area)
        {
            continue;
        }
//        if(c->cy<=40&&c->area>imageinfo.far_contour_max_area)
//        {
//            continue;
//        }
        //大目标判断逻辑,方差/最大灰度/质心坐标/长宽比
        if(c->area>img_process_params.small_area){
           if(c->Std_Varaince>=img_process_params.max_std &&c->max_gray>=img_process_params.max_gray &&c->boxratio>=imageinfo.boxratio_threshold){
               if (result_count < maxResult){
                         result[result_count++] = *c;
               }
           }
           continue;
        }
        //小目标,依靠维护的时空数组来更新
        else{
            if(Small_Tracker_with_timeinfo(tracked_list,num_tracked,c)){
                result[result_count++] = *c;
            }
            continue;
        }
    }
    clean_tracked_targets(tracked_list,num_tracked);
    return result_count;
}

int is_position_close(const Bbox* a, const Bbox* b, float max_dx, float max_dy) {
    float cx1 = a->x + a->w / 2.0f;
    float cy1 = a->y + a->h / 2.0f;
    float cx2 = b->x + b->w / 2.0f;
    float cy2 = b->y + b->h / 2.0f;

    float dx = fabsf(cx1 - cx2);  // 左右方向
    float dy = fabsf(cy1 - cy2);  // 前后方向

    return dx < max_dx && dy < max_dy;
}


int flicker_gate_c(Time_Small_TrackedTarget *track) {
    int count_mean = 0, count_max = 0;
    int min_mean = 255, max_mean = 0;
    int min_gray = 255, max_gray = 0;

    // 统计平均灰度滑动窗口
    for (int i = 0; i < WIN_LEN; ++i) {
        if (track->mean_gray[i] > 0) {
            count_mean++;
            if (track->mean_gray[i] > max_mean) max_mean = track->mean_gray[i];
            if (track->mean_gray[i] < min_mean) min_mean = track->mean_gray[i];
        }

        if (track->gray_q[i] > 0) {
            count_max++;
            if (track->gray_q[i] > max_gray) max_gray = track->gray_q[i];
            if (track->gray_q[i] < min_gray) min_gray = track->gray_q[i];
        }
    }

    // 数据不足，不做判断
    if (count_mean < 3 || count_max < 3)
        //return 1;
        return 0;

    int range_mean = max_mean - min_mean;
    int range_gray = max_gray - min_gray;

    // 只要平均灰度或最大灰度有一个波动显著即可认为是闪烁
    if (range_mean >= img_process_params.range_mean && range_gray >= img_process_params.range_gray)
        return 1;

    return 0;  // 不闪烁
}

int Small_Tracker_with_timeinfo_pro(
    Time_Small_TrackedTarget* tracked_list, // 追踪目标数组
    int* num_tracked,                       // 当前追踪目标数
    ContourInfo* contourinfo               // 当前单个目标
) {
    // 被某个区域命中
    for (int i = 0; i < *num_tracked; ++i) {
        float iou_val = bbox_iou(&tracked_list[i].bbox, &contourinfo->bbox);
        float distance = calc_centroid_distance(&tracked_list[i].bbox, &contourinfo->bbox);

        //if (iou_val > IOU || distance < MIN_DISTANCE)
        if (iou_val >= img_process_params.iou||distance <= img_process_params.max_distance||is_position_close(&tracked_list[i].bbox, &contourinfo->bbox,img_process_params.max_dx,img_process_params.max_dy)){
            // 命中：更新 bbox 和 miss
            tracked_list[i].bbox = contourinfo->bbox;
            tracked_list[i].miss = 0;

            // 更新滑动窗口
            tracked_list[i].idx_q = (tracked_list[i].idx_q + 1) % WIN_LEN;
            int idx = tracked_list[i].idx_q;

            tracked_list[i].std_q[idx]      = contourinfo->Std_Varaince;
            tracked_list[i].gray_q[idx]     = contourinfo->max_gray;
            tracked_list[i].mean_gray[idx]  = (uint8_t)(contourinfo->ave_gray);

            // 统计最大值与平均灰度
            float max_std = 0;
            uint8_t max_gray = 0;
            int valid_gray_sum = 0;
            int valid_gray_cnt = 0;

            for (int j = 0; j < WIN_LEN; ++j) {
                if (tracked_list[i].std_q[j] > max_std)
                    max_std = tracked_list[i].std_q[j];
                if (tracked_list[i].gray_q[j] > max_gray)
                    max_gray = tracked_list[i].gray_q[j];
                if (tracked_list[i].gray_q[j] > 0) {
                    valid_gray_sum += tracked_list[i].gray_q[j];
                    valid_gray_cnt++;
                }
            }

            tracked_list[i].max_std = max_std;
            tracked_list[i].max_gray = max_gray;

            // （可选）整体灰度平均值
   //         float avg_gray = (valid_gray_cnt > 0) ? (float)valid_gray_sum / valid_gray_cnt : 0.0f;
            // tracked_list[i].avg_gray = avg_gray; // 如果你未来想加 avg 字段

            // === 闪烁判断 ===
            if (!flicker_gate_c(&tracked_list[i])) {
                return 0;  // 不闪烁，排除
            }

            return 1; // 命中并通过
        }
    }

    // 完全未命中，是否创建新 track
    if (contourinfo->Std_Varaince>img_process_params.min_std && contourinfo->max_gray>img_process_params.min_gray) {
        if (*num_tracked < MAX_TIME_TRACK) {
            Time_Small_TrackedTarget* new_track = &tracked_list[*num_tracked];
            new_track->bbox     = contourinfo->bbox;
            new_track->miss     = 0;
            new_track->max_std  = contourinfo->Std_Varaince;
            new_track->max_gray = contourinfo->max_gray;
            new_track->idx_q    = 0;

            for (int j = 0; j < WIN_LEN; ++j) {
                new_track->std_q[j]      = (j == 0) ? contourinfo->Std_Varaince : 0;
                new_track->gray_q[j]     = (j == 0) ? contourinfo->max_gray     : 0;
                new_track->mean_gray[j]  = (j == 0) ? (contourinfo->ave_gray) : 0;
            }

            (*num_tracked)++;
              //return 1;  // 创建成功，默认通过
              return 0;
        }
    }

    // 未命中，且不符合创建条件
    return 0;
}

int clean_tracked_targets_pro(
    Time_Small_TrackedTarget* tracked_list,
    int* num_tracked
) {
    int alive_idx = 0;
    for (int i = 0; i < *num_tracked; ++i) {
        Time_Small_TrackedTarget* t = &tracked_list[i];

        uint8_t max_std = 0, max_gray = 0;
        for (int j = 0; j < WIN_LEN; ++j) {
            if (t->std_q[j] > max_std)   max_std = t->std_q[j];
            if (t->gray_q[j] > max_gray) max_gray = t->gray_q[j];
        }

        int keep = 1;
        if (t->miss > img_process_params.max_miss){
//            printf("miss\r\n");
            keep = 0;
        }
//      if (t->miss == 0 && (max_std<MIN_STD)){
//            printf("miss_std\r\n");
//            keep = 0;
//        }
        if (t->miss == 0 && (max_std < img_process_params.min_std||max_gray<img_process_params.min_gray)) {
            //printf("[miss_std] 因为滑窗最大std = %.1f < MIN_STD = %d ⇒ 删除该track\n", max_std, MIN_STD);

            // 打印整个滑动窗口内容
//            printf("[miss_std] std_q: ");
//            for (int k = 0; k < WIN_LEN; ++k) {
//                printf("%.1f ", t->std_q[k]);
//            }
 //           printf("miss_std\r\n");

            keep=0;
        }

        if (keep) {
            if (alive_idx != i) {
                tracked_list[alive_idx] = tracked_list[i];  // 覆盖式保留
            }
            alive_idx++;
        }
    }

    *num_tracked = alive_idx;  // 更新追踪目标数量
    return alive_idx;
}



int classify_targets_pro(
    Time_Small_TrackedTarget* tracked_list,
    int* num_tracked,                       // 当前追踪目标数(候选集数目)
    ContourInfo* contours,
    int numContours,
    ContourInfo* result,
    int maxResult
){
    int result_count=0;//返回值数量
    //假设所有候选集均未被命中
    SmallTracker_UpdateLife(tracked_list,*num_tracked);
    for(int i=0;i<numContours;i++){
        ContourInfo* c = &contours[i];
        //跳过位于图像最上方的目标
        if(c->cy<ROI_Y0){
            continue;
        }
        if(c->distance==100){
            continue;
        }
        if(c->cy>=80&&c->area<imageinfo.near_contour_max_area)
        {
            continue;
        }
        //大目标判断逻辑,方差/最大灰度/质心坐标/长宽比
        if(c->area>img_process_params.small_area){
           if(c->Std_Varaince>=img_process_params.max_std &&c->max_gray>=img_process_params.max_gray &&c->boxratio>=imageinfo.boxratio_threshold){
               if (result_count < maxResult){
                         result[result_count++] = *c;
               }
           }
           continue;
        }
        //小目标,依靠维护的时空数组来更新
        else{
            if(Small_Tracker_with_timeinfo_pro(tracked_list,num_tracked,c)){
                result[result_count++] = *c;
            }
            continue;
        }
    }
    clean_tracked_targets_pro(tracked_list,num_tracked);
    return result_count;
}
