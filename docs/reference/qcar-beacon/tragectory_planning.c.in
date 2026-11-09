/* tragectory_planning.c：信标轨迹规划。创建日期：2025年4月24日；作者：yuan。 */

#include "tragectory_planning.h"

Beacon beacons[MAX_BEACONS];
Turn_info turn_info;
int num_beacons = 0;
float distance_matrix[MAX_BEACONS][MAX_BEACONS];  // 信标角度，单位与调用方保持一致。
Path paths[MAX_PERMUTATIONS];  // 计算相邻信标之间的欧氏距离。
Path best_combined_show;

void polar_to_cartesian(Beacon *b) {
    b->x = b->r * cosf(b->theta);  // 使用 cosf 计算方向分量。
    b->y = b->r * sinf(b->theta);
}


float calc_distance(Beacon a, Beacon b) {
    float dx = a.x - b.x;
    float dy = a.y - b.y;
    return sqrtf(dx*dx + dy*dy);
}

void init_distances() {
    for (int i = 0; i < num_beacons; i++) {
        for (int j = 0; j < num_beacons; j++) {
            distance_matrix[i][j] = (i == j) ? 0 : calc_distance(beacons[i], beacons[j]);
        }
    }
}

float path_total_dist(int order[]) {
    if (num_beacons == 0) return 0;
    float dist = beacons[order[0]-1].r;
    for (int i = 0; i < num_beacons-1; i++) {
        dist += distance_matrix[order[i]-1][order[i+1]-1];
    }
    return dist;
}

float path_total_angle(int order[]) {
    if (num_beacons == 0) return 0.0f;

    float total = 0.0f;
    float prev_angle = PI / 2.0f;  // 初始朝向为 90 度。

    // 计算原点到首个信标的位移。
    if (num_beacons >= 1) {
        Beacon first = beacons[order[0]-1];
        float curr_angle = atan2f(first.y, first.x);
        float diff = fabsf(curr_angle - prev_angle);
        if (diff > PI) diff = 2*PI - diff;
        total += diff;
        prev_angle = curr_angle;
    }

    // 信标位置坐标。
    for (int i = 0; i < num_beacons-1; i++) {
        Beacon from = beacons[order[i]-1];
        Beacon to = beacons[order[i+1]-1];
        float dx = to.x - from.x;
        float dy = to.y - from.y;
        float curr_angle = atan2f(dy, dx);

        float diff = fabsf(curr_angle - prev_angle);
        if (diff > PI) diff = 2*PI - diff;
        total += diff;
        prev_angle = curr_angle;
    }

    return total * (180.0f / PI);  // 转换为角度制。
}

float calc_first_cx_diff(int order[]) {
    if (num_beacons == 0) return 0.0f;

    Beacon first = beacons[order[0]-1];
    return fabsf(first.cx - mono_beacon.dynamic_mid_line)>turn_info.diff_dead_zone?fabsf(first.cx - mono_beacon.dynamic_mid_line):0;
}

void generate_permutations(int *ids, int n) {
    int c[MAX_BEACONS] = {0};
    int i = 0, perm_idx = 0;

    // 计算路径距离与转向代价。
    for (int j = 0; j < n; j++) paths[perm_idx].order[j] = ids[j];
    paths[perm_idx].total_dist = path_total_dist(paths[perm_idx].order);
    paths[perm_idx].total_angle = path_total_angle(paths[perm_idx].order);
    paths[perm_idx].first_cx_diff = calc_first_cx_diff(paths[perm_idx].order);
    perm_idx++;

    while (i < n) {
        if (c[i] < i) {
            int swap_idx = (i % 2) ? 0 : c[i];
            int temp = ids[swap_idx];
            ids[swap_idx] = ids[i];
            ids[i] = temp;

            // 路径中信标的访问顺序。
            for (int j = 0; j < n; j++) paths[perm_idx].order[j] = ids[j];
            paths[perm_idx].total_dist = path_total_dist(paths[perm_idx].order);
            paths[perm_idx].total_angle = path_total_angle(paths[perm_idx].order);
            paths[perm_idx].first_cx_diff = calc_first_cx_diff(paths[perm_idx].order);
            perm_idx++;

            c[i]++;
            i = 0;
        } else {
            c[i] = 0;
            i++;
        }
    }
}

//void input_beacons(ImageInfo* imageinfo)
//{
//    num_beacons = imageinfo->numContours > MAX_BEACONS ? MAX_BEACONS : imageinfo->numContours;
////    for(int i=0; i< MAX_BEACONS; i++){
////        beacons[i].id = 0;
////        beacons[i].r = 0;
////        beacons[i].theta = 0;
////        beacons[i].x = 0;
////        beacons[i].y = 0;
////    }
//    for (int i = 0; i < num_beacons; i++) {
//        beacons[i].r = imageinfo->contour_filter[i].distance;
//        beacons[i].theta = imageinfo->contour_filter[i].angle;
// 历史角度到弧度的转换语句。
//        polar_to_cartesian(&beacons[i]);
// 历史实现：按输入顺序分配从 1 开始的信标编号。
//    }
//}
void input_beacons(ImageInfo* imageinfo)
{
    num_beacons = imageinfo->numContours > MAX_BEACONS ? MAX_BEACONS : imageinfo->numContours;
//    for(int i=0; i< MAX_BEACONS; i++){
//        beacons[i].id = 0;
//        beacons[i].r = 0;
//        beacons[i].theta = 0;
//        beacons[i].x = 0;
//        beacons[i].y = 0;
//    }
    for (int i = 0; i < num_beacons; i++) {
//        beacons[i].x = imageinfo->contour_filter[i].x;
//        beacons[i].y = imageinfo->contour_filter[i].y;
        beacons[i].r = imageinfo->contour_filter[i].distance;
        beacons[i].theta = imageinfo->contour_filter[i].angle;
        beacons[i].theta *= DEG2RAD;  // 将角度转换为弧度。
        beacons[i].cx = imageinfo->contour_filter[i].cx;
        beacons[i].cy = imageinfo->contour_filter[i].cy;
        polar_to_cartesian(&beacons[i]);
        beacons[i].id = i+1;        // 按输入顺序分配信标编号。
    }
}

void tragectory_planning_update(ImageInfo* imageinfo, Turn_info* turn_info)
{
    input_beacons(imageinfo);
// 历史调试：输出信标信息。
// 历史调试输出：信标编号、距离、角度、世界坐标和像素坐标。
//    for (int i = 0; i < num_beacons; i++) {
//        printf("%d\t%.2f\t%.2f\t\t%.2f\t%.2f\t%.2f\t%.2f\n",
//            beacons[i].id,
//            beacons[i].r,
//            beacons[i].theta * RAD2DEG,
//            beacons[i].x,
//            beacons[i].y,
//            beacons[i].cx,
//            beacons[i].cy);
//    }
    init_distances();

    int ids[MAX_BEACONS];
    for (int i = 0; i < num_beacons; i++)
        ids[i] = i+1;

    int num_perms = 1;
    for (int i = 2; i <= num_beacons; i++)
        num_perms *= i;

    generate_permutations(ids, num_beacons);

    float min_dist = paths[0].total_dist;
    float max_dist = paths[0].total_dist;
    float min_angle = paths[0].total_angle;
    float max_angle = paths[0].total_angle;
    float min_first_cx_diff = paths[0].first_cx_diff;
    float max_first_cx_diff = paths[0].first_cx_diff;

    for (int i = 1; i < num_perms; i++) {
        if (paths[i].total_dist < min_dist) min_dist = paths[i].total_dist;
        if (paths[i].total_dist > max_dist) max_dist = paths[i].total_dist;
        if (paths[i].total_angle < min_angle) min_angle = paths[i].total_angle;
        if (paths[i].total_angle > max_angle) max_angle = paths[i].total_angle;
        if (paths[i].first_cx_diff < min_first_cx_diff) min_first_cx_diff = paths[i].first_cx_diff;
        if (paths[i].first_cx_diff > max_first_cx_diff) max_first_cx_diff = paths[i].first_cx_diff;
    }

    // 选择路径综合评分所用权重。
// 历史路径距离权重配置。
// 历史转向角度权重配置。
// 历史首个目标横向偏差权重配置。

    Path *best_combined = &paths[0];
    float min_combined = 1e9;
    float combined =1e9 ;
    for (int i = 0; i < num_perms; i++) {
        // 将各项评价指标归一化到 0～1。
        float norm_dist = (max_dist != min_dist) ?
            (paths[i].total_dist - min_dist) / (max_dist - min_dist) : 0.0;
        float norm_angle = (max_angle != min_angle) ?
            (paths[i].total_angle - min_angle) / (max_angle - min_angle) : 0.0;
        float norm_first_cx_diff = (max_first_cx_diff != min_first_cx_diff) ?
            (paths[i].first_cx_diff - min_first_cx_diff) / (max_first_cx_diff - min_first_cx_diff) : 0.0;

        // 计算综合评分，分值越小越优。
        if(imageinfo->last_numContours==0)
        {
            combined = turn_info->weight_dist * norm_dist
                           + turn_info->weight_angle * norm_angle;
        }
        else
        {
            combined = turn_info->weight_dist * norm_dist
                           + turn_info->weight_angle * norm_angle
                           + turn_info->weight_first_cx_diff * norm_first_cx_diff;
        }
        if (combined < min_combined) {
            min_combined = combined;
            best_combined = &paths[i];
        }
    }
    best_combined_show=*best_combined;
    // 选择综合评分最低的路径。
//    Path min_dist_path = paths[0];
//    Path min_angle_path = paths[0];
//    Path min_first_cx_diff_path = paths[0];
//
//    for (int i = 1; i < num_perms; i++) {
//        if (paths[i].total_dist < min_dist_path.total_dist) min_dist_path = paths[i];
//        if (paths[i].total_angle < min_angle_path.total_angle) min_angle_path = paths[i];
//        if (paths[i].first_cx_diff < min_first_cx_diff_path.first_cx_diff) min_first_cx_diff_path = paths[i];
//    }

// 历史路径评分调试输出。
// 历史调试：输出推荐路径。
// 历史调试：输出最短路径顺序。
// 历史调试：输出综合最优路径的代价。
//           min_dist_path.total_dist, min_dist_path.total_angle, min_dist_path.first_cx_diff);
//
// 历史调试：输出最小转向角路径标题。
// 历史调试：输出最小转向角路径顺序。
// 历史调试：输出转向角、距离和首个目标横向偏差。
//           min_angle_path.total_angle, min_angle_path.total_dist, min_angle_path.first_cx_diff);
//
// 历史调试：输出首个目标横向偏差最小的路径。
// 历史调试：输出该路径的信标顺序。
// 历史调试：输出首个目标横向偏差及路径代价。
//           min_first_cx_diff_path.first_cx_diff, min_first_cx_diff_path.total_dist, min_first_cx_diff_path.total_angle);
//
// 历史调试：输出综合评分的各项权重。
//           weight_dist, weight_angle, weight_first_cx_diff);
// 历史调试：输出综合最优路径顺序。
// 历史调试：输出综合最优路径的代价。
//           best_combined->total_dist, best_combined->total_angle, best_combined->first_cx_diff);
//
// 历史调试：逐段分析转向角。
// 历史调试：输出最优路径的转向分析标题。
// 历史初始朝向：PI/2 弧度。
//
// 计算首段或相邻信标之间的方向。
//     int current_id = best_combined->order[0] - 1;
//     Beacon current = beacons[current_id];
//     float current_dir = atan2(current.y, current.x);
//
//     float diff = current_dir - prev_heading;
//     if (diff > PI) diff -= 2*PI;
//     else if (diff < -PI) diff += 2*PI;
//
//     prev_heading = current_dir;
//
//
// 计算所需转向角，并归一化到有效范围。
//     if (num_beacons >= 2) {
//         int current_id = best_combined->order[1] - 1;
//         Beacon current = beacons[current_id];
//         int prev_id = best_combined->order[0] - 1;
//         Beacon prev = beacons[prev_id];
//         float current_dir = atan2(current.y - prev.y, current.x - prev.x);
//
//         float diff = current_dir - prev_heading;
//         if (diff > PI) diff -= 2*PI;
//         else if (diff < -PI) diff += 2*PI;
//
//        //  turn_info->next_beacon=best_combined->order[0];
// 历史调试：输出相邻信标编号。
//         if (diff >= 0) {
//            //  turn_info->track_flag=left;
// 历史调试：输出当前转向方向。
//         } else {
//            //  turn_info->track_flag=right;
// 历史调试：输出当前转向方向。
//         }
//     }

     float prev_heading = PI / 2.0; // 初始朝向为 PI/2 弧度。

     // 计算首段或相邻信标之间的方向。
     int current_id = best_combined->order[0] - 1;
     Beacon current = beacons[current_id];
     float current_dir = atan2(current.y, current.x);

     float diff = current_dir - prev_heading;
     if (diff > PI) diff -= 2*PI;
     else if (diff < -PI) diff += 2*PI;

     prev_heading = current_dir;


     // 计算所需转向角并归一化。
     turn_info->next_beacon=best_combined->order[0];

     // 保存当前目标的像素坐标。
     int idx = turn_info->next_beacon - 1;  // 将从 1 开始的编号转换为从 0 开始的索引。
     turn_info->cx = imageinfo->contour_filter[idx].cx;
     turn_info->cy = imageinfo->contour_filter[idx].cy;

     if (num_beacons >= 2&&turn_info->cy < mono_beacon.track_threshold) {
         int current_id = best_combined->order[1] - 1;
         Beacon current = beacons[current_id];
         int prev_id = best_combined->order[0] - 1;
         Beacon prev = beacons[prev_id];
         float current_dir = atan2(current.y - prev.y, current.x - prev.x);

         float diff = current_dir - prev_heading;
         if (diff > PI) diff -= 2*PI;
         else if (diff < -PI) diff += 2*PI;

// 历史调试：输出相邻信标编号。
         if (diff >= 0) {
             turn_info->track_flag=left;
// 历史调试：输出当前转向方向。
         } else {
             turn_info->track_flag=right;
// 历史调试：输出当前转向方向。
         }
     }

}
//int main() {
//    input_beacons();
//    if (num_beacons == 0) {
// 历史调试：没有可用的信标路径。
//        return 0;
//    }
//
// 历史调试：输出信标检测结果。
//    for (int i = 0; i < num_beacons; i++) {
//        Beacon b = beacons[i];
// 历史调试：输出目标编号、距离、角度及世界坐标。
//                b.id, b.r, b.theta * (180.0 / PI), b.x, b.y);
//    }
//
//    init_distances();
//
//    int ids[MAX_BEACONS];
//    for (int i = 0; i < num_beacons; i++)
//        ids[i] = i+1;
//
//    int num_perms = 1;
//    for (int i = 2; i <= num_beacons; i++)
//        num_perms *= i;
//
//    Path *paths = malloc(num_perms * sizeof(Path));
//    int index = 0;
//    generate_permutations(ids, 0, num_beacons-1, paths, &index);
//
//    Path min_dist_path = paths[0];
//    Path min_angle_path = paths[0];
//    for (int i = 1; i < num_perms; i++) {
//        if (paths[i].total_dist < min_dist_path.total_dist) min_dist_path = paths[i];
//        if (paths[i].total_angle < min_angle_path.total_angle) min_angle_path = paths[i];
//    }
//
// 历史调试：输出路径距离及其他代价。
//    double min_dist = min_dist_path.total_dist;
//    double max_dist = min_dist_path.total_dist;
//    double min_angle = min_angle_path.total_angle;
//    double max_angle = min_angle_path.total_angle;
//    for (int i = 0; i < num_perms; i++) {
//        if (paths[i].total_dist < min_dist) min_dist = paths[i].total_dist;
//        if (paths[i].total_dist > max_dist) max_dist = paths[i].total_dist;
//        if (paths[i].total_angle < min_angle) min_angle = paths[i].total_angle;
//        if (paths[i].total_angle > max_angle) max_angle = paths[i].total_angle;
//    }
//
// 历史综合评分方案：路径距离权重占 50%。
//    double weight_dist = 0.5;
//    double weight_angle = 0.5;
//    Path *best_combined = &paths[0];
//    double min_combined = 1e9;
//
//    for (int i = 0; i < num_perms; i++) {
//        double norm_dist = (max_dist != min_dist) ?
//            (paths[i].total_dist - min_dist) / (max_dist - min_dist) : 0.0;
//        double norm_angle = (max_angle != min_angle) ?
//            (paths[i].total_angle - min_angle) / (max_angle - min_angle) : 0.0;
//        double combined = weight_dist * norm_dist + weight_angle * norm_angle;
//
//        if (combined < min_combined) {
//            min_combined = combined;
//            best_combined = &paths[i];
//        }
//    }
//
// 历史调试：输出推荐路径。
// 历史调试：输出最短路径顺序。
// 历史调试：输出路径总距离。
//
// 历史调试：输出最小转向角路径标题。
// 历史调试：输出最小转向角路径顺序。
// 历史调试：输出最小总转向角。
// 历史调试：输出综合评分所用权重。
// 历史调试：输出综合最优路径顺序。
// 历史调试：输出路径距离及总转向角。
//        best_combined->total_dist, best_combined->total_angle);
//
// 历史调试：逐段计算路径转向角。
// 历史调试：输出转向分析标题。
// 历史初始朝向：PI/2 弧度。
//
//    for (int i = 0; i < num_beacons; i++) {
//        int current_id = best_combined->order[i] - 1;
//        Beacon current = beacons[current_id];
//
// 计算当前路段的目标朝向。
//        double current_dir;
//        if (i == 0) {
// 计算原点到首个信标的位移。
//            current_dir = atan2(current.y, current.x);
//        } else {
// 计算上一信标到当前信标的位移。
//            int prev_id = best_combined->order[i-1] - 1;
//            Beacon prev = beacons[prev_id];
//            current_dir = atan2(current.y - prev.y, current.x - prev.x);
//        }
//
// 计算目标朝向与上次朝向的角度差。
//        double diff = current_dir - prev_heading;
// 将角度差归一化到 -180 度至 180 度。
//        if (diff > PI) diff -= 2*PI;
//        else if (diff < -PI) diff += 2*PI;
//
// 历史调试：输出当前路段转向信息。
//        if (i == 0) {
// 历史调试：输出原点到首个信标的路段。
//        } else {
// 历史调试：输出相邻信标编号。
//        }
//
//        if (diff > 0) {
// 历史调试：输出右转提示。
//        } else {
// 历史调试：输出右转提示。
//        }
//
// 保存当前朝向，供下一路段计算使用。
//        prev_heading = current_dir;
//    }
//
//    free(paths);
//    return 0;
//}


