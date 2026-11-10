#ifndef BEACON_CORE_H
#define BEACON_CORE_H
#include <stdint.h>
/* Shared POD snapshots only. Legacy classifier/planner types stay private. */
typedef struct {
    int mode;                 /* 0=lane, 1=distance, 2=time, 3=time-pro */
    int threshold, radius, min_gray, max_miss;
    float min_std, min_ratio, near_area, match_px;
    float speed_mps, yaw_limit_dps, mid_x;
    float weight_dist, weight_angle, weight_continuity;
} beacon_config_t;
typedef struct {
    uint8_t valid, count, total, next, turn, track;
    uint8_t order[3];
    float cx, cy, distance_m, bearing_deg;
    float route_distance_m, route_angle_deg;
} beacon_result_t;
void beacon_defaults(beacon_config_t *config);
void beacon_reset(void); /* CPU1: discard tracking history after camera restart */
/* CPU1 only, exact 188x120 frame; binary borrows the existing vision buffer. */
void beacon_process(const uint8_t *gray, uint8_t (*binary)[188],
                    const beacon_config_t *config, beacon_result_t *result);
#endif
