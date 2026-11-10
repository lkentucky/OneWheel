#if defined(__TASKING__)
#pragma section all "cpu1_dsram"
#pragma section code "beacon_flash"
#endif
#include "beacon_core.h"
#include "beacon_platform.h"

Mono_beacon mono_beacon;
static int previous_mode = -1;

void beacon_reset(void) { previous_mode = -1; }


void beacon_process(const uint8_t *gray, uint8_t (*binary)[188],
                    const beacon_config_t *c, beacon_result_t *r)
{
    int count,selected;
    if(!r) return;
    memset(r,0,sizeof(*r));
    if(!gray || !binary || !c || c->mode<1 || c->mode>3) {previous_mode=-1;return;}
    if(previous_mode!=c->mode) {
        /* Hot paths do not need the old heap-allocated morphology kernels. */
        memset(&imageinfo,0,sizeof(imageinfo));
        memset(&turn_info,0,sizeof(turn_info));
        memset(&mono_beacon,0,sizeof(mono_beacon));
        memset(&best_combined_show,0,sizeof(best_combined_show));
        imageinfo.width=188; imageinfo.height=120; imageinfo.minrow=19;
        cnt=0;
        previous_mode=c->mode;
    }
    imageinfo.image=(uint8_t *)gray; imageinfo.binaryimage=binary;
    imageinfo.contour_filter_sort=imageinfo.contourinfo;
    imageinfo.fixed_threshold=(float)c->threshold;
    imageinfo.domain_radius=(float)c->radius;
    imageinfo.std_var_threshold=c->min_std;
    imageinfo.boxratio_threshold=c->min_ratio;
    imageinfo.near_contour_max_area=c->near_area;
    img_process_params.max_std=40; img_process_params.max_gray=200;
    img_process_params.iou=.01f; img_process_params.min_std=c->min_std;
    img_process_params.min_gray=(float)c->min_gray;
    img_process_params.max_miss=(float)c->max_miss;
    img_process_params.max_distance=c->match_px;
    img_process_params.small_area=20; img_process_params.range_mean=25;
    img_process_params.range_gray=25; img_process_params.max_dx=8; img_process_params.max_dy=8;
    mono_beacon.img_mid_line=c->mid_x; mono_beacon.dynamic_mid_line=c->mid_x;
    mono_beacon.turn_threshold=110; mono_beacon.track_threshold=50;
    turn_info.weight_dist=c->weight_dist; turn_info.weight_angle=c->weight_angle;
    turn_info.weight_first_cx_diff=c->weight_continuity; turn_info.diff_dead_zone=2;
    memset(imageinfo.contourinfo,0,sizeof(imageinfo.contourinfo));
    count=c->mode==1?image_process_distance(&imageinfo,&turn_info):
          c->mode==2?image_process_time(&imageinfo,&turn_info):
                     image_process_time_pro(&imageinfo,&turn_info);
    r->total=(uint8_t)imageinfo.totalContours;
    r->turn=(uint8_t)turn_info.turn_flag; r->track=(uint8_t)turn_info.track_flag;
    if(count<1 || count>MAX_BEACONS) return;
    selected=turn_info.next_beacon-1;
    if(selected<0 || selected>=count) return;
    r->cx=turn_info.cx; r->cy=turn_info.cy;
    r->distance_m=imageinfo.contour_filter[selected].distance;
    r->bearing_deg=imageinfo.contour_filter[selected].angle;
    if(!isfinite(r->cx)||!isfinite(r->cy)||!isfinite(r->distance_m)||
       r->cx<0||r->cx>=188||r->cy<0||r->cy>=120||r->distance_m<=0||r->distance_m>=100) return;
    r->valid=1; r->count=(uint8_t)count; r->next=(uint8_t)(selected+1);
    for(selected=0;selected<count;selected++)r->order[selected]=(uint8_t)best_combined_show.order[selected];
    r->route_distance_m=best_combined_show.total_dist; r->route_angle_deg=best_combined_show.total_angle;
}
