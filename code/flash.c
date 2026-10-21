#include "flash.h"

//-------------------------------------------------------------------------------------------------------------------
// 检查指定 Flash 页的首个字是否非零。
// sector_num：需要写入的逻辑扇区编号。
// page_num：当前扇区内的物理页编号。
// 返回 1 表示首个字非零，返回 0 表示首个字为零；重写已编程区域前需擦除扇区。
//  @since      v1.0
// 示例：my_flash_check(0, 0)，检查扇区 0 的第 0 页。
//-------------------------------------------------------------------------------------------------------------------
uint8 my_flash_check(uint32 sector_num, uint32 page_num)
{
    zf_assert(QCAR_EEPROM_SECTOR_NUM > sector_num);
    zf_assert(QCAR_EEPROM_SECTOR_PAGE_NUM > page_num);
    uint32 sector_addr = IfxFlash_dFlashTableEepLog[sector_num].start;
    uint32 page_addr = sector_addr + page_num * IFXFLASH_DFLASH_PAGE_LENGTH;

    uint32 temp = *(uint32 *)page_addr;
    if(0 !=temp) return 1;
    return 0;


}

//-------------------------------------------------------------------------------------------------------------------
// 擦除指定逻辑扇区。
// sector_num：需要写入的逻辑扇区编号。
//  @return     void
//  @since      v1.0
//  Sample usage:               eeprom_erase_sector(0);
//-------------------------------------------------------------------------------------------------------------------
void eeprom_erase_sector(uint32 sector_num)
{
    zf_assert(QCAR_EEPROM_SECTOR_NUM > sector_num);

    uint32 flash = 0;
    uint16 end_init_sfty_pw;
    uint32 sector_addr = IfxFlash_dFlashTableEepLog[sector_num].start;

    end_init_sfty_pw = IfxScuWdt_getSafetyWatchdogPassword();

    IfxScuWdt_clearSafetyEndinit(end_init_sfty_pw);
    IfxFlash_eraseSector(sector_addr);
    IfxScuWdt_setSafetyEndinit(end_init_sfty_pw);

    IfxFlash_waitUnbusy(flash, IfxFlash_FlashType_D0);

}

//-------------------------------------------------------------------------------------------------------------------
// 编程指定 Flash 物理页。
// sector_num：需要写入的逻辑扇区编号。
// page_num：扇区内需要写入的物理页编号。
// buf：待写入的 uint32 数据地址。
//  @return     void
//  @since      v1.0
//  Sample usage:               eeprom_page_program(0,0, &buf);
//-------------------------------------------------------------------------------------------------------------------
void eeprom_page_program(uint32 sector_num, uint32 page_num, uint32 *buf)
{
    zf_assert(QCAR_EEPROM_SECTOR_NUM>sector_num);
    zf_assert(QCAR_EEPROM_SECTOR_PAGE_NUM>page_num);

    uint32 errors = 0;
    uint32 flash = 0;
    uint16 end_init_sfty_pw;
    uint32 sector_addr = IfxFlash_dFlashTableEepLog[sector_num].start;

    end_init_sfty_pw = IfxScuWdt_getSafetyWatchdogPassword();

    uint32 page_addr = sector_addr + page_num * IFXFLASH_DFLASH_PAGE_LENGTH;
    errors = IfxFlash_enterPageMode(page_addr);
    zf_assert(0 == errors);

    IfxFlash_waitUnbusy(flash, IfxFlash_FlashType_D0);

    IfxFlash_loadPage(page_addr, buf[0], 0);

    IfxScuWdt_clearSafetyEndinit(end_init_sfty_pw);
    IfxFlash_writePage(page_addr);
    IfxScuWdt_setSafetyEndinit(end_init_sfty_pw);

    IfxFlash_waitUnbusy(flash, IfxFlash_FlashType_D0);

}

/*
 * дflash pid
 */
void pid_write_eeprom(pid_param_t * cascade_vel, pid_param_t * cascade_vel_p, pid_param_t * cascade_ang, pid_param_t * cascade_ang_p, pid_param_t * cascade_ang_vel, pid_param_t * cascade_ang_vel_p,pid_param_t * cascade_vel_y,pid_param_t * cascade_ang_y,pid_param_t * cascade_ang_vel_y)
{
    uint32 write_buf = 0;

    write_buf =my_float_conversion_uint32(cascade_ang_vel->kp);
    eeprom_page_program(0, 1, &write_buf);
    write_buf =my_float_conversion_uint32(cascade_ang_vel->ki);
    eeprom_page_program(0, 2, &write_buf);
    write_buf =my_float_conversion_uint32(cascade_ang_vel->kd);
    eeprom_page_program(0, 3, &write_buf);

    write_buf =my_float_conversion_uint32(cascade_ang->kp);
    eeprom_page_program(0, 4, &write_buf);
    write_buf =my_float_conversion_uint32(cascade_ang->ki);
    eeprom_page_program(0, 5, &write_buf);
    write_buf =my_float_conversion_uint32(cascade_ang->kd);
    eeprom_page_program(0, 6, &write_buf);

    write_buf =my_float_conversion_uint32(cascade_vel->kp);
    eeprom_page_program(0, 7, &write_buf);
    write_buf =my_float_conversion_uint32(cascade_vel->ki);
    eeprom_page_program(0, 8, &write_buf);
    write_buf =my_float_conversion_uint32(cascade_vel->kd);
    eeprom_page_program(0, 9, &write_buf);

    write_buf =my_float_conversion_uint32(cascade_ang_vel_p->kp);
    eeprom_page_program(0, 10, &write_buf);
    write_buf =my_float_conversion_uint32(cascade_ang_vel_p->ki);
    eeprom_page_program(0, 11, &write_buf);
    write_buf =my_float_conversion_uint32(cascade_ang_vel_p->kd);
    eeprom_page_program(0, 12, &write_buf);

    write_buf =my_float_conversion_uint32(cascade_ang_p->kp);
    eeprom_page_program(0, 13, &write_buf);
    write_buf =my_float_conversion_uint32(cascade_ang_p->ki);
    eeprom_page_program(0, 14, &write_buf);
    write_buf =my_float_conversion_uint32(cascade_ang_p->kd);
    eeprom_page_program(0, 15, &write_buf);

    write_buf =my_float_conversion_uint32(cascade_vel_p->kp);
    eeprom_page_program(0, 16, &write_buf);
    write_buf =my_float_conversion_uint32(cascade_vel_p->ki);
    eeprom_page_program(0, 17, &write_buf);
    write_buf =my_float_conversion_uint32(cascade_vel_p->kd);
    eeprom_page_program(0, 18, &write_buf);

    write_buf =my_float_conversion_uint32(cascade_ang_vel_y->kp);
    eeprom_page_program(0, 19, &write_buf);
    write_buf =my_float_conversion_uint32(cascade_ang_vel_y->ki);
    eeprom_page_program(0, 20, &write_buf);
    write_buf =my_float_conversion_uint32(cascade_ang_vel_y->kd);
    eeprom_page_program(0, 21, &write_buf);

    write_buf =my_float_conversion_uint32(cascade_ang_y->kp);
    eeprom_page_program(0, 22, &write_buf);
    write_buf =my_float_conversion_uint32(cascade_ang_y->ki);
    eeprom_page_program(0, 23, &write_buf);
    write_buf =my_float_conversion_uint32(cascade_ang_y->kd);
    eeprom_page_program(0, 24, &write_buf);

}

void mono_params_write_eeprom(Mono_params * mono_params)
{
    uint32 write_buf = 0;

    write_buf =my_float_conversion_uint32(mono_params->setspeed);
    eeprom_page_program(0, 25, &write_buf);
    write_buf =my_float_conversion_uint32(mono_params->balance_angle_x);
    eeprom_page_program(0, 26, &write_buf);
    write_buf =my_float_conversion_uint32(mono_params->balance_angle_y);
    eeprom_page_program(0, 27, &write_buf);
    write_buf =my_float_conversion_uint32(mono_params->turn_k1_l);
    eeprom_page_program(0, 28, &write_buf);
    write_buf =my_float_conversion_uint32(mono_params->turn_k1_r);
    eeprom_page_program(0, 29, &write_buf);
    write_buf =my_float_conversion_uint32(mono_params->dead_zone_p);
    eeprom_page_program(0, 30, &write_buf);
    write_buf =my_float_conversion_uint32(mono_params->turn_k1_p_l);
    eeprom_page_program(0, 31, &write_buf);
    write_buf =my_float_conversion_uint32(mono_params->turn_k2_l);
    eeprom_page_program(0, 32, &write_buf);
    write_buf =my_float_conversion_uint32(mono_params->turn_k2_r);
    eeprom_page_program(0, 33, &write_buf);
    write_buf =my_float_conversion_uint32(mono_params->dead_zone);
    eeprom_page_program(0, 34, &write_buf);
    write_buf =my_float_conversion_uint32(mono_params->turn_k1_p_r);
    eeprom_page_program(0, 35, &write_buf);
    write_buf =my_float_conversion_uint32(mono_params->fast_speed);
    eeprom_page_program(0, 36, &write_buf);
    write_buf =my_float_conversion_uint32(mono_params->slow_speed);
    eeprom_page_program(0, 37, &write_buf);
    write_buf =my_float_conversion_uint32(mono_params->turn_k2_p_l);
    eeprom_page_program(0, 68, &write_buf);
    write_buf =my_float_conversion_uint32(mono_params->turn_k2_p_r);
    eeprom_page_program(0, 69, &write_buf);
    write_buf =my_float_conversion_uint32(mono_params->speed_flag);
    eeprom_page_program(0, 70, &write_buf);
}

void mono_beacon_write_eeprom(Mono_beacon * mono_beacon)
{
    uint32 write_buf = 0;

    write_buf =my_float_conversion_uint32(mono_beacon->yaw_increment);
    eeprom_page_program(0, 38, &write_buf);
    write_buf =my_float_conversion_uint32(mono_beacon->dynamic_zero_threshold);
    eeprom_page_program(0, 39, &write_buf);
    write_buf =my_float_conversion_uint32(mono_beacon->track_threshold);
    eeprom_page_program(0, 40, &write_buf);
    write_buf =my_float_conversion_uint32(mono_beacon->turn_threshold);
    eeprom_page_program(0, 41, &write_buf);
    write_buf =my_float_conversion_uint32(mono_beacon->dynamic_mid_line_kp);
    eeprom_page_program(0, 42, &write_buf);
    write_buf =my_float_conversion_uint32(mono_beacon->dynamic_mid_line_offset);
    eeprom_page_program(0, 43, &write_buf);
    write_buf =my_float_conversion_uint32(mono_beacon->img_mid_line);
    eeprom_page_program(0, 44, &write_buf);
    write_buf =my_float_conversion_uint32(mono_beacon->angle_err_max);
    eeprom_page_program(0, 45, &write_buf);
    write_buf =my_float_conversion_uint32(mono_beacon->dynamic_mid_line_kp_2);
    eeprom_page_program(0, 55, &write_buf);
    write_buf =my_float_conversion_uint32(mono_beacon->dynamic_mid_line_offset_2);
    eeprom_page_program(0, 56, &write_buf);
}

void img_process_write_eeprom(ImageInfo * imageinfo)
{
    uint32 write_buf = 0;

    write_buf =my_float_conversion_uint32(imageinfo->fixed_threshold);
    eeprom_page_program(0, 46, &write_buf);
    write_buf =my_float_conversion_uint32(imageinfo->domain_radius);
    eeprom_page_program(0, 47, &write_buf);
    write_buf =my_float_conversion_uint32(imageinfo->std_var_threshold);
    eeprom_page_program(0, 48, &write_buf);
    write_buf =my_float_conversion_uint32(imageinfo->far_contour_max_area);
    eeprom_page_program(0, 49, &write_buf);
    write_buf =my_float_conversion_uint32(imageinfo->near_contour_max_area);
    eeprom_page_program(0, 50, &write_buf);
    write_buf =my_float_conversion_uint32(imageinfo->near_contour_min_area);
    eeprom_page_program(0, 51, &write_buf);
    write_buf =my_float_conversion_uint32(imageinfo->boxratio_threshold);
    eeprom_page_program(0, 52, &write_buf);
    write_buf =my_float_conversion_uint32(imageinfo->near_threshold);
    eeprom_page_program(0, 53, &write_buf);
    write_buf =my_float_conversion_uint32(imageinfo->far_threshold);
    eeprom_page_program(0, 54, &write_buf);

}

void img_process_write_eeprom_2(Img_process_params * img_process_params)
{
    uint32 write_buf = 0;

    write_buf =my_float_conversion_uint32(img_process_params->max_std);
    eeprom_page_program(0, 57, &write_buf);
    write_buf =my_float_conversion_uint32(img_process_params->max_gray);
    eeprom_page_program(0, 58, &write_buf);
    write_buf =my_float_conversion_uint32(img_process_params->iou);
    eeprom_page_program(0, 59, &write_buf);
    write_buf =my_float_conversion_uint32(img_process_params->min_std);
    eeprom_page_program(0, 60, &write_buf);
    write_buf =my_float_conversion_uint32(img_process_params->min_gray);
    eeprom_page_program(0, 61, &write_buf);
    write_buf =my_float_conversion_uint32(img_process_params->max_miss);
    eeprom_page_program(0, 62, &write_buf);
    write_buf =my_float_conversion_uint32(img_process_params->max_distance);
    eeprom_page_program(0, 63, &write_buf);
    write_buf =my_float_conversion_uint32(img_process_params->img_process_mode);
    eeprom_page_program(0, 64, &write_buf);
    write_buf =my_float_conversion_uint32(img_process_params->small_area);
    eeprom_page_program(0, 71, &write_buf);
    write_buf =my_float_conversion_uint32(img_process_params->range_mean);
    eeprom_page_program(0, 73, &write_buf);
    write_buf =my_float_conversion_uint32(img_process_params->range_gray);
    eeprom_page_program(0, 74, &write_buf);
    write_buf =my_float_conversion_uint32(img_process_params->max_dx);
    eeprom_page_program(0, 75, &write_buf);
    write_buf =my_float_conversion_uint32(img_process_params->max_dy);
    eeprom_page_program(0, 76, &write_buf);
}

void tragectory_planning_write_eeprom(Turn_info *turn_info)
{
    uint32 write_buf = 0;

    write_buf =my_float_conversion_uint32(turn_info->weight_dist);
    eeprom_page_program(0, 65, &write_buf);
    write_buf =my_float_conversion_uint32(turn_info->weight_angle);
    eeprom_page_program(0, 66, &write_buf);
    write_buf =my_float_conversion_uint32(turn_info->weight_first_cx_diff);
    eeprom_page_program(0, 67, &write_buf);
    write_buf =my_float_conversion_uint32(turn_info->diff_dead_zone);
    eeprom_page_program(0, 72, &write_buf);
}

void pid_get_eeprom(pid_param_t * cascade_vel, pid_param_t * cascade_vel_p, pid_param_t * cascade_ang, pid_param_t * cascade_ang_p, pid_param_t * cascade_ang_vel, pid_param_t * cascade_ang_vel_p,pid_param_t * cascade_vel_y,pid_param_t * cascade_ang_y,pid_param_t * cascade_ang_vel_y)
{
    uint32 uint32_data;

    uint32_data =  my_flash_read(0,1,uint32);
    cascade_ang_vel->kp =  my_uint32_conversion_float(uint32_data);
    uint32_data =  my_flash_read(0,2,uint32);
    cascade_ang_vel->ki =  my_uint32_conversion_float(uint32_data);
    uint32_data =  my_flash_read(0,3,uint32);
    cascade_ang_vel->kd =  my_uint32_conversion_float(uint32_data);

    uint32_data =  my_flash_read(0,4,uint32);
    cascade_ang->kp =  my_uint32_conversion_float(uint32_data);
    uint32_data =  my_flash_read(0,5,uint32);
    cascade_ang->ki =  my_uint32_conversion_float(uint32_data);
    uint32_data =  my_flash_read(0,6,uint32);
    cascade_ang->kd =  my_uint32_conversion_float(uint32_data);

    uint32_data =  my_flash_read(0,7,uint32);
    cascade_vel->kp =  my_uint32_conversion_float(uint32_data);
    uint32_data =  my_flash_read(0,8,uint32);
    cascade_vel->ki = my_uint32_conversion_float(uint32_data);
    uint32_data =  my_flash_read(0,9,uint32);
    cascade_vel->kd =  my_uint32_conversion_float(uint32_data);

    uint32_data =  my_flash_read(0,10,uint32);
    cascade_ang_vel_p->kp =  my_uint32_conversion_float(uint32_data);
    uint32_data =  my_flash_read(0,11,uint32);
    cascade_ang_vel_p->ki =  my_uint32_conversion_float(uint32_data);
    uint32_data =  my_flash_read(0,12,uint32);
    cascade_ang_vel_p->kd =  my_uint32_conversion_float(uint32_data);

    uint32_data =  my_flash_read(0,13,uint32);
    cascade_ang_p->kp =  my_uint32_conversion_float(uint32_data);
    uint32_data =  my_flash_read(0,14,uint32);
    cascade_ang_p->ki =  my_uint32_conversion_float(uint32_data);
    uint32_data =  my_flash_read(0,15,uint32);
    cascade_ang_p->kd =  my_uint32_conversion_float(uint32_data);

    uint32_data =  my_flash_read(0,16,uint32);
    cascade_vel_p->kp =  my_uint32_conversion_float(uint32_data);
    uint32_data =  my_flash_read(0,17,uint32);
    cascade_vel_p->ki =  my_uint32_conversion_float(uint32_data);
    uint32_data =  my_flash_read(0,18,uint32);
    cascade_vel_p->kd =  my_uint32_conversion_float(uint32_data);

    uint32_data =  my_flash_read(0,19,uint32);
    cascade_ang_vel_y->kp =  my_uint32_conversion_float(uint32_data);
    uint32_data =  my_flash_read(0,20,uint32);
    cascade_ang_vel_y->ki =  my_uint32_conversion_float(uint32_data);
    uint32_data =  my_flash_read(0,21,uint32);
    cascade_ang_vel_y->kd =  my_uint32_conversion_float(uint32_data);

    uint32_data =  my_flash_read(0,22,uint32);
    cascade_ang_y->kp =  my_uint32_conversion_float(uint32_data);
    uint32_data =  my_flash_read(0,23,uint32);
    cascade_ang_y->ki =  my_uint32_conversion_float(uint32_data);
    uint32_data =  my_flash_read(0,24,uint32);
    cascade_ang_y->kd =  my_uint32_conversion_float(uint32_data);

}

void mono_params_get_eeprom(Mono_params * mono_params)
{
    uint32 uint32_data;

    uint32_data =  my_flash_read(0,25,uint32);
    mono_params->setspeed =  my_uint32_conversion_float(uint32_data);
    uint32_data =  my_flash_read(0,26,uint32);
    mono_params->balance_angle_x =  my_uint32_conversion_float(uint32_data);
    uint32_data =  my_flash_read(0,27,uint32);
    mono_params->balance_angle_y =  my_uint32_conversion_float(uint32_data);
    uint32_data =  my_flash_read(0,28,uint32);
    mono_params->turn_k1_l =  my_uint32_conversion_float(uint32_data);
    uint32_data =  my_flash_read(0,29,uint32);
    mono_params->turn_k1_r =  my_uint32_conversion_float(uint32_data);
    uint32_data =  my_flash_read(0,30,uint32);
    mono_params->dead_zone_p =  my_uint32_conversion_float(uint32_data);
    uint32_data =  my_flash_read(0,31,uint32);
    mono_params->turn_k1_p_l =  my_uint32_conversion_float(uint32_data);
    uint32_data =  my_flash_read(0,32,uint32);
    mono_params->turn_k2_l =  my_uint32_conversion_float(uint32_data);
    uint32_data =  my_flash_read(0,33,uint32);
    mono_params->turn_k2_r =  my_uint32_conversion_float(uint32_data);
    uint32_data =  my_flash_read(0,34,uint32);
    mono_params->dead_zone =  my_uint32_conversion_float(uint32_data);
    uint32_data =  my_flash_read(0,35,uint32);
    mono_params->turn_k1_p_r =  my_uint32_conversion_float(uint32_data);
    uint32_data =  my_flash_read(0,36,uint32);
    mono_params->fast_speed =  my_uint32_conversion_float(uint32_data);
    uint32_data =  my_flash_read(0,37,uint32);
    mono_params->slow_speed =  my_uint32_conversion_float(uint32_data);
    uint32_data =  my_flash_read(0,68,uint32);
    mono_params->turn_k2_p_l =  my_uint32_conversion_float(uint32_data);
    uint32_data =  my_flash_read(0,69,uint32);
    mono_params->turn_k2_p_r =  my_uint32_conversion_float(uint32_data);
    uint32_data =  my_flash_read(0,70,uint32);
    mono_params->speed_flag =  my_uint32_conversion_float(uint32_data);
}

void mono_beacon_get_eeprom(Mono_beacon * mono_beacon)
{
    uint32 uint32_data;

    uint32_data =  my_flash_read(0,38,uint32);
    mono_beacon->yaw_increment =  my_uint32_conversion_float(uint32_data);
    uint32_data =  my_flash_read(0,39,uint32);
    mono_beacon->dynamic_zero_threshold =  my_uint32_conversion_float(uint32_data);
    uint32_data =  my_flash_read(0,40,uint32);
    mono_beacon->track_threshold =  my_uint32_conversion_float(uint32_data);
    uint32_data =  my_flash_read(0,41,uint32);
    mono_beacon->turn_threshold =  my_uint32_conversion_float(uint32_data);
    uint32_data =  my_flash_read(0,42,uint32);
    mono_beacon->dynamic_mid_line_kp =  my_uint32_conversion_float(uint32_data);
    uint32_data =  my_flash_read(0,43,uint32);
    mono_beacon->dynamic_mid_line_offset =  my_uint32_conversion_float(uint32_data);
    uint32_data =  my_flash_read(0,44,uint32);
    mono_beacon->img_mid_line =  my_uint32_conversion_float(uint32_data);
    uint32_data =  my_flash_read(0,45,uint32);
    mono_beacon->angle_err_max =  my_uint32_conversion_float(uint32_data);
    uint32_data =  my_flash_read(0,55,uint32);
    mono_beacon->dynamic_mid_line_kp_2 =  my_uint32_conversion_float(uint32_data);
    uint32_data =  my_flash_read(0,56,uint32);
    mono_beacon->dynamic_mid_line_offset_2 =  my_uint32_conversion_float(uint32_data);
}

void img_process_get_eeprom(ImageInfo * imageinfo)
{
    uint32 uint32_data;

    uint32_data =  my_flash_read(0,46,uint32);
    imageinfo->fixed_threshold =  my_uint32_conversion_float(uint32_data);
    uint32_data =  my_flash_read(0,47,uint32);
    imageinfo->domain_radius =  my_uint32_conversion_float(uint32_data);
    uint32_data =  my_flash_read(0,48,uint32);
    imageinfo->std_var_threshold =  my_uint32_conversion_float(uint32_data);
    uint32_data =  my_flash_read(0,49,uint32);
    imageinfo->far_contour_max_area =  my_uint32_conversion_float(uint32_data);
    uint32_data =  my_flash_read(0,50,uint32);
    imageinfo->near_contour_max_area =  my_uint32_conversion_float(uint32_data);
    uint32_data =  my_flash_read(0,51,uint32);
    imageinfo->near_contour_min_area =  my_uint32_conversion_float(uint32_data);
    uint32_data =  my_flash_read(0,52,uint32);
    imageinfo->boxratio_threshold =  my_uint32_conversion_float(uint32_data);
    uint32_data =  my_flash_read(0,53,uint32);
    imageinfo->near_threshold =  my_uint32_conversion_float(uint32_data);
    uint32_data =  my_flash_read(0,54,uint32);
    imageinfo->far_threshold =  my_uint32_conversion_float(uint32_data);

}

void img_process_get_eeprom_2(Img_process_params * img_process_params)
{
    uint32 uint32_data;

    uint32_data =  my_flash_read(0,57,uint32);
    img_process_params->max_std =  my_uint32_conversion_float(uint32_data);
    uint32_data =  my_flash_read(0,58,uint32);
    img_process_params->max_gray =  my_uint32_conversion_float(uint32_data);
    uint32_data =  my_flash_read(0,59,uint32);
    img_process_params->iou =  my_uint32_conversion_float(uint32_data);
    uint32_data =  my_flash_read(0,60,uint32);
    img_process_params->min_std =  my_uint32_conversion_float(uint32_data);
    uint32_data =  my_flash_read(0,61,uint32);
    img_process_params->min_gray =  my_uint32_conversion_float(uint32_data);
    uint32_data =  my_flash_read(0,62,uint32);
    img_process_params->max_miss =  my_uint32_conversion_float(uint32_data);
    uint32_data =  my_flash_read(0,63,uint32);
    img_process_params->max_distance =  my_uint32_conversion_float(uint32_data);
    uint32_data =  my_flash_read(0,64,uint32);
    img_process_params->img_process_mode =  my_uint32_conversion_float(uint32_data);
    uint32_data =  my_flash_read(0,71,uint32);
    img_process_params->small_area =  my_uint32_conversion_float(uint32_data);
    uint32_data =  my_flash_read(0,73,uint32);
    img_process_params->range_mean =  my_uint32_conversion_float(uint32_data);
    uint32_data =  my_flash_read(0,74,uint32);
    img_process_params->range_gray =  my_uint32_conversion_float(uint32_data);
    uint32_data =  my_flash_read(0,75,uint32);
    img_process_params->max_dx =  my_uint32_conversion_float(uint32_data);
    uint32_data =  my_flash_read(0,76,uint32);
    img_process_params->max_dy =  my_uint32_conversion_float(uint32_data);
}

void tragectory_planning_get_eeprom(Turn_info *turn_info)
{
    uint32 uint32_data;

    uint32_data =  my_flash_read(0,65,uint32);
    turn_info->weight_dist =  my_uint32_conversion_float(uint32_data);
    uint32_data =  my_flash_read(0,66,uint32);
    turn_info->weight_angle =  my_uint32_conversion_float(uint32_data);
    uint32_data =  my_flash_read(0,67,uint32);
    turn_info->weight_first_cx_diff =  my_uint32_conversion_float(uint32_data);
    uint32_data =  my_flash_read(0,72,uint32);
    turn_info->diff_dead_zone =  my_uint32_conversion_float(uint32_data);
}
// 以下为历史 PID 参数写入代码。
//void pid_ec11_write(pid_param_t * cascade_vel, pid_param_t * cascade_vel_p, pid_param_t * cascade_ang, pid_param_t * cascade_ang_p, pid_param_t * cascade_ang_vel, pid_param_t * cascade_ang_vel_p,pid_param_t * cascade_vel_y,pid_param_t * cascade_ang_y,pid_param_t * cascade_ang_vel_y, char count)
//{
//    float ec_step = ec11_change_pid();
//    count = count % 28;
//    switch(count)
//    {
//        case 0: break;
//        case 1 :cascade_vel->kp += ec_step;ips114_show_float(0, 10, cascade_vel->kp, 3, 3);break;
//        case 2 :cascade_vel->ki += ec_step;ips114_show_float(0, 10, cascade_vel->ki, 3, 3);break;
//        case 3 :cascade_vel->kd += ec_step;ips114_show_float(0, 10, cascade_vel->kd, 3, 3);break;
//
//        case 4 :cascade_vel_p->kp += ec_step;ips114_show_float(0, 10, cascade_vel_p->kp, 3, 3);break;
//        case 5 :cascade_vel_p->ki += ec_step;ips114_show_float(0, 10, cascade_vel_p->ki, 3, 3);break;
//        case 6 :cascade_vel_p->kd += ec_step;ips114_show_float(0, 10, cascade_vel_p->kd, 3, 3);break;
//
//        case 7 :cascade_ang->kp += ec_step;ips114_show_float(0, 10, cascade_ang->kp, 3, 3);break;
//        case 8 :cascade_ang->ki += ec_step;ips114_show_float(0, 10, cascade_ang->ki, 3, 3);break;
//        case 9 :cascade_ang->kd += ec_step;ips114_show_float(0, 10, cascade_ang->kd, 3, 3);break;
//
//        case 10 :cascade_ang_p->kp += ec_step;ips114_show_float(0, 10, cascade_ang_p->kp, 3, 3);break;
//        case 11 :cascade_ang_p->ki += ec_step;ips114_show_float(0, 10, cascade_ang_p->ki, 3, 3);break;
//        case 12 :cascade_ang_p->kd += ec_step;ips114_show_float(0, 10, cascade_ang_p->kd, 3, 3);break;
//
//        case 13 :cascade_ang_vel->kp += ec_step;ips114_show_float(0, 10, cascade_ang_vel->kp, 3, 3);break;
//        case 14 :cascade_ang_vel->ki += ec_step;ips114_show_float(0, 10, cascade_ang_vel->ki, 3, 3);break;
//        case 15 :cascade_ang_vel->kd += ec_step;ips114_show_float(0, 10, cascade_ang_vel->kd, 3, 3);break;
//
//        case 16 :cascade_ang_vel_p->kp += ec_step;ips114_show_float(0, 10, cascade_ang_vel_p->kp, 3, 3);break;
//        case 17 :cascade_ang_vel_p->ki += ec_step;ips114_show_float(0, 10, cascade_ang_vel_p->ki, 3, 3);break;
//        case 18 :cascade_ang_vel_p->kd += ec_step;ips114_show_float(0, 10, cascade_ang_vel_p->kd, 3, 3);break;
//
//        case 19 :cascade_vel_y->kp += ec_step;ips114_show_float(0, 10, cascade_vel_y->kp, 3, 3);break;
//        case 20 :cascade_vel_y->ki += ec_step;ips114_show_float(0, 10, cascade_vel_y->ki, 3, 3);break;
//        case 21 :cascade_vel_y->kd += ec_step;ips114_show_float(0, 10, cascade_vel_y->kd, 3, 3);break;
//
//        case 22 :cascade_ang_y->kp += ec_step;ips114_show_float(0, 10, cascade_ang_y->kp, 3, 3);break;
//        case 23 :cascade_ang_y->ki += ec_step;ips114_show_float(0, 10, cascade_ang_y->ki, 3, 3);break;
//        case 24 :cascade_ang_y->kd += ec_step;ips114_show_float(0, 10, cascade_ang_y->kd, 3, 3);break;
//
//        case 25 :cascade_ang_vel_y->kp += ec_step;ips114_show_float(0, 10, cascade_ang_vel_y->kp, 3, 3);break;
//        case 26 :cascade_ang_vel_y->ki += ec_step;ips114_show_float(0, 10, cascade_ang_vel_y->ki, 3, 3);break;
//        case 27 :cascade_ang_vel_y->kd += ec_step;ips114_show_float(0, 10, cascade_ang_vel_y->kd, 3, 3);break;
//    }
//}

void write_flash(void)
{
    eeprom_erase_sector(0);
    pid_write_eeprom(&cascade_vel_roll ,&cascade_vel_pitch ,&cascade_ang_roll ,&cascade_ang_pitch ,&cascade_ang_vel_roll ,&cascade_ang_vel_pitch,&cascade_vel_yaw,&cascade_ang_yaw,&cascade_ang_vel_yaw);
    mono_params_write_eeprom(&mono_params);
    mono_beacon_write_eeprom(&mono_beacon);
    img_process_write_eeprom(&imageinfo);
    img_process_write_eeprom_2(&img_process_params);
    tragectory_planning_write_eeprom(&turn_info);
    uint32 magic = 0x51434131u;
    eeprom_page_program(0, 0, &magic);
}
void get_flash(void)
{
    param_pid_set();
    init_mono_params();
    init_mono_beacon_params();
    init_img_process_params();
    init_img_process_params_2();
    turn_info_init();
    mono_params.speed_flag = slow;
    turn_info.weight_dist = 1.0f;
    turn_info.weight_angle = 1.0f;
    turn_info.weight_first_cx_diff = 1.0f;
    if (my_flash_read(0, 0, uint32) != 0x51434131u) return;
    pid_get_eeprom(&cascade_vel_roll ,&cascade_vel_pitch ,&cascade_ang_roll ,&cascade_ang_pitch ,&cascade_ang_vel_roll ,&cascade_ang_vel_pitch,&cascade_vel_yaw,&cascade_ang_yaw,&cascade_ang_vel_yaw);
    mono_params_get_eeprom(&mono_params);
    mono_beacon_get_eeprom(&mono_beacon);
    img_process_get_eeprom(&imageinfo);
    img_process_get_eeprom_2(&img_process_params);
    tragectory_planning_get_eeprom(&turn_info);
}
