#ifndef QCAR_DISPLAY_H
#define QCAR_DISPLAY_H
void ips200_show_gray_image_with_highlight(
    uint16 x, uint16 y, const uint8 *image, uint16 width, uint16 height,
    uint16 dis_width, uint16 dis_height, uint8 threshold,
    int16_t highlight_row, int16_t highlight_col);
void ips200_show_gray_image_with_multi_highlight(
    uint16 x, uint16 y, ImageInfo* imageinfo,
    uint16 dis_width, uint16 dis_height,uint8_t threshold);
void ips200_show_gray_image_with_multi_highlight_track(
    uint16 x, uint16 y, ImageInfo* imageinfo,
    uint16 dis_width, uint16 dis_height, uint8_t threshold,
    Turn_info *marker,Path *optimized_path);
void ips200_show_gray_image_with_filterContours(
    uint16 x, uint16 y, ImageInfo* imageinfo,
    uint16 dis_width, uint16 dis_height);
void ips200_show_gray_image_with_filterContours_sort(
    uint16 x, uint16 y, ImageInfo* imageinfo,
    uint16 dis_width, uint16 dis_height);
void ips200_show_gray_image_with_contourinfo(
    uint16 x, uint16 y, ImageInfo* imageinfo,
    uint16 dis_width, uint16 dis_height);
void ips200_show_dilated_image(uint16 x, uint16 y, const uint8 *image, uint16 width, uint16 height, uint16 dis_width, uint16 dis_height);
#endif
