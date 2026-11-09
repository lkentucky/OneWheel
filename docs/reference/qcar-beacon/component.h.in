/* component.h：连通域标记接口。创建日期：2025年5月7日；作者：Lenovo。 */

#ifndef CODE_COMPONENT_H_
#define CODE_COMPONENT_H_

#include "zf_common_headfile.h"

#define HEIGHT 120
#define WIDTH 188
#define MAX_LABELS QCAR_MAX_CONTOURS

/* 八邻域连通域标记，并统计区域面积、质心和灰度。bin 为 WIDTH × HEIGHT 二值图，gray 为同尺寸灰度图，contours 为容量 MAX_LABELS 的输出数组；返回连通域数量。 */
int label_components(const uint8_t bin[HEIGHT][WIDTH],
                     const uint8_t gray[HEIGHT][WIDTH],
                     ContourInfo contours[MAX_LABELS]);
int label_components_change(const uint8_t bin[HEIGHT][WIDTH],
                     const uint8_t gray[HEIGHT][WIDTH],
                     ContourInfo contours[MAX_LABELS],
                     int R) ;
void comgray2binary(const uint8_t input[HEIGHT][WIDTH],
                 uint8_t output[HEIGHT][WIDTH],
                 uint8_t threshold);

#endif /* CODE_COMPONENT_H_ */
