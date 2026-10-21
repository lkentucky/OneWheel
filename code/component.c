/* component.c：连通域标记。创建日期：2025年5月7日；作者：Lenovo。 */

#include "component.h"
/* 静态并查集。 */
static int parent[MAX_LABELS];
static inline int uf_find(int x)
{
    /* 路径压缩。 */
    while (parent[x] != x) x = parent[x] = parent[parent[x]];
    return x;
}
static inline void uf_union(int a, int b)
{
    a = uf_find(a); b = uf_find(b);
    if (a != b) parent[b] = a;
}

/* 静态标签工作区与统计结构。 */
static uint16_t labels[HEIGHT*WIDTH] __attribute__((section(".bss.cpu0_dsram")));                    /* 每个像素的连通域标签。 */

typedef struct {
    int cnt;
    int sx, sy;
    long long int sg;           // sum of gray
    long long int sgsq;         // sum of gray^2
    uint8_t maxg;     // max gray
    uint8_t ming;     // min gray
    int16 minx, maxx, miny, maxy;
} Acc;

static Acc acc[MAX_LABELS];                          /* 各临时标签的累计统计量。 */

//int label_components(const uint8_t bin[HEIGHT][WIDTH],
//                     const uint8_t gray[HEIGHT][WIDTH],
//                     ContourInfo contours[MAX_LABELS])
//{
// 历史实现：初始化标签、统计量与并查集。
//    memset(labels, 0, sizeof(labels));
//    memset(acc, 0, sizeof(acc));
//    for (int i = 0; i < MAX_LABELS; ++i) parent[i] = i;
//
//    uint16_t next_label = 1;
//
// 历史实现第一遍扫描：分配标签、合并连通域并累计统计。
//    for (int y = 0; y < HEIGHT; ++y) {
//        for (int x = 0; x < WIDTH; ++x) {
//            if (bin[y][x] == 0) {
//                labels[y * WIDTH + x] = 0;
//                continue;
//            }
//
// 读取已扫描邻居的标签。
//            uint16_t nb_l  = (x > 0)             ? labels[y * WIDTH + (x - 1)]     : 0;
//            uint16_t nb_u  = (y > 0)             ? labels[(y - 1) * WIDTH + x]     : 0;
//            uint16_t nb_ul = (y > 0 && x > 0)    ? labels[(y - 1) * WIDTH + (x - 1)] : 0;
//            uint16_t nb_ur = (y > 0 && x < WIDTH - 1) ? labels[(y - 1) * WIDTH + (x + 1)] : 0;
//
// 从相邻像素的有效标签中选择最小标签。
//            uint16_t lab = 0;
//            if (nb_l)  lab = nb_l;
//            if (nb_u  && (!lab || nb_u  < lab)) lab = nb_u;
//            if (nb_ul && (!lab || nb_ul < lab)) lab = nb_ul;
//            if (nb_ur && (!lab || nb_ur < lab)) lab = nb_ur;
//            if (!lab) { if (next_label >= MAX_LABELS) continue; lab = next_label++; }
//
//            labels[y * WIDTH + x] = lab;
//
// 合并相邻标签对应的并查集。
//            if (nb_l)  uf_union(lab, nb_l);
//            if (nb_u)  uf_union(lab, nb_u);
//            if (nb_ul) uf_union(lab, nb_ul);
//            if (nb_ur) uf_union(lab, nb_ur);
//
// 累计区域统计量。
//            Acc *a = &acc[lab];
//            a->cnt += 1;
//            a->sx  += x;
//            a->sy  += y;
//            a->sg  += gray[y][x];
//        }
//    }
//
//    printf("Next label: %d\r\n", next_label);
// 历史实现第二遍扫描：统一根标签并聚合统计量。
//    uint16_t final_id[MAX_LABELS] = {0};
//    int label_cnt = 0;
//
//    for (uint16_t l = 1; l < next_label; ++l) {
//        if (!acc[l].cnt) continue;
//        uint16_t root = uf_find(l);
//        if (!final_id[root]) final_id[root] = ++label_cnt;
//        if (l != root) {
//            acc[root].cnt += acc[l].cnt;
//            acc[root].sx  += acc[l].sx;
//            acc[root].sy  += acc[l].sy;
//            acc[root].sg  += acc[l].sg;
//        }
//    }
//
// 将统计结果写入输出轮廓数组。
//    for (uint16_t root = 1; root < next_label; ++root) {
//        int id = final_id[root];
//        if (!id) continue;
//        Acc *a = &acc[root];
//        ContourInfo *c = &contours[id - 1];
//        c->area     = (float)a->cnt;
//        c->cx       = (float)a->sx / c->area;
//        c->cy       = (float)a->sy / c->area;
//        c->ave_gray = (float)a->sg / c->area;
// 历史可选位置估计调用。
//    }
//
//    return label_cnt;
//}


int label_components(const uint8_t bin[HEIGHT][WIDTH],
                     const uint8_t gray[HEIGHT][WIDTH],
                     ContourInfo contours[MAX_LABELS])
{
    /* 初始化标签与统计缓存。 */
    memset(labels, 0, sizeof(labels));
    memset(acc,    0, sizeof(acc));
    for (int i = 0; i < MAX_LABELS; ++i) {
        parent[i] = i;
        acc[i].minx =  WIDTH;   /* 背景像素。 */
        acc[i].miny =  HEIGHT;
        acc[i].maxx = -1;       /* 选择最小的有效邻居标签。 */
        acc[i].maxy = -1;
    }

    uint16_t next_label = 1;

    /* 第一次扫描：分配临时标签，并通过并查集合并连通区域。 */
    for (uint8_t y = 0; y < HEIGHT; ++y) {
        for (uint8_t x = 0; x < WIDTH; ++x) {

            if (bin[y][x] == 0) {        /* 背景像素。 */
                labels[y * WIDTH + x] = 0;
                continue;
            }

            /* 检查左、上、左上和右上四个已扫描邻居，形成八连通关系。 */
            uint16_t nb_l  = (x > 0) ? labels[y * WIDTH + (x - 1)] : 0;
            uint16_t nb_u  = (y > 0) ? labels[(y - 1) * WIDTH + x] : 0;
            uint16_t nb_ul = (y > 0 && x > 0) ? labels[(y - 1) * WIDTH + (x - 1)] : 0;
            uint16_t nb_ur = (y > 0 && x < WIDTH - 1) ? labels[(y - 1) * WIDTH + (x + 1)] : 0;

            /* 选择邻域中最小的非零标签。 */
            uint16_t lab = 0;
            if (nb_l)  lab = nb_l;
            if (nb_u  && (!lab || nb_u  < lab)) lab = nb_u;
            if (nb_ul && (!lab || nb_ul < lab)) lab = nb_ul;
            if (nb_ur && (!lab || nb_ur < lab)) lab = nb_ur;
            if (!lab) { if (next_label >= MAX_LABELS) continue; lab = next_label++; }

            labels[y * WIDTH + x] = lab;

            /* 合并邻居标签。 */
            if (nb_l)  uf_union(lab, nb_l);
            if (nb_u)  uf_union(lab, nb_u);
            if (nb_ul) uf_union(lab, nb_ul);
            if (nb_ur) uf_union(lab, nb_ur);

            /* 累计像素数量、坐标与灰度统计。 */
            Acc *a = &acc[lab];
            a->cnt += 1;
            a->sx  += x;
            a->sy  += y;
            a->sg  += gray[y][x];

            uint8_t g = gray[y][x];
            a->sgsq += g * g;
            if (g > a->maxg) a->maxg = g;

            if (x < a->minx) a->minx = x;
            if (x > a->maxx) a->maxx = x;
            if (y < a->miny) a->miny = y;
            if (y > a->maxy) a->maxy = y;
        }
    }

    /* 第二遍扫描：合并根标签的统计结果。 */
    uint16_t final_id[MAX_LABELS] = {0};
    uint16_t      label_cnt = 0;

    for (uint16_t l = 1; l < next_label; ++l) {

        if (!acc[l].cnt) continue;          /* 跳过没有像素统计的标签。 */

        int root = uf_find(l);
        if (!final_id[root]) final_id[root] = ++label_cnt;

        if (l != root) {                    /* 将当前标签统计合并到根标签。 */
            acc[root].cnt += acc[l].cnt;
            acc[root].sx  += acc[l].sx;
            acc[root].sy  += acc[l].sy;
            acc[root].sg  += acc[l].sg;
            if (acc[l].maxg > acc[root].maxg) acc[root].maxg = acc[l].maxg;
            if (acc[l].minx < acc[root].minx) acc[root].minx = acc[l].minx;
            if (acc[l].maxx > acc[root].maxx) acc[root].maxx = acc[l].maxx;
            if (acc[l].miny < acc[root].miny) acc[root].miny = acc[l].miny;
            if (acc[l].maxy > acc[root].maxy) acc[root].maxy = acc[l].maxy;
        }
    }

    /* 生成各连通域的输出信息。 */
    for (uint16_t root = 1; root < next_label; ++root) {
        int id = final_id[root];
        if (!id) continue;                  /* 跳过无效根标签。 */

        Acc *a = &acc[root];
        ContourInfo *c = &contours[id - 1];

        c->area     = (float)a->cnt;
        c->cx       = (float)a->sx / c->area;
        c->cy       = (float)a->sy / c->area;
        c->ave_gray = (float)a->sg / c->area;

        double mean    = (double)a->sg / a->cnt;
        double mean_sq = (double)a->sgsq / a->cnt;
        if(mean_sq>=mean * mean)
        {
            c->Std_Varaince = sqrtf(mean_sq - mean * mean);  // 计算灰度标准差。
        }
        int w = a->maxx - a->minx + 1;
        int h = a->maxy - a->miny + 1;
        c->boxratio = h ? (float)w / (float)h : 0.0f;

//      location_estimate(c);
        float world[3];
        pixel2world(c->cx,c->cy,world,c);
    }

    return label_cnt;   /* 指定邻域半径的连通域标记实现。 */
}

int label_components_change(const uint8_t bin[HEIGHT][WIDTH],
                     const uint8_t gray[HEIGHT][WIDTH],
                     ContourInfo contours[MAX_LABELS],
                     int R)
{
    /* 限制邻域半径，防止访问图像范围之外。 */
    if (R < 1) R = 1;
    if (R > HEIGHT - 1) R = HEIGHT - 1;
    if (R > WIDTH  - 1) R = WIDTH  - 1;

    /* 初始化标签与统计缓存。 */
    memset(labels, 0, sizeof(labels));
    memset(acc,    0, sizeof(acc));
    for (int i = 0; i < MAX_LABELS; ++i) {
        parent[i] = i;
        acc[i].minx =  WIDTH;
        acc[i].miny =  HEIGHT;
        acc[i].maxx = -1;
        acc[i].maxy = -1;
    }

    uint16_t next_label = 1;

    /* 第一次扫描：分配标签并合并指定邻域内的连通区域。 */
    for (uint8_t y = 0; y < HEIGHT; ++y) {
        for (uint8_t x = 0; x < WIDTH; ++x) {

            if (bin[y][x] == 0) {                    /* 背景像素。 */
                labels[y * WIDTH + x] = 0;
                continue;
            }

            /* 在已扫描的邻域内查找最小非零标签。 */
            uint16_t lab = 0;
            for (int dr = -R; dr <= 0; ++dr) {
                int yy = y + dr;
                if (yy < 0) continue;

                int dc_left  = -R;
                int dc_right = (dr == 0) ? -1 :  R;  /* 当前行只查找已扫描过的左侧像素。 */

                for (int dc = dc_left; dc <= dc_right; ++dc) {
                    if (dr == 0 && dc == 0) continue;
                    int xx = x + dc;
                    if (xx < 0 || xx >= WIDTH) continue;

                    uint16_t nb = labels[yy * WIDTH + xx];
                    if (nb && (!lab || nb < lab))
                        lab = nb;
                }
            }
            if (!lab) { if (next_label >= MAX_LABELS) continue; lab = next_label++; }            /* 没有可用邻居时分配新标签。 */
            labels[y * WIDTH + x] = lab;

            /* 将当前像素与邻域中的已扫描像素合并。 */
            for (int dr = -R; dr <= 0; ++dr) {
                int yy = y + dr;
                if (yy < 0) continue;

                int dc_left  = -R;
                int dc_right = (dr == 0) ? -1 :  R;

                for (int dc = dc_left; dc <= dc_right; ++dc) {
                    if (dr == 0 && dc == 0) continue;
                    int xx = x + dc;
                    if (xx < 0 || xx >= WIDTH) continue;

                    uint16_t nb = labels[yy * WIDTH + xx];
                    if (nb) uf_union(lab, nb);
                }
            }

            /* 累计区域统计量。 */
            Acc *a = &acc[lab];
            a->cnt  += 1;
            a->sx   += x;
            a->sy   += y;
            uint8_t g = gray[y][x];
            a->sg   += g;
            a->sgsq += g * g;
            if (g > a->maxg) a->maxg = g;
            if (g < a->ming) a->ming = g;
            if (x < a->minx) a->minx = x;
            if (x > a->maxx) a->maxx = x;
            if (y < a->miny) a->miny = y;
            if (y > a->maxy) a->maxy = y;
        }
    }

    /* 第二遍扫描：合并根标签的统计结果。 */
    uint16_t final_id[MAX_LABELS] = {0};
    uint16_t      label_cnt = 0;

    for (uint16_t l = 1; l < next_label; ++l) {
        if (!acc[l].cnt) continue;                 /* 跳过没有像素统计的标签。 */
        int root = uf_find(l);
        if (!final_id[root]) final_id[root] = ++label_cnt;

        if (l != root) {                           /* 累计到根标签。 */
            acc[root].cnt  += acc[l].cnt;
            acc[root].sx   += acc[l].sx;
            acc[root].sy   += acc[l].sy;
            acc[root].sg   += acc[l].sg;
            acc[root].sgsq += acc[l].sgsq;

            if (acc[l].maxg > acc[root].maxg) acc[root].maxg = acc[l].maxg;
            if (acc[l].ming < acc[root].ming) acc[root].ming = acc[l].ming;
            if (acc[l].minx < acc[root].minx) acc[root].minx = acc[l].minx;
            if (acc[l].maxx > acc[root].maxx) acc[root].maxx = acc[l].maxx;
            if (acc[l].miny < acc[root].miny) acc[root].miny = acc[l].miny;
            if (acc[l].maxy > acc[root].maxy) acc[root].maxy = acc[l].maxy;
        }
    }

    /* 输出每个有效连通域的信息。 */
    for (uint16_t root = 1; root < next_label; ++root) {
        int id = final_id[root];
        if (!id) continue;

        Acc *a = &acc[root];
        ContourInfo *c = &contours[id - 1];

        c->area     = (float)a->cnt;
        c->cx       = (float)a->sx / c->area;
        c->cy       = (float)a->sy / c->area;
        c->ave_gray = (float)a->sg / c->area;

        double mean    = (double)a->sg   / a->cnt;
        double mean_sq = (double)a->sgsq / a->cnt;

        if((mean_sq - mean * mean)>=0)
        {
            c->Std_Varaince = (float)sqrt(mean_sq - mean * mean);
        }


        c->bbox.x = a->minx;
        c->bbox.y = a->miny;
        c->bbox.w = a->maxx - a->minx + 1;
        c->bbox.h = a->maxy - a->miny + 1;
//      c->w=w;
//      c->h=h;
        c->boxratio = c->bbox.w  ? (float)c->bbox.w / (float)c->bbox.h : 0.f;

        c->max_gray       = a->maxg;
        c->mingray        = a->ming;
        c->contrast_ratio = c->mingray ? (float)c->max_gray / c->mingray : 0.f;

        float world[3];
        pixel2world(c->cx, c->cy, world, c);       /* 以下字段可根据应用需求启用。 */
    }

    return label_cnt;
}

void comgray2binary(const uint8_t input[HEIGHT][WIDTH],
                 uint8_t output[HEIGHT][WIDTH],
                 uint8_t threshold)
{
    for (int y = 0; y < HEIGHT; ++y) {
        for (int x = 0; x < WIDTH; ++x) {
            output[y][x] = (input[y][x] >= threshold) ? 255 : 0;
        }
    }
}
