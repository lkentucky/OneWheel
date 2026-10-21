"""Host smoke test for the bounded TC264 label buffers; requires GCC."""
from pathlib import Path
import shutil
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory(prefix='qcar-labels-') as temp:
    work = Path(temp)
    shutil.copy2(root / 'code/qcar_types.h', work)
    (work / 'zf_common_headfile.h').write_text('''
#include <stdint.h>
#include <string.h>
#include <math.h>
typedef uint8_t uint8;
typedef int16_t int16;
#include "qcar_types.h"
void pixel2world(float u, float v, float world[3], ContourInfo *c);
''')
    (work / 'check.c').write_text('''
#include <assert.h>
#include <stdio.h>
#include "component.h"
static uint8_t binary[HEIGHT][WIDTH], gray[HEIGHT][WIDTH];
static struct {
    uint32_t before;
    ContourInfo contours[QCAR_MAX_CONTOURS];
    uint32_t after;
} output;
void pixel2world(float u, float v, float world[3], ContourInfo *c) {
    (void)u; (void)v; (void)world; c->distance = 0;
}
int main(void) {
    output.before = output.after = 0x12345678;
    memset(gray, 100, sizeof(gray));
    for (int mode = 0; mode < 2; mode++) {
        memset(binary, 0, sizeof(binary));
        int n = mode ? label_components_change(binary, gray, output.contours, 1)
                     : label_components(binary, gray, output.contours);
        assert(n == 0);
        memset(binary, 1, sizeof(binary));
        n = mode ? label_components_change(binary, gray, output.contours, 1)
                 : label_components(binary, gray, output.contours);
        assert(n == 1);
        assert(output.contours[0].area == HEIGHT * WIDTH);
        assert(output.contours[0].ave_gray == 100);
        memset(binary, 0, sizeof(binary));
        for (int y = 0; y < HEIGHT; y += 3)
            for (int x = 0; x < WIDTH; x += 3) binary[y][x] = 1;
        n = mode ? label_components_change(binary, gray, output.contours, 1)
                 : label_components(binary, gray, output.contours);
        assert(n == QCAR_MAX_CONTOURS - 1);
        assert(output.before == 0x12345678 && output.after == 0x12345678);
    }
    puts("PASS: empty, full-frame and label-overflow images, both algorithms");
    return 0;
}
''')
    executable = work / 'check.exe'
    subprocess.run(['gcc', '-std=c99', '-I' + str(work), '-I' + str(root / 'code'),
                    str(root / 'code/component.c'), str(work / 'check.c'),
                    '-lm', '-o', str(executable)], check=True)
    subprocess.run([str(executable)], check=True)
