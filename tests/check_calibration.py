"""Real calibration: hand tremor tolerance, sustained motion rejection and recovery."""
from pathlib import Path
import subprocess
import tempfile
root=Path(__file__).resolve().parents[1]
source=r"""
#include "attitude.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
int main(void) {
    ImuCalibration c;
    float g[3], a[3];
    calibration_reset(&c);
    for(unsigned i=0;i<CALIBRATION_SAMPLES;i++) {
        float phase=6.28318530718f*4*(float)i/CALIBRATION_SAMPLES;
        g[0]=0.3f+2.5f*sinf(phase);
        g[1]=-0.2f+1.8f*cosf(phase);
        g[2]=0.1f+0.6f*sinf(phase);
        a[0]=0.02f*sinf(phase); a[1]=0.015f*cosf(phase); a[2]=1;
        int ready=calibration_push(&c,g,a);
        assert(ready==(i+1==CALIBRATION_SAMPLES));
    }
    assert(fabsf(c.bias_dps[0]-.3f)<1e-4f && fabsf(c.bias_dps[1]+.2f)<1e-4f);
    assert(fabsf(c.bias_dps[2]-.1f)<1e-4f && fabsf(c.initial_accel[0])<1e-4f);
    calibration_reset(&c); g[0]=g[1]=0; g[2]=5; a[0]=a[1]=0; a[2]=1;
    for(unsigned i=0;i<CALIBRATION_SAMPLES*3;i++) assert(!calibration_push(&c,g,a));
    assert(c.count==0); /* steady yaw does not become a 5deg/s bias */
    g[2]=0; for(int i=0;i<20;i++) assert(!calibration_push(&c,g,a));
    g[0]=11; assert(!calibration_push(&c,g,a) && c.count==0);
    g[0]=0; a[2]=1.2f; assert(!calibration_push(&c,g,a) && c.count==0);
    a[2]=1; g[1]=-4; assert(!calibration_push(&c,g,a));
    g[1]=4; assert(!calibration_push(&c,g,a) && c.count==0);
    g[1]=0; a[0]=-.07f; assert(!calibration_push(&c,g,a));
    a[0]=.07f; assert(!calibration_push(&c,g,a) && c.count==0);
    a[0]=0; g[0]=NAN; assert(!calibration_push(&c,g,a) && c.count==0);
    g[0]=0;
    for(unsigned i=0;i<CALIBRATION_SAMPLES;i++)
        assert(calibration_push(&c,g,a)==(i+1==CALIBRATION_SAMPLES));
    assert(c.bias_dps[0]==0 && c.bias_dps[1]==0 && c.bias_dps[2]==0);
    puts("PASS: hand tremor completes, unbiased averaging, sustained turn/spikes/acceleration/NaN rejected, recovery");
}
"""
with tempfile.TemporaryDirectory(prefix="qcar-calibration-") as directory:
    work=Path(directory); test=work/"test.c"; exe=work/"test.exe"
    test.write_text(source,encoding="utf-8")
    subprocess.run(["gcc","-std=c99","-O2","-Wall","-Wextra","-Werror","-I"+str(root/"code"),str(root/"code/attitude.c"),str(test),"-lm","-o",str(exe)],check=True)
    subprocess.run([str(exe)],check=True,timeout=5)
