"""CPU0 menu must run even when CPU1 has not been resumed by debugger."""
from pathlib import Path
import re,subprocess,tempfile
root=Path(__file__).resolve().parents[1]
s=re.sub(r'^#include.*$', '',(root/"user/cpu0_main.c").read_text("utf-8"),flags=re.M)
s=s.replace('int core0_main(void)','int firmware_main(void)')
c=(root/"libraries/zf_common/zf_common_clock.c").read_text("utf-8");a=c.index('void cpu_init_notify(void)');b=c.index('\n}',a)+2
prefix=r"""
#include <stdint.h>
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#define TRUE 1
#define QCAR_BOOT_IMU_WIFI 0
#define IPS200_PORTAIT 0
#define RGB565_WHITE 1
#define RGB565_BLACK 0
#define IPS200_TYPE_SPI 0
#define IPS200_6X8_FONT 0
static volatile uint8_t cpu_init_finish[2];
static int irq,init,key1_count;
static jmp_buf finished;
static int IfxCpu_getCoreId(void) {return 0;}
static void clock_init(void) {}
static void debug_init(void) {}
static void ips200_set_dir(int x) {(void)x;}
static void ips200_set_color(int x,int y) {(void)x;(void)y;}
static void ips200_init(int x) {(void)x;}
static void ips200_set_font(int x) {(void)x;}
static void ips200_show_string(int x,int y,const char *s) {(void)x;(void)y;(void)s;}
/* CPU0 must draw the menu before it publishes its ready flag. */
static void monotroch_init(void) {assert(cpu_init_finish[0]==0);init=1;}
static void menu_auto_telemetry_init(void) {assert(init&&irq&&cpu_init_finish[0]==1);}
static void interrupt_global_enable(int mask) {assert(mask==0);irq=1;}
static void menu(int key) {
    (void)key;
    assert(init&&irq&&cpu_init_finish[0]==1);
    assert(cpu_init_finish[1]==0);   /* CPU1 still halted, CPU0 keeps running */
    longjmp(finished,1);
}
"""
main=r"""
int main(void) {
 if(!setjmp(finished))firmware_main();
 puts("PASS: real CPU0 menu entry publishes ready and enables interrupts without waiting for halted CPU1");
}
"""
with tempfile.TemporaryDirectory(prefix="qcar-menu-boot-") as d:
 p=Path(d);f=p/"test.c";exe=p/"test.exe";f.write_text(prefix+c[a:b]+s+main,encoding="utf-8")
 subprocess.run(["gcc","-std=c99","-Wall","-Wextra","-Werror",str(f),"-o",str(exe)],check=True)
 subprocess.run([str(exe)],check=True,timeout=5)
