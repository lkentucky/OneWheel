"""Real bounded SPI transfer with simulated QSPI FIFO/events/STM."""
from pathlib import Path
import subprocess
import tempfile
root=Path(__file__).resolve().parents[1]
s=(root/"libraries/zf_driver/zf_driver_spi.c").read_text("utf-8")
a=s.index("uint8 spi_transfer_8bit_timeout(");b=s.index("\nvoid spi_transfer_8bit (",a)
source=s[a:b]
prefix=r"""
#include <stdint.h>
#include <stddef.h>
#include <assert.h>
#include <stdio.h>
typedef uint8_t uint8;typedef uint32_t uint32;typedef int spi_index_enum;typedef int IfxQspi_Index;
typedef struct {struct {struct {uint32 RXFIFOLEVEL,PT1F;} B;} STATUS;} Ifx_QSPI;
static Ifx_QSPI port;
static struct {uint32 U;} bacon[4];
static int MODULE_STM0,mode,writes,reads;
static uint32 ticks;
static uint8 received;
static Ifx_QSPI *IfxQspi_getAddress(IfxQspi_Index n) {assert(n==3);return &port;}
static float IfxStm_getFrequency(int *p) {(void)p;return 1000000;}
static uint32 IfxStm_getLower(int *p) {(void)p;return ticks++;}
static void spi_clear_fifo(volatile Ifx_QSPI *p) {p->STATUS.B.RXFIFOLEVEL=0;}
static void IfxQspi_clearAllEventFlags(volatile Ifx_QSPI *p) {p->STATUS.B.PT1F=0;}
static void IfxQspi_writeBasicConfigurationBeginStream(volatile Ifx_QSPI *p,uint32 b) {(void)p;(void)b;}
static void IfxQspi_writeBasicConfigurationEndStream(volatile Ifx_QSPI *p,uint32 b) {(void)p;(void)b;}
static void IfxQspi_writeTransmitFifo(volatile Ifx_QSPI *p,uint8 tx) {
    writes++;received=tx^0x5a;
    p->STATUS.B.RXFIFOLEVEL=mode==1?0:1;
    p->STATUS.B.PT1F=mode==2?0:1;
}
static uint32 IfxQspi_readReceiveFifo(volatile Ifx_QSPI *p) {reads++;p->STATUS.B.RXFIFOLEVEL=0;return received;}
"""
main=r"""
int main(void) {
    uint8 data[3]={1,2,3},out[3];
    assert(spi_transfer_8bit_timeout(3,data,out,3,20)==0);
    assert(out[0]==(1^0x5a) && out[2]==(3^0x5a) && reads==3);
    assert(spi_transfer_8bit_timeout(3,data,data,3,20)==0 && data[0]==(1^0x5a));
    int before=reads;assert(spi_transfer_8bit_timeout(3,data,NULL,3,20)==0 && reads==before+3);
    assert(spi_transfer_8bit_timeout(3,NULL,out,3,20)==0 && out[0]==0x5a);
    before=writes;assert(spi_transfer_8bit_timeout(3,NULL,NULL,0,20)==0 && writes==before);
    assert(spi_transfer_8bit_timeout(3,data,out,3,0)==1 && writes==before);
    mode=1;ticks=0xfffffff0u;uint32 start=ticks;
    assert(spi_transfer_8bit_timeout(3,data,out,3,1)==1);
    assert((uint32)(ticks-start)<1100 && writes==before+1);
    mode=2;start=ticks;assert(spi_transfer_8bit_timeout(3,data,out,3,1)==1 && (uint32)(ticks-start)<1100);
    mode=0;assert(spi_transfer_8bit_timeout(3,data,out,3,20)==0);
    puts("PASS: real SPI timeout covers missing RX/EOF, clock wrap, duplex/in-place/write-only draining and recovery");
}
"""
with tempfile.TemporaryDirectory(prefix="qcar-spi-deadline-") as directory:
    p=Path(directory);src=p/"test.c";exe=p/"test.exe";src.write_text(prefix+source+main,encoding="utf-8")
    subprocess.run(["gcc","-std=c99","-Wall","-Wextra","-Werror",str(src),"-o",str(exe)],check=True)
    subprocess.run([str(exe)],check=True,timeout=5)
