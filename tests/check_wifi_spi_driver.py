"""Exercise the production TC264 WiFi driver setup with fault injection."""
from pathlib import Path
import subprocess
import tempfile
root=Path(__file__).resolve().parents[1]
s=(root/"libraries/zf_device/zf_device_wifi_spi.c").read_text("utf-8")
def function(signature):
    a=s.index(signature);b=s.index("{",a)+1;depth=1
    while depth:
        depth+=(s[b]=="{")-(s[b]=="}");b+=1
    return s[a:b]
start=s.index("static void (*wifi_spi_wait_callback)")
end=s.index("//----------------------------------------------------------------",s.index("static uint8 wifi_spi_wait_idle"))
parts=s[start:end]
for signature in ["static uint8 wifi_spi_write (","static uint8 wifi_spi_transfer_command (","static uint8 wifi_spi_transfer_data (","static uint8 wifi_spi_set_parameter (","static uint8 wifi_spi_get_parameter (","uint8 wifi_spi_get_version (","static uint8 wifi_spi_get_mac_addr (","static uint8 wifi_spi_get_ip_addr_port (","uint8 wifi_spi_init ("]:
    parts+=function(signature)+"\n"
prefix=r"""
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <assert.h>
#define SPI_3 3
#define SPI3_SCLK_P22_3 3
#define SPI3_MOSI_P22_0 4
#define SPI3_MISO_P22_1 5
#define P22_2 6
#define P15_8 7
#define P23_1 8
#include "zf_device_wifi_spi.h"
#define OTHER_TIME_OUT 1000
#define SPI_MODE0 0
#define SPI_CS_NULL 99
#define GPO 1
#define GPI 0
#define GPO_PUSH_PULL 1
#define GPI_PULL_DOWN 0
#define FIFO_DATA_8BIT 0
typedef int fifo_struct;
char wifi_spi_version[12],wifi_spi_mac_addr[20],wifi_spi_ip_addr_port[25];
volatile uint32 wifi_spi_config_error,wifi_spi_config_command,wifi_spi_config_reply,wifi_spi_config_reply_length;
static fifo_struct wifi_spi_fifo;
static uint8 wifi_spi_buffer[WIFI_SPI_RECVIVE_FIFO_SIZE];
static volatile wifi_spi_state_enum wifi_spi_mutex;
static uint32 ticks;
static int ready=1,fail_spi,cs=1,mode=-1,callback_calls,transfers,check_padding,fail_on_read,block_after_write,transfer_base;
static uint8 response_command=WIFI_SPI_REPLY_VERSION;
static uint16 response_length=4;
static int MODULE_STM0;
static uint32 IfxStm_getLower(int *p) {(void)p;return ticks++;}
static float IfxStm_getFrequency(int *p) {(void)p;return 1000000;}
static int gpio_get_level(int p) {assert(p==WIFI_SPI_INT_PIN);return ready && !(block_after_write && transfers>transfer_base);}
static void gpio_low(int p) {assert(p==WIFI_SPI_CS_PIN);cs=0;}
static void gpio_high(int p) {assert(p==WIFI_SPI_CS_PIN);cs=1;}
static void system_delay_us(uint32 n) {ticks+=n;}
static void system_delay_ms(uint32 n) {ticks+=n*1000;}
static void gpio_init(int p,int a,int b,int c) {(void)p;(void)a;(void)b;(void)c;}
static void gpio_set_level(int p,int v) {(void)p;(void)v;}
static void fifo_init(fifo_struct *f,int t,uint8 *b,int n) {(void)f;(void)t;(void)b;(void)n;}
static void spi_init(int a,int b,int c,int d,int e,int f,int g) {(void)a;(void)c;(void)d;(void)e;(void)f;(void)g;mode=b;}
static uint8 spi_transfer_8bit_timeout(int spi,const uint8 *tx,uint8 *rx,uint32 n,uint32 timeout) {
    (void)tx;assert(spi==SPI_3 && !cs && timeout==20);transfers++;
    if(fail_spi || (fail_on_read && rx))return 1;
    if(rx && n==sizeof(wifi_spi_head_struct)) {
        wifi_spi_head_struct h={response_command,0,response_length};memcpy(rx,&h,sizeof h);
    } else if(rx) {
        if(check_padding) { for(uint32 i=3;i<n;i++) assert(tx[i]==0); check_padding=0; }
        memset(rx,'X',n);memcpy(rx,"V2.1",4);
    }
    return 0;
}
uint8 wifi_spi_wifi_connect(char *a,char *b) {(void)a;(void)b;return 0;}
uint8 wifi_spi_socket_connect(char *a,char *b,char *c,char *d) {(void)a;(void)b;(void)c;(void)d;return 0;}
"""
main=r"""
static void cancel(void) {callback_calls++;wifi_spi_cancel_wait();}
int main(void) {
    assert(wifi_spi_init(NULL,NULL)==0 && mode==0 && !strcmp(wifi_spi_version,"V2.1") && cs);
    fail_spi=1;assert(wifi_spi_init(NULL,NULL)==1 && cs);fail_spi=0;
    response_length=65535;assert(wifi_spi_get_version()==1 && cs);
    response_length=12;assert(wifi_spi_get_version()==0 && strlen(wifi_spi_version)==11 && cs);
    response_length=0;assert(wifi_spi_get_version()==1 && cs);
    response_length=4;response_command=0xff;assert(wifi_spi_get_version()==1 && cs);
    response_command=WIFI_SPI_REPLY_MAC_ADDR;response_length=20;assert(wifi_spi_get_mac_addr()==0 && strlen(wifi_spi_mac_addr)==19);
    response_command=WIFI_SPI_REPLY_IP_ADDR;response_length=25;assert(wifi_spi_get_ip_addr_port()==0 && strlen(wifi_spi_ip_addr_port)==24);
    uint8 payload[80]={1,2,3}; wifi_spi_packets_struct packet;
    response_command=WIFI_SPI_REPLY_DATA_END;response_length=4;
    check_padding=1;assert(wifi_spi_transfer_data(payload,&packet,3)==0 && !check_padding && cs);
    int old_transfers=transfers;assert(wifi_spi_transfer_data(payload,&packet,80)==0 && transfers==old_transfers+3 && cs);
    fail_spi=1;assert(wifi_spi_transfer_data(payload,&packet,3)==1 && cs);fail_spi=0;
    ready=0;ticks=0xffffff00u;wifi_spi_wait_cancelled=0;uint32 before=ticks;
    assert(wifi_spi_wait_idle(1)==1 && (uint32)(ticks-before)<1100);
    wifi_spi_set_wait_callback(cancel);callback_calls=0;
    assert(wifi_spi_wait_idle(1000)==1 && callback_calls==1);
    ready=1;assert(wifi_spi_wait_idle(1)==1); /* cancellation remains latched */
    wifi_spi_set_wait_callback(NULL);response_command=WIFI_SPI_REPLY_VERSION;response_length=4;
    assert(wifi_spi_init(NULL,NULL)==0); /* init clears previous cancellation */
    uint8 setting=0;
    response_command=WIFI_SPI_REPLY_OK;response_length=0;
    assert(wifi_spi_set_parameter(WIFI_SPI_SET_WIFI_INFORMATION,&setting,1,1)==0);
    assert(wifi_spi_config_error==0 && wifi_spi_config_reply==0x80 && wifi_spi_config_command==0x10);
    response_command=WIFI_SPI_REPLY_ERROR;
    assert(wifi_spi_set_parameter(WIFI_SPI_SET_WIFI_INFORMATION,&setting,1,1)==1);
    assert(wifi_spi_config_error==5 && wifi_spi_config_reply==0x81);
    ready=0;assert(wifi_spi_set_parameter(WIFI_SPI_SET_WIFI_INFORMATION,&setting,1,1)==1 && wifi_spi_config_error==1);ready=1;
    fail_spi=1;assert(wifi_spi_set_parameter(WIFI_SPI_SET_WIFI_INFORMATION,&setting,1,1)==1 && wifi_spi_config_error==2);fail_spi=0;
    fail_on_read=1;assert(wifi_spi_set_parameter(WIFI_SPI_SET_WIFI_INFORMATION,&setting,1,1)==1 && wifi_spi_config_error==4);fail_on_read=0;
    transfer_base=transfers;block_after_write=1;
    assert(wifi_spi_set_parameter(WIFI_SPI_SET_WIFI_INFORMATION,&setting,1,1)==1 && wifi_spi_config_error==3);
    block_after_write=0;
    puts("PASS: actual WiFi driver uses TC264 reference Mode0; SPI errors release CS; corrupt lengths/commands rejected, padded strings bounded; INT timeout/wrap and cancellation return");
}
"""
with tempfile.TemporaryDirectory(prefix="qcar-wifi-driver-") as directory:
    p=Path(directory);(p/"zf_common_typedef.h").write_text("#include <stdint.h>\ntypedef uint8_t uint8; typedef uint16_t uint16; typedef uint32_t uint32;\n")
    test=p/"test.c";exe=p/"test.exe";test.write_text(prefix+parts+main,encoding="utf-8")
    subprocess.run(["gcc","-std=c99","-Wall","-Wextra","-Werror","-I"+str(p),"-I"+str(root/"libraries/zf_device"),"-I"+str(root/"code"),str(test),"-o",str(exe)],check=True)
    subprocess.run([str(exe)],check=True,timeout=5)
