/* Execute the production C, including private parser/scheduler entry points.
 * Hardware functions below are mocks; this cannot validate a real SPI module.
 */
#include <assert.h>
#include "wifi_link.c"
#include "wifi_link_io.c"

/* TinyCC's older Windows CRT exports the double math entry points only. */
float atan2f(float y,float x) { return (float)atan2(y,x); }
float sqrtf(float x) { return (float)sqrt(x); }
float fmodf(float x,float y) { return (float)fmod(x,y); }

int MODULE_STM0;
cascada_value_struct balance_cascade;
ICM_DATA icm_data_lpf;
BeaconInfo beacon;
uint8 image_binary[MT9V03X_H][MT9V03X_W];
volatile uint32 system_count;
volatile uint8 run_flag;
volatile float k1=1.25f, k2=.5f;
float expect_speed=160;
float erro;
int16 encoder_data_1, car_speed, bujin_last_duty;
mock_motor motor_value;
char wifi_spi_version[12], wifi_spi_mac_addr[20], wifi_spi_ip_addr_port[25];
static uint32 ticks, assertions, socket_calls;
static uint8 busy, spi_stage, command, reply_code=WIFI_SPI_REPLY_OK;
static uint16 wire_length, captured_length, rx_length;
static uint8 captured[4088], rx_data[32];
static char output_directory[512];

float IfxStm_getFrequency(void *stm) { (void)stm; return 1000000; }
uint32 IfxStm_getLower(void *stm) { (void)stm; return ticks; }
uint8 gpio_get_level(int pin) { (void)pin; return !busy; }
void gpio_low(int pin) { (void)pin; assertions++; spi_stage=0; }
void gpio_high(int pin) { (void)pin; }
void system_delay_us(uint32 us) { ticks+=us; }
float imu660rb_gyro_transition(int16 raw) { return raw / 14.3f; }
float imu660rb_acc_transition(int16 raw) { return raw / 4098.0f; }
uint8 wifi_spi_init(char *a,char *b) { (void)a; (void)b; return 0; }
uint8 wifi_spi_socket_connect(char *a,char *b,char *c,char *d)
{ (void)a; (void)b; (void)c; (void)d; socket_calls++; return 0; }
void spi_transfer_8bit(int index,const uint8 *tx,uint8 *rx,uint32 n)
{
    (void)index;
    ticks += n;
    if (!spi_stage++)
    {
        wifi_spi_head_struct sent;
        assert(n==4);
        memcpy(&sent,tx,4);
        command=sent.command;
        wire_length=sent.length;
        memset(rx,0,n);
        rx[0]=reply_code;
        rx[2]=(uint8)rx_length;
        if (command==WIFI_SPI_DATA && wire_length) captured_length=0;
    }
    else
    {
        assert(n==32);
        if (command==WIFI_SPI_DATA && wire_length)
        {
            uint32 count=wire_length<32 ? wire_length:32;
            memcpy(captured,tx,count);
            captured_length=(uint16)count;
        }
        memcpy(rx,rx_data,n);
    }
}
void spi_write_8bit_array(int index,const uint8 *tx,uint32 n)
{
    (void)index;
    assert(captured_length+n<=sizeof(captured));
    memcpy(captured+captured_length,tx,n);
    captured_length+=(uint16)n;
    ticks+=n;
}
static void reset(void)
{
    ticks=0; busy=0; reply_code=WIFI_SPI_REPLY_OK; rx_length=0;
    memset((void *)&wl_share,0,sizeof(wl_share));
    wifi_link_init();
    wl_state=WL_ST_RUN;
    wl_c0_len=0;
    wl_share.cpu1_ready=1;
    run_flag=0;
}
static void cmd(const char *text)
{ wl_command_bytes((const uint8 *)text,(uint32)strlen(text)); wl_c0_log[wl_c0_len]=0; }
static void clear_log(void) { wl_c0_len=0; }
static void finish_tx(void)
{
    int i;
    for(i=0;i<3;i++)
    {
        uint32 before=assertions;
        wl_io_service();
        assert(assertions-before==1);
    }
    assert(!wifi_link_io_busy());
}
static void save(const char *name,const void *bytes,uint32 n)
{
    uint32 i; const uint8 *p=(const uint8 *)bytes;
    printf("PACKET %s ",name);
    for(i=0;i<n;i++)printf("%02x",p[i]);
    puts("");
}
static void test_parser(void)
{
    const char *bad[]={"nan","NaN","inf","-inf","1e999","1x","-1001","1001"};
    unsigned i; char line[200];
    reset(); wifi_target_speed=160;
    for(i=0;i<sizeof(bad)/sizeof(bad[0]);i++)
    {
        clear_log(); snprintf(line,sizeof(line),"MCAR SET 1 speed %s\n",bad[i]); cmd(line);
        assert(wifi_target_speed==160); assert(strstr(wl_c0_log,"ERR")!=NULL);
    }
    clear_log(); cmd("MCAR SET 1 speed 123\n"); assert(wifi_target_speed==123);
    clear_log(); cmd("MCAR SET 1 speed 10 extra\n"); assert(wifi_target_speed==123);
    memset(line,'x',80); strcpy(line+80,"MCAR SET 1 speed 5\n"); clear_log();cmd(line);
    assert(wifi_target_speed==123);
    clear_log();cmd("MCAR SET 1 speed 1\00100\n"); assert(wifi_target_speed==123);
    clear_log();cmd("MCAR SET 1 run 1\n"); assert(run_flag==0);
    run_flag=1;clear_log();cmd("MCAR SET 1 roll_angle_p 6\n");assert(balance_cascade.angle_cycle.p==0);
    clear_log();cmd("MCAR SET 1 run 0\n");assert(run_flag==0);
    clear_log();cmd("MCAR SET 1 ramp_ms 500\n");assert(fabs(wifi_ramp_step-.32f)<1e-6);
    clear_log();cmd("MCAR SET 1 cruise_cms 10\n"); assert(wifi_target_speed==123);
    clear_log();cmd("MCAR SET 1 host 999.1.1.1\n");assert(!wl_next_host[0]);
    clear_log();cmd("MCAR SET 1 host 192.168.1.2\n");assert(!strcmp(wl_next_host,"192.168.1.2"));assert(socket_calls==0);
    clear_log();wl_share.cpu1_ready=0;cmd("MCAR SET 1 speed 20\n");assert(wifi_target_speed==123);
    wl_share.cpu1_ready=1;clear_log();cmd("MCAR GET 42 ALL\n");
    assert(wl_c0_len<1800);save("getall.txt",wl_c0_log,wl_c0_len);
}
static void test_transport(void)
{
    uint32 before;
    reset();wifi_link_log("retain this\n");wl_send_logs();
    assert(wl_c0_len==12);
    busy=1;before=assertions;wl_io_service();assert(before==assertions);
    busy=0;finish_tx();assert(wl_c0_len==0);assert(captured_length==20);
    wifi_link_log("failure\n");wl_send_logs();
    reply_code=WIFI_SPI_REPLY_ERROR;wl_io_service();assert(wl_c0_len>=8);
    assert(wl_state==WL_ST_BACKOFF);
    reset();wifi_link_log("timeout\n");wl_send_logs();busy=1;wl_now_ms=250;
    wl_io_service();assert(wl_c0_len>=8);assert(wl_state==WL_ST_BACKOFF);
    reset();wifi_link_io_receive(0);reply_code=WIFI_SPI_REPLY_DATA_START;
    rx_length=1;rx_data[0]='x';before=assertions;wl_io_service();assert(assertions-before==1);
    assert(wifi_link_io_busy());wl_now_ms=250;wl_io_service();assert(!wifi_link_io_busy());
    reset();wifi_link_io_receive(0);reply_code=WIFI_SPI_REPLY_DATA_END;rx_length=33;
    wl_io_service();assert(wl_state==WL_ST_BACKOFF);
    reset();wifi_link_io_receive(0);reply_code=WIFI_SPI_INVALID2;
    wl_io_service();assert(wl_state==WL_ST_BACKOFF);
    /* Incoming full-duplex command spanning multiple transactions is drained
       before UDP flush; no command bytes are lost at the handshake boundary. */
    reset();wifi_link_log("tx\n");wl_send_logs();
    reply_code=WIFI_SPI_REPLY_DATA_START;rx_length=18;
    memcpy(rx_data,"MCAR SET 9 speed ",17);rx_data[17]='1';wl_io_service();
    assert(io_state==IO_DRAIN && wl_cmd_len==18);
    reply_code=WIFI_SPI_REPLY_DATA_END;rx_length=3;memcpy(rx_data,"23\n",3);
    wl_io_service();assert(wifi_target_speed==123);assert(io_state==IO_FLUSH);
    reply_code=WIFI_SPI_REPLY_OK;rx_length=0;wl_io_service();wl_io_service();
    assert(!wifi_link_io_busy());
    /* A slow RX completion must leave time for waiting outbound work. */
    reset();wifi_link_log("queued\n");wl_now_ms=100;
    wifi_link_io_receive(0);wl_io_service();
    assert(wl_last_rx_ms==100);wl_run();assert(io_state==IO_FLUSH);
}
static void test_data_and_camera(void)
{
    uint32 i; char name[30];
    reset();icm_data_lpf.accZ=4098;icm_data_lpf.gyroZ=14300;
    system_count=100;wifi_link_cpu1_service(1);system_count=120;wifi_link_cpu1_service(0);
    assert(fabs(wl_share.yaw_deg-20)<.001);system_count=220;wifi_link_cpu1_service(1);
    assert(fabs(wl_share.yaw_deg-120)<.001);assert(wl_share.vision_frames==2);
    assert(wl_share.vision_interval_us==120000);
    wifi_encoder_total=(uint32)-123;encoder_data_1=-12;bujin_last_duty=-1400;
    wl_send_data();finish_tx();save("data.bin",captured,captured_length);
    for(i=0;i<WIFI_LINK_IMG_SIZE;i++)wl_frame[i]=(uint8)(i*7);
    wl_share.frame_ready=1;wl_share.frame_index=99;
    for(i=0;i<WL_CAM_CHUNK_COUNT;i++)
    {
        wl_send_image_slice();assert(wl_frame_pos==i);finish_tx();
        snprintf(name,sizeof(name),"camera_%02u.bin",i);save(name,captured,captured_length);
    }
    assert(!wl_share.frame_ready);assert(wl_frame_pos==0);
    wl_share.frame_ready=1;wl_send_image_slice();reply_code=WIFI_SPI_REPLY_ERROR;
    wl_io_service();assert(wl_share.frame_ready);assert(wl_frame_pos==0);
}
int main(int argc,char **argv)
{
    assert(argc==2);strcpy(output_directory,argv[1]);
    test_parser();test_transport();test_data_and_camera();
    puts("PASS: real C parser, telemetry, camera, SPI handshakes and failure retention");
    return 0;
}
