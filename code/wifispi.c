/** 移植自 MCARpreparation/project/code/wifispi.c。
 * TC264 网络驱动、JustFloat 打包及可订阅遥测。
 * 收发均为同步操作，只在主循环且开中断时调用。
 */
#include "wifispi.h"
#include "imu.h"
#if QCAR_WIFI_ENABLED
#include "IfxStm.h"
#include "zf_common_interrupt.h"
#include "zf_device_wifi_spi.h"
#include "zf_driver_delay.h"
#include "pid.h"
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#endif
#include <string.h>

volatile int imu_wifi_status;
volatile uint32_t imu_wifi_tx_packets, imu_wifi_init_attempts;
volatile int imu_wifi_last_error;
volatile uint32_t wifi_telemetry_channel_count, wifi_telemetry_period_ms;
volatile uint32_t wifi_telemetry_commands, wifi_telemetry_command_errors;
volatile uint32_t wifi_telemetry_stream_enabled;

/* 编译期确认 float32；协议不依赖主机字节序。 */
typedef char float_must_be_32_bits[(sizeof(float)==4)?1:-1];
static void pack_float32_le(uint8_t *out,float value)
{
    uint32_t bits;
    unsigned j;
    memcpy(&bits,&value,4);
    for (j=0;j<4;++j) out[j]=(uint8_t)(bits>>(8*j));
}
size_t vofa_pack(uint8_t *out,size_t out_capacity,const float *channels,size_t count)
{
    size_t i,frame_bytes;
    if (out==NULL || channels==NULL || count==0u || count>VOFA_MAX_CHANNELS) return 0u;
    frame_bytes=VOFA_FRAME_BYTES(count);
    if (out_capacity<frame_bytes) return 0u;
    for (i=0u;i<count;++i) pack_float32_le(&out[4u*i],channels[i]);
    out[4u*count]=0x00u;
    out[4u*count+1u]=0x00u;
    out[4u*count+2u]=0x80u;
    out[4u*count+3u]=0x7fu;
    return frame_bytes;
}
#if QCAR_WIFI_ENABLED
static int wifi_ready;
static uint32 g_last_wifi_tx_ticks, g_last_wifi_rx_ticks, g_ticks_per_ms;

/* CPU0 owns the transport and snapshots its ISR-produced data. CPU1 never
 * accesses this module when QCAR_WIFI_ENABLED is set. */
#define TELEMETRY_VARIABLES(X) \
    X(roll_deg, imu_roll_deg) X(pitch_deg, imu_pitch_deg) X(yaw_deg, imu_yaw_deg) \
    X(accel_x_g, imu_accel_g[0]) X(accel_y_g, imu_accel_g[1]) X(accel_z_g, imu_accel_g[2]) \
    X(gyro_x_dps, imu_gyro_dps[0]) X(gyro_y_dps, imu_gyro_dps[1]) X(gyro_z_dps, imu_gyro_dps[2]) \
    X(imu_status, imu_attitude_status) X(imu_cal_percent, imu_calibration_percent) \
    X(gz_bias_dps, imu_gyro_bias_z_dps) X(imu_dt_ms, imu_sample_dt_ms) \
    X(imu_solve_us, imu_solve_us) X(imu_solve_max_us, imu_solve_max_us) \
    X(imu_accel_used, imu_accel_used) X(ekf_nis, imu_ekf_nis) \
    X(ekf_bias_x_dps, imu_ekf_bias_xy_dps[0]) X(ekf_bias_y_dps, imu_ekf_bias_xy_dps[1]) \
    X(imu_seq, imu_sample_sequence) X(imu_method, imu_attitude_method) \
    X(accel_norm_g, imu_accel_norm_g) X(gyro_z_integral_deg, imu_gyro_z_integral_deg) \
    X(wheel_speed, real_speed) X(target_speed, mono_params.setspeed) \
    X(roll_speed_out, cascade_vel_out) X(pitch_speed_out, cascade_vel_out_p) \
    X(yaw_speed_out, cascade_vel_out_y) X(roll_angle_out, cascade_angle_out) \
    X(pitch_angle_out, cascade_angle_out_p) X(yaw_angle_out, cascade_angle_out_y) \
    X(roll_gyro_out, cascade_angle_vel_out) X(pitch_gyro_out, cascade_angle_vel_out_p) \
    X(yaw_gyro_out, cascade_angle_vel_out_y)
#define VARIABLE_ID(name, expr) TV_##name,
enum { TELEMETRY_VARIABLES(VARIABLE_ID) TV_COUNT };
#undef VARIABLE_ID
#define VARIABLE_NAME(name, expr) #name,
static const char *const g_variable_names[]={TELEMETRY_VARIABLES(VARIABLE_NAME)};
#undef VARIABLE_NAME
typedef char variable_ids_fit_byte[(TV_COUNT<=256)?1:-1];
static uint8_t g_channels[VOFA_MAX_CHANNELS];
static char g_command[1024];
static size_t g_command_length;
static int g_command_discard;
static int g_slider_receiving;

/* A slider names one public parameter, never an address. Add mappings here
 * when another application variable should be editable from the host. */
typedef struct {
    const char *name;
    float *value;
    float minimum, maximum;
} slider_parameter_t;
#define SLIDER_PID(pid, term) {#pid "." #term, &pid.term, 0.0f, 1000.0f}
#define SLIDER_GAINS(pid) SLIDER_PID(pid, kp), SLIDER_PID(pid, ki), SLIDER_PID(pid, kd)
static const slider_parameter_t g_slider_parameters[] = {
    SLIDER_GAINS(cascade_vel_roll), SLIDER_GAINS(cascade_vel_pitch), SLIDER_GAINS(cascade_vel_yaw),
    SLIDER_GAINS(cascade_ang_roll), SLIDER_GAINS(cascade_ang_pitch), SLIDER_GAINS(cascade_ang_yaw),
    SLIDER_GAINS(cascade_ang_vel_roll), SLIDER_GAINS(cascade_ang_vel_pitch), SLIDER_GAINS(cascade_ang_vel_yaw),
    {"balance_angle_x", &mono_params.balance_angle_x, -45.0f, 45.0f},
    {"balance_angle_y", &mono_params.balance_angle_y, -45.0f, 45.0f}
};
#undef SLIDER_GAINS
#undef SLIDER_PID

static void telemetry_reset(void)
{
    g_channels[0]=TV_roll_deg; g_channels[1]=TV_pitch_deg; g_channels[2]=TV_yaw_deg;
    wifi_telemetry_channel_count=3;
    wifi_telemetry_period_ms=QCAR_WIFI_PERIOD_MS;
    wifi_telemetry_stream_enabled=1;
    wifi_telemetry_commands=wifi_telemetry_command_errors=0;
    g_command_length=0; g_command_discard=0;
    g_slider_receiving=0;
}

static void send_reply(const char *text)
{
    if (wifi_spi_send_buffer((const uint8 *)text,(uint32)strlen(text))!=0 ||
        wifi_spi_udp_send_now()!=0) imu_wifi_status=-4;
}
static void command_error(const char *reason)
{
    char reply[96];
    ++wifi_telemetry_command_errors;
    (void)snprintf(reply,sizeof(reply),"MCAR ERR %s\n",reason);
    send_reply(reply);
}
static char *trim(char *text)
{
    char *end;
    while (*text==' ' || *text=='\t') ++text;
    end=text+strlen(text);
    while (end>text && (end[-1]==' ' || end[-1]=='\t')) --end;
    *end='\0';
    return text;
}
static char *slider_token(char *text)
{
    size_t length;
    text=trim(text); length=strlen(text);
    if (length>=2 && text[0]=='"' && text[length-1]=='"') {
        text[length-1]='\0'; ++text;
    }
    if (*text=='\0' || strchr(text,'"')!=NULL) return NULL;
    return text;
}
/* Decimal number, optionally signed and with exponent; not a general JSON
 * parser. Strings/escapes, extra fields, NaN/Inf and hex numbers are rejected. */
static int slider_number(const char *text, float *out)
{
    const char *p=text;
    char *end;
    if (*p=='-' || *p=='+') ++p;
    if (*p<'0' || *p>'9') return 0;
    while (*p>='0' && *p<='9') ++p;
    if (*p=='.') {
        ++p;
        if (*p<'0' || *p>'9') return 0;
        while (*p>='0' && *p<='9') ++p;
    }
    if (*p=='e' || *p=='E') {
        ++p;
        if (*p=='+' || *p=='-') ++p;
        if (*p<'0' || *p>'9') return 0;
        while (*p>='0' && *p<='9') ++p;
    }
    if (*p!='\0') return 0;
    *out=strtof(text,&end);
    return *end=='\0' && isfinite(*out);
}
static void process_slider(char *command)
{
    char *kind, *name, *number, *comma;
    size_t length=strlen(command), i;
    float value;
    const slider_parameter_t *parameter=NULL;
    uint32 primask;
    char reply[96];
    if (length<2 || command[length-1]!=']') goto bad_packet;
    command[length-1]='\0'; kind=command+1;
    comma=strchr(kind,',');
    if (comma==NULL) goto bad_packet;
    *comma='\0'; name=comma+1;
    comma=strchr(name,',');
    if (comma==NULL) goto bad_packet;
    *comma='\0'; number=trim(comma+1);
    kind=slider_token(kind); name=slider_token(name);
    if (kind==NULL || name==NULL || !slider_number(number,&value)) goto bad_packet;
    if (strcmp(kind,"slider")!=0) { command_error("unknown control type"); return; }
    for (i=0;i<sizeof(g_slider_parameters)/sizeof(g_slider_parameters[0]);++i) {
        if (strcmp(name,g_slider_parameters[i].name)==0) { parameter=&g_slider_parameters[i]; break; }
    }
    if (parameter==NULL) { command_error("unknown parameter"); return; }
    if (value<parameter->minimum || value>parameter->maximum) {
        command_error("parameter out of range"); return;
    }
    primask=interrupt_global_disable();
    *parameter->value=value;
    value=*parameter->value;
    interrupt_global_enable(primask);
    (void)snprintf(reply,sizeof(reply),"MCAR SLIDER %s %.9g\n",name,(double)value);
    send_reply(reply);
    return;
bad_packet:
    command_error("expected [slider,parameter,number]");
}
static void reply_names(int all)
{
    /* Maximum names + delimiters are checked at append time. One UDP datagram. */
    char reply[1400];
    size_t used=0, i, count=all?TV_COUNT:wifi_telemetry_channel_count;
    int n=snprintf(reply,sizeof(reply),all?"MCAR LIST ":"MCAR SUB ");
    used=(size_t)n;
    for (i=0;i<count;++i) {
        n=snprintf(reply+used,sizeof(reply)-used,"%s%s",i?",":"",
                   g_variable_names[all?i:g_channels[i]]);
        if (n<0 || (size_t)n>=sizeof(reply)-used-1) {
            command_error("reply too long"); return;
        }
        used+=(size_t)n;
    }
    reply[used++]='\n'; reply[used]='\0';
    send_reply(reply);
}
static void process_command(char *line)
{
    char *command=trim(line), *arg, *end;
    uint8_t selected[VOFA_MAX_CHANNELS];
    size_t count=0, id;
    unsigned long number;
    char reply[80];
    ++wifi_telemetry_commands;
    if (*command=='[') { process_slider(command); return; }
    if (strcmp(command,"LIST?")==0) { reply_names(1); return; }
    if (strcmp(command,"GET?")==0) {
        reply_names(0);
        (void)snprintf(reply,sizeof(reply),"MCAR RATE %lu\n",(unsigned long)wifi_telemetry_period_ms);
        send_reply(reply);
        (void)snprintf(reply,sizeof(reply),"MCAR STREAM %lu\n",(unsigned long)wifi_telemetry_stream_enabled);
        send_reply(reply); return;
    }
    arg=strpbrk(command," \t");
    if (arg==NULL) { command_error("expected SUB/RATE/STREAM/LIST?/GET?"); return; }
    *arg++='\0'; arg=trim(arg);
    if (strcmp(command,"SUB")==0) {
        /* Validate the entire list before replacing anything. Empty fields,
         * duplicate names, unknown names and >40 channels leave the old list intact. */
        while (1) {
            char *comma=strchr(arg,',');
            size_t j;
            if (comma!=NULL) *comma='\0';
            arg=trim(arg);
            if (*arg=='\0' || count==VOFA_MAX_CHANNELS) { command_error("invalid channel count"); return; }
            for (id=0;id<TV_COUNT;++id) if (strcmp(arg,g_variable_names[id])==0) break;
            if (id==TV_COUNT) { command_error("unknown variable"); return; }
            for (j=0;j<count;++j) if (selected[j]==id) { command_error("duplicate variable"); return; }
            selected[count++]=(uint8_t)id;
            if (comma==NULL) break;
            arg=comma+1;
        }
        memcpy(g_channels,selected,count);
        wifi_telemetry_channel_count=(uint32_t)count;
        reply_names(0); return;
    }
    if (*arg<'0' || *arg>'9') { command_error("expected unsigned integer"); return; }
    errno=0;
    number=strtoul(arg,&end,10);
    if (errno!=0 || *end!='\0') { command_error("invalid integer"); return; }
    if (strcmp(command,"RATE")==0 && number>=2 && number<=1000) {
        wifi_telemetry_period_ms=(uint32_t)number;
        (void)snprintf(reply,sizeof(reply),"MCAR RATE %lu\n",number);
    } else if (strcmp(command,"STREAM")==0 && number<=1) {
        wifi_telemetry_stream_enabled=(uint32_t)number;
        (void)snprintf(reply,sizeof(reply),"MCAR STREAM %lu\n",number);
    } else { command_error("RATE 2..1000 or STREAM 0/1"); return; }
    send_reply(reply);
}
static void receive_commands(void)
{
    uint8_t bytes[256];
    uint32 i, count=wifi_spi_read_buffer(bytes,sizeof(bytes));
    for (i=0;i<count;++i) {
        uint8_t ch=bytes[i];
        if (ch=='[' && !g_slider_receiving && !g_command_discard) {
            g_command_length=0; g_slider_receiving=1;
        }
        if (ch=='\n' || (g_slider_receiving && ch==']')) {
            if (ch==']' && !g_command_discard) {
                if (g_command_length==sizeof(g_command)-1) g_command_discard=1;
                else g_command[g_command_length++]=(char)ch;
            }
            if (g_command_discard) {
                ++wifi_telemetry_commands;
                command_error("invalid or oversized line");
            }
            else if (g_command_length!=0) {
                g_command[g_command_length]='\0'; process_command(g_command);
            }
            g_command_length=0; g_command_discard=0; g_slider_receiving=0;
        } else if (ch=='\r') {
            /* Accept both LF and CRLF from the existing host text sender. */
        } else if (g_command_discard) {
            /* Recover only at the next newline. */
        } else if ((ch<32 && ch!='\t') || ch>126 || g_command_length==sizeof(g_command)-1) {
            g_command_discard=1;
        } else g_command[g_command_length++]=(char)ch;
    }
}
static size_t capture_channels(float channels[VOFA_MAX_CHANNELS])
{
    size_t i, count=wifi_telemetry_channel_count;
    uint32 primask=interrupt_global_disable();
    for (i=0;i<count;++i) {
        switch (g_channels[i]) {
#define VARIABLE_VALUE(name, expr) case TV_##name: channels[i]=(float)(expr); break;
            TELEMETRY_VARIABLES(VARIABLE_VALUE)
#undef VARIABLE_VALUE
            default: channels[i]=0.0f; break;
        }
    }
    interrupt_global_enable(primask);
    return count;
}
static int imu_wifi_init_once(void)
{
    memset(wifi_spi_version,0,sizeof(wifi_spi_version));
    memset(wifi_spi_ip_addr_port,0,sizeof(wifi_spi_ip_addr_port));
    imu_wifi_status=2;
    if (wifi_spi_init(0,0)!=0 || strncmp(wifi_spi_version,"V2",2)!=0) return -1;
    imu_wifi_status=3;
    if (wifi_spi_wifi_connect(QCAR_WIFI_SSID,QCAR_WIFI_PASSWORD)!=0) return -2;
    imu_wifi_status=4;
    if (wifi_spi_socket_connect("UDP",QCAR_WIFI_TARGET_IP,
                               QCAR_WIFI_TARGET_PORT,QCAR_WIFI_LOCAL_PORT)!=0) return -3;
    return 1;
}
#endif
int wifispi_init(void)
{
#if QCAR_WIFI_ENABLED
    uint32_t attempt;
    int result;
    wifi_ready=0;
    imu_wifi_status=0; imu_wifi_init_attempts=0; imu_wifi_last_error=0;
    /* 保留原初始化统计语义：启用模式重新初始化不清发送累计数。 */
    system_delay_ms(QCAR_WIFI_STARTUP_DELAY_MS);
    for (attempt=0;attempt<QCAR_WIFI_INIT_ATTEMPTS;++attempt) {
        if (attempt!=0) system_delay_ms(QCAR_WIFI_RETRY_DELAY_MS);
        ++imu_wifi_init_attempts;
        result=imu_wifi_init_once();
        if (result==1) { wifi_ready=1; imu_wifi_status=1; return 1; }
        imu_wifi_last_error=result; imu_wifi_status=result;
    }
    return 0;
#else
    /* 禁用时绝不初始化 SPI/GPIO，防止覆盖电机引脚复用。 */
    imu_wifi_status=-5; imu_wifi_init_attempts=0;
    imu_wifi_last_error=0; imu_wifi_tx_packets=0;
    return 0;
#endif
}
int wifispi_send_floats(const float *channels,size_t count)
{
#if QCAR_WIFI_ENABLED
    uint8_t packet[VOFA_MAX_FRAME_BYTES];
    size_t packet_bytes;
    if (!wifi_ready) return 0;
    packet_bytes=vofa_pack(packet,sizeof(packet),channels,count);
    if (packet_bytes==0u) return 0;
    /* send_buffer 返回未发送字节数；send_now 结束本 UDP 数据报。 */
    if (wifi_spi_send_buffer(packet,(uint32_t)packet_bytes)!=0 ||
        wifi_spi_udp_send_now()!=0) { imu_wifi_status=-4; return 0; }
    imu_wifi_status=1; ++imu_wifi_tx_packets;
    return 1;
#else
    (void)channels; (void)count;
    return 0;
#endif
}
int wifispi_send_imu(const float angles_deg[3])
{
    return wifispi_send_floats(angles_deg,3u);
}
void wifispi_telemetry_init(void)
{
#if QCAR_WIFI_ENABLED
    telemetry_reset();
#endif
    (void)wifispi_init();
#if QCAR_WIFI_ENABLED
    g_ticks_per_ms=(uint32)(IfxStm_getFrequency(&MODULE_STM0)/1000.0f);
    g_last_wifi_tx_ticks=g_last_wifi_rx_ticks=IfxStm_getLower(&MODULE_STM0);
#endif
}
void wifispi_telemetry_service(void)
{
#if QCAR_WIFI_ENABLED
    uint32 now;
    float channels[VOFA_MAX_CHANNELS];
    size_t count;
    if (!wifi_ready) return;
    now=IfxStm_getLower(&MODULE_STM0);
    /* Poll independently of streaming so STREAM 0 and slow RATE cannot lock
     * the host out. Limit SPI polling to 100 Hz in the main loop. */
    if ((uint32)(now-g_last_wifi_rx_ticks)>=g_ticks_per_ms*10u) {
        g_last_wifi_rx_ticks=now; receive_commands();
    }
    now=IfxStm_getLower(&MODULE_STM0);
    if (!wifi_telemetry_stream_enabled ||
        (uint32)(now-g_last_wifi_tx_ticks)<g_ticks_per_ms*wifi_telemetry_period_ms) return;
    g_last_wifi_tx_ticks=now;
    count=capture_channels(channels);
    (void)wifispi_send_floats(channels,count);
#endif
}

void wifispi_telemetry_stop(void)
{
#if QCAR_WIFI_ENABLED
    int was_ready=wifi_ready;
    wifi_ready=0;
    wifi_telemetry_stream_enabled=0;
    if (was_ready && wifi_spi_socket_disconnect()!=0) { imu_wifi_status=-4; return; }
#endif
    imu_wifi_status=0;
}
int wifispi_is_ready(void)
{
#if QCAR_WIFI_ENABLED
    return wifi_ready;
#else
    return 0;
#endif
}
