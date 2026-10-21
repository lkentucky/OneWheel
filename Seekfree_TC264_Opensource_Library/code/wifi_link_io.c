/* Cooperative implementation of the existing Seekfree WiFi-SPI wire protocol.
 * Exactly one CS assertion per poll. Never call the SDK's wait/drain loops.
 * Initialization and socket configuration remain owned by the SDK, outside RUN.
 */
#include "zf_common_headfile.h"
#include "wifi_link_io.h"
#include "IfxStm.h"

#define IO_TIMEOUT_MS (250u)

typedef enum { IO_IDLE, IO_DATA, IO_FLUSH, IO_ACK, IO_RX, IO_DRAIN } io_phase;
static io_phase io_state;
static io_phase io_after_drain;
static wifi_link_rx_fn io_receive;
static const uint8 *io_data;
static uint16 io_length;
static uint32 io_since_ms;
static uint32 io_ticks_per_us;
static uint32 io_last_cost;
static uint32 io_peak_cost;

static uint8 io_exchange(uint8 command, const uint8 *data, uint16 length,
                         uint8 clock_payload)
{
    wifi_spi_packets_struct packet;
    uint8 outgoing[WIFI_SPI_RECVIVE_SIZE];
    uint16 first = (length > WIFI_SPI_RECVIVE_SIZE) ? WIFI_SPI_RECVIVE_SIZE : length;
    uint32 start = IfxStm_getLower(&MODULE_STM0);
    memset(&packet, 0, sizeof(packet));
    memset(outgoing, 0, sizeof(outgoing));
    packet.head.command = command;
    packet.head.length = length;
    if (data && first)
    {
        memcpy(outgoing, data, first);
    }
    gpio_low(WIFI_SPI_CS_PIN);
    spi_transfer_8bit(WIFI_SPI_INDEX, (uint8 *)&packet.head,
                     (uint8 *)&packet.head, sizeof(packet.head));
    if (clock_payload)
    {
        /* The module always returns up to 32 bytes, including on short writes. */
        spi_transfer_8bit(WIFI_SPI_INDEX, outgoing, packet.buffer, WIFI_SPI_RECVIVE_SIZE);
        if (data && length > WIFI_SPI_RECVIVE_SIZE)
        {
            spi_write_8bit_array(WIFI_SPI_INDEX, data + WIFI_SPI_RECVIVE_SIZE,
                                length - WIFI_SPI_RECVIVE_SIZE);
        }
    }
    gpio_high(WIFI_SPI_CS_PIN);
    io_last_cost = (IfxStm_getLower(&MODULE_STM0) - start) / io_ticks_per_us;
    if (io_last_cost > io_peak_cost)
    {
        io_peak_cost = io_last_cost;
    }
    if (packet.head.command == WIFI_SPI_REPLY_DATA_START ||
        packet.head.command == WIFI_SPI_REPLY_DATA_END)
    {
        if (!clock_payload || packet.head.length > WIFI_SPI_RECVIVE_SIZE)
        {
            return WIFI_SPI_REPLY_ERROR;
        }
        if (packet.head.length && io_receive)
        {
            io_receive(packet.buffer, packet.head.length);
        }
    }
    return packet.head.command;
}

void wifi_link_io_init(wifi_link_rx_fn receive, uint32 ticks_per_us)
{
    io_state = IO_IDLE;
    io_receive = receive;
    io_data = NULL;
    io_length = 0;
    io_ticks_per_us = ticks_per_us ? ticks_per_us : 1u;
    io_last_cost = 0;
    io_peak_cost = 0;
}

uint8 wifi_link_io_busy(void)
{
    return io_state != IO_IDLE;
}

uint8 wifi_link_io_send(const uint8 *data, uint16 length, uint32 now_ms)
{
    if (io_state != IO_IDLE || !data || !length || length > WIFI_SPI_TRANSFER_SIZE)
    {
        return 1;
    }
    io_data = data;
    io_length = length;
    io_since_ms = now_ms;
    io_state = IO_DATA;
    return 0;
}

void wifi_link_io_receive(uint32 now_ms)
{
    if (io_state == IO_IDLE)
    {
        io_since_ms = now_ms;
        io_state = IO_RX;
    }
}

wifi_link_io_result wifi_link_io_poll(uint32 now_ms)
{
    uint8 reply;
    if (io_state == IO_IDLE)
    {
        return WIFI_LINK_IO_IDLE;
    }
    if ((uint32)(now_ms - io_since_ms) >= IO_TIMEOUT_MS)
    {
        io_state = IO_IDLE;
        return WIFI_LINK_IO_ERROR;
    }
    if (!gpio_get_level(WIFI_SPI_INT_PIN))
    {
        return WIFI_LINK_IO_BUSY;
    }
    switch (io_state)
    {
        case IO_DATA:
            reply = io_exchange(WIFI_SPI_DATA, io_data, io_length, 1);
            io_state = IO_FLUSH;
            if (reply == WIFI_SPI_REPLY_DATA_START)
            {
                io_after_drain = IO_FLUSH;
                io_state = IO_DRAIN;
            }
            break;
        case IO_FLUSH:
            reply = io_exchange(WIFI_SPI_UDP_SEND, NULL, 0, 1);
            io_state = IO_ACK;
            if (reply == WIFI_SPI_REPLY_DATA_START)
            {
                io_after_drain = IO_ACK;
                io_state = IO_DRAIN;
            }
            break;
        case IO_DRAIN:
            reply = io_exchange(WIFI_SPI_DATA, NULL, 0, 1);
            if (reply != WIFI_SPI_REPLY_DATA_START) { io_state = io_after_drain; }
            break;
        case IO_ACK:
            reply = io_exchange(WIFI_SPI_DATA, NULL, 0, 0);
            io_state = IO_IDLE;
            return reply == WIFI_SPI_REPLY_OK ? WIFI_LINK_IO_SENT : WIFI_LINK_IO_ERROR;
        case IO_RX:
            reply = io_exchange(WIFI_SPI_DATA, NULL, 0, 1);
            if (reply != WIFI_SPI_REPLY_DATA_START)
            {
                io_state = IO_IDLE;
                return (reply == WIFI_SPI_REPLY_OK || reply == WIFI_SPI_REPLY_DATA_END) ?
                    WIFI_LINK_IO_RECEIVED : WIFI_LINK_IO_ERROR;
            }
            break;
        default:
            io_state = IO_IDLE;
            return WIFI_LINK_IO_ERROR;
    }
    if (reply != WIFI_SPI_REPLY_OK && reply != WIFI_SPI_REPLY_DATA_START &&
        reply != WIFI_SPI_REPLY_DATA_END)
    {
        io_state = IO_IDLE;
        return WIFI_LINK_IO_ERROR;
    }
    /* A multi-chunk RX burst retains its original deadline. A transmitter may
       start a new deadline when it advances to the next handshake phase. */
    if (io_state != IO_RX && io_state != IO_DRAIN)
    {
        io_since_ms = now_ms;
    }
    return WIFI_LINK_IO_BUSY;
}

uint32 wifi_link_io_last_us(void) { return io_last_cost; }
uint32 wifi_link_io_peak_us(void) { return io_peak_cost; }
