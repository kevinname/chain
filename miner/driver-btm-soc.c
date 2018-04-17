#include "config.h"
#include <assert.h>

#include <limits.h>
#include <pthread.h>
#include <stdio.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/file.h>
#include <dirent.h>
#include <unistd.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <assert.h>
#include <unistd.h>
#include <math.h>

#ifndef WIN32
#include <sys/select.h>
//#include <termios.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <net/if.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <errno.h>
#include <sys/stat.h>

#ifndef O_CLOEXEC
#define O_CLOEXEC 0
#endif
#else
#include "compat.h"
#include <windows.h>
#include <io.h>
#endif

#include "elist.h"
//#include "usbutils.h"
#include "driver-btm-soc.h"
#include "util.h"


/****************** checked ***************************/
int BM1680_ack_record[BITMAIN_MAX_CHAIN_NUM][2][BM1680_MAX_CMD_NUM] = {0};

int const red_led = 941;
int const green_led = 942;

unsigned char bt8d = 0x1a;
int i2c_slave_addr = 0;

pthread_mutex_t reinit_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t i2c_mutex = PTHREAD_MUTEX_INITIALIZER;  // used when cpu operates i2c interface
pthread_mutex_t iic_mutex = PTHREAD_MUTEX_INITIALIZER;  // used when cpu communicate with pic
pthread_mutex_t reg_read_mutex = PTHREAD_MUTEX_INITIALIZER;     // used when read ASIC register periodic in pthread
pthread_mutex_t work_queue_mutex = PTHREAD_MUTEX_INITIALIZER;

unsigned int gChipOffset = 0;   // record register CHIP_OFFSET value
unsigned int gCoreOffset = 0;   // record register CORE_OFFSET value

int opt_bitmain_fan_pwm = 50;       // if not control fan speed according to temperature, use this parameter to control fan speed
bool opt_bitmain_fan_ctrl = false;  // false: control fan speed according to temperature; true: use opt_bitmain_fan_pwm to control fan speed
int opt_bitmain_B3_freq = 200;

int last_temperature = 0, temp_highest = 0;

struct all_parameters dev;
struct thr_info *pic_heart_beat;
struct thr_info *scan_reg_id;
struct thr_info *read_temp_id;
struct thr_info *check_status_id;
struct thr_info *check_miner_status_id;                  // thread id for check system
struct thr_info *read_hash_rate;
struct thr_info *restore_voltage;
struct thr_info *check_fan_id;
struct dev_info dev_info[BITMAIN_MAX_CHAIN_NUM];
bool is_rt = true;
enum I2C_TYPE { LOCAL, REMOTE, OFFSET, ID};
int32_t global_workid = 0;


unsigned char g_CHIP_ADDR_reg_value_num[BITMAIN_MAX_CHAIN_NUM] = {0};                   // record receive how many CHIP_ADDR register value each chain
unsigned int g_CHIP_ADDR_reg_value[BITMAIN_MAX_CHAIN_NUM][128] = {{0}};             // record receive CHIP_ADDR register value each chain
unsigned char g_HASH_RATE_reg_value_num[BITMAIN_MAX_CHAIN_NUM] = {0};                   // record receive how many HASH_RATE register value each chain
unsigned int g_HASH_RATE_reg_value[BITMAIN_MAX_CHAIN_NUM][128] = {{0}};             // record receive HASH_RATE register value each chain
bool g_HASH_RATE_reg_value_from_which_asic[BITMAIN_MAX_CHAIN_NUM][128] = {{0}}; // record receive HASH_RATE register value from which asic
unsigned char g_CHIP_STATUS_reg_value_num[BITMAIN_MAX_CHAIN_NUM] = {0};             // record receive how many CHIP_STATUS register value each chain
unsigned int g_CHIP_STATUS_reg_value[BITMAIN_MAX_CHAIN_NUM][128] = {{0}};           // record receive CHIP_STATUS register value each chain


uint64_t rate[BITMAIN_MAX_CHAIN_NUM] = {0};
unsigned char rate_error[BITMAIN_MAX_CHAIN_NUM] = {0};  // record not receive all ASIC's HASH_RATE register value time
char displayed_rate[BITMAIN_MAX_CHAIN_NUM][16];
unsigned char pic_version[BITMAIN_MAX_CHAIN_NUM] = {0};
bool gLost_internet_10_min = false;     // lost internet for 10 minutes
bool gGot_Temperature_value = false;    // wether read out new temperature value
bool gMinerStatus_Low_Hashrate = false;         // hash rate is too low
bool gMinerStatus_High_Temp = false;            // the temperature is higher than MAX_TEMP
bool gMinerStatus_Not_read_all_sensor = false;  // do not read out all sensor's temperature
bool gMinerStatus_Lost_connection_to_pool = false;  // can't receive job from pool
bool gFan_Error = false;                        // lost fan or fan speed to low

bool once_error = false;
bool status_error = false;
bool stop = false;

unsigned char gMinerStatus_High_Temp_Counter = 0;           // the temperature is higher than MAX_TEMP counter

/****************** checked end ***************************/

struct thr_info *read_nonce_reg_id;                 // thread id for read nonce and register
uint64_t h = 0;
uint64_t h_each_chain[BITMAIN_MAX_CHAIN_NUM] = {0};
double each_chain_h_avg[BITMAIN_MAX_CHAIN_NUM] = {0};
double geach_chain_h_all = 0;
unsigned char hash_board_id[BITMAIN_MAX_CHAIN_NUM][12];
unsigned char voltage[BITMAIN_MAX_CHAIN_NUM] = {0,0,0,0};
pthread_mutex_t reg_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t nonce_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t tty_write_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t temp_buf_mutex = PTHREAD_MUTEX_INITIALIZER;
static bool start_send[BITMAIN_MAX_CHAIN_NUM] = {false};
static bool reiniting[BITMAIN_MAX_CHAIN_NUM] = {false};
static bool start_recv[BITMAIN_MAX_CHAIN_NUM] = {false};
static bool need_reinit[BITMAIN_MAX_CHAIN_NUM] = {false};
static bool update_asic_num[BITMAIN_MAX_CHAIN_NUM] = {false};
static bool need_recheck[BITMAIN_MAX_CHAIN_NUM] = {false};

bool check_rate = false;
bool gBegin_get_nonce[BITMAIN_MAX_CHAIN_NUM] = {false};
bool send_heart = true;
bool new_block[BITMAIN_MAX_CHAIN_NUM] = {false};
bool update_seed[BITMAIN_MAX_CHAIN_NUM] = {false};

uint64_t hashboard_average_hash_rate[BITMAIN_MAX_CHAIN_NUM] = {0};
uint64_t hashboard_real_time_hash_rate[BITMAIN_MAX_CHAIN_NUM] = {0};
struct nonce_buf nonce_fifo;
struct reg_buf reg_fifo;
struct timeval tv_send_job = {0, 0};

uint8_t chain_voltage[BITMAIN_MAX_CHAIN_NUM] = {0};
/******************** global functions ********************/

int fpga_fd;
int fd_fpga_mem;                                // fpga memory
unsigned int *fpga_mem_addr = NULL;             // fpga memory address
unsigned int *axi_fpga_addr = NULL;

pthread_mutex_t uart_send_mutex[BITMAIN_MAX_CHAIN_NUM] = {PTHREAD_MUTEX_INITIALIZER};
pthread_mutex_t uart_receive_mutex[BITMAIN_MAX_CHAIN_NUM] = {PTHREAD_MUTEX_INITIALIZER};


int bitmain_axi_init()
{
    unsigned int data;
    int ret=0;

    fpga_fd = open("/dev/axi_fpga_dev", O_RDWR);
    if(fpga_fd < 0)
    {
        printf("/dev/axi_fpga_dev open failed. fd = %d\n", fpga_fd);
        return -1;
    }

    axi_fpga_addr = mmap(NULL, TOTAL_LEN, PROT_READ|PROT_WRITE, MAP_SHARED, fpga_fd, 0);
    if(!axi_fpga_addr)
    {
        printf("mmap axi_fpga_addr failed. axi_fpga_addr = 0x%x\n", axi_fpga_addr);
        return -1;
    }
    printf("mmap axi_fpga_addr = 0x%x\n", axi_fpga_addr);

    //check the value in address 0xff200000
    data = (*axi_fpga_addr & 0x0000ffff);
    printf("axi_fpga_addr data = 0x%x\n", data);

    fd_fpga_mem = open("/dev/fpga_mem", O_RDWR);
    if(fd_fpga_mem < 0)
    {
        printf("/dev/fpga_mem open failed. fd_fpga_mem = %d\n", fd_fpga_mem);
        return -1;
    }

    fpga_mem_addr = mmap(NULL, FPGA_MEM_TOTAL_LEN, PROT_READ|PROT_WRITE, MAP_SHARED, fd_fpga_mem, 0);
    if(!fpga_mem_addr)
    {
        printf("mmap fpga_mem_addr failed. fpga_mem_addr = 0x%x\n", fpga_mem_addr);
        return -1;
    }
    printf("mmap fpga_mem_addr = 0x%x\n", fpga_mem_addr);

    return ret;
}

int bitmain_axi_close()
{
    int ret = 0;

    printf("\n\n--- %s\n", __FUNCTION__);

    ret = munmap((void *)axi_fpga_addr, TOTAL_LEN);
    if(ret<0)
    {
        printf("munmap failed!\n");
    }

    ret = munmap((void *)fpga_mem_addr, FPGA_MEM_TOTAL_LEN);
    if(ret<0)
    {
        printf("munmap failed!\n");
    }

    close(fpga_fd);
    close(fd_fpga_mem);
}

void print_bin(uint8_t * cmd, size_t len)
{
    char *hex_buff = NULL;
    hex_buff = bin2hex(cmd, len);
    applog(LOG_NOTICE,"%s",hex_buff);
    free(hex_buff);
    hex_buff = NULL;
}
unsigned int read_axi_fpga(unsigned int address)
{
    volatile unsigned int data = 0xffffffff;
    data = *((unsigned int *)(axi_fpga_addr + address));
    //applog(LOG_NOTICE,"%s:%x %x %x", __FUNCTION__,axi_fpga_addr, address, data);
    return data;
}

void write_axi_fpga(unsigned int address, unsigned int data)
{
    *((unsigned int *)(axi_fpga_addr + address)) = data;
}

void init_fpga(void)
{
    unsigned int data = RESET_FPGA | RST;

    printf("\n--- %s\n", __FUNCTION__);

    write_axi_fpga(QN_WRITE_DATA_COMMAND, data);

    while(read_axi_fpga(QN_WRITE_DATA_COMMAND) & RST)
    {
        cgsleep_us(10000);
    }

    cgsleep_us(100*1000);
}

unsigned char asic_baud_to_fpga_baud(unsigned char asic_baud)
{
    switch(asic_baud)
    {
        case 0:
            return 1;

        case 1:
            return 3;

        case 2:
            return 5;

        case 6:
            return 13;

        case 26:
            return 53;

        default:
            printf("%s: Don't support ASIC baud = %d, error!!!\n", __FUNCTION__, asic_baud);
            return 53;
    }
}

void set_fpga_baud(unsigned char asic_baud)
{
    unsigned char fpga_baud;
    unsigned char ret;
    unsigned int value = 0;

    //fpga_baud = asic_baud_to_fpga_baud(asic_baud);

    write_axi_fpga(BT8D, asic_baud);
}


unsigned int check_how_many_uart_data_in_fpga(unsigned char which_uart)
{
    //printf("%s\n", __FUNCTION__);
    switch(which_uart)
    {
        case 1:
            return ((read_axi_fpga(RECEIVE_FIFO1_2_STATUS) >> 16) & 0x000003ff);

        case 2:
            return (read_axi_fpga(RECEIVE_FIFO1_2_STATUS) & 0x000003ff);

        case 3:
            return ((read_axi_fpga(RECEIVE_FIFO3_8_STATUS) >> 16) & 0x000003ff);

        case 8:
            return (read_axi_fpga(RECEIVE_FIFO3_8_STATUS) & 0x000003ff);

        case 9:
            return ((read_axi_fpga(RECEIVE_FIFO9_10_STATUS) >> 16) & 0x000003ff);

        case 10:
            return (read_axi_fpga(RECEIVE_FIFO9_10_STATUS) & 0x000003ff);

        case 11:
            return ((read_axi_fpga(RECEIVE_FIFO11_12_STATUS) >> 16) & 0x000003ff);

        case 12:
            return (read_axi_fpga(RECEIVE_FIFO11_12_STATUS) & 0x000003ff);

        case 13:
            return ((read_axi_fpga(RECEIVE_FIFO13_14_STATUS) >> 16) & 0x000003ff);

        case 14:
            return (read_axi_fpga(RECEIVE_FIFO13_14_STATUS) & 0x000003ff);

        default:
            printf("%s: The uart%d is not supported!!!\n", __FUNCTION__, which_uart);
            return 0;
    }
}


unsigned int read_uart_data_in_fpga(unsigned char which_uart, unsigned char *buf, unsigned int length)
{
    unsigned int chain_read_enable_addr = 0, chain_read_data_addr = 0;
    unsigned int data = 0, read_loop = 0, i = 0, ret_data = 0, ret_length = 0;

    switch(which_uart)
    {
        case 1:
            chain_read_enable_addr = CHAIN1_READ_ENABLE;
            chain_read_data_addr   = CHAIN1_READ_DATA;
            break;

        case 2:
            chain_read_enable_addr = CHAIN2_READ_ENABLE;
            chain_read_data_addr   = CHAIN2_READ_DATA;
            break;

        case 3:
            chain_read_enable_addr = CHAIN3_READ_ENABLE;
            chain_read_data_addr   = CHAIN3_READ_DATA;
            break;

        case 8:
            chain_read_enable_addr = CHAIN8_READ_ENABLE;
            chain_read_data_addr   = CHAIN8_READ_DATA;
            break;

        case 9:
            chain_read_enable_addr = CHAIN9_READ_ENABLE;
            chain_read_data_addr   = CHAIN9_READ_DATA;
            break;

        case 10:
            chain_read_enable_addr = CHAIN10_READ_ENABLE;
            chain_read_data_addr   = CHAIN10_READ_DATA;
            break;

        case 11:
            chain_read_enable_addr = CHAIN11_READ_ENABLE;
            chain_read_data_addr   = CHAIN11_READ_DATA;
            break;

        case 12:
            chain_read_enable_addr = CHAIN12_READ_ENABLE;
            chain_read_data_addr   = CHAIN12_READ_DATA;
            break;

        case 13:
            chain_read_enable_addr = CHAIN13_READ_ENABLE;
            chain_read_data_addr   = CHAIN13_READ_DATA;
            break;

        case 14:
            chain_read_enable_addr = CHAIN14_READ_ENABLE;
            chain_read_data_addr   = CHAIN14_READ_DATA;
            break;

        default:
            applog(LOG_ERR,"%s: The uart%d is not supported!!!\n", __FUNCTION__);
            return 0;
    }

    // enable read uart, and tell fpga how many data will be read
    data = 0x80000000 | (length & 0x000003ff);
    write_axi_fpga(chain_read_enable_addr, data);

    // begin read data
    read_loop = length / 4;
    for(i=0; i<read_loop; i++)
    {
        ret_data = read_axi_fpga(chain_read_data_addr);
        *(buf + 4*i + 0) = (unsigned char)((ret_data >> 24) & 0x000000ff);
        *(buf + 4*i + 1) = (unsigned char)((ret_data >> 16) & 0x000000ff);
        *(buf + 4*i + 2) = (unsigned char)((ret_data >> 8) & 0x000000ff);
        *(buf + 4*i + 3) = (unsigned char)(ret_data & 0x000000ff);
    }
    ret_length = read_loop * 4;

    read_loop = length % 4;
    if(read_loop)
    {
        ret_data = read_axi_fpga(chain_read_data_addr);
        switch(read_loop)
        {
            case 1:
                *(buf + 4*i + 0) = (unsigned char)((ret_data >> 24) & 0x000000ff);
                ret_length += 1;
                break;

            case 2:
                *(buf + 4*i + 0) = (unsigned char)((ret_data >> 24) & 0x000000ff);
                *(buf + 4*i + 1) = (unsigned char)((ret_data >> 16) & 0x000000ff);
                ret_length += 2;
                break;

            case 3:
                *(buf + 4*i + 0) = (unsigned char)((ret_data >> 24) & 0x000000ff);
                *(buf + 4*i + 1) = (unsigned char)((ret_data >> 16) & 0x000000ff);
                *(buf + 4*i + 2) = (unsigned char)((ret_data >> 8) & 0x000000ff);
                ret_length += 3;
                break;

            default:
                applog(LOG_ERR,"%s: the uart%d left data is 4*N length, error!!!\n", __FUNCTION__, which_uart);
                break;
        }
    }

    return ret_length;
}

unsigned int uart_send(unsigned char which_uart, unsigned char *buf, unsigned int length)
{
    int ret=0, send_counter=0, send_loop = 0, i = 0, j = 0;
    unsigned char send_buf[160] = {0};
    unsigned int send_data_len = length, send_data = 0;
    unsigned int chain_send_fifo_status_addr = 0, chain_send_ready_addr = 0, chain_send_buffer_addr = 0;
    unsigned int max_fpga_can_send_uart_data_len = 0, bit_shift_num = 0;

    pthread_mutex_lock(&uart_send_mutex[which_uart]);

    switch(which_uart)
    {
        case 1:
            bit_shift_num = 24;
            chain_send_fifo_status_addr = CHAIN1_3_8_SEND_FIFO_STATUS;
            chain_send_ready_addr       = CHAIN1_SEND_READY;
            chain_send_buffer_addr      = CHAIN1_SEND_BUFFER;
            break;

        case 2:
            bit_shift_num = 16;
            chain_send_fifo_status_addr = CHAIN1_3_8_SEND_FIFO_STATUS;
            chain_send_ready_addr       = CHAIN2_SEND_READY;
            chain_send_buffer_addr      = CHAIN2_SEND_BUFFER;
            break;

        case 3:
            bit_shift_num = 8;
            chain_send_fifo_status_addr = CHAIN1_3_8_SEND_FIFO_STATUS;
            chain_send_ready_addr       = CHAIN3_SEND_READY;
            chain_send_buffer_addr      = CHAIN3_SEND_BUFFER;
            break;

        case 8:
            bit_shift_num = 0;
            chain_send_fifo_status_addr = CHAIN1_3_8_SEND_FIFO_STATUS;
            chain_send_ready_addr       = CHAIN8_SEND_READY;
            chain_send_buffer_addr      = CHAIN8_SEND_BUFFER;
            break;

        case 9:
            bit_shift_num = 24;
            chain_send_fifo_status_addr = CHAIN9_12_SEND_FIFO_STATUS;
            chain_send_ready_addr       = CHAIN9_SEND_READY;
            chain_send_buffer_addr      = CHAIN9_SEND_BUFFER;
            break;

        case 10:
            bit_shift_num = 16;
            chain_send_fifo_status_addr = CHAIN9_12_SEND_FIFO_STATUS;
            chain_send_ready_addr       = CHAIN10_SEND_READY;
            chain_send_buffer_addr      = CHAIN10_SEND_BUFFER;
            break;

        case 11:
            bit_shift_num = 8;
            chain_send_fifo_status_addr = CHAIN9_12_SEND_FIFO_STATUS;
            chain_send_ready_addr       = CHAIN11_SEND_READY;
            chain_send_buffer_addr      = CHAIN11_SEND_BUFFER;
            break;

        case 12:
            bit_shift_num = 0;
            chain_send_fifo_status_addr = CHAIN9_12_SEND_FIFO_STATUS;
            chain_send_ready_addr       = CHAIN12_SEND_READY;
            chain_send_buffer_addr      = CHAIN12_SEND_BUFFER;
            break;

        case 13:
            bit_shift_num = 24;
            chain_send_fifo_status_addr = CHAIN13_14_SEND_FIFO_STATUS;
            chain_send_ready_addr       = CHAIN13_SEND_READY;
            chain_send_buffer_addr      = CHAIN13_SEND_BUFFER;
            break;

        case 14:
            bit_shift_num = 16;
            chain_send_fifo_status_addr = CHAIN13_14_SEND_FIFO_STATUS;
            chain_send_ready_addr       = CHAIN14_SEND_READY;
            chain_send_buffer_addr      = CHAIN14_SEND_BUFFER;
            break;

        default:
            applog(LOG_ERR,"%s: The uart%d is not supported!!!\n", __FUNCTION__);
            return 0;
    }

    send_counter = 0;
    while(1)
    {
        max_fpga_can_send_uart_data_len = (read_axi_fpga(chain_send_fifo_status_addr) >> bit_shift_num) & 0x000000ff;

        if(max_fpga_can_send_uart_data_len >= length)
        {
            break;
        }
        else
        {
            usleep(3*1000);
            send_counter++;
            if(send_counter > 20)
            {
                pthread_mutex_unlock(&uart_send_mutex[which_uart]);
                applog(LOG_ERR,"%s: uart%d always dose not has enough send fifo space, break\n", __FUNCTION__, which_uart);
                return 0;
            }
        }
    }

    // check whether uart is ready
    send_counter = 0;
    while(read_axi_fpga(chain_send_ready_addr) & 0x80000000)
    {
        if(send_counter > 20)
        {
            pthread_mutex_unlock(&uart_send_mutex[which_uart]);
            applog(LOG_ERR,"%s: uart%d always busy, break\n", __FUNCTION__, which_uart);
            return 0;
        }
        send_counter++;
        usleep(3*1000);
    }

    // prepare sending data
    memcpy(send_buf, buf, length);

    send_loop = send_data_len / 4;
    for(i=0; i<send_loop; i++)
    {
        send_data = (send_buf[4*i + 0] << 24) | (send_buf[4*i + 1] << 16) | (send_buf[4*i + 2] << 8) | send_buf[4*i + 3];
        write_axi_fpga(chain_send_buffer_addr, send_data);
    }

    send_loop = send_data_len % 4;
    if(send_loop)
    {
        switch(send_loop)
        {
            case 1:
                send_data = send_buf[4*i + 0] << 24;
                break;

            case 2:
                send_data = (send_buf[4*i + 0] << 24) | (send_buf[4*i + 1] << 16);
                break;

            case 3:
                send_data = (send_buf[4*i + 0] << 24) | (send_buf[4*i + 1] << 16) | (send_buf[4*i + 2] << 8);
                break;

            default:
                applog(LOG_ERR,"%s: the uart%d send left data is 4*N length, error!!!\n", __FUNCTION__, which_uart);
                break;
        }
        write_axi_fpga(chain_send_buffer_addr, send_data);
    }

    // send data
    write_axi_fpga(chain_send_ready_addr, 0x80000000 | send_data_len);

    pthread_mutex_unlock(&uart_send_mutex[which_uart]);
	cgsleep_ms(5);
    return send_data_len;
}

unsigned int uart_receive(unsigned char which_uart, unsigned char *buf, unsigned int buf_length)
{
    unsigned int len=0, nbytes=0, nbytes2=0, nbytes3=0, asic_return_data_len = 0 ;

    pthread_mutex_lock(&uart_receive_mutex[which_uart]);
    nbytes = check_how_many_uart_data_in_fpga(which_uart);
    cgsleep_ms(10);
    nbytes2 = check_how_many_uart_data_in_fpga(which_uart);
    cgsleep_ms(10);
    nbytes3 = check_how_many_uart_data_in_fpga(which_uart);
    cgsleep_ms(10);

    if(nbytes == nbytes2 && nbytes2 == nbytes3)
    {
        if(nbytes > buf_length)
        {
            nbytes = buf_length;
        }


        len = read_uart_data_in_fpga(which_uart, buf, nbytes);
        if(len != nbytes)
        {
            //printf("!!! %s: There are %d bytes in UART, but we just read out %d bytes\n", __FUNCTION__, nbytes, len);
        }
        pthread_mutex_unlock(&uart_receive_mutex[which_uart]);
        return len;

    }
    else
    {
        pthread_mutex_unlock(&uart_receive_mutex[which_uart]);
        return 0;
    }
}


unsigned int clear_uart_send_fifo(unsigned char which_chain)
{
    unsigned int chain_send_fifo_status_addr = 0, chain_send_ready_addr = 0, chain_send_buffer_addr = 0;
    unsigned int max_fpga_can_send_uart_data_len = 0, bit_shift_num = 0, send_counter = 0;

    pthread_mutex_lock(&uart_send_mutex[which_chain]);

    switch(which_chain)
    {
        case 1:
            bit_shift_num = 24;
            chain_send_fifo_status_addr = CHAIN1_3_8_SEND_FIFO_STATUS;
            chain_send_ready_addr       = CHAIN1_SEND_READY;
            chain_send_buffer_addr      = CHAIN1_SEND_BUFFER;
            break;

        case 2:
            bit_shift_num = 16;
            chain_send_fifo_status_addr = CHAIN1_3_8_SEND_FIFO_STATUS;
            chain_send_ready_addr       = CHAIN2_SEND_READY;
            chain_send_buffer_addr      = CHAIN2_SEND_BUFFER;
            break;

        case 3:
            bit_shift_num = 8;
            chain_send_fifo_status_addr = CHAIN1_3_8_SEND_FIFO_STATUS;
            chain_send_ready_addr       = CHAIN3_SEND_READY;
            chain_send_buffer_addr      = CHAIN3_SEND_BUFFER;
            break;

        case 8:
            bit_shift_num = 0;
            chain_send_fifo_status_addr = CHAIN1_3_8_SEND_FIFO_STATUS;
            chain_send_ready_addr       = CHAIN8_SEND_READY;
            chain_send_buffer_addr      = CHAIN8_SEND_BUFFER;
            break;

        case 9:
            bit_shift_num = 24;
            chain_send_fifo_status_addr = CHAIN9_12_SEND_FIFO_STATUS;
            chain_send_ready_addr       = CHAIN9_SEND_READY;
            chain_send_buffer_addr      = CHAIN9_SEND_BUFFER;
            break;

        case 10:
            bit_shift_num = 16;
            chain_send_fifo_status_addr = CHAIN9_12_SEND_FIFO_STATUS;
            chain_send_ready_addr       = CHAIN10_SEND_READY;
            chain_send_buffer_addr      = CHAIN10_SEND_BUFFER;
            break;

        case 11:
            bit_shift_num = 8;
            chain_send_fifo_status_addr = CHAIN9_12_SEND_FIFO_STATUS;
            chain_send_ready_addr       = CHAIN11_SEND_READY;
            chain_send_buffer_addr      = CHAIN11_SEND_BUFFER;
            break;

        case 12:
            bit_shift_num = 0;
            chain_send_fifo_status_addr = CHAIN9_12_SEND_FIFO_STATUS;
            chain_send_ready_addr       = CHAIN12_SEND_READY;
            chain_send_buffer_addr      = CHAIN12_SEND_BUFFER;
            break;

        case 13:
            bit_shift_num = 24;
            chain_send_fifo_status_addr = CHAIN13_14_SEND_FIFO_STATUS;
            chain_send_ready_addr       = CHAIN13_SEND_READY;
            chain_send_buffer_addr      = CHAIN13_SEND_BUFFER;
            break;

        case 14:
            bit_shift_num = 16;
            chain_send_fifo_status_addr = CHAIN13_14_SEND_FIFO_STATUS;
            chain_send_ready_addr       = CHAIN14_SEND_READY;
            chain_send_buffer_addr      = CHAIN14_SEND_BUFFER;
            break;

        default:
            printf("%s: The uart%d is not supported!!!\n", __FUNCTION__, which_chain);
            pthread_mutex_unlock(&uart_send_mutex[which_chain]);
            return 1;
    }

    send_counter = 0;
    while(1)
    {
        max_fpga_can_send_uart_data_len = (read_axi_fpga(chain_send_fifo_status_addr) >> bit_shift_num) & 0x000000ff;
        if(max_fpga_can_send_uart_data_len == 0x000000ff)
        {
            pthread_mutex_unlock(&uart_send_mutex[which_chain]);
            return 0;
        }
        else
        {
            printf("%s: waiting fpga uart%d clear send fifo space ...\n", __FUNCTION__, which_chain);
            usleep(3*1000);
            send_counter++;
            if(send_counter > 20)
            {
                printf("%s: uart%d always dose not has enough send fifo space, break\n", __FUNCTION__, which_chain);
                pthread_mutex_unlock(&uart_send_mutex[which_chain]);
                return max_fpga_can_send_uart_data_len;
            }
        }
    }
}

unsigned int clear_uart_rx_fifo(unsigned char which_chain)
{
    unsigned int len=0, nbytes=0;
    unsigned char *buf = NULL;

    //printf("\n--- %s\n", __FUNCTION__);

    pthread_mutex_lock(&uart_receive_mutex[which_chain]);

    nbytes = check_how_many_uart_data_in_fpga(which_chain);
    if(nbytes == 0)
    {
        pthread_mutex_unlock(&uart_receive_mutex[which_chain]);
        return 0;
    }

    buf = malloc(nbytes);
    if(!buf)
    {
        printf("%s: uart%d malloc buffer error\n", __FUNCTION__, which_chain);
        buf = NULL;
        usleep(500*1000);
        pthread_mutex_unlock(&uart_receive_mutex[which_chain]);
        return 1;
    }
    else
    {
        len = read_uart_data_in_fpga(which_chain, buf, nbytes);
        if(len != nbytes)
        {
            printf("%s: uart%d clear rx fifo error. nbytes = %d, len = %d\n", __FUNCTION__, which_chain, nbytes, len);
            free(buf);
            buf = NULL;
            usleep(10*1000);
            pthread_mutex_unlock(&uart_receive_mutex[which_chain]);
            return 1;
        }

        free(buf);
        buf = NULL;
        usleep(10*1000);
        pthread_mutex_unlock(&uart_receive_mutex[which_chain]);
        return 1;
    }
}


static void hexdump(const uint8_t *p, unsigned int len)
{
    unsigned int i, addr;
    unsigned int wordlen = sizeof(unsigned int);
    unsigned char v, line[BYTES_PER_LINE * 5];

    for (addr = 0; addr < len; addr += BYTES_PER_LINE)
    {
        /* clear line */
        for (i = 0; i < sizeof(line); i++)
        {
            if (i == wordlen * 2 + 52 ||
                i == wordlen * 2 + 69)
            {
                line[i] = '|';
                continue;
            }

            if (i == wordlen * 2 + 70)
            {
                line[i] = '\0';
                continue;
            }

            line[i] = ' ';
        }

        /* print address */
        for (i = 0; i < wordlen * 2; i++)
        {
            v = addr >> ((wordlen * 2 - i - 1) * 4);
            line[i] = nibble[v & 0xf];
        }

        /* dump content */
        for (i = 0; i < BYTES_PER_LINE; i++)
        {
            int pos = (wordlen * 2) + 3 + (i / 8);

            if (addr + i >= len)
                break;

            v = p[addr + i];
            line[pos + (i * 3) + 0] = nibble[v >> 4];
            line[pos + (i * 3) + 1] = nibble[v & 0xf];

            /* character printable? */
            line[(wordlen * 2) + 53 + i] =
                (v >= ' ' && v <= '~') ? v : '.';
        }

        hex_print(line);
    }
}
unsigned char CRC5(unsigned char *ptr, unsigned char len)
{
    unsigned char i, j, k;
    unsigned char crc = 0x1f;

    unsigned char crcin[5] = {1, 1, 1, 1, 1};
    unsigned char crcout[5] = {1, 1, 1, 1, 1};
    unsigned char din = 0;

    j = 0x80;
    k = 0;
    for (i = 0; i < len; i++)
    {
        if (*ptr & j)
        {
            din = 1;
        }
        else
        {
            din = 0;
        }
        crcout[0] = crcin[4] ^ din;
        crcout[1] = crcin[0];
        crcout[2] = crcin[1] ^ crcin[4] ^ din;
        crcout[3] = crcin[2];
        crcout[4] = crcin[3];

        j = j >> 1;
        k++;
        if (k == 8)
        {
            j = 0x80;
            k = 0;
            ptr++;
        }
        memcpy(crcin, crcout, 5);
    }
    crc = 0;
    if(crcin[4])
    {
        crc |= 0x10;
    }
    if(crcin[3])
    {
        crc |= 0x08;
    }
    if(crcin[2])
    {
        crc |= 0x04;
    }
    if(crcin[1])
    {
        crc |= 0x02;
    }
    if(crcin[0])
    {
        crc |= 0x01;
    }
    return crc;
}

/** CRC table for the CRC ITU-T V.41 0x0x1021 (x^16 + x^12 + x^5 + 1) */
const uint16_t crc_itu_t_table[256] =
{
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
    0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
    0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
    0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
    0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
    0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
    0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
    0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
    0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
    0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
    0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
    0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
    0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
    0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
    0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
    0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
    0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
    0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
    0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
    0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
    0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
    0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
    0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
};


static inline uint16_t crc_itu_t_byte(uint16_t crc, const uint8_t data)
{
    return (crc << 8) ^ crc_itu_t_table[((crc >> 8) ^ data) & 0xff];
}


unsigned short CRC16(unsigned short crc, unsigned char *buffer, int len)
{
    while (len--)
        crc = crc_itu_t_byte(crc, *buffer++);
    return crc;
}


uint16_t crc_itu_t(uint16_t crc, const uint8_t *buffer, int len)
{
    while (len--)
        crc = crc_itu_t_byte(crc, *buffer++);
    return crc;
}

/****************** global functions end ******************/


void init_asic_display_status()
{
    unsigned char which_chain, which_asic, offset;

    for(which_chain=0; which_chain < BITMAIN_MAX_CHAIN_NUM; which_chain++)
    {
        if(dev.chain_exist[which_chain])
        {
            offset = 0;
            for(which_asic = 0; which_asic < dev.chain_asic_num[which_chain]; which_asic++)
            {
                if(which_asic % 8 == 0)
                {
                    if ( ( which_asic + offset ) > ( ASIC_NUM_EACH_CHAIN + 16 ))
                    {
                        applog(LOG_ERR, "offset[%d] ERR", (which_asic + offset));
                    }
                    dev.chain_asic_status_string[which_chain][which_asic + offset] = ' ';
                    offset++;
                }

                if ( ( which_asic + offset ) > ( ASIC_NUM_EACH_CHAIN + 16 ))
                {
                    applog(LOG_ERR, "offset[%d] ERR", (which_asic + offset));
                }
                dev.chain_asic_status_string[which_chain][which_asic + offset] = 'o';
                dev.chain_asic_nonce[which_chain][which_asic] = 0;
            }

            if ( ( which_asic + offset ) > ( ASIC_NUM_EACH_CHAIN + 16 ))
            {
                applog(LOG_ERR, "offset[%d] ERR", (which_asic + offset));
            }
            dev.chain_asic_status_string[which_chain][which_asic + offset] = '\0';
        }
    }
}


void tty_init_chain(uint8_t which_chain, struct bitmain_B3_info *info)
{
    if(dev.chain_exist[which_chain])
    {
        int ret;
        applog(LOG_NOTICE, "%s",__FUNCTION__);

        dev_info[which_chain].chainid = which_chain;
        applog(LOG_NOTICE, "%s chainid = %d",__FUNCTION__,dev_info[which_chain].chainid);

        start_recv[which_chain] = true;
        ret = thr_info_create(&info->uart_rx_t[which_chain], NULL, get_asic_response, (void *)&dev_info[which_chain]);
        if(unlikely(ret != 0))
        {
            applog(LOG_ERR,"create rx read thread for chain %d failed", which_chain);
        }
        else
        {
            applog(LOG_ERR,"create rx read thread for chain %d ok", which_chain);
        }

        cgsleep_ms(50);
        struct bitmian_B3_info_with_index info_with_index;
        info_with_index.info = info;
        info_with_index.chain_index = which_chain;
        ret = thr_info_create(&info->uart_tx_t[which_chain], NULL, B3_fill_work, (void *)(&info_with_index));
        cgsleep_ms(200);
        if(unlikely(ret != 0))
        {
            applog(LOG_ERR,"create tx read thread for chain %d failed",which_chain);
        }
        else
        {
            applog(LOG_ERR,"create tx read thread for chain %d ok",which_chain);
        }

        applog(LOG_NOTICE,"open device over");


        cgsleep_ms(10);
    }
}


void tty_init(struct bitmain_B3_info *info)
{
    uint8_t which_chain = 0,ret;

    applog(LOG_NOTICE, "%s",__FUNCTION__);

    for(which_chain = 0; which_chain < BITMAIN_MAX_CHAIN_NUM; which_chain++)
    {
        tty_init_chain(which_chain, info);
    }
    cgsleep_ms(10);
}



int B3_write(int fd, uint8_t *buf, size_t bufLen)
{
    uart_send(fd,buf,bufLen);
}


int B3_read(int uart_fd, unsigned char *buf, size_t MAX_READ_BYTES)
{
    return uart_receive(uart_fd,buf,MAX_READ_BYTES);
}

void check_chain()
{
    unsigned char which_chain=0xff;
    int ret = 0;
    int other_chain = 0;

    applog(LOG_NOTICE,"%s", __FUNCTION__);

    dev.chain_num = 0;

    ret = read_axi_fpga(HASH_ON_PLUG);

    for(which_chain=0; which_chain < 4; which_chain++)
    {
        switch(which_chain)
        {
            case 1:
                other_chain = 8;
                break;
            case 2:
                other_chain = 10;
                break;
            case 3:
                other_chain = 12;
                break;
        }
        if((ret >> which_chain) & 0x1)
        {
            dev.chain_exist[which_chain] = 1;
            dev.chain_exist[other_chain] = 1;
            dev.chain_asic_num[which_chain] = 2;
            dev.chain_asic_num[other_chain] = 2;
            dev.chain_num += 2;
            applog(LOG_NOTICE, "Chain %d existed!", which_chain, other_chain);
        }
        else
        {
            dev.chain_exist[which_chain] = 0;
            dev.chain_exist[other_chain] = 0;
        }
    }
}


static void reset_hash_board_low(unsigned char which_chain)
{
    applog(LOG_NOTICE, "%s %d", __FUNCTION__, which_chain);
    write_axi_fpga(RESET_HASH_BOARD, 0x0 | (1 << which_chain));

}

static void reset_hash_board_high(unsigned char which_chain)
{
    applog(LOG_NOTICE, "%s %d", __FUNCTION__, which_chain);
    write_axi_fpga(RESET_HASH_BOARD, 0x0);
}

static void reset_all_hash_board_low(void)
{
    applog(LOG_NOTICE, "%s %d", __FUNCTION__);
    write_axi_fpga(RESET_HASH_BOARD, 0xffff);
}

static void reset_all_hash_board_high(void)
{
    applog(LOG_NOTICE, "%s %d", __FUNCTION__);
    write_axi_fpga(RESET_HASH_BOARD, 0x0);
}

#define RED_LED_DEV_XILINX "/sys/class/gpio/gpio37/value"
#define GREEN_LED_DEV_XILINX "/sys/class/gpio/gpio38/value"

void set_led(bool stop)
{
    static bool blink = true;
    char cmd[100];
    blink = !blink;

    if(stop)
    {
        sprintf(cmd,"echo %d > %s", 0,GREEN_LED_DEV_XILINX);
        system(cmd);
        sprintf(cmd,"echo %d > %s", (blink)?1:0,RED_LED_DEV_XILINX);
        system(cmd);
    }
    else
    {
        sprintf(cmd,"echo %d > %s", 0,RED_LED_DEV_XILINX);
        system(cmd);
        sprintf(cmd,"echo %d > %s", (blink)?1:0,GREEN_LED_DEV_XILINX);
        system(cmd);
    }
}

static unsigned int getNum(const char* buffer)
{
    char* pos = strstr(buffer, ":");

    while(*(++pos) == ' ');
    char* startPos = pos;

    while(*(++pos) != ' ');
    *pos = '\0';

    return (atoi(startPos));
}

int get_fan_speed(unsigned char *fan_id, unsigned int *fan_speed)
{
    int ret = -1;
    ret = read_axi_fpga(FAN_SPEED);
    *fan_speed = 0x000000ff & ret;
    *fan_id = (unsigned char)(0x00000007 & (ret >> 8));
    //applog(LOG_NOTICE,"%s:%x %x %x %x %x", __FUNCTION__,axi_fpga_addr, FAN_SPEED, ret, *fan_speed, *fan_id);
    return ret;
}

void check_fan_speed(void)
{
    int i=0, j=0;
    unsigned char fan_id = 0;
    unsigned int fan_speed;

    dev.fan_speed_top1 = 0;
    dev.fan_speed_low1 = 0;
    for(j=0; j < 2; j++)    //means check for twice to make sure find out all fan
    {
        for(i=0; i < BITMAIN_MAX_FAN_NUM; i++)
        {
            fan_speed = 0;
            fan_id = 0;
            if(get_fan_speed(&fan_id, &fan_speed) != -1)
            {
                dev.fan_speed_value[fan_id] = fan_speed * 60 * 2;
                if((fan_speed > 0) && (dev.fan_exist[fan_id] == 0))
                {
                    dev.fan_exist[fan_id] = 1;
                    dev.fan_num++;
                    dev.fan_exist_map |= (0x1 << fan_id);
                }
                else if((fan_speed == 0) && (dev.fan_exist[fan_id] == 1))
                {
                    dev.fan_exist[fan_id] = 0;
                    dev.fan_num--;
                    dev.fan_exist_map &= !(0x1 << fan_id);
                }

                if(dev.fan_speed_top1 < dev.fan_speed_value[fan_id])
                    dev.fan_speed_top1 = dev.fan_speed_value[fan_id];
                if((dev.fan_speed_low1 > dev.fan_speed_value[fan_id] && dev.fan_speed_value[fan_id] != 0) || dev.fan_speed_low1 == 0)
                    dev.fan_speed_low1 = dev.fan_speed_value[fan_id];
            }
            cgsleep_ms(50);
        }
    }
}

void set_PWM(unsigned char pwm_percent)
{
    int temp_pwm_percent = 0;
    temp_pwm_percent = pwm_percent;
    if(temp_pwm_percent < MIN_PWM_PERCENT)
    {
        temp_pwm_percent = MIN_PWM_PERCENT;
    }

    if(temp_pwm_percent > MAX_PWM_PERCENT || gMinerStatus_Not_read_all_sensor)
    {
        temp_pwm_percent = MAX_PWM_PERCENT;
    }
    dev.pwm_percent = temp_pwm_percent;
    //temp_pwm_percent = 100;

    uint32_t speed_t = (temp_pwm_percent << 16) | (100 - temp_pwm_percent);
    write_axi_fpga(FAN_CONTROL, speed_t);
}

void set_PWM_according_to_temperature(void)
{
    int pwm_percent = 0, temp_change = 0;

    temp_highest = dev.temp_top1;
    if(temp_highest >= MAX_FAN_TEMP)
    {
        //applog(LOG_DEBUG,"%s: Temperature is higher than %d 'C", __FUNCTION__, temp_highest);
    }

    if(dev.fan_eft)
    {
        if((dev.fan_pwm >= 0) && (dev.fan_pwm <= 100))
        {
            set_PWM(dev.fan_pwm);
            return;
        }
    }

    temp_change = temp_highest - last_temperature;

    if(temp_highest >= MAX_FAN_TEMP || temp_highest == 0)
    {
        set_PWM(MAX_PWM_PERCENT);
        dev.fan_pwm = MAX_PWM_PERCENT;
        //applog(LOG_DEBUG,"%s: Set PWM percent : MAX_PWM_PERCENT", __FUNCTION__);
        return;
    }

    if(temp_highest <= MIN_FAN_TEMP)
    {
        set_PWM(MIN_PWM_PERCENT);
        dev.fan_pwm = MIN_PWM_PERCENT;
        //applog(LOG_DEBUG,"%s: Set PWM percent : MIN_PWM_PERCENT", __FUNCTION__);
        return;
    }

    if(temp_change >= TEMP_INTERVAL || temp_change <= -TEMP_INTERVAL)
    {
        pwm_percent = MIN_PWM_PERCENT + (temp_highest -MIN_FAN_TEMP) * PWM_ADJUST_FACTOR;
        if(pwm_percent < 0)
        {
            pwm_percent = 0;
        }
        dev.fan_pwm = pwm_percent;
        applog(LOG_NOTICE,"%s: Set PWM percent : %d  temp %d", __FUNCTION__, pwm_percent,temp_highest);
        set_PWM(pwm_percent);
        last_temperature = temp_highest;
    }
}

void reset_chain(struct bitmain_B3_info *info, uint8_t chain)
{


}
void recheck_asic_num(struct bitmain_B3_info *info, uint8_t chain)
{
    pthread_mutex_lock(&reinit_mutex);
    reset_chain(info, chain);
    pthread_mutex_unlock(&reinit_mutex);

    clear_register_value_buf();
    cgsleep_ms(100);

    cgsleep_ms(200);
    applog(LOG_NOTICE,"%s DONE!", __FUNCTION__);
}
int bitmain_B3_init(struct bitmain_B3_info *info)
{
    uint16_t crc = 0, freq = 0;
    unsigned char which_chain = 0;
    int i = 0,check_asic_times = 0, ret = 0;
    bool check_asic_fail = false;

    applog(LOG_WARNING, "%s", __FUNCTION__);

    sprintf(g_miner_version, "A.0.0.1");
    dev.addrInterval = 1;

    // init the work queue which stores the latest work that sent to hash boards
    for(i=0; i < BITMAIN_MAX_QUEUE_NUM; i++)
    {
        info->work_queue[i] = NULL;
    }

    tty_init(info);

    cgsleep_ms(100);
    clear_register_value_buf();
    cgsleep_ms(100);
    sleep(10);

    for(which_chain=0; which_chain<BITMAIN_MAX_CHAIN_NUM; which_chain++)
    {
        if(dev.chain_exist[which_chain] && send_heart)
        {
            BM1680_send_init(which_chain,0);
            BM1680_send_init(which_chain,1);
        }
    }
    sleep(10);

#ifdef UPGRADE
    applog(LOG_WARNING, "%s %d: burning flash", __FUNCTION__, __LINE__);
    for(which_chain=0; which_chain<BITMAIN_MAX_CHAIN_NUM; which_chain++)
    {
        if(dev.chain_exist[which_chain] && send_heart)
        {
            BM1680_upgrade(which_chain,0);
            cgsleep_ms(10);
            BM1680_upgrade(which_chain,1);
            cgsleep_ms(10);
        }
    }
    cgsleep_ms(500);
    write_axi_fpga(SOCKET_ID, 0);
    reset_all_hash_board_low();
    cgsleep_ms(500);
    write_axi_fpga(SOCKET_ID, 7);
    cgsleep_ms(500);
    reset_all_hash_board_high();
    cgsleep_ms(100);

    cgsleep_ms(100);
    clear_register_value_buf();
    cgsleep_ms(100);


    for(which_chain=0; which_chain<BITMAIN_MAX_CHAIN_NUM; which_chain++)
    {
        if(dev.chain_exist[which_chain] && send_heart)
        {
            BM1680_send_init(which_chain,0);
            BM1680_send_init(which_chain,1);
        }
    }
    sleep(10);

#endif

    for(which_chain=0; which_chain<BITMAIN_MAX_CHAIN_NUM; which_chain++)
    {
        if(dev.chain_exist[which_chain] && send_heart)
        {
            BM1680_set_nonce_diff(which_chain, 0, TM);
            BM1680_set_nonce_diff(which_chain, 1, TM);
        }
    }

    for(which_chain=0; which_chain<BITMAIN_MAX_CHAIN_NUM; which_chain++)
    {
        if(dev.chain_exist[which_chain] && send_heart)
        {
            BM1680_set_nonce_interval(which_chain, 0, 0, 0xffffffff);
            BM1680_set_nonce_interval(which_chain, 1, 0, 0xffffffff);
        }
    }

    dev.fan_eft = opt_bitmain_fan_ctrl;
    dev.fan_pwm = opt_bitmain_fan_pwm;
    applog(LOG_NOTICE,"%s: fan_eft : %d  fan_pwm : %d", __FUNCTION__, dev.fan_eft, dev.fan_pwm);
    if(dev.fan_eft)
    {
        if((dev.fan_pwm >= 0) && (dev.fan_pwm <= 100))
        {
            set_PWM(dev.fan_pwm);
        }
        else
        {
            set_PWM_according_to_temperature();
        }
    }
    else
    {
        set_PWM_according_to_temperature();
    }

    // create some pthread
    ret = create_bitmain_check_miner_status_pthread(info);
    if(ret == -5)
    {
        return ret;
    }

    ret = create_bitmain_get_hash_rate_pthread();
    if(ret == -6)
    {
        return ret;
    }

    ret = create_bitmain_read_temp_pthread();
    if(ret == -7)
    {
        return ret;
    }

    ret = create_bitmain_check_fan_pthread();
    if(ret == -8)
    {
        return ret;
    }

    ret = create_bitmain_check_status_pthread();
    if(ret == -9)
    {
        return ret;
    }
    cgsleep_ms(FANINT * 1000);
    applog(LOG_NOTICE,"INIT DONE!");
    // init ASIC status which will be display on the web
    init_asic_display_status();

    for(i=0; i < BITMAIN_MAX_CHAIN_NUM; i++)
        start_send[i] = true;
    return 0;
}

void* bitmain_B3_reinit_chain(void * usrdata)
{
    pthread_detach(pthread_self());
    pthread_mutex_lock(&reinit_mutex);
    struct bitmian_B3_info_with_index *info_with_chain = (struct bitmian_B3_info_with_index *)usrdata;
    struct bitmain_B3_info *info = info_with_chain->info;
    uint8_t chain = info_with_chain->chain_index;

    start_send[chain] = false;
    start_recv[chain] = false;
    reiniting[chain] = true;

    thr_info_join(&info->uart_tx_t[chain]);
    thr_info_join(&info->uart_rx_t[chain]);
    reset_hash_board_low(chain);
    cgsleep_ms(100);
    reset_hash_board_high(chain);
    tty_init_chain(chain, info);

    BM1680_send_init(chain,0);
    BM1680_send_init(chain,1);

	sleep(10);

    BM1680_set_nonce_diff(chain, 0, TM);
    BM1680_set_nonce_diff(chain, 1, TM);

    BM1680_set_nonce_interval(chain, 0, 0, 0xffffffff);
    BM1680_set_nonce_interval(chain, 1, 0, 0xffffffff);


    reiniting[chain] = true;
    start_send[chain] = true;
	pthread_mutex_unlock(&reinit_mutex);

}

/****************** about control board & Hash Board end ******************/

#define TEMPERATURE_SENSOR
/****************** about temperature sensor ******************/


#define ABOUT_OTHER_FUNCTIONS
/****************** about other functions ******************/

void suffix_string_B3(uint64_t val, char *buf, size_t bufsiz, int sigdigits,bool display)
{
    const double  dkilo = 1000.0;
    const uint64_t kilo = 1000ull;
    const uint64_t mega = 1000000ull;
    const uint64_t giga = 1000000000ull;
    char suffix[2] = "";
    bool decimal = true;
    double dval;
    /*
        if (val >= exa)
        {
            val /= peta;
            dval = (double)val / dkilo;
            strcpy(suffix, "E");
        }
        else if (val >= peta)
        {
            val /= tera;
            dval = (double)val / dkilo;
            strcpy(suffix, "P");
        }
        else if (val >= tera)
        {
            val /= giga;
            dval = (double)val / dkilo;
            strcpy(suffix, "T");
        }
        else
    if (val >= giga)
    {
        val /= mega;
        dval = (double)val / dkilo;
        strcpy(suffix, "G");
    }
    else if (val >= mega)
    {
        val /= kilo;
        dval = (double)val / dkilo;
        strcpy(suffix, "M");
    }
    else if (val >= kilo)
    {
        dval = (double)val / dkilo;
        strcpy(suffix, "K");
    }
    else*/
    {
        dval = val;
        decimal = false;
    }

    if (!sigdigits)
    {
        if (decimal)
            snprintf(buf, bufsiz, "%.3g%s", dval, suffix);
        else
            snprintf(buf, bufsiz, "%d%s", (unsigned int)dval, suffix);
    }
    else
    {
        /* Always show sigdigits + 1, padded on right with zeroes
         * followed by suffix */
        int ndigits = sigdigits - 1 - (dval > 0.0 ? floor(log10(dval)) : 0);
        if(display)
            snprintf(buf, bufsiz, "%*.*f%s", sigdigits + 1, ndigits, dval, suffix);
        else
            snprintf(buf, bufsiz, "%*.*f", sigdigits + 1, ndigits, dval);

    }
}

void clear_register_value_buf(void)
{
    pthread_mutex_lock(&reg_mutex);
    reg_fifo.p_wr = 0;
    reg_fifo.p_rd = 0;
    reg_fifo.reg_value_num = 0;
    pthread_mutex_unlock(&reg_mutex);
}

void clear_nonce_fifo()
{
    int i = 0;
    pthread_mutex_lock(&nonce_mutex);
    nonce_fifo.p_wr = 0;
    nonce_fifo.p_rd = 0;
    nonce_fifo.nonce_num = 0;
    for(i=0; i < BITMAIN_MAX_CHAIN_NUM; i++)
        clear_uart_rx_fifo(i);
    pthread_mutex_unlock(&nonce_mutex);
}

static inline void my_be32enc(void *pp, uint32_t x)
{
    uint8_t *p = (uint8_t *)pp;
    p[3] = x & 0xff;
    p[2] = (x >> 8) & 0xff;
    p[1] = (x >> 16) & 0xff;
    p[0] = (x >> 24) & 0xff;
}

/**************** about BM1680 functions ****************/

void BM1680_send_init(uint8_t which_uart, uint8_t which_BM1680)
{
    uint8_t send_buf[BM1680_INIT_CMD_CMD_LEN] = {BM1680_SEND_CMD_HEADER1, BM1680_SEND_CMD_HEADER2, 0, BM1680_INIT_CMD, 0x0};
    uint32_t crc = 0, i = 0;

    send_buf[BM1680_CHIP_ADDRESS_ADDR] = which_BM1680;

    for(i=0; i<BM1680_INIT_CMD_CMD_LEN - 1; i++)
    {
        crc += send_buf[i];
    }
    send_buf[BM1680_INIT_CMD_CMD_LEN - 1] = (uint8_t)crc;
#if 0
    char * seed_hex = NULL;
    seed_hex = bin2hex(send_buf,sizeof(send_buf));
    applog(LOG_NOTICE,"Send init to chain %d asic %d  %s",which_uart,which_BM1680,seed_hex);
    free(seed_hex);
#endif
    uart_send(which_uart, send_buf, BM1680_INIT_CMD_CMD_LEN);
}

void BM1680_set_nonce_diff(uint8_t which_uart, uint8_t which_BM1680, uint8_t nonce_diff)
{
    uint8_t send_buf[BM1680_SET_NONCE_DIFF_CMD_LEN] = {BM1680_SEND_CMD_HEADER1, BM1680_SEND_CMD_HEADER2, 0, BM1680_SET_NONCE_DIFF, 0x0, 0x0};
    uint32_t crc = 0, i = 0;

    send_buf[BM1680_CHIP_ADDRESS_ADDR] = which_BM1680;
    send_buf[4] = nonce_diff;

    for(i=0; i<BM1680_SET_NONCE_DIFF_CMD_LEN - 1; i++)
    {
        crc += send_buf[i];
    }
    send_buf[BM1680_SET_NONCE_DIFF_CMD_LEN - 1] = (uint8_t)crc;

    uart_send(which_uart, send_buf, BM1680_SET_NONCE_DIFF_CMD_LEN);
}

void BM1680_set_seed(uint8_t which_uart, uint8_t which_BM1680, uint8_t *seed)
{
#if 1
    char * seed_hex = NULL;
    seed_hex = bin2hex(seed,32);
    applog(LOG_NOTICE,"Send seed to chain %d asic %d  %s",which_uart,which_BM1680,seed_hex);
    free(seed_hex);
#endif
    uint8_t send_buf[BM1680_SET_SEED_CMD_LEN] = {BM1680_SEND_CMD_HEADER1, BM1680_SEND_CMD_HEADER2, 0, BM1680_SET_SEED};
    uint32_t crc = 0, i = 0;

    send_buf[2] = which_BM1680;
    send_buf[BM1680_SET_SEED_CMD_LEN - 1] = 0;

    memcpy(send_buf + 4, seed, BM1680_SET_SEED_DATA_LEN);

    for(i=0; i<BM1680_SET_SEED_CMD_LEN - 1; i++)
    {
        crc += send_buf[i];
    }
    send_buf[BM1680_SET_SEED_CMD_LEN - 1] = (uint8_t)crc;

    uart_send(which_uart, send_buf, BM1680_SET_SEED_CMD_LEN);
}

void update_seed_and_wait_ok(uint8_t which_uart, uint8_t *seed)
{
    BM1680_set_seed(which_uart, 0, seed);
    BM1680_set_seed(which_uart, 1, seed);
    sleep(15);
}

void BM1680_set_message(uint8_t which_uart, uint8_t which_BM1680, uint8_t *message)
{
#if 1
    char * message_hex = NULL;
    message_hex = bin2hex(message,137);
    applog(LOG_NOTICE,"Send message to chain %d asic %d  %s",which_uart,which_BM1680,message_hex);
    free(message_hex);
#endif

    uint8_t send_buf[BM1680_SET_MESSAGE_CMD_LEN] = {BM1680_SEND_CMD_HEADER1, BM1680_SEND_CMD_HEADER2, 0, BM1680_SET_MESSAGE};
    uint32_t crc = 0, i = 0;

    send_buf[2] = which_BM1680;
    send_buf[BM1680_SET_MESSAGE_CMD_LEN - 1] = 0;

    memcpy(send_buf + 4, message, BM1680_SET_MESSAGE_DATA_LEN);

    for(i=0; i<BM1680_SET_MESSAGE_CMD_LEN - 1; i++)
    {
        crc += send_buf[i];
    }
    send_buf[BM1680_SET_MESSAGE_CMD_LEN - 1] = (uint8_t)crc;

    uart_send(which_uart, send_buf, BM1680_SET_MESSAGE_CMD_LEN);
}

void BM1680_check_status(uint8_t which_uart, uint8_t which_BM1680)
{
    uint8_t send_buf[BM1680_CHECK_STATUS_CMD_LEN] = {BM1680_SEND_CMD_HEADER1, BM1680_SEND_CMD_HEADER2, 0, BM1680_CHECK_STATUS, 0x0};
    uint32_t crc = 0, i = 0;

    send_buf[2] = which_BM1680;
    for(i=0; i<BM1680_CHECK_STATUS_CMD_LEN - 1; i++)
    {
        crc += send_buf[i];
    }
    send_buf[BM1680_CHECK_STATUS_CMD_LEN - 1] = (uint8_t)crc;

    uart_send(which_uart, send_buf, BM1680_CHECK_STATUS_CMD_LEN);

    #if 1
    char * status_hex = NULL;
    status_hex = bin2hex(send_buf, BM1680_CHECK_STATUS_CMD_LEN);
    applog(LOG_NOTICE, "Send status to chain %d asic %d  %s",which_uart, which_BM1680, status_hex);
    free(status_hex);
    #endif
}

void BM1680_set_nonce_interval(uint8_t which_uart, uint8_t which_BM1680, uint64_t start_nonce, uint64_t end_nonce)
{
    uint8_t send_buf[BM1680_SET_NONCE_INTERVAL_CMD_LEN] = {BM1680_SEND_CMD_HEADER1, BM1680_SEND_CMD_HEADER2, 0, BM1680_SET_NONCE_INTERVAL};
    uint32_t crc = 0, i = 0;

    send_buf[2] = which_BM1680;
    send_buf[BM1680_SET_NONCE_INTERVAL_CMD_LEN - 1] = 0;

    memcpy(send_buf + 4, &start_nonce, sizeof(uint64_t));
    memcpy(send_buf + 4 + sizeof(uint64_t), &end_nonce, sizeof(uint64_t));

    for(i=0; i<BM1680_SET_NONCE_INTERVAL_CMD_LEN - 1; i++)
    {
        crc += send_buf[i];
    }
    send_buf[BM1680_SET_NONCE_INTERVAL_CMD_LEN - 1] = (uint8_t)crc;

    uart_send(which_uart, send_buf, BM1680_SET_NONCE_INTERVAL_CMD_LEN);
}

void BM1680_get_board_temperature(uint8_t which_uart, uint8_t which_BM1680)
{
    uint8_t send_buf[BM1680_GET_BOARD_TEMPERATURE_CMD_LEN] = {BM1680_SEND_CMD_HEADER1, BM1680_SEND_CMD_HEADER2, 0, BM1680_GET_BOARD_TEMPERATURE, 0x0};
    uint32_t crc = 0, i = 0;

    send_buf[2] = which_BM1680;

    for(i=0; i<BM1680_GET_BOARD_TEMPERATURE_CMD_LEN - 1; i++)
    {
        crc += send_buf[i];
    }
    send_buf[BM1680_GET_BOARD_TEMPERATURE_CMD_LEN - 1] = (uint8_t)crc;

#if 1
    char * message_hex = NULL;
    message_hex = bin2hex(send_buf,sizeof(send_buf));
    applog(LOG_NOTICE,"Get PCB temp from chain %d asic %d  %s",which_uart,which_BM1680,message_hex);
    free(message_hex);
#endif

    uart_send(which_uart, send_buf, BM1680_GET_BOARD_TEMPERATURE_CMD_LEN);
}

void BM1680_get_asic_temperature(uint8_t which_uart, uint8_t which_BM1680)
{
    uint8_t send_buf[BM1680_GET_ASIC_TEMPERATURE_CMD_LEN] = {BM1680_SEND_CMD_HEADER1, BM1680_SEND_CMD_HEADER2, 0, BM1680_GET_ASIC_TEMPERATURE, 0x0};
    uint32_t crc = 0, i = 0;

    send_buf[2] = which_BM1680;

    for(i=0; i<BM1680_GET_ASIC_TEMPERATURE_CMD_LEN - 1; i++)
    {
        crc += send_buf[i];
    }
    send_buf[BM1680_GET_ASIC_TEMPERATURE_CMD_LEN - 1] = (uint8_t)crc;
#if 1
    char * message_hex = NULL;
    message_hex = bin2hex(send_buf,sizeof(send_buf));
    applog(LOG_NOTICE,"Get CHIP temp from chain %d asic %d  %s",which_uart,which_BM1680,message_hex);
    free(message_hex);
#endif


    uart_send(which_uart, send_buf, BM1680_GET_ASIC_TEMPERATURE_CMD_LEN);
}

void BM1680_upgrade_bin_file_without_ending(unsigned char which_uart, unsigned char which_BM1680, unsigned char *data)
{
    unsigned char send_buf[BM1680_UPGRADE_BIN_FILE_WITHOUT_ENDING_CMD_LEN] = {BM1680_SEND_CMD_HEADER1, BM1680_SEND_CMD_HEADER2, 0, BM1680_UPGRADE_BIN_FILE_WITHOUT_ENDING};
    unsigned char *receive_buf = NULL;
    unsigned int crc = 0, receive_data_len = 0, i = 0;
    int receive_data_header_pointer = 0, ret = 0;

    send_buf[2] = which_BM1680;
    send_buf[BM1680_UPGRADE_BIN_FILE_WITHOUT_ENDING_CMD_LEN - 1] = 0;

    for(i=0; i<BM1680_UPGRADE_BIN_FILE_WITHOUT_ENDING_DATA_LEN; i++)
    {
        send_buf[4 + i] = data[i];
    }

    for(i=0; i<BM1680_UPGRADE_BIN_FILE_WITHOUT_ENDING_CMD_LEN - 1; i++)
    {
        crc += send_buf[i];
    }
    send_buf[BM1680_UPGRADE_BIN_FILE_WITHOUT_ENDING_CMD_LEN - 1] = (unsigned char)crc;

    cgsleep_ms(5);
    uart_send(which_uart, send_buf, BM1680_UPGRADE_BIN_FILE_WITHOUT_ENDING_CMD_LEN/2);
    cgsleep_ms(5);
    uart_send(which_uart, send_buf + BM1680_UPGRADE_BIN_FILE_WITHOUT_ENDING_CMD_LEN/2, BM1680_UPGRADE_BIN_FILE_WITHOUT_ENDING_CMD_LEN - BM1680_UPGRADE_BIN_FILE_WITHOUT_ENDING_CMD_LEN/2);
}

void BM1680_upgrade_bin_file_with_ending(unsigned char which_uart, unsigned char which_BM1680, unsigned char *data, unsigned int data_len)
{
    unsigned char send_buf[BM1680_UPGRADE_BIN_FILE_WITH_ENDING_CMD_LEN] = {BM1680_SEND_CMD_HEADER1, BM1680_SEND_CMD_HEADER2, 0, BM1680_UPGRADE_BIN_FILE_WITH_ENDING};
    unsigned char *receive_buf = NULL;
    unsigned int crc = 0, receive_data_len = 0, i = 0;
    int receive_data_header_pointer = 0, ret = 0;

    // step 1: prepare data for sending
    memset(send_buf, 0, BM1680_UPGRADE_BIN_FILE_WITH_ENDING_CMD_LEN);
    send_buf[0] = BM1680_SEND_CMD_HEADER1;
    send_buf[1] = BM1680_SEND_CMD_HEADER2;
    send_buf[2] = which_BM1680;
    send_buf[3] = BM1680_UPGRADE_BIN_FILE_WITH_ENDING;
    send_buf[BM1680_UPGRADE_BIN_FILE_WITH_ENDING_CMD_LEN - 1] = 0;

    for(i=0; i<data_len; i++)
    {
        send_buf[4 + i] = data[i];
    }

    for(i=0; i<BM1680_UPGRADE_BIN_FILE_WITH_ENDING_CMD_LEN - 1; i++)
    {
        crc += send_buf[i];
    }
    send_buf[BM1680_UPGRADE_BIN_FILE_WITH_ENDING_CMD_LEN - 1] = (unsigned char)crc;

    cgsleep_ms(5);
    uart_send(which_uart, send_buf, BM1680_UPGRADE_BIN_FILE_WITH_ENDING_CMD_LEN/2);
    cgsleep_ms(5);
    uart_send(which_uart, send_buf + BM1680_UPGRADE_BIN_FILE_WITH_ENDING_CMD_LEN/2, BM1680_UPGRADE_BIN_FILE_WITH_ENDING_CMD_LEN - BM1680_UPGRADE_BIN_FILE_WITH_ENDING_CMD_LEN/2);

}


void BM1680_upgrade(unsigned char which_uart, unsigned char which_BM1680)
{
    unsigned int i=0, error_counter = 0;
    unsigned int upgrade_file_size = 0;
    int ret = 0;
    unsigned char *upgrade_file_buf = NULL;
    FILE *upgrade_file;
    struct stat statbuf;
    applog(LOG_NOTICE,"Update Chain %d Asic%d", which_uart, which_BM1680);
    stat(BM1680_UPGRADE_FILE, &statbuf);
    upgrade_file_size = statbuf.st_size;

    upgrade_file_buf = malloc(upgrade_file_size+1);
    if(!upgrade_file_buf)
    {
        printf("\n!!! %s: malloc upgrade_file_buf failed!\n\n", __FUNCTION__);
        return;
    }

    upgrade_file = fopen(BM1680_UPGRADE_FILE, "r");
    if(!upgrade_file)
    {
        printf("\n!!! %s: open BM1680_UPGRADE_FILE failed\n\n", __FUNCTION__);
        fclose(upgrade_file);
        free(upgrade_file_buf);
        return;
    }

    fseek(upgrade_file, 0, SEEK_SET);
    fread(upgrade_file_buf, 1, upgrade_file_size+1, upgrade_file);

    for(i=0; i<upgrade_file_size/BM1680_UPGRADE_BIN_FILE_WITHOUT_ENDING_DATA_LEN; i++)
    {
        BM1680_upgrade_bin_file_without_ending(which_uart, which_BM1680, upgrade_file_buf + BM1680_UPGRADE_BIN_FILE_WITHOUT_ENDING_DATA_LEN*i);
    }

    if(upgrade_file_size % BM1680_UPGRADE_BIN_FILE_WITHOUT_ENDING_DATA_LEN)
    {
        BM1680_upgrade_bin_file_with_ending(which_uart, which_BM1680, upgrade_file_buf + BM1680_UPGRADE_BIN_FILE_WITHOUT_ENDING_DATA_LEN*i, upgrade_file_size % BM1680_UPGRADE_BIN_FILE_WITHOUT_ENDING_DATA_LEN);
    }
    applog(LOG_NOTICE,"Update Chain %d Asic%d end", which_uart, which_BM1680);
    free(upgrade_file_buf);
}



/**************** about BM1680 functions end ****************/


/**************** about other functions end ****************/

static __inline void flip_swab(void *dest_p, const void *src_p, unsigned int length)
{
    uint32_t *dest = dest_p;
    const uint32_t *src = src_p;
    int i;

    for (i = 0; i < length/4; i++)
        dest[i] = swab32(src[i]);
}

#define ABOUT_CGMINER_PTHREAD
/******************** about cgminer pthread ********************/
//#define PATTEN

void *bitmain_scanhash(void *arg)
{
    struct thr_info *thr = (struct thr_info*)arg;
    struct cgpu_info *bitmain_B3 = thr->cgpu;
    struct bitmain_B3_info *info = bitmain_B3->device_data;
    struct timeval current;
    uint8_t nonce_bin[4],crc_check,which_asic_nonce,which_core_nonce;
    uint32_t i,k, *work_nonce=NULL;
    uint64_t nonce;
    int submitfull = 0;
    bool submitnonceok = true;
    unsigned char work_id = 0, chain_id = 0, nonce_diff = 0, nonce_crc5 = 0,tm = 0;
    unsigned char pworkdata[128]= {0},  hash1[32]= {0};
    unsigned int endiandata[32]= {0};
    unsigned char *ob_hex=NULL;
    uint64_t ret = 0;

    struct work *work = NULL;
    cgtime(&current);
    h = 0;
    pthread_mutex_lock(&nonce_mutex);
    cg_rlock(&info->update_lock);

    while(nonce_fifo.nonce_num)
    {
        nonce_fifo.nonce_num--;

        nonce = (nonce_fifo.nonce_buffer[nonce_fifo.p_rd].nonce);
        work_id = nonce_fifo.nonce_buffer[nonce_fifo.p_rd].wc;
        chain_id = nonce_fifo.nonce_buffer[nonce_fifo.p_rd].chainid;
        tm = nonce_fifo.nonce_buffer[nonce_fifo.p_rd].tm;
        pthread_mutex_lock(&work_queue_mutex);
        if(info->work_queue[work_id] != NULL)
            work = copy_work(info->work_queue[work_id]);
        pthread_mutex_unlock(&work_queue_mutex);
        if(work)
        {
            if(tm >= work->btm_dff)
            {
                applog(LOG_NOTICE,"diff:%d %d",work->btm_dff,tm);
                submit_nonce_direct(thr, work, nonce);
            }
            h++;
            h_each_chain[chain_id] += (1<<TM);
            which_asic_nonce = nonce_fifo.nonce_buffer[nonce_fifo.p_rd].which_asic;

            applog(LOG_DEBUG,"%s: chain %d which_asic_nonce %d ", __FUNCTION__, chain_id, which_asic_nonce);

            if (( chain_id > BITMAIN_MAX_CHAIN_NUM ) || (!dev.chain_exist[chain_id]))
            {
                applog(LOG_ERR, "ChainID Cause Error! ChainID:[%d]", chain_id);
                goto crc_error;
            }

            if ( which_asic_nonce >= 2)
            {
                applog(LOG_DEBUG, "Which Nonce Cause Err![%d] %08x", which_asic_nonce,nonce);
                goto crc_error;
            }

            dev.chain_asic_nonce[chain_id][which_asic_nonce]++;

            cg_logwork(work, nonce_bin, submitnonceok);
            free_work(work);

        }
        else
        {
            applog(LOG_ERR, "%s %d: work %02x not find error", bitmain_B3->drv->name, bitmain_B3->device_id, work_id);
        }
    crc_error:
        if(nonce_fifo.p_rd < MAX_NONCE_NUMBER_IN_FIFO)
        {
            nonce_fifo.p_rd++;
        }
        else
        {
            nonce_fifo.p_rd = 0;
        }
    }

    cg_runlock(&info->update_lock);
    pthread_mutex_unlock(&nonce_mutex);
    cgsleep_ms(1);

    h = h * (1<<TM);
    return h;
}

void * check_fan_thr(void *arg)
{

    int i=0, j=0;
    unsigned char fan_id = 0;
    unsigned int fan_speed;
    while( 1 )
    {

        dev.fan_speed_top1 = 0;
        dev.fan_speed_low1 = 0;
        for(j=0; j < 2; j++)    //means check for twice to make sure find out all fan
        {
            for(i=0; i < BITMAIN_MAX_FAN_NUM; i++)
            {
                fan_speed = 0;
                fan_id = 0;
                if(get_fan_speed(&fan_id, &fan_speed) != -1)
                {
                    dev.fan_speed_value[fan_id] = fan_speed * 60 * 2;
                    if((fan_speed > 0) && (dev.fan_exist[fan_id] == 0))
                    {
                        dev.fan_exist[fan_id] = 1;
                        dev.fan_num++;
                        dev.fan_exist_map |= (0x1 << fan_id);
                    }
                    else if((fan_speed == 0) && (dev.fan_exist[fan_id] == 1))
                    {
                        dev.fan_exist[fan_id] = 0;
                        dev.fan_num--;
                        dev.fan_exist_map &= !(0x1 << fan_id);
                    }
                    if(dev.fan_speed_top1 < dev.fan_speed_value[fan_id])
                        dev.fan_speed_top1 = dev.fan_speed_value[fan_id];
                    if((dev.fan_speed_low1 > dev.fan_speed_value[fan_id] && dev.fan_speed_value[fan_id] != 0) || dev.fan_speed_low1 == 0)
                        dev.fan_speed_low1 = dev.fan_speed_value[fan_id];
                }
                cgsleep_ms(50);
            }
        }
        //dev.fan_speed_value[0] = dev.fan_speed_top1;
        //dev.fan_speed_value[1] = dev.fan_speed_low1;
        sleep(FANINT);
    }
}

int create_bitmain_check_fan_pthread(void)
{
    check_fan_id = calloc(1,sizeof(struct thr_info));
    if(thr_info_create(check_fan_id, NULL, check_fan_thr, NULL))
    {
        applog(LOG_DEBUG,"%s: create thread for check miner_status", __FUNCTION__);
        return -8;
    }
    pthread_detach(check_fan_id->pth);
    cgsleep_ms(500);
}

int fan_error_num = 0;
inline int check_fan_ok()
{
    int ret = 0;
    if(dev.fan_num < MIN_FAN_NUM)
    {
        ret = 1;
        goto err;
    }
    if(dev.fan_speed_value[0] < (FAN1_MAX_SPEED * dev.fan_pwm / 130))
    {
        ret = 2;
        goto err;
    }
    if(dev.fan_speed_value[1] < (FAN2_MAX_SPEED * dev.fan_pwm / 130))
    {
        ret = 3;
        goto err;
    }
    if((dev.pwm_percent == 100) && (dev.fan_speed_value[0] < (FAN1_MAX_SPEED * 90 / 100) || dev.fan_speed_value[1] < (FAN2_MAX_SPEED * 90 / 100)))
    {
        ret = 4;
        goto err;
    }
err:
    if(ret != 0)
    {
        fan_error_num++;
        applog(LOG_DEBUG, "fan_error_num:%d fan_num %d fan_pwm %d fan_speed_value[0] %d fan_speed_value[1] %d", fan_error_num,dev.fan_num,dev.fan_pwm,dev.fan_speed_value[0],dev.fan_speed_value[1]);
        if(fan_error_num > (FANINT * 10))
            return ret;
        else
        {
            return 0;
        }
    }
    else
    {
        fan_error_num = 0;
        return 0;
    }
}

void *check_miner_status(void *arg)
{
    struct bitmain_B3_info *info = (struct bitmain_B3_info*)arg;
    struct timeval tv_start = {0, 0}, diff = {0, 0}, tv_end,tv_send;
    double ghs = 0;
    int i = 0, j = 0;
    cgtime(&tv_end);
    cgtime(&tv_send);
    copy_time(&tv_start, &tv_end);
    copy_time(&tv_send_job,&tv_send);

    unsigned int asic_num = 0, error_asic = 0, avg_num = 0;
    unsigned int which_chain, which_asic, which_sensor;
    unsigned int offset = 0;
    int fan_ret = 0;
    int low_hash_times = 0;
    struct bitmian_B3_info_with_index info_with_index[BITMAIN_MAX_CHAIN_NUM];
    pthread_t reinit_id[BITMAIN_MAX_CHAIN_NUM];

    while(1)
    {
        diff.tv_sec = 0;
        diff.tv_usec = 0;
        cgtime(&tv_end);
        timersub(&tv_end, &tv_start, &diff);


        if (diff.tv_sec > 120)
        {
            asic_num = 0, error_asic = 0, avg_num = 0;

            for(which_chain=0; which_chain<BITMAIN_MAX_CHAIN_NUM; which_chain++)
            {
                if(dev.chain_exist[which_chain])
                {
                    asic_num += dev.chain_asic_num[which_chain];
                    for(which_asic=0; which_asic<dev.chain_asic_num[which_chain]; which_asic++)
                    {
                        avg_num += dev.chain_asic_nonce[which_chain][which_asic];
                        applog(LOG_DEBUG,"%s: chain %d asic %d asic_nonce_num %d", __FUNCTION__, which_chain,which_asic,dev.chain_asic_nonce[which_chain][which_asic]);
                    }
                }
            }
            if (asic_num != 0)
            {
                applog(LOG_DEBUG,"%s: avg_num %d asic_num %d", __FUNCTION__, avg_num,asic_num);
                avg_num = avg_num / asic_num / 8;
            }
            else
            {
                avg_num = 1;
            }

            for(which_chain=0; which_chain<BITMAIN_MAX_CHAIN_NUM; which_chain++)
            {
                if(dev.chain_exist[which_chain])
                {
                    offset = 0;

                    for(which_asic=0; which_asic<dev.chain_asic_num[which_chain]; which_asic++)
                    {
                        if(which_asic % 8 == 0)
                        {
                            if ( ( which_asic + offset ) > (ASIC_NUM_EACH_CHAIN + 16) )
                                applog(LOG_ERR, "asic num err![%d]", (which_asic + offset));
                            dev.chain_asic_status_string[which_chain][which_asic+offset] = ' ';
                            offset++;
                        }

                        if(dev.chain_asic_nonce[which_chain][which_asic]>avg_num)
                        {
                            if ( ( which_asic +offset ) > (ASIC_NUM_EACH_CHAIN + 16) )
                                applog(LOG_ERR, "asic num err![%d]", (which_asic + offset));
                            dev.chain_asic_status_string[which_chain][which_asic+offset] = 'o';
                        }
                        else
                        {
                            if ( ( which_asic + offset ) > (ASIC_NUM_EACH_CHAIN + 16) )
                                applog(LOG_ERR, "asic num err![%d]", (which_asic + offset));
                            dev.chain_asic_status_string[which_chain][which_asic+offset] = 'x';
                            error_asic++;
                        }

                        if ( ( which_asic ) > (ASIC_NUM_EACH_CHAIN + 16) )
                            applog(LOG_ERR, "asic num err![%d]", (which_asic));
                        dev.chain_asic_nonce[which_chain][which_asic] = 0;
                    }

                    if ( ( which_asic + offset ) > (ASIC_NUM_EACH_CHAIN + 16) )
                        applog(LOG_ERR, "asic num err![%d]", (which_asic + offset));
                    dev.chain_asic_status_string[which_chain][which_asic+offset] = '\0';
                    if(need_reinit[which_chain])//((each_chain_h_avg[which_chain]/1000000/10) < (double)(( ASIC_NUM_EACH_CHAIN * dev.frequency * dev.corenum * 0.80))) && (!status_error))
                    {
                        info_with_index[which_chain].info = info;
                        info_with_index[which_chain].chain_index = which_chain;
                        pthread_create(&reinit_id[which_chain], NULL, bitmain_B3_reinit_chain, (void *)(&info_with_index[which_chain]));
                        gMinerStatus_Low_Hashrate = true;
                    }
                    else
                    {
                        gMinerStatus_Low_Hashrate = false;
                    }
                }
            }

            copy_time(&tv_start, &tv_end);
        }

        // check temperature
        if(dev.temp_top1 > MAX_TEMP)
        {
            gMinerStatus_High_Temp_Counter++;
            if(gMinerStatus_High_Temp_Counter > 2)
            {
                gMinerStatus_High_Temp = true;
                applog(LOG_ERR,"%s: the temperature %d is too high, close PIC and need reboot!!!", __FUNCTION__, dev.temp_top1);
            }
            else
            {
                applog(LOG_ERR, "Temperature %d is higher than 85'C for %d time, PWM is %d", dev.temp_top1, gMinerStatus_High_Temp_Counter, dev.pwm_percent);
            }
        }
        else
        {
            gMinerStatus_High_Temp_Counter = 0;
            gMinerStatus_High_Temp = false;
        }

        // check fan
        fan_ret = 0;//check_fan_ok();
        if(fan_ret != 0)
        {
            gFan_Error = true;
            switch (fan_ret)
            {
                case 1:
                    applog(LOG_ERR, "Fan Err! Disable PIC! Fan num is %d",dev.fan_num);
                    break;
                case 2:
                    applog(LOG_ERR, "Fan Err! Disable PIC! Fan1 speed is too low %d pwm %d ",dev.fan_speed_value[0],dev.pwm_percent);
                    break;
                case 3:
                    applog(LOG_ERR, "Fan Err! Disable PIC! Fan2 speed is too low %d pwm %d ",dev.fan_speed_value[1],dev.pwm_percent);
                    break;
                case 4:
                    applog(LOG_ERR, "Fan Err! Disable PIC! Fan1:%d Fan2:%d pwm %d",dev.fan_speed_value[0],dev.fan_speed_value[1],dev.pwm_percent);
                    break;
            }
        }
        else
        {
            gFan_Error = false;
        }

        if(gMinerStatus_Low_Hashrate || gMinerStatus_High_Temp || gFan_Error || gMinerStatus_Not_read_all_sensor)
        {
            //stop = true;
            if((!once_error) && (gMinerStatus_High_Temp || gFan_Error ))
            {
                status_error = true;
                once_error = true;
                write_axi_fpga(SOCKET_ID, 7);
            }
        }
        else
        {
            stop = false;
            status_error = false;
            if (once_error)
            {
                stop = true;
                status_error = true;
            }
        }
        set_led(stop);
        cgsleep_ms(1000);
    }
}


int create_bitmain_check_miner_status_pthread(struct bitmain_B3_info *info)
{
    check_miner_status_id = calloc(1,sizeof(struct thr_info));
    if(thr_info_create(check_miner_status_id, NULL, check_miner_status, info))
    {
        applog(LOG_DEBUG,"%s: create thread for check miner_status", __FUNCTION__);
        return -5;
    }
    pthread_detach(check_miner_status_id->pth);
    cgsleep_ms(500);
}

void *get_hash_rate()
{
    uint32_t which_chain = 0, i = 0;
    struct timeval old_h, new_h, diff;
    double each_chain_h[BITMAIN_MAX_CHAIN_NUM][10] = {{0}};
    double each_chain_h_all = 0;
    int index[BITMAIN_MAX_CHAIN_NUM] = {0};
    cgtime(&old_h);
    cgtime(&new_h);

    while(1)
    {
        cgtime(&new_h);
        timersub(&new_h, &old_h, &diff);
        each_chain_h_all = 0;
        for(which_chain = 0; which_chain < BITMAIN_MAX_CHAIN_NUM; which_chain++)
        {
            if(dev.chain_exist[which_chain])
            {
                each_chain_h[which_chain][index[which_chain]] = (double)(h_each_chain[which_chain] );

                h_each_chain[which_chain] = 0;
                each_chain_h[which_chain][index[which_chain]] = each_chain_h[which_chain][index[which_chain]] / (diff.tv_sec + ((double)(diff.tv_usec + 1) / 1000000));
                each_chain_h_avg[which_chain] = 0;
                for( i = 0; i < 10; i++)
                {
                    each_chain_h_avg[which_chain]+= each_chain_h[which_chain][i];
                }

                sprintf(displayed_rate[which_chain],"%.2f",each_chain_h_avg[which_chain]/10);
                each_chain_h_all += each_chain_h_avg[which_chain]/10;

                index[which_chain]++;
                if (index[which_chain] >= 10)
                {
                    index[which_chain] = 0;
                }
            }
        }
        sprintf(displayed_hash_rate,"%.2f",each_chain_h_all);
        geach_chain_h_all = each_chain_h_all;
        copy_time(&old_h,&new_h);

        sleep(READ_HASH_RATE_TIME_GAP);
    }
}


int create_bitmain_get_hash_rate_pthread(void)
{
    read_hash_rate = calloc(1,sizeof(struct thr_info));
    if(thr_info_create(read_hash_rate, NULL, get_hash_rate, read_hash_rate))
    {
        applog(LOG_DEBUG,"%s: create thread for get hashrate from asic failed", __FUNCTION__);
        return -6;
    }
    pthread_detach(read_hash_rate->pth);
    cgsleep_ms(500);
}


void *check_status_func()
{
    unsigned char which_chain;

    applog(LOG_DEBUG, "%s", __FUNCTION__);
    while(1)
    {

        for ( which_chain = 0; which_chain < BITMAIN_MAX_CHAIN_NUM; which_chain++ )
        {
            if ( dev.chain_exist[which_chain] == 1 && gBegin_get_nonce[which_chain])
            {
                BM1680_check_status(which_chain,0);
                BM1680_check_status(which_chain,1);
            }
        }
        sleep(5);
    }
}


void *read_temp_func()
{
    unsigned char which_chain;
    int16_t tmpTemp = 0;
    uint8_t temp = 0;

    applog(LOG_DEBUG, "%s", __FUNCTION__);
    while(1)
    {
        tmpTemp = 0;
        for ( which_chain = 0; which_chain < BITMAIN_MAX_CHAIN_NUM; which_chain++ )
        {
            if ( dev.chain_exist[which_chain] == 1 && start_send[which_chain])
            {
                BM1680_get_board_temperature(which_chain, 0);
                BM1680_get_asic_temperature(which_chain, 0);
                BM1680_get_board_temperature(which_chain, 1);
                BM1680_get_asic_temperature(which_chain, 1);
            }
        }

        sleep(READ_TEMPERATURE_TIME_GAP);
        for ( which_chain = 0; which_chain < BITMAIN_MAX_CHAIN_NUM; which_chain++ )
        {
            if ( dev.chain_asic_temp[which_chain][0][1] > tmpTemp)
            {
                tmpTemp = dev.chain_asic_temp[which_chain][0][1];
            }
        }
        dev.temp_top1 = tmpTemp;
        if(stop)
        {
            set_PWM(MAX_PWM_PERCENT);
        }
        else
        {
            set_PWM_according_to_temperature();
        }
    }
}

int create_bitmain_check_status_pthread(void)
{
    check_status_id = calloc(1,sizeof(struct thr_info));
    if(thr_info_create(check_status_id, NULL, check_status_func, check_status_id))
    {
        applog(LOG_DEBUG,"%s: create thread for read temp", __FUNCTION__);
        return -7;
    }
    pthread_detach(check_status_id->pth);
    cgsleep_ms(500);
}


int create_bitmain_read_temp_pthread(void)
{
    read_temp_id = calloc(1,sizeof(struct thr_info));
    if(thr_info_create(read_temp_id, NULL, read_temp_func, read_temp_id))
    {
        applog(LOG_DEBUG,"%s: create thread for read temp", __FUNCTION__);
        return -7;
    }
    pthread_detach(read_temp_id->pth);
    cgsleep_ms(500);
}

void *B3_fill_work(void *usrdata)
{
    pthread_detach(pthread_self());
    applog(LOG_DEBUG, "Start To Fill Work!");
    struct bitmian_B3_info_with_index *info_with_chain = (struct bitmian_B3_info_with_index *)usrdata;
    struct bitmain_B3_info *info = info_with_chain->info;
    uint8_t chainid = info_with_chain->chain_index;
    struct thr_info * thr = info->thr;
    struct timeval send_start, last_send, send_elapsed;
    struct work *work = NULL;
    struct pool *pool = NULL;
    char * current_job_id = NULL;
    int8_t seed[32], workdata0[137],workdata1[137];
    int8_t workid = 0;
    unsigned int i = 0;
    bool send_tm = true;

    applog(LOG_DEBUG, "Start To Fill Work!ChainIndex:[%d]", chainid);

    while(1 && !reiniting[chainid])
    {

        if(!start_send[chainid])
        {
            cgsleep_ms(10);
            continue;
        }

        pool = current_pool();
        if(pool == NULL)
        {
            cgsleep_ms(10);
            continue;
        }

        cg_rlock(&pool->data_lock);
        if(current_job_id == NULL)
        {
            if(pool->swork.job_id != NULL)
            {
                current_job_id = strdup(pool->swork.job_id);
                new_block[chainid] = true;
            }
            else
            {
                cgsleep_ms(10);
                cg_runlock(&pool->data_lock);
                continue;
            }
        }
        else if(pool->swork.job_id != NULL)
        {
            if(strcmp(current_job_id,pool->swork.job_id) != 0)
            {
                free(current_job_id);
                current_job_id = strdup(pool->swork.job_id);
                new_block[chainid] = true;
            }
            else
            {
                cgsleep_ms(10);
                cg_runlock(&pool->data_lock);
                continue;
            }
        }
        if(pool->update_seed)
        {
            for(i = 0; i < BITMAIN_MAX_CHAIN_NUM; i++)
                update_seed[i] = true;
            pool->update_seed = false;
        }
        cg_runlock(&pool->data_lock);

        if(update_seed[chainid] == true)
        {
            update_seed_and_wait_ok(chainid,pool->btm_seed);
            update_seed[chainid] = false;
        }

        if(new_block[chainid])
        {
            cgtime(&last_send);
            work = make_work();
            gen_stratum_work(pool,work);

            workid = (global_workid++) & 0x7f;
            new_block[chainid] = false;
            memset(workdata0, 0, sizeof(workdata0));
            workdata0[0] = workid;
            memcpy(workdata0+1, work->data, 136);
            workdata0[129] = chainid;
            workdata0[132] = chainid;
            pthread_mutex_lock(&work_queue_mutex);
            if(info->work_queue[workid])
            {
                free_work(info->work_queue[workid]);
                info->work_queue[workid] = NULL;
            }
            info->work_queue[workid] = copy_work(work);
            pthread_mutex_unlock(&work_queue_mutex);
            applog(LOG_DEBUG, "ChainID[%d] Wirte Work. workid=%d", chainid, workid);
            BM1680_set_message(chainid,0,workdata0);


            workid = (global_workid++) & 0x7f;
            memset(workdata1, 0, sizeof(workdata1));
            workdata1[0] = workid;
            memcpy(workdata1+1, work->data, 136);
            workdata1[129] = chainid << 4;
            workdata1[132] = chainid << 4;
            pthread_mutex_lock(&work_queue_mutex);
            if(info->work_queue[workid])
            {
                free_work(info->work_queue[workid]);
                info->work_queue[workid] = NULL;
            }
            info->work_queue[workid] = copy_work(work);
            pthread_mutex_unlock(&work_queue_mutex);
            applog(LOG_DEBUG, "ChainID[%d] Wirte Work. workid=%d", chainid, workid);
            BM1680_set_message(chainid,1,workdata1);
            gBegin_get_nonce[chainid] = true;
        }

        cgsleep_us(500);
        if(work != NULL)
        {
            free_work(work);
        }
    }
	if(current_job_id != NULL)
		free(current_job_id);
}

void add_point(int * point,int MAX_SIZE)
{
    (*point)++;
    if((*point) >= MAX_SIZE)
        (*point) = 0;
}

inline int use_point_sub_1(int point,int MAX_SIZE)
{
    return (point == 0) ? MAX_SIZE -1 : point -1;
}

int use_point_add_1(int point,int MAX_SIZE)
{
    return (point >= MAX_SIZE -1) ? 0 : (point + 1) ;
}

int get_data_len(uint8_t cmd, uint8_t ack)
{
    int data_len = 0;
    switch(cmd)
    {
        case 0:
            data_len = 1;
            break;
        case 1:
        case 2:
        case 3:
            data_len = 0;
            break;
        case 4:
            data_len = (ack & 0x7f) * BM1680_NONCE_DATA_LEN + 1;
            break;
        case 5:
        case 6:
        case 7:
            data_len = 0;
            break;
        case 8:
            data_len = 9;
            break;
        case 9:
        case 10:
            data_len = 4;
            break;
    }
    return data_len;
}

bool is_nonce_or_reg_value(uint8_t ack)
{
    return ((ack >> 7) > 0) ? true : false;
}

void process_ack(uint8_t * data, int len, int chain_id)
{
    int8_t asic = data[BM1680_CHIP_ADDRESS_ADDR];
    int8_t ack = data[BM1680_ACK_ADDR];
    int8_t cmd = data[BM1680_CMD_ADDR];
    int8_t ret = 0;
    if((ack & 0x80) == 0x0)
    {
        switch(ack)
        {
            case BM1680_EXECUTE_OK:
                ret = BM1680_EXECUTE_OK;
                break;

            case BM1680_RECEIVED_DATA_CRC_ERROR:
                ret = BM1680_RECEIVED_DATA_CRC_ERROR;
                break;

            case BM1680_NOT_ENOUGH_MEMORY:
                ret = BM1680_NOT_ENOUGH_MEMORY;
                break;

            case BM1680_LOCAL_MEMORY_NOT_ENOUGH:
                ret = BM1680_LOCAL_MEMORY_NOT_ENOUGH;
                break;

            case BM1680_CMD_PARAMETER_ERROR:
                ret = BM1680_CMD_PARAMETER_ERROR;
                break;

            case BM1680_CMD_INDEX_ERROR:
                ret = BM1680_CMD_INDEX_ERROR;
                break;

            case BM1680_CHECK_STATUS_ERROR:
                ret = BM1680_CHECK_STATUS_ERROR;
                break;
            default:
                ret = BM1680_NOT_SUPPORT_THIS_CMD;
                break;
        }
    }
    BM1680_ack_record[chain_id][asic][cmd] = ret;
    if(ret == BM1680_EXECUTE_OK)
    {
        switch(cmd)
        {
            case BM1680_INIT_CMD:
                applog(LOG_NOTICE,"Chain %d Asic %d version is %d",chain_id, asic, data[BM1680_ACK_DATA_BEGIN_ADDR]);
                break;
            case BM1680_SET_MESSAGE:
                applog(LOG_NOTICE,"Chain %d Asic %d set message ok",chain_id, asic);
                break;
            case BM1680_GET_BOARD_TEMPERATURE:
                dev.temp[chain_id][asic][0] = data[BM1680_ACK_DATA_BEGIN_ADDR];
                applog(LOG_NOTICE,"Chain %d Asic %d local temp %d",chain_id, asic,dev.temp[chain_id][asic][0]);
                break;
            case BM1680_GET_ASIC_TEMPERATURE:
                dev.temp[chain_id][asic][1] = data[BM1680_ACK_DATA_BEGIN_ADDR];
                applog(LOG_NOTICE,"Chain %d Asic %d remote temp %d",chain_id, asic,dev.temp[chain_id][asic][1]);
                break;
        }
        dev.chain_asic_temp[chain_id][0][0] = dev.temp[chain_id][asic][0];
        dev.chain_asic_temp[chain_id][0][1] = dev.temp[chain_id][asic][1];
    }
    else
    {
        applog(LOG_NOTICE,"Chain %d Asic %d cmd %x ret is %d",chain_id, asic, cmd, ret);
		if(cmd == BM1680_CHECK_STATUS || ret == BM1680_CHECK_STATUS_ERROR) need_reinit[chain_id] = true;
    }
}
void *get_asic_response(void* arg)
{
    pthread_detach(pthread_self());

    uint32_t  nonce_number, read_loop;
    unsigned char nonce_bin[7],chainid;

    struct dev_info *dev_i = (struct dev_info*)arg;
    chainid = dev_i->chainid;

    unsigned char receive_buf[10 * 100] = {0};    // used to receive data
    unsigned char tmp[10 * MAX_NONCE_NUMBER + BM1680_RECEIVE_CMD_LEN_EXCLUDE_DATA] = {0};          // store 9 bytes data that from data_buf
    unsigned char data_buf[10 * 512] = {0};
    int data_buf_rp = 0, data_buf_wp = 0;
    ssize_t len = 0;
    ssize_t data_len = 0;
    int i = 0;
    int max = 10 * 512;
    char * hex_buff = NULL;

    applog(LOG_NOTICE, "Start A New Asic Response.Chain Id:[%d]", chainid);
    applog(LOG_DEBUG, "%s %d",__FUNCTION__,chainid);

    clear_uart_rx_fifo(chainid);
    clear_uart_rx_fifo(chainid);
    clear_uart_rx_fifo(chainid);

    while(start_recv[chainid])
    {
        cgsleep_ms(100);
        len = B3_read(chainid, receive_buf, sizeof(receive_buf));
#if 0
        if(len != 0)
        {
            char *receive_hex = NULL;
            receive_hex = bin2hex(receive_buf,len);
            applog(LOG_NOTICE,"chain %d read out : %d  %s", chainid, len, receive_hex);
            free(receive_hex);
        }
#endif
        for(i = 0; i < len; i++)
        {
            data_buf[data_buf_wp] = receive_buf[i];
            add_point(&data_buf_wp,max);
        }

        if(data_buf_rp != data_buf_wp)
        {
            len = (data_buf_wp > data_buf_rp) ? (data_buf_wp - data_buf_rp):( sizeof(data_buf) - data_buf_rp + data_buf_wp);
        }
        else
        {
            continue;
        }
        while(len >= BM1680_RECEIVE_CMD_LEN_EXCLUDE_DATA)
        {
            if(data_buf[data_buf_rp] == BM1680_RECV_CMD_HEADER1 && data_buf[use_point_add_1(data_buf_rp,max)] == BM1680_RECV_CMD_HEADER2)
            {
                break;
            }
            else
            {
                add_point(&data_buf_rp,max);
                len--;
            }
        }

        if(len >= BM1680_RECEIVE_CMD_LEN_EXCLUDE_DATA)
            data_len = get_data_len(data_buf[data_buf_rp + 3], data_buf[data_buf_rp + 4]);

        while(len >= (BM1680_RECEIVE_CMD_LEN_EXCLUDE_DATA + data_len))
        {
            for(i = 0; i < BM1680_RECEIVE_CMD_LEN_EXCLUDE_DATA + data_len; i++)
            {
                tmp[i] = data_buf[data_buf_rp];
                add_point(&data_buf_rp,max);
            }

            len = len - (BM1680_RECEIVE_CMD_LEN_EXCLUDE_DATA + data_len);
#if 0
            char* return_data = NULL;
            return_data = bin2hex(tmp,BM1680_RECEIVE_CMD_LEN_EXCLUDE_DATA + data_len);
            applog(LOG_NOTICE,"return_data : %s",return_data);
            free(return_data);
#endif
            if(is_nonce_or_reg_value(tmp[BM1680_ACK_ADDR])) // get a nonce
            {
                applog(LOG_NOTICE,"Chain %d Asic %d get nonce count %d", chainid, tmp[2], tmp[4] & 0x7f);
                if(gBegin_get_nonce[chainid])
                {
                    pthread_mutex_lock(&nonce_mutex);
                    for (i = 0; i < (data_len -1) / BM1680_NONCE_DATA_LEN; i++)
                    {
                        memcpy((unsigned char *)(&nonce_fifo.nonce_buffer[nonce_fifo.p_wr].nonce), &tmp[7 + i*BM1680_NONCE_DATA_LEN], 8);    // we do not swap32 here, but swap it when we analyse nonce
                        nonce_fifo.nonce_buffer[nonce_fifo.p_wr].wc             = tmp[5] & 0x7f;
                        nonce_fifo.nonce_buffer[nonce_fifo.p_wr].chainid        = chainid;
                        nonce_fifo.nonce_buffer[nonce_fifo.p_wr].tm             = tmp[6 + i*BM1680_NONCE_DATA_LEN];
                        nonce_fifo.nonce_buffer[nonce_fifo.p_wr].which_asic     = tmp[BM1680_CHIP_ADDRESS_ADDR];
#if 0
                        applog(LOG_NOTICE,"%llx, %x, %x, %x, %x",nonce_fifo.nonce_buffer[nonce_fifo.p_wr].nonce,nonce_fifo.nonce_buffer[nonce_fifo.p_wr].wc,
                               nonce_fifo.nonce_buffer[nonce_fifo.p_wr].chainid,nonce_fifo.nonce_buffer[nonce_fifo.p_wr].tm,nonce_fifo.nonce_buffer[nonce_fifo.p_wr].which_asic);
#endif
                        if(nonce_fifo.p_wr < MAX_NONCE_NUMBER_IN_FIFO)
                        {
                            nonce_fifo.p_wr++;
                        }
                        else
                        {
                            nonce_fifo.p_wr = 0;
                        }

                        if(nonce_fifo.nonce_num < MAX_NONCE_NUMBER_IN_FIFO)
                        {
                            nonce_fifo.nonce_num++;
                        }
                        else
                        {
                            nonce_fifo.nonce_num = MAX_NONCE_NUMBER_IN_FIFO;
                            applog(LOG_WARNING, "%s: nonce fifo full!!!", __FUNCTION__);
                        }
                    }
                    pthread_mutex_unlock(&nonce_mutex);
                }
            }
            else    // get a register value
            {
                process_ack(tmp,BM1680_RECEIVE_CMD_LEN_EXCLUDE_DATA + data_len,chainid);
            }

            while(len >= BM1680_RECEIVE_CMD_LEN_EXCLUDE_DATA)
            {
                if(data_buf[data_buf_rp] == BM1680_RECV_CMD_HEADER1 && data_buf[use_point_add_1(data_buf_rp,max)] == BM1680_RECV_CMD_HEADER2)
                {
                    break;
                }
                else
                {
                    add_point(&data_buf_rp,max);
                    len--;
                }
            }
            data_len = get_data_len(data_buf[data_buf_rp + 3], data_buf[data_buf_rp + 4]);
        }
    }
}

/****************** about cgminer pthread end ******************/



#define ABOUT_CGMINER_DRIVER
/******************** about cgminer driver ********************/

static void bitmain_B3_detect(__maybe_unused bool hotplug)
{
    struct cgpu_info *cgpu = calloc(1, sizeof(*cgpu));
    struct device_drv *drv = &bitmain_B3_drv;
    assert(cgpu);
    cgpu->drv = drv;
    cgpu->deven = DEV_ENABLED;
    cgpu->threads = 1;
    cgpu->device_data = calloc(sizeof(struct bitmain_B3_info), 1);
    if (unlikely(!(cgpu->device_data)))
        quit(1, "Failed to calloc cgpu_info data");

    assert(add_cgpu(cgpu));
    applog(LOG_DEBUG,"%s detect new device",__FUNCTION__);
}

static bool bitmain_B3_prepare(struct thr_info *thr)
{
    struct cgpu_info *bitmain_B3 = thr->cgpu;
    struct bitmain_B3_info *info = bitmain_B3->device_data;

    info->thr = thr;
    mutex_init(&info->lock);
    cglock_init(&info->update_lock);

    bitmain_B3_init(info);

    return true;
}

static int64_t bitmain_B3_scanhash(struct thr_info *thr)
{
    h = 0;
    pthread_t send_id;
    pthread_create(&send_id, NULL, bitmain_scanhash, (void*)thr);
    pthread_join(send_id, NULL);

    return h;
}

static void bitmain_B3_update(struct cgpu_info *bitmain)
{
    int i = 0;
    applog(LOG_DEBUG, "Updated Work!");
    for ( ; i < BITMAIN_MAX_CHAIN_NUM; ++i )
    {
        new_block[i] = true;
    }
}

static double hwp = 0;

static struct api_data *bitmain_api_stats(struct cgpu_info *cgpu)
{
    struct api_data *root = NULL;
    int i = 0;
    uint64_t hash_rate_all = 0;
    bool copy_data = false;

    root = api_add_uint8(root, "miner_count", &(dev.chain_num), copy_data);
    root = api_add_string(root, "frequency", "750", copy_data);
    root = api_add_uint8(root, "fan_num", &(dev.fan_num), copy_data);

    // dev.fan_speed_value[0] is FAN1
    // dev.fan_speed_value[1] is FAN2
    for(i = 0; i < BITMAIN_MAX_FAN_NUM; i++)
    {
        char fan_name[12];
        sprintf(fan_name,"fan%d",i+1);
        root = api_add_uint32(root, fan_name, &(dev.fan_speed_value[i]), copy_data);
    }

    root = api_add_uint8(root, "temp_num", &(dev.chain_num), copy_data);

    for(i = 0; i < BITMAIN_MAX_CHAIN_NUM; i++)
    {
        char temp_name[12];
        sprintf(temp_name,"temp%d",i+1);
        root = api_add_int16(root, temp_name, &(dev.chain_asic_temp[i][0][0]), copy_data);
    }


    for(i = 0; i < BITMAIN_MAX_CHAIN_NUM; i++)
    {
        char temp2_name[12];
        sprintf(temp2_name,"temp2_%d",i+1);
        root = api_add_int16(root, temp2_name, &(dev.chain_asic_temp[i][0][1]), copy_data);
    }

    root = api_add_uint32(root, "temp_max", &(dev.temp_top1), copy_data);

    total_diff1 = total_diff_accepted + total_diff_rejected + total_diff_stale;
    hwp = (hw_errors + total_diff1) ?
          (double)(hw_errors) / (pow(2, 8)) / (double)(hw_errors + total_diff1) : 0;

    root = api_add_percent(root, "Device Hardware%", &hwp, true);
    root = api_add_int(root, "no_matching_work", &hw_errors, true);

    for(i = 0; i < BITMAIN_MAX_CHAIN_NUM; i++)
    {
        char chain_name[12];
        sprintf(chain_name,"chain_acn%d",i+1);
        root = api_add_uint8(root, chain_name, &(dev.chain_asic_num[i]), copy_data);

    }

    for(i = 0; i < BITMAIN_MAX_CHAIN_NUM; i++)
    {
        char chain_asic_name[12];
        sprintf(chain_asic_name,"chain_acs%d",i+1);
        root = api_add_string(root, chain_asic_name, dev.chain_asic_status_string[i], copy_data);
    }

    for(i = 0; i < BITMAIN_MAX_CHAIN_NUM; i++)
    {
        char chain_hw[16];
        sprintf(chain_hw,"chain_hw%d",i+1);
        root = api_add_uint32(root, chain_hw, &(dev.chain_hw[i]), copy_data);
    }

    for(i = 0; i < BITMAIN_MAX_CHAIN_NUM; i++)
    {
        char chain_rate[16];
        sprintf(chain_rate,"chain_rate%d",i+1);
        root = api_add_string(root, chain_rate, displayed_rate[i], copy_data);
    }

    return root;
}

static void bitmain_B3_reinit_device(struct cgpu_info *bitmain)
{
    if(!status_error)
        system("/etc/init.d/cgminer.sh restart > /dev/null 2>&1 &");
}

static void get_bitmain_statline_before(char *buf, size_t bufsiz, struct cgpu_info *bitmain_B3)
{
    struct bitmain_B3_info *info = bitmain_B3->device_data;
}

static void bitmain_B3_shutdown(struct thr_info *thr)
{
    thr_info_cancel(check_miner_status_id);
    thr_info_cancel(check_fan_id);
    thr_info_cancel(read_hash_rate);
    thr_info_cancel(read_temp_id);
}

struct device_drv bitmain_B3_drv =
{
    .drv_id = DRIVER_bitmain_B3,
    .dname = "Bitmain_B3",
    .name = "B3",
    .drv_detect = bitmain_B3_detect,
    .thread_prepare = bitmain_B3_prepare,
    .hash_work = &hash_driver_work,
    .scanwork = bitmain_B3_scanhash,
    .update_work = bitmain_B3_update,
    .get_api_stats = bitmain_api_stats,
    .reinit_device = bitmain_B3_reinit_device,
    .get_statline_before = get_bitmain_statline_before,
    .thread_shutdown = bitmain_B3_shutdown,
};

/****************** about cgminer driver end ******************/
