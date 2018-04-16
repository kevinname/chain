#ifndef __DRIVER_BTM_B3_H__

#define __DRIVER_BTM_B3_H__

#include <poll.h>
#include <termios.h>
#include "miner.h"
//#define UP_FREQ_MODE
#define HIGH_VOLTAGE_OPEN_CORE

/************************************** MARCO for 1680 **************************************/


#define FIRST_BM1680			0
#define SECOND_BM1680			1

#define BOARD_TEMP				0
#define ASIC_TEMP				1

// header
#define BM1680_SEND_CMD_HEADER1					    0xAA
#define BM1680_SEND_CMD_HEADER2					    0x55
#define BM1680_RECV_CMD_HEADER1				        0x55
#define BM1680_RECV_CMD_HEADER2				        0xAA

// command
#define BM1680_MAX_CMD_NUM							16
#define BM1680_INIT_CMD							    0x00
#define BM1680_SET_NONCE_DIFF						0x01
#define BM1680_SET_SEED								0x02
#define BM1680_SET_MESSAGE							0x03
#define BM1680_CHECK_STATUS							0x04
#define BM1680_SET_NONCE_INTERVAL					0x05
#define BM1680_UPGRADE_BIN_FILE_WITHOUT_ENDING		0x06
#define BM1680_UPGRADE_BIN_FILE_WITH_ENDING			0x07
#define BM1680_TEST_CMD							    0x08
#define BM1680_GET_BOARD_TEMPERATURE				0x09
#define BM1680_GET_ASIC_TEMPERATURE					0x0A

// command length
#define BM1680_SEND_CMD_LEN_EXCLUDE_DATA					    5
#define BM1680_SET_NONCE_DIFF_DATA_LEN							4
#define BM1680_SET_SEED_DATA_LEN								32
#define BM1680_SET_MESSAGE_DATA_LEN								137
#define BM1680_SET_NONCE_INTERVAL_DATA_LEN						16
#define BM1680_UPGRADE_BIN_FILE_WITHOUT_ENDING_DATA_LEN			256
#define BM1680_UPGRADE_BIN_FILE_WITH_ENDING_DATA_LEN			256
#define BM1680_TEST_CMD_DATA_LEN							    9

#define BM1680_INIT_CMD_CMD_LEN							    (BM1680_SEND_CMD_LEN_EXCLUDE_DATA)
#define BM1680_SET_NONCE_DIFF_CMD_LEN						(BM1680_SEND_CMD_LEN_EXCLUDE_DATA + BM1680_SET_NONCE_DIFF_DATA_LEN)
#define BM1680_SET_SEED_CMD_LEN								(BM1680_SEND_CMD_LEN_EXCLUDE_DATA + BM1680_SET_SEED_DATA_LEN)
#define BM1680_SET_MESSAGE_CMD_LEN							(BM1680_SEND_CMD_LEN_EXCLUDE_DATA + BM1680_SET_MESSAGE_DATA_LEN)
#define BM1680_CHECK_STATUS_CMD_LEN							(BM1680_SEND_CMD_LEN_EXCLUDE_DATA)
#define BM1680_SET_NONCE_INTERVAL_CMD_LEN					(BM1680_SEND_CMD_LEN_EXCLUDE_DATA + BM1680_SET_NONCE_INTERVAL_DATA_LEN)
#define BM1680_UPGRADE_BIN_FILE_WITHOUT_ENDING_CMD_LEN		(BM1680_SEND_CMD_LEN_EXCLUDE_DATA + BM1680_UPGRADE_BIN_FILE_WITHOUT_ENDING_DATA_LEN)
#define BM1680_UPGRADE_BIN_FILE_WITH_ENDING_CMD_LEN			(BM1680_SEND_CMD_LEN_EXCLUDE_DATA + BM1680_UPGRADE_BIN_FILE_WITH_ENDING_DATA_LEN)
#define BM1680_TEST_CMD_CMD_LEN							    (BM1680_SEND_CMD_LEN_EXCLUDE_DATA + BM1680_TEST_CMD_DATA_LEN)
#define BM1680_GET_BOARD_TEMPERATURE_CMD_LEN				(BM1680_SEND_CMD_LEN_EXCLUDE_DATA)
#define BM1680_GET_ASIC_TEMPERATURE_CMD_LEN					(BM1680_SEND_CMD_LEN_EXCLUDE_DATA)



// command return data length
#define BM1680_RECEIVE_CMD_LEN_EXCLUDE_DATA					    6

#define BM1680_MESSAGE_ID_LEN									1
#define BM1680_VERSION_LEN  									1
#define BM1680_NONCE_TM_LEN										1
#define BM1680_NONCE_LEN										8
#define BM1680_NONCE_DATA_LEN									(BM1680_NONCE_TM_LEN + BM1680_NONCE_LEN)
#define BM1680_TEST_CMD_RETURN_DATA_LEN						    9
#define BM1680_GET_BOARD_TEMPERATURE_CMD_RETURN_DATA_LEN	    4
#define BM1680_GET_ASIC_TEMPERATURE_CMD_RETURN_DATA_LEN		    4

#define BM1680_COMMON_RETURN_LEN							(BM1680_RECEIVE_CMD_LEN_EXCLUDE_DATA)
#define BM1680_INIT_RETURN_LEN							    (BM1680_RECEIVE_CMD_LEN_EXCLUDE_DATA + BM1680_VERSION_LEN)
#define BM1680_TEST_CMD_RETURN_LEN							(BM1680_RECEIVE_CMD_LEN_EXCLUDE_DATA + BM1680_TEST_CMD_RETURN_DATA_LEN)
#define BM1680_GET_BOARD_TEMPERATURE_CMD_RETURN_LEN			(BM1680_RECEIVE_CMD_LEN_EXCLUDE_DATA + BM1680_GET_BOARD_TEMPERATURE_CMD_RETURN_DATA_LEN)
#define BM1680_GET_ASIC_TEMPERATURE_CMD_RETURN_LEN			(BM1680_RECEIVE_CMD_LEN_EXCLUDE_DATA + BM1680_GET_ASIC_TEMPERATURE_CMD_RETURN_DATA_LEN)


//
#define BM1680_CHIP_ADDRESS_ADDR		2
#define BM1680_CMD_ADDR					3
#define BM1680_ACK_ADDR					4
#define BM1680_ACK_DATA_BEGIN_ADDR		5

// error code
#define BM1680_EXECUTE_OK					0
#define BM1680_RECEIVED_DATA_CRC_ERROR		16
#define BM1680_NOT_ENOUGH_MEMORY			17
#define BM1680_LOCAL_MEMORY_NOT_ENOUGH		18
#define BM1680_CMD_PARAMETER_ERROR		    19
#define BM1680_CMD_INDEX_ERROR			    20
#define BM1680_CHECK_STATUS_ERROR			21
#define BM1680_NOT_SUPPORT_THIS_CMD		    99

// upgrade file
//#define UPGRADE
#define BM1680_UPGRADE_FILE		"/usr/bin/bm1680_v3.bin"

#define MAX_NONCE_NUMBER        100           // max 100 nonce


void BM1680_send_init(uint8_t which_uart, uint8_t which_BM1680);
void BM1680_set_nonce_diff(uint8_t which_uart, uint8_t which_BM1680, uint8_t nonce_diff);
void BM1680_set_seed(uint8_t which_uart, uint8_t which_BM1680, uint8_t *seed);
void BM1680_set_message(uint8_t which_uart, uint8_t which_BM1680, uint8_t *message);
void BM1680_check_status(uint8_t which_uart, uint8_t which_BM1680);
void BM1680_set_nonce_interval(uint8_t which_uart, uint8_t which_BM1680, uint64_t start_nonce, uint64_t end_nonce);
void BM1680_get_board_temperature(uint8_t which_uart, uint8_t which_BM1680);
void BM1680_get_asic_temperature(uint8_t which_uart, uint8_t which_BM1680);
void BM1680_upgrade_bin_file_with_ending(unsigned char which_uart, unsigned char which_BM1680, unsigned char *data, unsigned int data_len);
void BM1680_upgrade_bin_file_without_ending(unsigned char which_uart, unsigned char which_BM1680, unsigned char *data);
void BM1680_upgrade(unsigned char which_uart, unsigned char which_BM1680);


/**************************************END MARCO for 1680 **************************************/

// FPGA macro define
#define TOTAL_LEN                       0x300
#define FPGA_MEM_TOTAL_LEN              (16*1024*1024)  // 16M bytes


//FPGA rgister Address Map
#define HARDWARE_VERSION                (0x00000000/sizeof(int))
#define FAN_SPEED                       (0x00000004/sizeof(int))
#define HASH_ON_PLUG                    (0x00000008/sizeof(int))
#define BUFFER_SPACE                    (0x0000000c/sizeof(int))
#define RETURN_NONCE                    (0x00000010/sizeof(int))
#define NONCE_NUMBER_IN_FIFO            (0x00000018/sizeof(int))
#define NONCE_FIFO_INTERRUPT            (0x0000001c/sizeof(int))
#define TEMPERATURE_0_3                 (0x00000020/sizeof(int))
#define TEMPERATURE_4_7                 (0x00000024/sizeof(int))
#define TEMPERATURE_8_11                (0x00000028/sizeof(int))
#define TEMPERATURE_12_15               (0x0000002c/sizeof(int))
#define IIC_COMMAND                     (0x00000030/sizeof(int))
#define RESET_HASH_BOARD                (0x00000034/sizeof(int))
#define BT8D							(0x0000003C/sizeof(int))
#define TW_WRITE_COMMAND                (0x00000040/sizeof(int))
#define QN_WRITE_DATA_COMMAND           (0x00000080/sizeof(int))
#define FAN_CONTROL                     (0x00000084/sizeof(int))
#define TIME_OUT_CONTROL                (0x00000088/sizeof(int))
#define TICKET_MASK_FPGA                (0x0000008c/sizeof(int))
#define HASH_COUNTING_NUMBER_FPGA       (0x00000090/sizeof(int))
#define SNO                             (0x00000094/sizeof(int))
#define SOCKET_ID						(0x00000098/sizeof(int))
#define BC_WRITE_COMMAND                (0x000000c0/sizeof(int))
#define BC_COMMAND_BUFFER               (0x000000c4/sizeof(int))
#define FPGA_CHIP_ID_ADDR               (0x000000f0/sizeof(int))
#define CRC_ERROR_CNT                   (0x000000f8/sizeof(int))
#define DHASH_ACC_CONTROL               (0x00000100/sizeof(int))
#define COINBASE_AND_NONCE2_LENGTH      (0x00000104/sizeof(int))
#define WORK_NONCE_2                    (0x00000108/sizeof(int))
#define NONCE2_AND_JOBID_STORE_ADDRESS  (0x00000110/sizeof(int))
#define MERKLE_BIN_NUMBER               (0x00000114/sizeof(int))
#define JOB_START_ADDRESS               (0x00000118/sizeof(int))
#define JOB_LENGTH                      (0x0000011c/sizeof(int))
#define JOB_DATA_READY                  (0x00000120/sizeof(int))
#define JOB_ID                          (0x00000124/sizeof(int))
#define BLOCK_HEADER_VERSION            (0x00000130/sizeof(int))
#define TIME_STAMP                      (0x00000134/sizeof(int))
#define TARGET_BITS                     (0x00000138/sizeof(int))
#define PRE_HEADER_HASH                 (0x00000140/sizeof(int))
#define SHIFT_READY						(0x000001C0/sizeof(int))
#define SHIFT_BUFFER					(0x000001C4/sizeof(int))
#define CHAIN1_3_8_SEND_FIFO_STATUS		(0x00000180/sizeof(int))
#define CHAIN9_12_SEND_FIFO_STATUS		(0x00000184/sizeof(int))
#define CHAIN13_14_SEND_FIFO_STATUS		(0x00000188/sizeof(int))
#define CHAIN1_SEND_READY				(0x00000190/sizeof(int))
#define CHAIN1_SEND_BUFFER				(0x00000194/sizeof(int))
#define CHAIN2_SEND_READY				(0x00000198/sizeof(int))
#define CHAIN2_SEND_BUFFER				(0x0000019C/sizeof(int))
#define CHAIN3_SEND_READY				(0x000001A0/sizeof(int))
#define CHAIN3_SEND_BUFFER				(0x000001A4/sizeof(int))
#define CHAIN8_SEND_READY				(0x000001A8/sizeof(int))
#define CHAIN8_SEND_BUFFER				(0x000001AC/sizeof(int))
#define CHAIN9_SEND_READY				(0x000001B0/sizeof(int))
#define CHAIN9_SEND_BUFFER				(0x000001B4/sizeof(int))
#define CHAIN10_SEND_READY				(0x000001B8/sizeof(int))
#define CHAIN10_SEND_BUFFER				(0x000001BC/sizeof(int))
#define CHAIN11_SEND_READY				(0x000001C0/sizeof(int))
#define CHAIN11_SEND_BUFFER				(0x000001C4/sizeof(int))
#define CHAIN12_SEND_READY				(0x000001C8/sizeof(int))
#define CHAIN12_SEND_BUFFER				(0x000001CC/sizeof(int))
#define CHAIN13_SEND_READY				(0x000001D0/sizeof(int))
#define CHAIN13_SEND_BUFFER				(0x000001D4/sizeof(int))
#define CHAIN14_SEND_READY				(0x000001D8/sizeof(int))
#define CHAIN14_SEND_BUFFER				(0x000001DC/sizeof(int))
#define RECEIVE_FIFO1_2_STATUS			(0x000001F0/sizeof(int))
#define RECEIVE_FIFO3_8_STATUS			(0x000001F4/sizeof(int))
#define RECEIVE_FIFO9_10_STATUS			(0x000001F8/sizeof(int))
#define RECEIVE_FIFO11_12_STATUS		(0x000001FC/sizeof(int))
#define RECEIVE_FIFO13_14_STATUS		(0x00000200/sizeof(int))
#define CHAIN1_READ_ENABLE				(0x00000210/sizeof(int))
#define CHAIN1_READ_DATA				(0x00000214/sizeof(int))
#define CHAIN2_READ_ENABLE				(0x00000218/sizeof(int))
#define CHAIN2_READ_DATA				(0x0000021C/sizeof(int))
#define CHAIN3_READ_ENABLE				(0x00000220/sizeof(int))
#define CHAIN3_READ_DATA				(0x00000224/sizeof(int))
#define CHAIN8_READ_ENABLE				(0x00000228/sizeof(int))
#define CHAIN8_READ_DATA				(0x0000022C/sizeof(int))
#define CHAIN9_READ_ENABLE				(0x00000230/sizeof(int))
#define CHAIN9_READ_DATA				(0x00000234/sizeof(int))
#define CHAIN10_READ_ENABLE				(0x00000238/sizeof(int))
#define CHAIN10_READ_DATA				(0x0000023C/sizeof(int))
#define CHAIN11_READ_ENABLE				(0x00000240/sizeof(int))
#define CHAIN11_READ_DATA				(0x00000244/sizeof(int))
#define CHAIN12_READ_ENABLE				(0x00000248/sizeof(int))
#define CHAIN12_READ_DATA				(0x0000024C/sizeof(int))
#define CHAIN13_READ_ENABLE				(0x00000250/sizeof(int))
#define CHAIN13_READ_DATA				(0x00000254/sizeof(int))
#define CHAIN14_READ_ENABLE				(0x00000258/sizeof(int))
#define CHAIN14_READ_DATA				(0x0000025C/sizeof(int))

//FPGA registers bit map
// HARDWARE_VERSION
#define HW_TYPE_T9_PLUS					(1 << 31)
#define	DHASH_ENGINE_BYPASS				(1 << 29)

//QN_WRITE_DATA_COMMAND
#define RST                             (1 << 31)
#define RESET_ALL                       (1 << 23)
#define CHAIN_ID(id)                    (id << 16)
#define RESET_FPGA                      (1 << 15)
#define RESET_TIME(time)                (time << 0)
#define TIME_OUT_VALID                  (1 << 31)
//RETURN_NONCE
#define WORK_ID_OR_CRC                  (1 << 31)
#define WORK_ID_OR_CRC_VALUE(value)     ((value >> 16) & 0x7fff)
#define NONCE_INDICATOR                 (1 << 7)
#define CHAIN_NUMBER(value)             (value & 0xf)
#define REGISTER_DATA_CRC(value)        ((value >> 24) & 0x7f)
//BC_WRITE_COMMAND
#define BC_COMMAND_BUFFER_READY         (1 << 31)
#define BC_COMMAND_EN_CHAIN_ID          (1 << 23)
#define BC_COMMAND_EN_NULL_WORK         (1 << 22)
//DHASH_ACC_CONTROL
#define VIL_MODE                        (1 << 15)
#define VIL_MIDSTATE_NUMBER(value)      ((value &0x0f) << 8)
#define NEW_BLOCK                       (1 << 7)
#define RUN_BIT                         (1 << 6)
#define OPERATION_MODE                  (1 << 5)
//NONCE_FIFO_INTERRUPT
#define FLUSH_NONCE3_FIFO               (1 << 16)
// i2c command
#define IIC_SELECT(which_i2c)			(which_i2c << 26)
#define IIC_READ_WRITE					(1 << 25)	// read default
#define IIC_REG_ADDR_VALID              (1 << 24)
#define HIGH_4_BITS(bits)				(bits << 20)
#define IIC_COMMAND_CHAIN_NUMBER(which_chain)		(which_chain << 16)
#define IIC_SLAVE_REG_ADDR(addr)		(addr << 8)



/******************** about B3 miner *********************/

#define BITMAIN_MAX_CHAIN_NUM           16

#define TM								0x8
// something about fan
#define BITMAIN_MAX_FAN_NUM             6
#define PWM_PERIOD_NS                   100000
#define MIN_FAN_NUM                     1
#define FAN_WANN_SPEED                  6600
#define FAN1_MAX_SPEED                  6000
#define FAN2_MAX_SPEED                  4300
#define FAN_SPEED_OK_PERCENT            (0.85)
#define MIN_PWM_PERCENT                 40
#define MAX_PWM_PERCENT                 100
#define TEMP_INTERVAL                   2
#define MAX_TEMP                        80
#define MAX_FAN_TEMP                    50
#define MIN_FAN_TEMP                    20
#define PWM_ADJUST_FACTOR               ((100 - MIN_PWM_PERCENT)/(MAX_FAN_TEMP - MIN_FAN_TEMP))
#define FANINT                          1
#define PROCFILENAME                    "/proc/interrupts"


/****************** about B3 miner end *******************/
#define ASIC_NUM_EACH_CHAIN 60



/******************** other MACRO ********************/

#define MAX_RETURNED_NONCE_NUM                      100


#define BITMAIN_MAX_QUEUE_NUM                       128

// swap unsigned int data
#define Swap32(l)                                   (((l) >> 24) | (((l) & 0x00ff0000) >> 8) | (((l) & 0x0000ff00) << 8) | ((l) << 24))
#define Swap16(l)                                   (l >> 8) | ((l & 0xff) << 8)
#define hex_print(p)                                applog(LOG_DEBUG, "%s", p)
#define BYTES_PER_LINE                              0x10
#define INIT_CONFIG_TYPE                            0x51
#define WAIT_REG_VALUE_COUNTER                      4       // every time we check register value, we should wait WAIT_REG_VALUE_COUNTER times to make sure recieve the return value
#define READ_LOOP                                   3       // read temperature sensor times
#define READ_TEMPERATURE_TIME_GAP                   5       // 1s
#define READ_HASH_RATE_TIME_GAP                     5       // 5s

/****************** other MACRO end ******************/


/******************** struct definition ********************/

struct init_config
{
    uint8_t     token_type;
    uint8_t     version;
    uint16_t    length;
    uint32_t    baud;
    uint8_t     reset                   :1;
    uint8_t     fan_eft                 :1;
    uint8_t     timeout_eft             :1;
    uint8_t     frequency_eft           :1;
    uint8_t     voltage_eft             :1;
    uint8_t     chain_check_time_eft    :1;
    uint8_t     chip_config_eft         :1;
    uint8_t     hw_error_eft            :1;
    uint8_t     beeper_ctrl             :1;
    uint8_t     temp_ctrl               :1;
    uint8_t     chain_freq_eft          :1;
    uint8_t     auto_read_temp          :1;
    uint8_t     reserved1               :4;
    uint8_t     reserved2[2];
    uint8_t     chain_num;
    uint8_t     asic_num;
    uint8_t     fan_pwm_percent;
    uint8_t     temperature;
    uint16_t    frequency;
    uint8_t     voltage[2];
    uint8_t     chain_check_time_integer;
    uint8_t     chain_check_time_fractions;
    uint8_t     timeout_data_integer;
    uint8_t     timeout_data_fractions;
    uint32_t    reg_data;
    uint8_t     chip_address;
    uint8_t     reg_address;
    uint16_t    crc;
} __attribute__((packed, aligned(4)));

struct bitmian_B3_info_with_index
{
    struct bitmain_B3_info *info;
    uint8_t chain_index;
}__attribute__((packed, aligned(4)));

struct bitmain_B3_info
{
    cglock_t    update_lock;
    uint8_t     data_type;
    uint8_t     version;
    uint16_t    length;
    uint8_t     chip_value_eft  :1;
    uint8_t     reserved1       :7;
    uint8_t     chain_num;
    uint16_t    reserved2;
    uint8_t     fan_num;
    uint8_t     temp_num;
    uint8_t     reserved3[2];
    uint32_t    fan_exist;
    uint32_t    temp_exist;
    uint16_t    diff;
    uint16_t    reserved4;
    uint32_t    reg_value;
    uint32_t    chain_asic_exist[BITMAIN_MAX_CHAIN_NUM][ASIC_NUM_EACH_CHAIN/32];
    uint32_t    chain_asic_status[BITMAIN_MAX_CHAIN_NUM][ASIC_NUM_EACH_CHAIN/32];
    uint8_t     chain_asic_num[BITMAIN_MAX_CHAIN_NUM];
    uint8_t     temp[BITMAIN_MAX_CHAIN_NUM];
    uint8_t     chain_status[BITMAIN_MAX_CHAIN_NUM];    // 1: chain exist; 0: chain not exist
    uint8_t     fan_speed_value[BITMAIN_MAX_FAN_NUM];
    uint16_t    freq[BITMAIN_MAX_CHAIN_NUM];
    uint32_t    i2c_fd;
    struct work *work_queue[BITMAIN_MAX_QUEUE_NUM];     // store the latest works that sent to Hash boards
    struct thr_info *thr;
    struct thr_info uart_tx_t[BITMAIN_MAX_CHAIN_NUM];
    struct thr_info uart_rx_t[BITMAIN_MAX_CHAIN_NUM];
    pthread_mutex_t lock;
    uint16_t    crc;
} __attribute__((packed, aligned(4)));

struct all_parameters
{

    uint32_t    pwm_value;
    uint32_t    duty_ns;
    uint8_t     chain_exist[BITMAIN_MAX_CHAIN_NUM];     // 1: chain exist; 0: chain not exist
    uint8_t     chain_asic_in_full[BITMAIN_MAX_CHAIN_NUM];
    uint32_t    timeout;
    uint32_t    fan_exist_map;
    uint32_t    temp_sensor_map;
    uint32_t    nonce_error;
    uint32_t    chain_asic_exist[BITMAIN_MAX_CHAIN_NUM][8];
    uint32_t    chain_asic_status[BITMAIN_MAX_CHAIN_NUM][8];
    int16_t     chain_asic_temp[BITMAIN_MAX_CHAIN_NUM][4][2];
    int8_t      chain_asic_iic[ASIC_NUM_EACH_CHAIN];
    uint32_t    chain_hw[BITMAIN_MAX_CHAIN_NUM];
    uint32_t    chain_asic_nonce[BITMAIN_MAX_CHAIN_NUM][ASIC_NUM_EACH_CHAIN];
    char        chain_asic_status_string[BITMAIN_MAX_CHAIN_NUM][ASIC_NUM_EACH_CHAIN + 16];

    uint32_t    total_nonce_num;

    uint32_t    fan_fd[BITMAIN_MAX_FAN_NUM];
    uint8_t     fan_exist[BITMAIN_MAX_FAN_NUM];
    uint32_t    fan_event_count[BITMAIN_MAX_FAN_NUM];
    uint32_t    fan_speed_value[BITMAIN_MAX_FAN_NUM];
    uint32_t    temp[BITMAIN_MAX_CHAIN_NUM][2][2];
    uint8_t     chain_asic_num[BITMAIN_MAX_CHAIN_NUM];
    uint8_t     check_bit;
    uint8_t     pwm_percent;
    uint8_t     chain_num;
    uint8_t     fan_num;
    uint8_t     temp_num;
    uint32_t    fan_speed_top1;
	uint32_t	fan_speed_low1;
    uint32_t    temp_top1;
    uint32_t    temp_top_i[BITMAIN_MAX_CHAIN_NUM];
    uint32_t    temp_top1_last;
    uint8_t     corenum;
    uint8_t     addrInterval;
    uint8_t     max_asic_num_in_one_chain;
    uint8_t     baud;
    uint8_t     diff;
    uint8_t     fan_eft;
    uint8_t     fan_pwm;

    uint16_t    frequency;
    char        frequency_t[10];
    uint16_t    freq[BITMAIN_MAX_CHAIN_NUM];
    uint32_t    i2c_fd;
    struct pollfd pfd[BITMAIN_MAX_FAN_NUM];

} __attribute__((packed, aligned(4)));

typedef enum
{
    BUF_START,
    BUF_READY,
    BUF_READING,
    BUF_WRITING,
    BUF_IDLE
} BUF_STATE_E;


typedef struct
{
    BUF_STATE_E state;
    unsigned char data;
} ASIC_TEMP_T;


struct dev_info
{
    uint32_t    chainid;
};

#define MAX_NONCE_NUMBER_IN_FIFO 512

struct nonce_ctx
{
    uint64_t nonce;
    uint8_t wc;         // Bit[7]: Reserved. Bit[6:0]: work count
    uint8_t tm;
	uint8_t which_asic;
    uint8_t chainid;
} __attribute__((packed, aligned(4)));


struct nonce_buf
{
    uint32_t p_wr;
    uint32_t p_rd;
    uint32_t nonce_num;
    struct nonce_ctx nonce_buffer[MAX_NONCE_NUMBER_IN_FIFO];
} __attribute__((packed, aligned(4)));


struct reg_ctx
{
    uint32_t reg_value;
    uint8_t crc5;       //Bit[7:5]:0, Bit[4:0] crc5
    uint8_t chainid;
} __attribute__((packed, aligned(4)));


struct reg_buf
{
    uint32_t p_wr;
    uint32_t p_rd;
    uint32_t reg_value_num;
    struct reg_ctx reg_buffer[MAX_NONCE_NUMBER_IN_FIFO];
} __attribute__((packed, aligned(4)));


/******************** global variable ********************/

static char nibble[] =
{
    '0', '1', '2', '3', '4', '5', '6', '7',
    '8', '9', 'a', 'b', 'c', 'd', 'e', 'f'
};


/****************** global variable end ******************/


/******************** referencing functions from other files ********************/

extern void rev(unsigned char *s, size_t l);
extern void cg_logwork(struct work *work, unsigned char *nonce_bin, bool ok);

/****************** referencing functions from other files end ******************/

void check_asic_reg(unsigned char which_chain, unsigned char chip_addr, unsigned char reg, bool mode);
void *get_asic_response(void* arg);
void *B3_fill_work(void *usrdata);
void suffix_string_B3(uint64_t val, char *buf, size_t bufsiz, int sigdigits,bool display);
void clear_register_value_buf(void);
void check_sensor_ID(void);
void set_PWM(unsigned char pwm_percent);

extern int opt_bitmain_fan_pwm;       // if not control fan speed according to temperature, use this parameter to control fan speed
extern bool opt_bitmain_fan_ctrl;  // false: control fan speed according to temperature; true: use opt_bitmain_fan_pwm to control fan speed
extern int opt_bitmain_B3_freq;


#endif
