#ifndef MODM100_H
#define	MODM100_H

#define HEXIN_MAGICRF_HEAD                         0xBB
#define HEXIN_MAGICRF_TAIL                         0x7E


#define HEXIN_MAGICRF_CMD_INFO                     0x03
#define HEXIN_MAGICRF_CMD_QUERY                    0x22
#define HEXIN_MAGICRF_CMD_MUL_QUERY                0x27
#define HEXIN_MAGICRF_CMD_STOP                     0x28
#define HEXIN_MAGICRF_CMD_SET_SELECT               0x0C
#define HEXIN_MAGICRF_CMD_SEND_SELECT              0x12
#define HEXIN_MAGICRF_CMD_READ_DATA                0x39
#define HEXIN_MAGICRF_CMD_WRITE_DATA               0x49
#define HEXIN_MAGICRF_CMD_LOCK                     0x82
#define HEXIN_MAGICRF_CMD_KILL                     0x65
#define HEXIN_MAGICRF_CMD_GET_QUERY                0x0D
#define HEXIN_MAGICRF_CMD_SET_QUERY                0x0E
#define HEXIN_MAGICRF_CMD_SET_REGION               0x07
#define HEXIN_MAGICRF_CMD_SET_RF_CHANNEL           0xAB
#define HEXIN_MAGICRF_CMD_GET_RF_CHANNEL           0xAA
#define HEXIN_MAGICRF_CMD_SET_HFSS                 0xAD
#define HEXIN_MAGICRF_CMD_GET_RF_POWER             0xB7
#define HEXIN_MAGICRF_CMD_SET_RF_POWER             0xB6
#define HEXIN_MAGICRF_CMD_SET_RF_CARRIER           0xB0
#define HEXIN_MAGICRF_CMD_GET_RF_GAIN              0xF1
#define HEXIN_MAGICRF_CMD_SET_RF_GAIN              0xF0
#define HEXIN_MAGICRF_CMD_TEST_SCANJAMMER          0xF2
#define HEXIN_MAGICRF_CMD_TEST_RSSI                0xF3
#define HEXIN_MAGICRF_CMD_CTRL_IO                  0x1A
#define HEXIN_MAGICRF_CMD_DEEP_SLEEP               0x17
#define HEXIN_MAGICRF_CMD_DEEPSLEEP_TIME           0x1D
#define HEXIN_MAGICRF_CMD_IDLE                     0x04
#define HEXIN_MAGICRF_CMD_ERROR                    0xFF


#define HEXIN_UCHAR2USHORT(msb, lsb)    ((((unsigned short)msb << 8) & 0xFF00) | \
                                         (((unsigned short)lsb << 0) & 0x00FF))

typedef enum {
    HEAD_OFFSET = 0x00,
    TYPE_OFFSET,
    COMMAND_OFFSET,
    LENGTH_MSB_OFFSET,
    LENGTH_LSB_OFFSET,
    PAYLOAD_OFFSET
} request_packet_offset_t;

typedef enum {
    BANK_RFU = 0x00,
    BANK_EPC,
    BANK_TID,
    BANK_USER,
    BANK_MAX
} module_memory_bank_t;


unsigned int setPaPower( float  power,         unsigned char *pbuf );
unsigned int getPaPower(                       unsigned char *pbuf );
unsigned int query     ( unsigned short loop , unsigned char *pbuf );

unsigned int getQueryParam  ( unsigned char *pbuf );
unsigned int setQueryParam  ( unsigned char select,
                              unsigned char session,
                              unsigned char target,
                              unsigned char qvalue,
                              unsigned char *pbuf );

unsigned int stop      (      unsigned char *pbuf );

unsigned int unpackFrame    ( unsigned char *frame,
                              unsigned char *param,
                              unsigned int  *paramLength );

unsigned int setSelectParam ( unsigned char select,      /* Target(3bit) + Action(3bit) + Membank(2bit) */
                              unsigned int  ptr,         /* 4Bytes */
                              const    char *mask,
                              unsigned char maskLen,
                              unsigned char maskflag,    /* Mask format. 0: Hex format; 1: String format. */
                              unsigned char truncate,
                              unsigned char *pbuf );

unsigned int readData(        const unsigned char *pwd,    /* Access password. */
                              unsigned char pwdflag,       /* Password format. 0: Hex format; 1: String format. */
                              module_memory_bank_t bank,   /* Memory bank. */
                              unsigned short sa,           /* Data address offset. */
                              unsigned short dl,           /* Data length. */
                              unsigned char *pbuf );

unsigned int writeData(       const unsigned char *pwd,   /* Access password. */
                              unsigned char pwdflag,      /* Password format. 0: Hex format; 1: String format. */
                              module_memory_bank_t bank,  /* Memory bank. */
                              unsigned short sa,          /* Data address offset. */
                              unsigned short dl,          /* Data length. */
                              unsigned char *dt,          /* Write data buffer. */
                              unsigned char *pbuf );

#define HEXIN_M100_BUFFER_MAX_SIZE          (128)

/******************************************************************************
 DEFINE PRIVATE TYPES
 ******************************************************************************/

typedef struct {
    mp_obj_base_t       base;
    mp_obj_t            handler;
    mp_obj_t            handler_arg;
    uint32_t            trigger;
    bool                init;
    uint8_t             command;
    uint32_t            value_len;
    uint8_t             value[HEXIN_M100_BUFFER_MAX_SIZE];
} m100_obj_t;


typedef enum {
    STATE_HEAD = 0x00,
    STATE_TYPE,
    STATE_COMD,
    STATE_SIZE,
    STATE_DATA,
    STATE_OVER,
} rx_state_machine_t;

#endif /* MODM100_H */
