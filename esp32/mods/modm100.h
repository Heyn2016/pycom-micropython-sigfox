#ifndef MODM100_H
#define	MODM100_H

#define HEXIN_MAGICRF_HEAD                         0xBB
#define HEXIN_MAGICRF_TAIL                         0x7E


#define HEXIN_ERROR                                ( 0x00000000 )

#define HEXIN_MAGICRF_BASE                         ( 0x00000001 )
#define HEXIN_MAGICRF_INFO                         ( HEXIN_MAGICRF_BASE <<  0 )
#define HEXIN_MAGICRF_QUERY                        ( HEXIN_MAGICRF_BASE <<  1 )
#define HEXIN_MAGICRF_MUL_QUERY                    ( HEXIN_MAGICRF_BASE <<  2 )
#define HEXIN_MAGICRF_STOP                         ( HEXIN_MAGICRF_BASE <<  3 )
#define HEXIN_MAGICRF_SET_SELECT                   ( HEXIN_MAGICRF_BASE <<  4 )
#define HEXIN_MAGICRF_SEND_SELECT                  ( HEXIN_MAGICRF_BASE <<  5 )
#define HEXIN_MAGICRF_READ_DATA                    ( HEXIN_MAGICRF_BASE <<  6 )
#define HEXIN_MAGICRF_WRITE_DATA                   ( HEXIN_MAGICRF_BASE <<  7 )
#define HEXIN_MAGICRF_LOCK                         ( HEXIN_MAGICRF_BASE <<  8 )
#define HEXIN_MAGICRF_KILL                         ( HEXIN_MAGICRF_BASE <<  9 )
#define HEXIN_MAGICRF_GET_QUERY                    ( HEXIN_MAGICRF_BASE << 10 )
#define HEXIN_MAGICRF_SET_QUERY                    ( HEXIN_MAGICRF_BASE << 11 )
#define HEXIN_MAGICRF_SET_REGION                   ( HEXIN_MAGICRF_BASE << 12 )
#define HEXIN_MAGICRF_INSERT_RF_CHANNEL            ( HEXIN_MAGICRF_BASE << 13 )
#define HEXIN_MAGICRF_SET_RF_CHANNEL               ( HEXIN_MAGICRF_BASE << 14 )
#define HEXIN_MAGICRF_GET_RF_CHANNEL               ( HEXIN_MAGICRF_BASE << 15 )
#define HEXIN_MAGICRF_SET_HFSS                     ( HEXIN_MAGICRF_BASE << 16 )
#define HEXIN_MAGICRF_GET_RF_POWER                 ( HEXIN_MAGICRF_BASE << 17 )
#define HEXIN_MAGICRF_SET_RF_POWER                 ( HEXIN_MAGICRF_BASE << 18 )
#define HEXIN_MAGICRF_SET_RF_CARRIER               ( HEXIN_MAGICRF_BASE << 19 )
#define HEXIN_MAGICRF_GET_RF_GAIN                  ( HEXIN_MAGICRF_BASE << 20 )
#define HEXIN_MAGICRF_SET_RF_GAIN                  ( HEXIN_MAGICRF_BASE << 21 )
#define HEXIN_MAGICRF_TEST_SCANJAMMER              ( HEXIN_MAGICRF_BASE << 22 )
#define HEXIN_MAGICRF_TEST_RSSI                    ( HEXIN_MAGICRF_BASE << 23 )
#define HEXIN_MAGICRF_SET_MODE                     ( HEXIN_MAGICRF_BASE << 24 )
#define HEXIN_MAGICRF_CTRL_IO                      ( HEXIN_MAGICRF_BASE << 25 )
#define HEXIN_MAGICRF_DEEP_SLEEP                   ( HEXIN_MAGICRF_BASE << 26 )
#define HEXIN_MAGICRF_DEEPSLEEP_TIME               ( HEXIN_MAGICRF_BASE << 27 )
#define HEXIN_MAGICRF_IDLE                         ( HEXIN_MAGICRF_BASE << 28 )

#define HEXIN_MAGICRF_NOTHING                      ( HEXIN_MAGICRF_BASE << 30 )
#define HEXIN_MAGICRF_ERROR                        ( 0x10000000 )


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

typedef enum {
    GPIO_PIN_1 = 0x01,
    GPIO_PIN_2,
    GPIO_PIN_3,
    GPIO_PIN_4,
    GPIO_PIN_MAX
} module_gpio_pin_t;

typedef enum {
    GPIO_TYPE_INIT = 0x00,  /* Init GPIO direction  */
    GPIO_TYPE_WRITE,        /* Write value to GPIO  */
    GPIO_TYPE_READ,         /* Read value from GPIO */
    GPIO_TYPE_MAX
} module_gpio_type_t;


unsigned int version        ( unsigned char param,  unsigned char *pbuf );
unsigned int setPaPower     ( float  power,         unsigned char *pbuf );
unsigned int getPaPower     (                       unsigned char *pbuf );
unsigned int query          ( unsigned short loop , unsigned char *pbuf );

unsigned int setMode        ( unsigned char mode,   unsigned char *pbuf );
unsigned int getRFChannel   (                       unsigned char *pbuf );
unsigned int setRFChannel   ( unsigned char index,  unsigned char *pbuf );
unsigned int setHFSS        ( unsigned char status, unsigned char *pbuf );
unsigned int scanJammer     (                       unsigned char *pbuf );
unsigned int testRSSI       (                       unsigned char *pbuf );

unsigned int gpio           ( module_gpio_pin_t  port,
                              module_gpio_type_t type,
                              unsigned char value,
                              unsigned char *pbuf );

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

unsigned int writeEPC(        const unsigned char *pwd,   /* Access password. */
                              unsigned char pwdflag,      /* Password format. 0: Hex format; 1: String format. */
                              unsigned char *dt,          /* Write data buffer. */
                              unsigned short dl,          /* Data length. */
                              unsigned char *pbuf );

unsigned int insertRFChannel( unsigned char start,
                              unsigned char stop,
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
    uint8_t             errorcode;
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

typedef struct {
    unsigned int  length;
    unsigned char data[HEXIN_M100_BUFFER_MAX_SIZE];
} m100_task_rsp_data_t;

#endif /* MODM100_H */
