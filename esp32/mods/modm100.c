#include <stdint.h>
#include <string.h>

#include "py/mpconfig.h"
#include "py/obj.h"
#include "py/runtime.h"
#include "py/objlist.h"
#include "py/stream.h"
#include "py/mphal.h"
#include "pybioctl.h"
#include "py/mperrno.h"
#include "readline.h"
#include "serverstask.h"
#include "mpexception.h"
#include "mpirq.h"

#include "esp_heap_caps.h"
#include "sdkconfig.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "nvs_flash.h"
#include "esp_event.h"

#include "esp_types.h"
#include "esp_attr.h"
#include "esp_intr.h"

#include "driver/uart.h"
#include "driver/gpio.h"

#include "rom/ets_sys.h"
#include "soc/uart_struct.h"
#include "soc/dport_reg.h"
#include "soc/gpio_sig_map.h"


#include "machpin.h"
#include "pins.h"

#include "modm100.h"


#define MICROPY_M100_TX1_PIN                 (&PIN_MODULE_P3)    //GPIO4  -> TX
#define MICROPY_M100_RX1_PIN                 (&PIN_MODULE_P4)    //GPIO15 -> RX

#define MICROPY_M100_TX2_PIN                 (&PIN_MODULE_P8)    //GPIO2  -> TX
#define MICROPY_M100_RX2_PIN                 (&PIN_MODULE_P9)    //GPIO12 -> RX


/******************************************************************************
 DECLARE PRIVATE DATA
 ******************************************************************************/

TaskHandle_t            xM100TaskHandle;
static QueueHandle_t    xRxQueue;

static uint8_t          uart_port           = 1;
static uart_dev_t*      uart_driver_m100    = &UART1;
static uart_config_t    m100_uart_config;

static m100_obj_t m100_obj = { .handler     = mp_const_none,
                               .handler_arg = mp_const_none,
                               .init        = false };

const mp_obj_type_t m100_type;

// this function will be called by the interrupt thread
STATIC void m100_payload_callback_handler(void *arg) {
    m100_obj_t *self = arg;

    if (self->handler && self->handler != mp_const_none) {
        mp_call_function_1(self->handler, self->handler_arg);
    }
}

static void TASK_M100 (void *pvParameters) {
    size_t   length = 0;
    uint8_t  pos    = 0, command = 0;
    uint32_t size   = 0;
    volatile rx_state_machine_t state = STATE_HEAD;
    uint8_t  rx_buff[HEXIN_M100_BUFFER_MAX_SIZE] = { 0x00 };

    while (1) {
        uart_get_buffered_data_len( uart_port, &length );

        if ( 0 == length ) {
            vTaskDelay (2 / portTICK_PERIOD_MS);
            continue;
        }

        // mp_printf(&mp_plat_print, "uart_get_buffered_data_len = %d\n", length);

        switch ( state ) {  /* STATE MACHINE */

            case STATE_HEAD:
                for ( pos=0; pos <length; pos++ ) {
                    uart_read_bytes(uart_port, rx_buff + HEAD_OFFSET, 1, 0);
                    if ( rx_buff[HEAD_OFFSET] == HEXIN_MAGICRF_HEAD ) {
                        state = STATE_TYPE;
                        break;
                    }
                }
                break;

            case STATE_TYPE:
                uart_read_bytes(uart_port, rx_buff + TYPE_OFFSET,    1, 0);
                state = STATE_COMD;
                break;

            case STATE_COMD:
                uart_read_bytes(uart_port, rx_buff + COMMAND_OFFSET, 1, 0);
                state = STATE_SIZE;
                break;

            case STATE_SIZE:
                if ( length >= 2 ) {
                    uart_read_bytes(uart_port, rx_buff + LENGTH_MSB_OFFSET, 2, 0);
                    size  = HEXIN_UCHAR2USHORT(rx_buff[LENGTH_MSB_OFFSET], rx_buff[LENGTH_LSB_OFFSET]);
                    state = STATE_DATA;
                }
                break;

            case STATE_DATA:
                if ( length < (size + 2) ) {
                    break;
                }

                uart_read_bytes(uart_port, rx_buff + PAYLOAD_OFFSET, size + 2, 0);
                command = unpackFrame( rx_buff, m100_obj.value, &size );
                if ( (0 == command) || (0xFF == command) ) {
                    state = STATE_HEAD;
                    break;
                }

                // mp_printf(&mp_plat_print, "trigger = %02X\n", m100_obj.trigger );
                // mp_printf(&mp_plat_print, "command = %02X\n", command );

                if ( ( m100_obj.trigger == command ) || ( HEXIN_MAGICRF_CMD_READ_DATA == command ) ) {
                    m100_obj.value_len = size;
                    mp_irq_queue_interrupt(m100_payload_callback_handler, (void *)&m100_obj);
                }

                switch ( command ) {
                    case HEXIN_MAGICRF_CMD_GET_RF_CHANNEL:
                        m100_obj.command   = HEXIN_MAGICRF_CMD_GET_RF_CHANNEL;
                        m100_obj.value_len = size;
                        mp_irq_queue_interrupt(m100_payload_callback_handler, (void *)&m100_obj);
                        break;

                    case HEXIN_MAGICRF_CMD_TEST_RSSI:
                    case HEXIN_MAGICRF_CMD_TEST_SCANJAMMER:
                        m100_obj.command   = command;
                        m100_obj.value_len = size;
                        mp_irq_queue_interrupt(m100_payload_callback_handler, (void *)&m100_obj);
                        break;

                    default:
                        break;
                }

                state = STATE_HEAD;
                break;

            default:
                state = STATE_HEAD;
                break;
        } /* End STATE MACHINE */
    }
}

/// \method callback(trigger, handler, arg)
STATIC mp_obj_t mod_m100_callback(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    STATIC const mp_arg_t allowed_args[] = {
        { MP_QSTR_trigger,      MP_ARG_REQUIRED | MP_ARG_OBJ,   },
        { MP_QSTR_handler,      MP_ARG_OBJ,     {.u_obj = mp_const_none} },
        { MP_QSTR_arg,          MP_ARG_OBJ,     {.u_obj = mp_const_none} },
    };

    // parse arguments
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(args), allowed_args, args);
    m100_obj_t *self = pos_args[0];

    if ( (args[0].u_obj != mp_const_none) && (args[1].u_obj != mp_const_none) ) {
        self->trigger = mp_obj_get_int(args[0].u_obj);
        self->handler = args[1].u_obj;
        mp_irq_add(self, args[1].u_obj);
        if (args[2].u_obj == mp_const_none) {
            self->handler_arg = self;
        } else {
            self->handler_arg = args[2].u_obj;
        }
    } else {
        self->trigger = 0;
        mp_irq_remove(self);
        INTERRUPT_OBJ_CLEAN(self);
    }

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(mod_m100_callback_obj, 1, mod_m100_callback);


STATIC mp_obj_t mod_m100_rf_power(mp_obj_t self_in, mp_obj_t rfpower) {
    uint32_t ret = 0;
    float    power = mp_obj_get_float(rfpower);
    uint8_t  param[HEXIN_M100_BUFFER_MAX_SIZE] = { 0x00 };

    ret = setPaPower(power, param);
    uart_write_bytes(uart_port, (char*)(param), ret);

    return mp_const_true; // return mp_obj_new_bytes((byte *)&param, ret);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(mod_m100_rf_power_obj, mod_m100_rf_power);


STATIC mp_obj_t mod_m100_query(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_loop,     MP_ARG_INT, {.u_int = 1} },
    };

    uint32_t ret = 0;
    uint8_t  param[HEXIN_M100_BUFFER_MAX_SIZE] = { 0x00 };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(args), allowed_args, args);

    if ( args[0].u_int > 65535 ) {
        mp_raise_ValueError(mpexception_value_invalid_arguments);
    }
    ret = query(args[0].u_int, param);
    uart_write_bytes(uart_port, (char*)(param), ret);
    return mp_const_true;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(mod_m100_query_obj, 1, mod_m100_query);


STATIC mp_obj_t mod_m100_stop( mp_obj_t self_in ) {
    uint32_t ret = 0;
    uint8_t  param[HEXIN_M100_BUFFER_MAX_SIZE] = { 0x00 };

    ret = stop(param);

    uart_write_bytes(uart_port, (char*)(param), ret);
    return mp_const_true;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mod_m100_stop_obj, mod_m100_stop);


STATIC mp_obj_t mod_m100_param(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_select,       MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_session,      MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_target,       MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_q,            MP_ARG_INT, {.u_int = 4} },
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(args), allowed_args, args);

    uint32_t ret = 0;
    uint8_t  param[HEXIN_M100_BUFFER_MAX_SIZE] = { 0x00 };

    ret = setQueryParam(args[0].u_int, args[1].u_int, args[2].u_int, args[3].u_int, param);

    uart_write_bytes(uart_port, (char*)(param), ret);
    return mp_const_true;
}

STATIC MP_DEFINE_CONST_FUN_OBJ_KW(mod_m100_param_obj, 1, mod_m100_param);


STATIC mp_obj_t m100_init_helper(m100_obj_t *self, const mp_arg_val_t *args) {
    // mp_printf(&mp_plat_print, "m100 port = %d\n", args[0].u_int);
    // mp_printf(&mp_plat_print, "m100 baud = %d\n", args[1].u_int);

    uart_port = args[0].u_int;

    if ( (uart_port < 0) || (uart_port > 2) ) {
        uart_port = 1;
    }

    uart_driver_delete(uart_port);

    m100_uart_config.baud_rate = args[1].u_int;
    m100_uart_config.data_bits = UART_DATA_8_BITS;
    m100_uart_config.parity    = UART_PARITY_DISABLE;
    m100_uart_config.stop_bits = UART_STOP_BITS_1;
    m100_uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    m100_uart_config.rx_flow_ctrl_thresh = 64;

    uart_param_config(uart_port, &m100_uart_config);

    pin_deassign(MICROPY_M100_TX1_PIN);
    gpio_pullup_dis(MICROPY_M100_TX1_PIN->pin_number);

    pin_deassign(MICROPY_M100_RX1_PIN);
    gpio_pullup_dis(MICROPY_M100_RX1_PIN->pin_number);

    pin_config(MICROPY_M100_TX1_PIN, -1, U1TXD_OUT_IDX, GPIO_MODE_OUTPUT, MACHPIN_PULL_NONE, 1);
    pin_config(MICROPY_M100_RX1_PIN, U1RXD_IN_IDX, -1, GPIO_MODE_INPUT, MACHPIN_PULL_NONE, 1);

    uart_driver_install(uart_port, 2048, 2048, 0, NULL, 0, NULL);

    switch( uart_port ) {
        case 0:
            uart_driver_m100 = &UART0;
            break;
        case 1:
            uart_driver_m100 = &UART1;
            break;
        case 2:
            uart_driver_m100 = &UART2;
            break;
        default:
            uart_driver_m100 = &UART1;
            break;
    }

    // disable the delay between transfers
    uart_driver_m100->idle_conf.tx_idle_num = 0;

    // configure the rx timeout threshold
    uart_driver_m100->conf1.rx_tout_thrhd = 10 & UART_RX_TOUT_THRHD_V;

    xRxQueue = xQueueCreate(1, 2048);
    xTaskCreatePinnedToCore(TASK_M100, "M100Module", 4096 / sizeof(StackType_t), NULL, 7, &xM100TaskHandle, 1);


    return mp_const_none;
}

STATIC const mp_arg_t m100_init_args[] = {
    { MP_QSTR_id,                                MP_ARG_INT,        {.u_int = 0} },
    { MP_QSTR_port,                              MP_ARG_INT,        {.u_int = 1} },
    { MP_QSTR_baudrate,                          MP_ARG_INT,        {.u_int = 115200} },
};

STATIC mp_obj_t m100_make_new(const mp_obj_type_t *type, mp_uint_t n_args, mp_uint_t n_kw, const mp_obj_t *all_args) {

    // parse args
    mp_map_t kw_args;
    mp_map_init_fixed_table(&kw_args, n_kw, all_args + n_args);
    mp_arg_val_t args[MP_ARRAY_SIZE(m100_init_args)];
    mp_arg_parse_all(n_args, all_args, &kw_args, MP_ARRAY_SIZE(args), m100_init_args, args);

    // check the peripheral id
    if (args[0].u_int != 0) {
        nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, mpexception_os_resource_not_avaliable));
    }

    // setup the object
    m100_obj_t *self = (m100_obj_t *)&m100_obj;
    self->base.type = &m100_type;

    // start the peripheral
    m100_init_helper(self, &args[1]);

    return (mp_obj_t)self;
}

//
STATIC mp_obj_t m100_char_value(mp_obj_t self_in) {
    m100_obj_t *self = self_in;
    return mp_obj_new_bytes(self->value, self->value_len);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mod_m100_value_obj, m100_char_value);

//
STATIC mp_obj_t m100_command_error(mp_obj_t self_in) {
    m100_obj_t *self = self_in;
    return mp_obj_new_int(self->errorcode);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mod_m100_error_obj, m100_command_error);

//
STATIC mp_obj_t mod_m100_size(mp_obj_t self_in) {
    m100_obj_t *self = self_in;
    return mp_obj_new_int(self->value_len);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mod_m100_size_obj, mod_m100_size);


STATIC mp_obj_t mod_m100_select(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_epc,     MP_ARG_REQUIRED | MP_ARG_OBJ, },
        { MP_QSTR_target,  MP_ARG_KW_ONLY  | MP_ARG_INT,  {.u_int  = 0} },
        { MP_QSTR_action,  MP_ARG_KW_ONLY  | MP_ARG_INT,  {.u_int  = 0} },
        { MP_QSTR_bank,    MP_ARG_KW_ONLY  | MP_ARG_INT,  {.u_int  = 1} },
        { MP_QSTR_ptr,     MP_ARG_KW_ONLY  | MP_ARG_INT,  {.u_int  = 0x00000020} },
        { MP_QSTR_truncate,MP_ARG_KW_ONLY  | MP_ARG_BOOL, {.u_bool = false} },
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(args), allowed_args, args);

    const char *epc   = mp_obj_str_get_str(args[0].u_obj);
    uint8_t  target   = (args[1].u_int & 0x07) << 5;    // max 3bit
    uint8_t  action   = (args[2].u_int & 0x07) << 3;    // max 3bit
    uint8_t  bank     = (args[3].u_int & 0x03);         // max 2bit
    uint32_t ptr      = (args[4].u_int);
    uint8_t  truncate = (args[5].u_bool == false) ? 0x00 : 0x01;

    uint32_t ret    = 0;
    uint8_t  param[HEXIN_M100_BUFFER_MAX_SIZE] = { 0x00 };

    ret = setSelectParam( target|action|bank, ptr, epc, strlen(epc), 1, truncate, param );
    uart_write_bytes(uart_port, (char*)(param), ret);

    // return mp_const_false;
    return mp_obj_new_bytes((byte *)&param, ret);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(mod_m100_select_obj, 1, mod_m100_select);


STATIC mp_obj_t mod_m100_read_data(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_bank,    MP_ARG_REQUIRED | MP_ARG_INT,  },
        { MP_QSTR_pwd,     MP_ARG_KW_ONLY  | MP_ARG_OBJ,  {.u_obj  = MP_OBJ_NULL}},
        { MP_QSTR_addr,    MP_ARG_KW_ONLY  | MP_ARG_INT,  {.u_int  = 0x0000} },
        { MP_QSTR_length,  MP_ARG_KW_ONLY  | MP_ARG_INT,  {.u_int  = 0x0002} },
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(args), allowed_args, args);

    uint8_t  bank     = (args[0].u_int & 0x03);
    const char *pwd   = (args[1].u_obj == MP_OBJ_NULL) ? "00000000" : mp_obj_str_get_str(args[1].u_obj);
    uint16_t addr     = (args[2].u_int);
    uint16_t length   = (args[3].u_int);
    volatile unsigned char hexORstr = 1;    // 1 : string type. 0 : hex tpye.

    // TODO : data type is bytes or hex; (2018/10/15)
    if (args[1].u_obj != MP_OBJ_NULL) {     // BUGFIED
        if ( MP_OBJ_IS_TYPE(args[1].u_obj, &mp_type_bytes) ) {
            hexORstr = 0;
        } else {
            hexORstr = 1;
        }
    }
    // END

    if ( strlen(pwd) != 8 ) {
        mp_raise_ValueError("Invalid password, password must be 8 Bytes");
    }

    uint32_t ret    = 0;
    uint8_t  param[HEXIN_M100_BUFFER_MAX_SIZE] = { 0x00 };
    ret = readData( (const unsigned char *)pwd, hexORstr, (module_memory_bank_t)bank, addr, length, param );
    uart_write_bytes(uart_port, (char*)(param), ret);

    return mp_const_true;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(mod_m100_read_data_obj, 1, mod_m100_read_data);


STATIC mp_obj_t mod_m100_write_data(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_bank,    MP_ARG_REQUIRED | MP_ARG_INT,  },
        { MP_QSTR_data,    MP_ARG_REQUIRED | MP_ARG_KW_ONLY | MP_ARG_OBJ,  {.u_obj  = MP_OBJ_NULL} },
        { MP_QSTR_pwd,     MP_ARG_KW_ONLY  | MP_ARG_OBJ,  {.u_obj  = MP_OBJ_NULL} },
        { MP_QSTR_addr,    MP_ARG_KW_ONLY  | MP_ARG_INT,  {.u_int  = 0x0000} },
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(args), allowed_args, args);

    mp_uint_t length  = 0;
    uint8_t  bank     = (args[0].u_int & 0x03);
    const char *data  = mp_obj_str_get_data(args[1].u_obj, &length);
    const char *pwd   = (args[2].u_obj == MP_OBJ_NULL) ? "00000000" : mp_obj_str_get_str(args[2].u_obj);
    uint16_t addr     = (args[3].u_int);
    volatile unsigned char hexORstr = 1;    // 1 : string type. 0 : hex tpye.

    // TODO : password type is bytes or hex; (2018/10/15)
    if (args[2].u_obj != MP_OBJ_NULL) {
        if ( MP_OBJ_IS_TYPE(args[2].u_obj, &mp_type_bytes) ) {
            hexORstr = 0;
        } else {
            hexORstr = 1;
        }
    }
    // END

    if ( strlen(pwd) != 8 ) {
        mp_raise_ValueError("Invalid password, password must be 8 Bytes");
    }

    // mp_printf(&mp_plat_print, "write data length = %d\n", length);
    // if ( (length % 2) != 0 ) {
    //     mp_raise_ValueError("Invalid data, data must be an integer multiple of 2bytes.");
    // }

    uint32_t ret    = 0;
    uint8_t  param[HEXIN_M100_BUFFER_MAX_SIZE] = { 0x00 };
    ret = writeData( (const unsigned char *)pwd, hexORstr, (module_memory_bank_t)bank, addr, length, (unsigned char *)data, param );
    uart_write_bytes(uart_port, (char*)(param), ret);

    return mp_const_true;
}

STATIC MP_DEFINE_CONST_FUN_OBJ_KW(mod_m100_write_data_obj, 1, mod_m100_write_data);


STATIC mp_obj_t mod_m100_write_epc_data(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_epc,     MP_ARG_REQUIRED | MP_ARG_OBJ,  },
        { MP_QSTR_pwd,     MP_ARG_KW_ONLY  | MP_ARG_OBJ,  {.u_obj  = MP_OBJ_NULL} },
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(args), allowed_args, args);

    mp_uint_t epclen  = 0;
    const char *epc   = mp_obj_str_get_data(args[0].u_obj, &epclen);
    const char *pwd   = (args[1].u_obj == MP_OBJ_NULL) ? "00000000" : mp_obj_str_get_str(args[1].u_obj);
    volatile unsigned char hexORstr = 1;    // 1 : string type. 0 : hex tpye.

    // TODO : password type is bytes or hex; (2018/10/15)
    if (args[1].u_obj != MP_OBJ_NULL) {
        if ( MP_OBJ_IS_TYPE(args[1].u_obj, &mp_type_bytes) ) {
            hexORstr = 0;
        } else {
            hexORstr = 1;
        }
    }
    // END

    if ( strlen(pwd) != 8 ) {
        mp_raise_ValueError("Invalid password, password must be 8 Bytes");
    }

    uint32_t ret    = 0;
    uint8_t  param[HEXIN_M100_BUFFER_MAX_SIZE] = { 0x00 };

    ret = writeEPC((const unsigned char *)pwd, hexORstr, (unsigned char *)epc, epclen, param );
    uart_write_bytes(uart_port, (char*)(param), ret);

    return mp_const_true;
}

STATIC MP_DEFINE_CONST_FUN_OBJ_KW(mod_m100_write_epc_data_obj, 1, mod_m100_write_epc_data);


STATIC mp_obj_t mod_m100_mode(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_mode,     MP_ARG_INT, {.u_int = 0} },
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(args), allowed_args, args);

    uint32_t ret  = 0;
    uint8_t  mode = (unsigned char)args[0].u_int & 0xFF;
    uint8_t  param[HEXIN_M100_BUFFER_MAX_SIZE] = { 0x00 };

    ret = setMode(mode, param);
    uart_write_bytes(uart_port, (char*)(param), ret);
    return mp_const_true;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(mod_m100_mode_obj, 1, mod_m100_mode);


STATIC mp_obj_t mod_m100_getchannel( mp_obj_t self_in ) {
    uint32_t ret = 0;
    uint8_t  param[HEXIN_M100_BUFFER_MAX_SIZE] = { 0x00 };

    ret = getRFChannel(param);
    uart_write_bytes(uart_port, (char*)(param), ret);

    return mp_obj_new_bytes((byte *)&param, ret);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mod_m100_getchannel_obj, mod_m100_getchannel);


STATIC mp_obj_t mod_m100_setchannel(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_index,     MP_ARG_INT, {.u_int = 0} },
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(args), allowed_args, args);

    uint32_t ret  = 0;
    uint8_t  index = (unsigned char)args[0].u_int & 0xFF;
    uint8_t  param[HEXIN_M100_BUFFER_MAX_SIZE] = { 0x00 };

    ret = setRFChannel(index, param);
    uart_write_bytes(uart_port, (char*)(param), ret);

    return mp_obj_new_bytes((byte *)&param, ret);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(mod_m100_setchannel_obj, 1, mod_m100_setchannel);

STATIC mp_obj_t mod_m100_hfss(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_mode,     MP_ARG_INT, {.u_int = 0} },
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(args), allowed_args, args);

    uint32_t ret  = 0;
    uint8_t  mode = (unsigned char)args[0].u_int & 0xFF;
    uint8_t  param[HEXIN_M100_BUFFER_MAX_SIZE] = { 0x00 };

    ret = setHFSS(mode, param);
    uart_write_bytes(uart_port, (char*)(param), ret);

    return mp_obj_new_bytes((byte *)&param, ret);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(mod_m100_hfss_obj, 1, mod_m100_hfss);

STATIC mp_obj_t mod_m100_jammer( mp_obj_t self_in ) {
    uint32_t ret = 0;
    uint8_t  param[HEXIN_M100_BUFFER_MAX_SIZE] = { 0x00 };

    ret = scanJammer(param);
    uart_write_bytes(uart_port, (char*)(param), ret);

    return mp_const_true;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mod_m100_jammer_obj, mod_m100_jammer);

STATIC mp_obj_t mod_m100_rssi( mp_obj_t self_in ) {
    uint32_t ret = 0;
    uint8_t  param[HEXIN_M100_BUFFER_MAX_SIZE] = { 0x00 };

    ret = testRSSI(param);
    uart_write_bytes(uart_port, (char*)(param), ret);

    return mp_const_true;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mod_m100_rssi_obj, mod_m100_rssi);


STATIC const mp_map_elem_t m100_locals_dict_table[] = {
    { MP_OBJ_NEW_QSTR(MP_QSTR_power),                   (mp_obj_t)&mod_m100_rf_power_obj        },
    { MP_OBJ_NEW_QSTR(MP_QSTR_query),                   (mp_obj_t)&mod_m100_query_obj           },
    { MP_OBJ_NEW_QSTR(MP_QSTR_stop),                    (mp_obj_t)&mod_m100_stop_obj            },
    { MP_OBJ_NEW_QSTR(MP_QSTR_param),                   (mp_obj_t)&mod_m100_param_obj           },
    { MP_OBJ_NEW_QSTR(MP_QSTR_callback),                (mp_obj_t)&mod_m100_callback_obj        },
    { MP_OBJ_NEW_QSTR(MP_QSTR_value),                   (mp_obj_t)&mod_m100_value_obj           },
    { MP_OBJ_NEW_QSTR(MP_QSTR_error),                   (mp_obj_t)&mod_m100_error_obj           },
    { MP_OBJ_NEW_QSTR(MP_QSTR_size),                    (mp_obj_t)&mod_m100_size_obj            },
    { MP_OBJ_NEW_QSTR(MP_QSTR_select),                  (mp_obj_t)&mod_m100_select_obj          },
    { MP_OBJ_NEW_QSTR(MP_QSTR_read),                    (mp_obj_t)&mod_m100_read_data_obj       },
    { MP_OBJ_NEW_QSTR(MP_QSTR_write),                   (mp_obj_t)&mod_m100_write_data_obj      },
    { MP_OBJ_NEW_QSTR(MP_QSTR_write_epc),               (mp_obj_t)&mod_m100_write_epc_data_obj  },
    { MP_OBJ_NEW_QSTR(MP_QSTR_mode),                    (mp_obj_t)&mod_m100_mode_obj            },
    { MP_OBJ_NEW_QSTR(MP_QSTR_getchannel),              (mp_obj_t)&mod_m100_getchannel_obj      },
    { MP_OBJ_NEW_QSTR(MP_QSTR_setchannel),              (mp_obj_t)&mod_m100_setchannel_obj      },
    { MP_OBJ_NEW_QSTR(MP_QSTR_hfss),                    (mp_obj_t)&mod_m100_hfss_obj            },
    { MP_OBJ_NEW_QSTR(MP_QSTR_jammer),                  (mp_obj_t)&mod_m100_jammer_obj          },
    { MP_OBJ_NEW_QSTR(MP_QSTR_rssi),                    (mp_obj_t)&mod_m100_rssi_obj            },

    // constants
    { MP_OBJ_NEW_QSTR(MP_QSTR_PARAM_SELECT_ALL),        MP_OBJ_NEW_SMALL_INT(0)  },
    { MP_OBJ_NEW_QSTR(MP_QSTR_PARAM_SELECT_NSL),        MP_OBJ_NEW_SMALL_INT(2)  },
    { MP_OBJ_NEW_QSTR(MP_QSTR_PARAM_SELECT_SL),         MP_OBJ_NEW_SMALL_INT(3)  },

    { MP_OBJ_NEW_QSTR(MP_QSTR_PARAM_SESSION_S0),        MP_OBJ_NEW_SMALL_INT(0)  },
    { MP_OBJ_NEW_QSTR(MP_QSTR_PARAM_SESSION_S1),        MP_OBJ_NEW_SMALL_INT(1)  },
    { MP_OBJ_NEW_QSTR(MP_QSTR_PARAM_SESSION_S2),        MP_OBJ_NEW_SMALL_INT(2)  },
    { MP_OBJ_NEW_QSTR(MP_QSTR_PARAM_SESSION_S3),        MP_OBJ_NEW_SMALL_INT(3)  },

    { MP_OBJ_NEW_QSTR(MP_QSTR_PARAM_TARGET_A),          MP_OBJ_NEW_SMALL_INT(0)  },
    { MP_OBJ_NEW_QSTR(MP_QSTR_PARAM_TARGET_B),          MP_OBJ_NEW_SMALL_INT(1)  },

    { MP_OBJ_NEW_QSTR(MP_QSTR_TRIGGER_QUERY),           MP_OBJ_NEW_SMALL_INT(HEXIN_MAGICRF_CMD_QUERY) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_TRIGGER_STOP),            MP_OBJ_NEW_SMALL_INT(HEXIN_MAGICRF_CMD_STOP) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_TRIGGER_PA_POWER),        MP_OBJ_NEW_SMALL_INT(HEXIN_MAGICRF_CMD_SET_RF_POWER) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_TRIGGER_ERROR),           MP_OBJ_NEW_SMALL_INT(HEXIN_MAGICRF_CMD_ERROR) },

    { MP_OBJ_NEW_QSTR(MP_QSTR_BANK_RFU),                MP_OBJ_NEW_SMALL_INT(BANK_RFU)  },
    { MP_OBJ_NEW_QSTR(MP_QSTR_BANK_EPC),                MP_OBJ_NEW_SMALL_INT(BANK_EPC)  },
    { MP_OBJ_NEW_QSTR(MP_QSTR_BANK_TID),                MP_OBJ_NEW_SMALL_INT(BANK_TID)  },
    { MP_OBJ_NEW_QSTR(MP_QSTR_BANK_USER),               MP_OBJ_NEW_SMALL_INT(BANK_USER) },

    { MP_OBJ_NEW_QSTR(MP_QSTR_MODE_HIGH_SENSITIVITY),   MP_OBJ_NEW_SMALL_INT(0)         },
    { MP_OBJ_NEW_QSTR(MP_QSTR_MODE_DENSE_READER),       MP_OBJ_NEW_SMALL_INT(1)         },

    { MP_OBJ_NEW_QSTR(MP_QSTR_HFSS_AUTO),               MP_OBJ_NEW_SMALL_INT(0xFF)      },
    { MP_OBJ_NEW_QSTR(MP_QSTR_HFSS_STOP),               MP_OBJ_NEW_SMALL_INT(0x00)      },
};

STATIC MP_DEFINE_CONST_DICT(m100_locals_dict, m100_locals_dict_table);

const mp_obj_type_t m100_type = {
    { &mp_type_type },
    .name = MP_QSTR_m100,
    .make_new = m100_make_new,
    .locals_dict = (mp_obj_t)&m100_locals_dict,
};

/*****************************************************************************/

STATIC const mp_rom_map_elem_t mp_module_magicrf_globals_table[] = {
    { MP_OBJ_NEW_QSTR(MP_QSTR___name__),    MP_ROM_QSTR(MP_QSTR_umagicrf) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_m100),        (mp_obj_t)&m100_type },
};

STATIC MP_DEFINE_CONST_DICT(mp_module_magicrf_globals, mp_module_magicrf_globals_table);

const mp_obj_module_t mp_module_magicrf = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&mp_module_magicrf_globals,
};

/*
from magicrf import m100

reader = m100()

def uart_cb( char ):
    # print(char.size())
    print(char.value())


reader.callback(m100.TRIGGER_QUERY, uart_cb)

reader.power(22.0)
reader.mode(m100.MODE_HIGH_SENSITIVITY)
reader.mode(m100.MODE_DENSE_READER)

reader.query()
reader.query(100)

reader.select("52533A535349443132333435")
reader.read(m100.BANK_USER, length=8)
reader.write(m100.BANK_USER,data=b"\xf4Y4\x00RS:SSID\x31\x32\x33\x34\x35")
reader.write_epc("000000000000")

reader.rssi()
reader.jammer()

reader.hfss(m100.HFSS_STOP)
reader.hfss(m100.HFSS_AUTO)
reader.setchannel(1)
reader.getchannel()

*/
