#include <stdint.h>

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

#include "esp_heap_caps.h"
#include "sdkconfig.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "nvs_flash.h"
#include "esp_event.h"

#include "esp_types.h"
#include "esp_attr.h"
#include "esp_intr.h"

#include "rom/ets_sys.h"
#include "soc/uart_struct.h"
#include "soc/dport_reg.h"
#include "soc/gpio_sig_map.h"


#include "modm100.h"

#define HEXIN_M100_BUFFER_MAX_SIZE         (128)

STATIC mp_obj_t mod_m100_rf_power(size_t n_args, const mp_obj_t *args) {
    uint32_t ret = 0;
    uint8_t  param[HEXIN_M100_BUFFER_MAX_SIZE] = { 0x00 };

    if (n_args == 0) {
        ret = getRFPower(param);
    } else {
        ret = setRFPower(mp_obj_get_int(args[0]), param);
    }

    return mp_obj_new_bytes((byte *)&param, ret);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_m100_rf_power_obj, 0, 1, mod_m100_rf_power);


STATIC mp_obj_t mod_m100_query(size_t n_args, const mp_obj_t *args) {
    uint32_t ret = 0;
    uint8_t  param[HEXIN_M100_BUFFER_MAX_SIZE] = { 0x00 };

    if (n_args == 0) {
        ret = query(1, param);
    } else {
        uint32_t loop = mp_obj_get_int(args[0]);
        if (loop > 65535) {
            mp_raise_ValueError(mpexception_value_invalid_arguments);
        }
        ret = query(loop, param);
    }

    return mp_obj_new_bytes((byte *)&param, ret);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_m100_query_obj, 0, 1, mod_m100_query);


STATIC mp_obj_t mod_m100_stop( void ) {
    uint32_t ret = 0;
    uint8_t  param[HEXIN_M100_BUFFER_MAX_SIZE] = { 0x00 };

    ret = stop(param);

    return mp_obj_new_bytes((byte *)&param, ret);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mod_m100_stop_obj, mod_m100_stop);


STATIC mp_obj_t mod_m100_param(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_select,       MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_session,      MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_target,       MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_qvalue,       MP_ARG_INT, {.u_int = 4} },
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(args), allowed_args, args);

    uint32_t ret = 0;
    uint8_t  param[HEXIN_M100_BUFFER_MAX_SIZE] = { 0x00 };

    ret = setQueryParam(args[0].u_int, args[1].u_int, args[2].u_int, args[3].u_int, param);

    return mp_obj_new_bytes((byte *)&param, ret);
}

STATIC MP_DEFINE_CONST_FUN_OBJ_KW(mod_m100_param_obj, 0, mod_m100_param);


STATIC const mp_rom_map_elem_t mp_module_m100_globals_table[] = {
    { MP_OBJ_NEW_QSTR(MP_QSTR___name__),    MP_ROM_QSTR(MP_QSTR_m100) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_power),       MP_ROM_PTR(&mod_m100_rf_power_obj) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_query),       MP_ROM_PTR(&mod_m100_query_obj) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_stop),        MP_ROM_PTR(&mod_m100_stop_obj)  },
    { MP_OBJ_NEW_QSTR(MP_QSTR_param),       MP_ROM_PTR(&mod_m100_param_obj) },
};

STATIC MP_DEFINE_CONST_DICT(mp_module_m100_globals, mp_module_m100_globals_table);

const mp_obj_module_t mp_module_m100 = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&mp_module_m100_globals,
};