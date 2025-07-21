#pragma once

#include "ms.h"

#include <esp_err.h>

#ifdef __cplusplus
namespace sd {
    esp_err_t init();
    esp_err_t create_new_file();
    esp_err_t finalize_file();
    esp_err_t append_result(const ms::export_data_t* data);
    void deinit();
}
#endif