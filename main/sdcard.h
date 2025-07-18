#pragma once

#include "ms.h"

#include <esp_err.h>

namespace sd {
    esp_err_t init();
    esp_err_t append_result(const ms::export_data_t* data);
    void deinit();
}