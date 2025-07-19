/* SD card and FAT filesystem example.
   This example uses SPI peripheral to communicate with SD card.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include "my_sdcard.h"
#include "main.h"

#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include <esp_vfs_fat.h>
#include <sdmmc_cmd.h>
#include <esp_log.h>
#include <driver/gpio.h>

static const char *TAG = "SD";

// Pin assignments can be set in menuconfig, see "SD SPI Example Configuration" menu.
// You can also change the pin assignments here by changing the following 4 lines.
#define PIN_NUM_MISO GPIO_NUM_2
#define PIN_NUM_MOSI GPIO_NUM_15
#define PIN_NUM_CLK GPIO_NUM_14
#define PIN_NUM_CS GPIO_NUM_13

#define MOUNT_POINT "/sd"
#define MAX_FILES 10

namespace sd
{
    static sdmmc_host_t host;
    static sdmmc_card_t *card;
    static const char mount_point[] = MOUNT_POINT;
    static char file_path[32];

    esp_err_t init()
    {
        esp_err_t ret;

        // Options for mounting the filesystem.
        // If format_if_mount_failed is set to true, SD card will be partitioned and
        // formatted in case when mounting fails.
        esp_vfs_fat_sdmmc_mount_config_t mount_config = {
            .format_if_mount_failed = true,
            .max_files = MAX_FILES,
            .allocation_unit_size = 16 * 1024};

        ESP_LOGI(TAG, "Initializing SD card");

        // Use settings defined above to initialize SD card and mount FAT filesystem.
        // Note: esp_vfs_fat_sdmmc/sdspi_mount is all-in-one convenience functions.
        // Please check its source code and implement error recovery when developing
        // production applications.
        ESP_LOGD(TAG, "Using SPI peripheral");

        // By default, SD card frequency is initialized to SDMMC_FREQ_DEFAULT (20MHz)
        // For setting a specific frequency, use host.max_freq_khz (range 400kHz - 20MHz for SDSPI)
        // Example: for fixed frequency of 10MHz, use host.max_freq_khz = 10000;
        host = SDSPI_HOST_DEFAULT();

        spi_bus_config_t bus_cfg = {
            .mosi_io_num = PIN_NUM_MOSI,
            .miso_io_num = PIN_NUM_MISO,
            .sclk_io_num = PIN_NUM_CLK,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = 4000};

        ret = spi_bus_initialize((spi_host_device_t)(host.slot), &bus_cfg, SDSPI_DEFAULT_DMA);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to initialize bus.");
            return ret;
        }

        // This initializes the slot without card detect (CD) and write protect (WP) signals.
        // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
        sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
        slot_config.gpio_cs = PIN_NUM_CS;
        slot_config.host_id = (spi_host_device_t)(host.slot);

        ESP_LOGI(TAG, "Mounting filesystem");
        ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);

        if (ret != ESP_OK)
        {
            if (ret == ESP_FAIL)
            {
                ESP_LOGE(TAG, "Failed to mount filesystem. "
                              "If you want the card to be formatted, set the CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
            }
            else
            {
                ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                              "Make sure SD card lines have pull-up resistors in place.",
                         esp_err_to_name(ret));
            }
            return ret;
        }
        ESP_LOGI(TAG, "Filesystem mounted");

        // Use POSIX and C standard library functions to work with files.
        // Find first non-existing file name for this session
        struct stat st;
        int index = 0;
        do
        {
            snprintf(file_path, sizeof(file_path), MOUNT_POINT "/%i.txt", ++index);
            if (index > MAX_FILES) return ESP_ERR_INVALID_SIZE;
        } while (stat(file_path, &st) == 0);
        // Create and open a new file
        ESP_LOGI(TAG, "Opening file %s", file_path);
        FILE *f = fopen(file_path, "w");
        if (f == NULL)
        {
            ESP_LOGE(TAG, "Failed to open file for writing");
            return ESP_FAIL;
        }
        fprintf(f, "{ \"Temp\", \"Conc\" },\n[\n");
        fclose(f);
        ESP_LOGI(TAG, "File created");

        return ret;
    }

    esp_err_t append_result(const ms::export_data_t *data)
    {
        FILE *f = fopen(file_path, "a");
        if (f == NULL)
            return ESP_FAIL;
        fprintf(f, RESULTS_FORMAT, data->heater_temperature, data->concentration);
        fclose(f);
        return ESP_OK;
    }

    void deinit()
    {
        // All done, unmount partition and disable SPI peripheral
        esp_vfs_fat_sdcard_unmount(mount_point, card);
        ESP_LOGI(TAG, "Card unmounted");

        // deinitialize the bus after all devices are removed
        spi_bus_free((spi_host_device_t)(host.slot));
    }
} // namespace sd