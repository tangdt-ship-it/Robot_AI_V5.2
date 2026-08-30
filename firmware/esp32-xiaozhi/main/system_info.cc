#include "system_info.h"

#include <cstring>
#include <esp_heap_caps.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_flash.h>
#include <esp_mac.h>
#include <esp_system.h>
#include <esp_partition.h>
#include <esp_app_desc.h>
#include <esp_ota_ops.h>
#include <esp_pm.h>
#if CONFIG_IDF_TARGET_ESP32P4
#include "esp_wifi_remote.h"
#endif

#define TAG "SystemInfo"

namespace {

struct TaskStackProfile {
    const char* name;
    size_t allocated_bytes;
    const char* memory;
};

constexpr TaskStackProfile kTaskStackProfiles[] = {
    {"main", 6144, "INTERNAL"},
    {"sys_evt", 4096, "INTERNAL"},
    {"esp_timer", 3584, "INTERNAL"},
    {"activation", 8192, "INTERNAL"},
    {"acoustic_wifi", 4096, "INTERNAL"},
    {"wifi_cfg_delay", 4096, "INTERNAL"},
    {"ml307_net", 4096, "INTERNAL"},
    {"CameraInitTask", 4096, "INTERNAL"},
    {"audio_input", 5120, "INTERNAL"},
    {"audio_output", 4096, "INTERNAL"},
    {"opus_codec", 24576, "INTERNAL"},
    {"audio_communication", 4096, "INTERNAL"},
    {"audio_detection", 4096, "INTERNAL"},
    {"encode_wake_word", 28672, "PSRAM"},
    {"LedEvent", 2048, "INTERNAL"},
    {"robot_navigation", 10240, "PSRAM"},
    {"robot_return_home", 32768, "PSRAM"},
    {"robot_shadow_scan", 8192, "PSRAM"},
    {"robot_bypass_once", 12288, "PSRAM"},
    {"road_ai_upload", 4608, "INTERNAL"},
    {"robot_proto_recovery", 3072, "INTERNAL"},
    {"robot_uart_rx", 4096, "INTERNAL"},
    {"robot_link_test", 3072, "INTERNAL"},
    {"robot_heartbeat", 2048, "INTERNAL"},
    {"robot_diag_turn", 4096, "INTERNAL"},
    {"robot_diag_console", 4096, "INTERNAL"},
    {"blufi_deinit", 4096, "INTERNAL"},
    {"blufi_wifi_conn", 4096, "INTERNAL"},
    {"map_replay_b1", 4096, "INTERNAL"},
    {"map_replay_b2", 4096, "INTERNAL"},
    {"map_replay_b3", 4096, "INTERNAL"},
    {"map_replay_full", 6144, "INTERNAL"},
    {"map_replay_resume", 6144, "INTERNAL"},
};

const TaskStackProfile* FindTaskStackProfile(const char* name) {
    for (const auto& profile : kTaskStackProfiles) {
        if (std::strcmp(profile.name, name) == 0) return &profile;
    }
    return nullptr;
}

}  // namespace

size_t SystemInfo::GetFlashSize() {
    uint32_t flash_size;
    if (esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get flash size");
        return 0;
    }
    return (size_t)flash_size;
}

size_t SystemInfo::GetMinimumFreeHeapSize() {
    return esp_get_minimum_free_heap_size();
}

size_t SystemInfo::GetFreeHeapSize() {
    return esp_get_free_heap_size();
}

std::string SystemInfo::GetMacAddress() {
    uint8_t mac[6];
#if CONFIG_IDF_TARGET_ESP32P4
    esp_wifi_get_mac(WIFI_IF_STA, mac);
#else
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
#endif
    char mac_str[18];
    snprintf(mac_str, sizeof(mac_str), "%02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return std::string(mac_str);
}

std::string SystemInfo::GetChipModelName() {
    return std::string(CONFIG_IDF_TARGET);
}

std::string SystemInfo::GetUserAgent() {
    auto app_desc = esp_app_get_description();
    auto user_agent = std::string(BOARD_NAME "/") + app_desc->version;
    return user_agent;
}

esp_err_t SystemInfo::PrintTaskCpuUsage(TickType_t xTicksToWait) {
    #define ARRAY_SIZE_OFFSET 5
    TaskStatus_t *start_array = NULL, *end_array = NULL;
    UBaseType_t start_array_size, end_array_size;
    configRUN_TIME_COUNTER_TYPE start_run_time, end_run_time;
    esp_err_t ret;
    uint32_t total_elapsed_time;

    //Allocate array to store current task states
    start_array_size = uxTaskGetNumberOfTasks() + ARRAY_SIZE_OFFSET;
    start_array = (TaskStatus_t*)heap_caps_malloc(
        sizeof(TaskStatus_t) * start_array_size,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (start_array == NULL) {
        ret = ESP_ERR_NO_MEM;
        goto exit;
    }
    //Get current task states
    start_array_size = uxTaskGetSystemState(start_array, start_array_size, &start_run_time);
    if (start_array_size == 0) {
        ret = ESP_ERR_INVALID_SIZE;
        goto exit;
    }

    vTaskDelay(xTicksToWait);

    //Allocate array to store tasks states post delay
    end_array_size = uxTaskGetNumberOfTasks() + ARRAY_SIZE_OFFSET;
    end_array = (TaskStatus_t*)heap_caps_malloc(
        sizeof(TaskStatus_t) * end_array_size,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (end_array == NULL) {
        ret = ESP_ERR_NO_MEM;
        goto exit;
    }
    //Get post delay task states
    end_array_size = uxTaskGetSystemState(end_array, end_array_size, &end_run_time);
    if (end_array_size == 0) {
        ret = ESP_ERR_INVALID_SIZE;
        goto exit;
    }

    //Calculate total_elapsed_time in units of run time stats clock period.
    total_elapsed_time = (end_run_time - start_run_time);
    if (total_elapsed_time == 0) {
        ret = ESP_ERR_INVALID_STATE;
        goto exit;
    }

    printf("| Task | Run Time | Percentage\n");
    //Match each task in start_array to those in the end_array
    for (int i = 0; i < start_array_size; i++) {
        int k = -1;
        for (int j = 0; j < end_array_size; j++) {
            if (start_array[i].xHandle == end_array[j].xHandle) {
                k = j;
                //Mark that task have been matched by overwriting their handles
                start_array[i].xHandle = NULL;
                end_array[j].xHandle = NULL;
                break;
            }
        }
        //Check if matching task found
        if (k >= 0) {
            uint32_t task_elapsed_time = end_array[k].ulRunTimeCounter - start_array[i].ulRunTimeCounter;
            uint32_t percentage_time = (task_elapsed_time * 100UL) / (total_elapsed_time * CONFIG_FREERTOS_NUMBER_OF_CORES);
            printf("| %-16s | %8lu | %4lu%%\n", start_array[i].pcTaskName, task_elapsed_time, percentage_time);
        }
    }

    //Print unmatched tasks
    for (int i = 0; i < start_array_size; i++) {
        if (start_array[i].xHandle != NULL) {
            printf("| %s | Deleted\n", start_array[i].pcTaskName);
        }
    }
    for (int i = 0; i < end_array_size; i++) {
        if (end_array[i].xHandle != NULL) {
            printf("| %s | Created\n", end_array[i].pcTaskName);
        }
    }
    ret = ESP_OK;

exit:    //Common return path
    heap_caps_free(start_array);
    heap_caps_free(end_array);
    return ret;
}

void SystemInfo::PrintTaskList() {
    char buffer[1000];
    vTaskList(buffer);
    ESP_LOGI(TAG, "Task list: \n%s", buffer);
}

void SystemInfo::PrintTaskStackHighWaterMarks() {
    static uint32_t sample_count = 0;
    if (++sample_count % 3 != 0) return;

    const UBaseType_t capacity = uxTaskGetNumberOfTasks() + 5;
    auto* tasks = static_cast<TaskStatus_t*>(heap_caps_malloc(
        sizeof(TaskStatus_t) * capacity, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (tasks == nullptr) {
        ESP_LOGW(TAG, "TASK_STACK sampler allocation failed");
        return;
    }

    const UBaseType_t count = uxTaskGetSystemState(tasks, capacity, nullptr);
    for (UBaseType_t i = 0; i < count; ++i) {
        const TaskStackProfile* profile = FindTaskStackProfile(tasks[i].pcTaskName);
        if (profile == nullptr) continue;

        const size_t min_free_bytes =
            static_cast<size_t>(tasks[i].usStackHighWaterMark) * sizeof(StackType_t);
        const size_t peak_used_bytes = profile->allocated_bytes > min_free_bytes
            ? profile->allocated_bytes - min_free_bytes : 0;
        ESP_LOGI(TAG,
                 "TASK_STACK,NAME=%s,ALLOCATED=%u,MIN_FREE=%u,PEAK_USED=%u,HEADROOM=%u,MEMORY=%s,PRIORITY=%u",
                 tasks[i].pcTaskName,
                 static_cast<unsigned>(profile->allocated_bytes),
                 static_cast<unsigned>(min_free_bytes),
                 static_cast<unsigned>(peak_used_bytes),
                 static_cast<unsigned>(min_free_bytes),
                 profile->memory,
                 static_cast<unsigned>(tasks[i].uxCurrentPriority));
    }
    heap_caps_free(tasks);
}

void SystemInfo::PrintHeapStats() {
    const size_t internal_free = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    const size_t internal_min = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL);
    const size_t internal_largest = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
    const size_t dma_free = heap_caps_get_free_size(MALLOC_CAP_DMA);
    const size_t dma_min = heap_caps_get_minimum_free_size(MALLOC_CAP_DMA);
    const size_t psram_free = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    const size_t psram_min = heap_caps_get_minimum_free_size(MALLOC_CAP_SPIRAM);
    const size_t psram_largest = heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM);

    ESP_LOGI(TAG,
             "HEAP,INTERNAL_FREE=%u,INTERNAL_MIN=%u,INTERNAL_LARGEST=%u,"
             "DMA_FREE=%u,DMA_MIN=%u,PSRAM_FREE=%u,PSRAM_MIN=%u,"
             "PSRAM_LARGEST=%u",
             static_cast<unsigned>(internal_free),
             static_cast<unsigned>(internal_min),
             static_cast<unsigned>(internal_largest),
             static_cast<unsigned>(dma_free),
             static_cast<unsigned>(dma_min),
             static_cast<unsigned>(psram_free),
             static_cast<unsigned>(psram_min),
             static_cast<unsigned>(psram_largest));
    PrintTaskStackHighWaterMarks();
}

void SystemInfo::PrintMemoryCheckpoint(const char* checkpoint) {
    const size_t internal_free = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    const size_t internal_min = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL);
    const size_t internal_largest = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
    const size_t dma_free = heap_caps_get_free_size(MALLOC_CAP_DMA);
    const size_t dma_min = heap_caps_get_minimum_free_size(MALLOC_CAP_DMA);
    const size_t psram_free = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    const size_t psram_min = heap_caps_get_minimum_free_size(MALLOC_CAP_SPIRAM);

    ESP_LOGI(TAG,
             "MEM_CHECKPOINT,NAME=%s,INTERNAL_FREE=%u,INTERNAL_MIN=%u,"
             "INTERNAL_LARGEST=%u,DMA_FREE=%u,DMA_MIN=%u,PSRAM_FREE=%u,"
             "PSRAM_MIN=%u",
             checkpoint != nullptr ? checkpoint : "UNKNOWN",
             static_cast<unsigned>(internal_free),
             static_cast<unsigned>(internal_min),
             static_cast<unsigned>(internal_largest),
             static_cast<unsigned>(dma_free),
             static_cast<unsigned>(dma_min),
             static_cast<unsigned>(psram_free),
             static_cast<unsigned>(psram_min));
}

void SystemInfo::PrintPmLocks() {
    esp_pm_dump_locks(stdout);
}
