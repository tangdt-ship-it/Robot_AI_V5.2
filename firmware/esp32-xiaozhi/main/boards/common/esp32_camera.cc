#include "sdkconfig.h"

#include <esp_heap_caps.h>
#include <cstdio>
#include <cstring>
#include <esp_log.h>
#include <img_converters.h>

#include "esp32_camera.h"
#include "board.h"
#include "display.h"
#include "lvgl_display.h"
#include "mcp_server.h"
#include "system_info.h"
#include "jpg/image_to_jpeg.h"
#include "esp_timer.h"
#include <algorithm>
#include <atomic>
#include <cmath>
#include <chrono>

#define TAG "Esp32Camera"

namespace {
struct ConsoleJpegStats {
    size_t bytes = 0;
    size_t chunks = 0;
};

void LogVisionMemory(const char* stage) {
    ESP_LOGI(TAG,
             "VISION_MEM,STAGE=%s,INTERNAL_FREE=%u,INTERNAL_LARGEST=%u,PSRAM_FREE=%u,PSRAM_LARGEST=%u,STACK_HIGH_WATER=%u",
             stage,
             static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL)),
             static_cast<unsigned>(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL)),
             static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_SPIRAM)),
             static_cast<unsigned>(heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM)),
             static_cast<unsigned>(uxTaskGetStackHighWaterMark(nullptr)));
}

// HttpClient::Write() returns the complete HTTP chunk on-wire size when
// Transfer-Encoding is chunked: "<hex length>\\r\\n<payload>\\r\\n".
// EspTcp::Send() either sends that complete string or returns an error.
size_t ChunkedWireSize(size_t payload_size) {
    // Do not use printf's z length modifier here: the target's compact C
    // formatter can report the literal "zx" length for a large size_t.
    size_t hex_digits = 1;
    for (size_t value = payload_size; value >= 16; value >>= 4) {
        ++hex_digits;
    }
    return payload_size + hex_digits + 4U;
}

void PrintBase64Chunk(const uint8_t* data, size_t length) {
    static constexpr char kAlphabet[] =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    char encoded[245] = {};
    size_t output = 0;
    for (size_t input = 0; input < length; input += 3) {
        const uint32_t a = data[input];
        const uint32_t b = input + 1 < length ? data[input + 1] : 0;
        const uint32_t c = input + 2 < length ? data[input + 2] : 0;
        const uint32_t value = (a << 16) | (b << 8) | c;
        encoded[output++] = kAlphabet[(value >> 18) & 0x3f];
        encoded[output++] = kAlphabet[(value >> 12) & 0x3f];
        encoded[output++] = input + 1 < length
                                ? kAlphabet[(value >> 6) & 0x3f]
                                : '=';
        encoded[output++] = input + 2 < length ? kAlphabet[value & 0x3f] : '=';
    }
    encoded[output] = '\0';
    std::printf("CAMERA_JPEG_DATA:%s\n", encoded);
    std::fflush(stdout);
    vTaskDelay(1);
}

// Center-crop the camera frame to the LCD aspect ratio and resample it to the
// exact panel size. The full source frame remains available to the AI encoder.
uint8_t* MakeDisplayPreview(const camera_fb_t* fb, int output_width,
                            int output_height, bool swap_bytes,
                            size_t* output_size) {
    if (fb == nullptr ||
        (fb->format != PIXFORMAT_RGB565 &&
         fb->format != PIXFORMAT_YUV422) ||
        output_width <= 0 || output_height <= 0 || output_size == nullptr) {
        return nullptr;
    }
    *output_size = static_cast<size_t>(output_width) * output_height * 2U;
    auto* output = static_cast<uint16_t*>(heap_caps_malloc(
        *output_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (output == nullptr) return nullptr;

    int crop_width = fb->width;
    int crop_height = static_cast<int>(
        (static_cast<int64_t>(crop_width) * output_height) / output_width);
    if (crop_height > fb->height) {
        crop_height = fb->height;
        crop_width = static_cast<int>(
            (static_cast<int64_t>(crop_height) * output_width) /
            output_height);
    }
    const int crop_x = (fb->width - crop_width) / 2;
    const int crop_y = (fb->height - crop_height) / 2;
    auto read_pixel = [fb, swap_bytes](int x, int y) -> uint16_t {
        if (fb->format == PIXFORMAT_RGB565) {
            const auto* source = reinterpret_cast<const uint16_t*>(fb->buf);
            uint16_t value = source[static_cast<size_t>(y) * fb->width + x];
            return swap_bytes ? __builtin_bswap16(value) : value;
        }

        const size_t pair_offset =
            (static_cast<size_t>(y) * fb->width + (x & ~1)) * 2U;
        const int y_value = fb->buf[pair_offset + ((x & 1) ? 2U : 0U)];
        const int u_value = fb->buf[pair_offset + 1U] - 128;
        const int v_value = fb->buf[pair_offset + 3U] - 128;
        const int luminance = std::max(0, y_value - 16);
        const int red = std::clamp(
            (298 * luminance + 409 * v_value + 128) >> 8, 0, 255);
        const int green = std::clamp(
            (298 * luminance - 100 * u_value - 208 * v_value + 128) >> 8,
            0, 255);
        const int blue = std::clamp(
            (298 * luminance + 516 * u_value + 128) >> 8, 0, 255);
        return static_cast<uint16_t>(((red & 0xf8) << 8) |
                                     ((green & 0xfc) << 3) |
                                     (blue >> 3));
    };

    for (int y = 0; y < output_height; ++y) {
        const uint32_t sy_fp = output_height > 1
            ? static_cast<uint32_t>(
                  (static_cast<uint64_t>(y) * (crop_height - 1) << 8) /
                  (output_height - 1))
            : 0U;
        const int sy = crop_y + static_cast<int>(sy_fp >> 8);
        const int sy1 = std::min(sy + 1, crop_y + crop_height - 1);
        const uint32_t fy = sy_fp & 0xFFU;
        for (int x = 0; x < output_width; ++x) {
            const uint32_t sx_fp = output_width > 1
                ? static_cast<uint32_t>(
                      (static_cast<uint64_t>(x) * (crop_width - 1) << 8) /
                      (output_width - 1))
                : 0U;
            const int sx = crop_x + static_cast<int>(sx_fp >> 8);
            const int sx1 = std::min(sx + 1, crop_x + crop_width - 1);
            const uint32_t fx = sx_fp & 0xFFU;
            const uint16_t p00 = read_pixel(sx, sy);
            const uint16_t p10 = read_pixel(sx1, sy);
            const uint16_t p01 = read_pixel(sx, sy1);
            const uint16_t p11 = read_pixel(sx1, sy1);
            auto blend = [fx, fy](uint32_t c00, uint32_t c10,
                                  uint32_t c01, uint32_t c11) -> uint32_t {
                const uint32_t top = c00 * (256U - fx) + c10 * fx;
                const uint32_t bottom = c01 * (256U - fx) + c11 * fx;
                return (top * (256U - fy) + bottom * fy + 32768U) >> 16;
            };
            const uint32_t red = blend((p00 >> 11) & 0x1FU,
                                       (p10 >> 11) & 0x1FU,
                                       (p01 >> 11) & 0x1FU,
                                       (p11 >> 11) & 0x1FU);
            const uint32_t green = blend((p00 >> 5) & 0x3FU,
                                         (p10 >> 5) & 0x3FU,
                                         (p01 >> 5) & 0x3FU,
                                         (p11 >> 5) & 0x3FU);
            const uint32_t blue = blend(p00 & 0x1FU, p10 & 0x1FU,
                                        p01 & 0x1FU, p11 & 0x1FU);
            output[static_cast<size_t>(y) * output_width + x] =
                static_cast<uint16_t>((red << 11) | (green << 5) | blue);
        }
    }
    uint64_t luminance_sum = 0;
    uint64_t edge_sum = 0;
    for (int y = 0; y < output_height; ++y) {
        uint32_t previous_luma = 0;
        for (int x = 0; x < output_width; ++x) {
            const uint16_t pixel = output[static_cast<size_t>(y) * output_width + x];
            const uint32_t red = ((pixel >> 11) & 0x1FU) * 255U / 31U;
            const uint32_t green = ((pixel >> 5) & 0x3FU) * 255U / 63U;
            const uint32_t blue = (pixel & 0x1FU) * 255U / 31U;
            const uint32_t luma = (77U * red + 150U * green + 29U * blue) >> 8;
            luminance_sum += luma;
            if (x != 0) edge_sum += luma > previous_luma
                ? luma - previous_luma : previous_luma - luma;
            previous_luma = luma;
        }
    }
    const uint32_t mean_luma = static_cast<uint32_t>(
        luminance_sum / (static_cast<uint64_t>(output_width) * output_height));
    const uint32_t edge_score = static_cast<uint32_t>(
        edge_sum / (static_cast<uint64_t>(output_width - 1) * output_height));
    ESP_LOGI(TAG,
             "Preview center-crop: %dx%d crop=%dx%d@%d,%d -> %dx%d luma=%lu edge=%lu",
             fb->width, fb->height, crop_width, crop_height, crop_x, crop_y,
             output_width, output_height,
             static_cast<unsigned long>(mean_luma),
             static_cast<unsigned long>(edge_score));
    return reinterpret_cast<uint8_t*>(output);
}
}  // namespace

Esp32Camera::Esp32Camera(const camera_config_t &config) {
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_camera_init failed with error 0x%x", err);
        return;
    }

    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        if (s->id.PID == GC0308_PID) {
            s->set_hmirror(s, 0); // Control camera mirror: 1 for mirror, 0 for normal
        }
        // Natural-color profile for OV2640. Its driver does not implement
        // set_sharpness()/set_denoise(), so real detail comes from resolution,
        // optical focus, low sensor gain and high-quality JPEG encoding.
        int profile_errors = 0;
        auto apply = [&profile_errors](int result) {
            if (result != 0) ++profile_errors;
        };
        apply(s->set_brightness(s, 0));
        apply(s->set_contrast(s, 0));
        apply(s->set_saturation(s, 0));
        apply(s->set_special_effect(s, 0));
        apply(s->set_whitebal(s, 1));
        apply(s->set_awb_gain(s, 1));
        apply(s->set_wb_mode(s, 0));
        apply(s->set_exposure_ctrl(s, 1));
        apply(s->set_aec2(s, 1));
        apply(s->set_ae_level(s, 0));
        apply(s->set_gain_ctrl(s, 1));
        apply(s->set_gainceiling(s, GAINCEILING_8X));
        apply(s->set_raw_gma(s, 1));
        apply(s->set_bpc(s, 1));
        apply(s->set_wpc(s, 1));
        apply(s->set_lenc(s, 1));
        apply(s->set_dcw(s, 1));
        ESP_LOGI(TAG,
                 "Camera initialized: format=%d frame=%d profile=natural awb/aec/agc/gamma/lenc=1 gain_ceiling=8x profile_errors=%d sharpness=optical",
                 config.pixel_format, config.frame_size, profile_errors);
    }

    streaming_on_ = true;
}

Esp32Camera::~Esp32Camera() {
    if (streaming_on_) {
        if (current_fb_) {
            esp_camera_fb_return(current_fb_);
            current_fb_ = nullptr;
        }
        if (encode_buf_) {
            heap_caps_free(encode_buf_);
            encode_buf_ = nullptr;
            encode_buf_size_ = 0;
        }
        esp_camera_deinit();
        streaming_on_ = false;
    }
}

void Esp32Camera::SetExplainUrl(const std::string &url, const std::string &token) {
    explain_url_ = url;
    explain_token_ = token;
    ESP_LOGI(TAG, "VISION_CAPABILITY,VISION_URL=%s,TOKEN=%s",
             url.empty() ? "EMPTY" : "SET",
             token.empty() ? "EMPTY" : "SET");
}

bool Esp32Camera::Capture() {
    LogVisionMemory("BEFORE_CAPTURE");
    std::lock_guard<std::recursive_timed_mutex> lock(camera_mutex_);
    return CaptureLocked();
}

bool Esp32Camera::CaptureLocked() {
    if (encoder_thread_.joinable()) {
        encoder_thread_.join();
    }

    if (!streaming_on_) {
        return false;
    }

    // Get the latest frame, discard old frames for real-time performance
    for (int i = 0; i < 2; i++) {
        if (current_fb_) {
            esp_camera_fb_return(current_fb_);
        }
        current_fb_ = esp_camera_fb_get();
        if (!current_fb_) {
            ESP_LOGE(TAG, "Camera capture failed");
            return false;
        }
    }

    // Prepare encode buffer for RGB565 format (with optional byte swapping)
    if (current_fb_->format == PIXFORMAT_RGB565) {
        size_t pixel_count = current_fb_->width * current_fb_->height;
        size_t data_size = pixel_count * 2;

        // Allocate or reallocate encode buffer if needed
        if (encode_buf_size_ < data_size) {
            if (encode_buf_) {
                heap_caps_free(encode_buf_);
            }
            encode_buf_ = (uint8_t *)heap_caps_malloc(data_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
            if (encode_buf_ == nullptr) {
                ESP_LOGE(TAG, "Failed to allocate memory for encode buffer");
                encode_buf_size_ = 0;
                return false;
            }
            encode_buf_size_ = data_size;
        }

        // Copy data to encode buffer with optional byte swapping
        uint16_t *src = (uint16_t *)current_fb_->buf;
        uint16_t *dst = (uint16_t *)encode_buf_;
        if (swap_bytes_enabled_) {
            for (size_t i = 0; i < pixel_count; i++) {
                dst[i] = __builtin_bswap16(src[i]);
            }
        } else {
            memcpy(encode_buf_, current_fb_->buf, data_size);
        }

    } else if (current_fb_->format == PIXFORMAT_JPEG) {
        // JPEG format preview usually requires decoding, skip preview display for now, just log
        ESP_LOGW(TAG, "JPEG capture success, len=%zu, but not supported for preview", current_fb_->len);
    }

    // Create a display-native preview from either RGB565 or clean YUYV data.
    auto display = dynamic_cast<LvglDisplay *>(Board::GetInstance().GetDisplay());
    if (display != nullptr &&
        (current_fb_->format == PIXFORMAT_RGB565 ||
         current_fb_->format == PIXFORMAT_YUV422)) {
        size_t preview_size = 0;
        uint8_t* preview_data = MakeDisplayPreview(
            current_fb_, display->width(), display->height(),
            swap_bytes_enabled_, &preview_size);
        if (preview_data != nullptr) {
            display->SetPreviewImage(std::make_unique<LvglAllocatedImage>(
                preview_data, preview_size, display->width(),
                display->height(), display->width() * 2,
                LV_COLOR_FORMAT_RGB565));
        } else {
            ESP_LOGE(TAG, "Failed to create full-screen camera preview");
        }
    }

    ESP_LOGI(TAG, "Captured frame: %dx%d, len=%lu, format=%d",
             current_fb_->width, current_fb_->height,
             static_cast<unsigned long>(current_fb_->len), current_fb_->format);
    LogVisionMemory("AFTER_CAPTURE");

    return true;
}

bool Esp32Camera::AnalyzeCurrentFrame(CameraNavigationMetrics& metrics) const {
    metrics = {};
    if (current_fb_ == nullptr ||
        (current_fb_->format != PIXFORMAT_RGB565 &&
         current_fb_->format != PIXFORMAT_YUV422) ||
        current_fb_->width < 24 || current_fb_->height < 24) {
        return false;
    }

    struct RegionStats {
        uint64_t luma = 0;
        uint64_t edge = 0;
        uint32_t count = 0;
    } regions[3];

    const int width = current_fb_->width;
    const int height = current_fb_->height;
    auto luma_at = [this, width](int x, int y) -> int {
        if (current_fb_->format == PIXFORMAT_YUV422) {
            const size_t pair_offset =
                (static_cast<size_t>(y) * width + (x & ~1)) * 2U;
            return current_fb_->buf[pair_offset + ((x & 1) ? 2U : 0U)];
        }
        const auto* pixels =
            reinterpret_cast<const uint16_t*>(current_fb_->buf);
        uint16_t pixel = pixels[static_cast<size_t>(y) * width + x];
        if (swap_bytes_enabled_) pixel = __builtin_bswap16(pixel);
        const int red = ((pixel >> 11) & 0x1f) * 255 / 31;
        const int green = ((pixel >> 5) & 0x3f) * 255 / 63;
        const int blue = (pixel & 0x1f) * 255 / 31;
        return (77 * red + 150 * green + 29 * blue) >> 8;
    };

    constexpr int kStep = 8;
    const int first_y = height * 2 / 5;
    
    // Track traversability, edge density, dark/bright obstacles
    for (int y = first_y + kStep; y < height - kStep; y += kStep) {
        for (int x = kStep; x < width - kStep; x += kStep) {
            const int region = std::min(2, (x * 3) / width);
            const int luma = luma_at(x, y);
            const int horizontal = std::abs(luma - luma_at(x - kStep, y));
            const int vertical = std::abs(luma - luma_at(x, y - kStep));
            regions[region].luma += luma;
            regions[region].edge += horizontal + vertical;
            ++regions[region].count;
        }
    }

    float scores[3] = {};
    float confidence_sum = 0.0f;
    float edge_density_sum = 0.0f;
    unsigned total_samples = 0;
    unsigned dark_sample_count = 0;
    unsigned high_contrast_count = 0;
    
    for (int i = 0; i < 3; ++i) {
        if (regions[i].count == 0) return false;
        const float mean_luma = static_cast<float>(regions[i].luma) /
                                regions[i].count;
        const float mean_edge = static_cast<float>(regions[i].edge) /
                                regions[i].count;
        const float brightness_quality = std::max(
            0.0f, 100.0f - std::fabs(mean_luma - 128.0f) * 0.78f);
        const float smoothness = std::max(0.0f,
                                          100.0f - mean_edge * 1.35f);
        scores[i] = std::clamp(0.72f * smoothness +
                                   0.28f * brightness_quality,
                               0.0f, 100.0f);
        confidence_sum += std::clamp(20.0f + mean_edge * 1.6f,
                                     20.0f, 100.0f);
        edge_density_sum += mean_edge;
        total_samples += regions[i].count;
        
        // Detect dark obstacles (mean_luma < 60)
        if (mean_luma < 60.0f) dark_sample_count += regions[i].count;
        
        // Detect high contrast edges (mean_edge > 40)
        if (mean_edge > 40.0f) high_contrast_count += regions[i].count;
    }

    metrics.valid = true;
    metrics.left_open_pct = scores[0];
    metrics.center_open_pct = scores[1];
    metrics.right_open_pct = scores[2];
    metrics.corridor_offset = std::clamp((scores[2] - scores[0]) / 100.0f,
                                         -1.0f, 1.0f);
    metrics.confidence_pct = confidence_sum / 3.0f;
    metrics.sampled_pixels = total_samples;
    
    // NEW: Edge Density Detection (obstacle/cliff detection)
    metrics.edge_density_pct = std::clamp(edge_density_sum / 3.0f, 0.0f, 100.0f);
    
    // NEW: Dark Obstacle Detection (shadows, black objects)
    metrics.dark_obstacle = (dark_sample_count * 100 / total_samples) > 30;
    
    // NEW: High Contrast Edge Detection (sharp boundaries)
    metrics.high_contrast = (high_contrast_count * 100 / total_samples) > 25;
    
    // NEW: Estimate Obstacle Position & Height
    // Left/Right bias detection
    metrics.obstacle_left_pct = std::max(0.0f, 100.0f - scores[0]);
    metrics.obstacle_right_pct = std::max(0.0f, 100.0f - scores[2]);
    
    // If center is most blocked, obstacle is center; otherwise left/right
    if (scores[1] < std::min(scores[0], scores[2])) {
        metrics.obstacle_center_x = 0.5f;
    } else if (scores[0] < scores[2]) {
        metrics.obstacle_center_x = 0.25f;  // Left side
    } else {
        metrics.obstacle_center_x = 0.75f;  // Right side
    }
    
    // Obstacle detected if center_open < 40% or high edge density
    if (scores[1] < 40.0f || metrics.edge_density_pct > 55.0f) {
        metrics.obstacle_height_pct = 100.0f - std::max(scores[0], scores[2]);
    } else {
        metrics.obstacle_height_pct = 0.0f;
        metrics.obstacle_center_x = -1.0f;  // No obstacle
    }
    
    // NEW: Time-to-Collision Estimation (simplified)
    // High edge density near camera center suggests fast approach
    if (metrics.high_contrast && scores[1] < 40.0f) {
        // Assume 20 cm/s forward speed, estimate collision distance
        // High contrast + blocked center = collision in ~500-800ms
        metrics.motion_flow_magnitude = 85.0f;
        metrics.time_to_collision_ms = 600.0f;
        metrics.collision_imminent = true;
    } else if (metrics.edge_density_pct > 60.0f) {
        metrics.motion_flow_magnitude = 65.0f;
        metrics.time_to_collision_ms = 1200.0f;
        metrics.collision_imminent = true;
    } else if (metrics.edge_density_pct > 45.0f && scores[1] < 50.0f) {
        metrics.motion_flow_magnitude = 45.0f;
        metrics.time_to_collision_ms = 1800.0f;
        metrics.collision_imminent = false;
    } else {
        metrics.motion_flow_magnitude = 0.0f;
        metrics.time_to_collision_ms = 99999.0f;
        metrics.collision_imminent = false;
    }
    
    return true;
}

bool Esp32Camera::AnalyzeNavigation(CameraNavigationMetrics& metrics) {
    std::lock_guard<std::recursive_timed_mutex> lock(camera_mutex_);
    if (!CaptureLocked()) {
        metrics = {};
        return false;
    }
    const bool ok = AnalyzeCurrentFrame(metrics);
    if (ok) {
        ESP_LOGI(TAG,
                 "Navigation view: open L=%.0f C=%.0f R=%.0f offset=%.2f confidence=%.0f",
                 metrics.left_open_pct, metrics.center_open_pct,
                 metrics.right_open_pct, metrics.corridor_offset,
                 metrics.confidence_pct);
    }
    return ok;
}

bool Esp32Camera::DumpJpegToConsole() {
    std::lock_guard<std::recursive_timed_mutex> lock(camera_mutex_);
    if (!CaptureLocked() || current_fb_ == nullptr) return false;

    v4l2_pix_fmt_t format;
    switch (current_fb_->format) {
        case PIXFORMAT_RGB565:
            format = V4L2_PIX_FMT_RGB565;
            break;
        case PIXFORMAT_YUV422:
            format = V4L2_PIX_FMT_YUYV;
            break;
        default:
            ESP_LOGE(TAG, "Console JPEG does not support format %d",
                     current_fb_->format);
            return false;
    }

    uint8_t* source = current_fb_->buf;
    size_t source_length = current_fb_->len;
    if (current_fb_->format == PIXFORMAT_RGB565 && encode_buf_ != nullptr) {
        source = encode_buf_;
        source_length = encode_buf_size_;
    }

    ConsoleJpegStats stats;
    // Keep asynchronous Wi-Fi/MQTT logs from interleaving with Base64 lines.
    // Logging is restored immediately after this diagnostic transfer.
    esp_log_level_set("*", ESP_LOG_NONE);
    std::printf("CAMERA_JPEG_BEGIN,W=%u,H=%u,FORMAT=%d,QUALITY=96\n",
                current_fb_->width, current_fb_->height, current_fb_->format);
    std::fflush(stdout);
    const bool encoded = image_to_jpeg_cb(
        source, source_length, current_fb_->width, current_fb_->height,
        format, 96,
        [](void* argument, size_t, const void* data, size_t length) -> size_t {
            auto* stats = static_cast<ConsoleJpegStats*>(argument);
            if (data == nullptr || length == 0) return 0;
            const auto* bytes = static_cast<const uint8_t*>(data);
            for (size_t offset = 0; offset < length; offset += 180) {
                const size_t chunk_length =
                    std::min<size_t>(180, length - offset);
                PrintBase64Chunk(bytes + offset, chunk_length);
                ++stats->chunks;
            }
            stats->bytes += length;
            return length;
        },
        &stats);
    std::printf("CAMERA_JPEG_END,OK=%d,LEN=%lu,CHUNKS=%lu\n",
                encoded && stats.bytes > 0,
                static_cast<unsigned long>(stats.bytes),
                static_cast<unsigned long>(stats.chunks));
    std::fflush(stdout);
    esp_log_level_set("*", ESP_LOG_INFO);
    return encoded && stats.bytes > 0;
}

bool Esp32Camera::SetHMirror(bool enabled) {
    std::lock_guard<std::recursive_timed_mutex> lock(camera_mutex_);
    sensor_t *s = esp_camera_sensor_get();
    if (!s) {
        return false;
    }
    s->set_hmirror(s, enabled ? 1 : 0);
    return true;
}

bool Esp32Camera::SetVFlip(bool enabled) {
    std::lock_guard<std::recursive_timed_mutex> lock(camera_mutex_);
    sensor_t *s = esp_camera_sensor_get();
    if (!s) {
        return false;
    }
    s->set_vflip(s, enabled ? 1 : 0);
    return true;
}

bool Esp32Camera::SetSwapBytes(bool enabled) {
    std::lock_guard<std::recursive_timed_mutex> lock(camera_mutex_);
    swap_bytes_enabled_ = enabled;
    return true;
}

bool Esp32Camera::TryAcquireVision(uint32_t timeout_ms) {
    return camera_mutex_.try_lock_for(std::chrono::milliseconds(timeout_ms));
}

void Esp32Camera::ReleaseVision() {
    camera_mutex_.unlock();
}

std::string Esp32Camera::Explain(const std::string &question) {
    // Hold the frame lock for the complete encode/upload lifetime. A camera
    // framebuffer is driver-owned, so releasing it while an encoder thread is
    // reading it risks a concurrent Capture() returning the buffer to ESP32.
    std::unique_lock<std::recursive_timed_mutex> lock(camera_mutex_);
    if (explain_url_.empty()) {
        throw std::runtime_error("Image explain URL or token is not set");
    }
    if (current_fb_ == nullptr) {
        throw std::runtime_error("No camera frame captured");
    }

    v4l2_pix_fmt_t format;
    switch (current_fb_->format) {
        case PIXFORMAT_RGB565: format = V4L2_PIX_FMT_RGB565; break;
        case PIXFORMAT_YUV422: format = V4L2_PIX_FMT_YUYV; break;
        case PIXFORMAT_YUV420: format = V4L2_PIX_FMT_YUV420; break;
        case PIXFORMAT_GRAYSCALE: format = V4L2_PIX_FMT_GREY; break;
        case PIXFORMAT_JPEG: format = V4L2_PIX_FMT_JPEG; break;
        case PIXFORMAT_RGB888: format = V4L2_PIX_FMT_RGB24; break;
        default: throw std::runtime_error("Unsupported camera format");
    }
    const uint16_t width = current_fb_->width;
    const uint16_t height = current_fb_->height;
    uint8_t* source = current_fb_->buf;
    size_t source_length = current_fb_->len;
    if (current_fb_->format == PIXFORMAT_RGB565 && encode_buf_ != nullptr) {
        source = encode_buf_;
        source_length = encode_buf_size_;
    }

    // Queue descriptors stay in internal SRAM; every JPEG payload is a small,
    // independently owned PSRAM allocation and is released immediately after
    // its HTTP write. This prevents a full-image JPEG gather buffer.
    QueueHandle_t jpeg_queue = xQueueCreate(8, sizeof(JpegChunk));
    if (jpeg_queue == nullptr) {
        throw std::runtime_error("Failed to create JPEG queue");
    }
    std::atomic<bool> encoder_ok{true};
    struct EncoderContext {
        QueueHandle_t queue;
        std::atomic<bool>* ok;
    } context{jpeg_queue, &encoder_ok};
    auto release_queue = [jpeg_queue]() {
        JpegChunk chunk{};
        while (xQueueReceive(jpeg_queue, &chunk, 0) == pdTRUE) {
            if (chunk.data != nullptr) heap_caps_free(chunk.data);
        }
        vQueueDelete(jpeg_queue);
    };
    auto join_and_drain_encoder = [this, jpeg_queue]() {
        if (!encoder_thread_.joinable()) return;
        // Keep consuming on error: the encoder may be blocked in xQueueSend
        // because the HTTP consumer has stopped. The explicit sentinel marks
        // that it is then safe to join.
        while (true) {
            JpegChunk chunk{};
            xQueueReceive(jpeg_queue, &chunk, portMAX_DELAY);
            if (chunk.data != nullptr) heap_caps_free(chunk.data);
            if (chunk.data == nullptr && chunk.len == 0) break;
        }
        encoder_thread_.join();
    };

    LogVisionMemory("BEFORE_JPEG_ENCODER");
    encoder_thread_ = std::thread([source, source_length, width, height,
                                    format, &context, &encoder_ok, jpeg_queue]() {
        const bool encoded = image_to_jpeg_cb(
            source, source_length, width, height, format, 85,
            [](void* argument, size_t, const void* data, size_t length) -> size_t {
                auto* context = static_cast<EncoderContext*>(argument);
                if (data == nullptr || length == 0 || !context->ok->load()) return 0;
                auto* copy = static_cast<uint8_t*>(heap_caps_aligned_alloc(
                    16, length, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
                if (copy == nullptr) {
                    context->ok->store(false);
                    return 0;
                }
                memcpy(copy, data, length);
                JpegChunk chunk{copy, length};
                if (xQueueSend(context->queue, &chunk, portMAX_DELAY) != pdTRUE) {
                    heap_caps_free(copy);
                    context->ok->store(false);
                    return 0;
                }
                return length;
            }, &context);
        if (!encoded) encoder_ok.store(false);
        const JpegChunk done{nullptr, 0};
        xQueueSend(jpeg_queue, &done, portMAX_DELAY);
    });

    // The production encoder emits one complete JPEG callback (then its end
    // signal), as verified by the B1 runtime logs. Join before opening HTTP so
    // the default 3072-byte pthread stack is reclaimed before the 4096-byte
    // TCP receive task and its required internal network allocations exist.
    // The JPEG payload remains the same PSRAM-owned queue chunk; no image is
    // captured or copied again.
    if (encoder_thread_.joinable()) encoder_thread_.join();
    if (!encoder_ok.load()) {
        release_queue();
        throw std::runtime_error("Failed to encode image to JPEG");
    }
    LogVisionMemory("AFTER_JPEG_ENCODER_JOIN");

    auto network = Board::GetInstance().GetNetwork();
    LogVisionMemory("BEFORE_CREATE_HTTP");
    auto http = network->CreateHttp(3);
    bool http_open = false;
    // Road monitoring is best-effort and must release its task stack quickly
    // when the vision endpoint or Wi-Fi stalls. The generic client defaults
    // to 30 s per operation, which can keep scarce internal SRAM reserved for
    // more than a minute across the multipart writes.
    http->SetTimeout(8000);
    const std::string boundary = "----ESP32_CAMERA_BOUNDARY";
    http->SetHeader("Device-Id", SystemInfo::GetMacAddress().c_str());
    http->SetHeader("Client-Id", Board::GetInstance().GetUuid().c_str());
    if (!explain_token_.empty()) {
        http->SetHeader("Authorization", "Bearer " + explain_token_);
    }
    http->SetHeader("Content-Type", "multipart/form-data; boundary=" + boundary);
    http->SetHeader("Transfer-Encoding", "chunked");
    try {
        if (!http->Open("POST", explain_url_)) {
            throw std::runtime_error("Failed to connect to explain URL");
        }
        http_open = true;
        LogVisionMemory("AFTER_HTTP_OPEN");
        const std::string question_field =
            "--" + boundary +
            "\r\nContent-Disposition: form-data; name=\"question\"\r\n\r\n" +
            question + "\r\n";
        const int question_written = http->Write(question_field.c_str(), question_field.size());
        const size_t question_wire_size = ChunkedWireSize(question_field.size());
        ESP_LOGI(TAG, "VISION_WRITE,QUESTION_REQUESTED=%u,WIRE_EXPECTED=%u,QUESTION_WRITTEN=%d",
                 static_cast<unsigned>(question_field.size()),
                 static_cast<unsigned>(question_wire_size), question_written);
        if (question_written != static_cast<int>(question_wire_size)) {
            throw std::runtime_error("Failed to upload vision question");
        }
        const std::string file_header =
            "--" + boundary +
            "\r\nContent-Disposition: form-data; name=\"file\"; filename=\"camera.jpg\""
            "\r\nContent-Type: image/jpeg\r\n\r\n";
        const int header_written = http->Write(file_header.c_str(), file_header.size());
        const size_t header_wire_size = ChunkedWireSize(file_header.size());
        ESP_LOGI(TAG, "VISION_WRITE,HEADER_REQUESTED=%u,WIRE_EXPECTED=%u,HEADER_WRITTEN=%d",
                 static_cast<unsigned>(file_header.size()),
                 static_cast<unsigned>(header_wire_size), header_written);
        if (header_written != static_cast<int>(header_wire_size)) {
            throw std::runtime_error("Failed to upload vision file header");
        }

        size_t jpeg_bytes = 0;
        size_t jpeg_chunks = 0;
        uint8_t first[2]{};
        uint8_t last[2]{};
        while (true) {
            JpegChunk chunk{};
            xQueueReceive(jpeg_queue, &chunk, portMAX_DELAY);
            if (chunk.data == nullptr && chunk.len == 0) break;
            if (chunk.data == nullptr || chunk.len == 0) {
                throw std::runtime_error("Invalid JPEG stream chunk");
            }
            if (jpeg_bytes == 0) {
                first[0] = chunk.data[0];
                first[1] = chunk.len > 1 ? chunk.data[1] : 0;
            }
            if (chunk.len > 1) {
                last[0] = chunk.data[chunk.len - 2];
                last[1] = chunk.data[chunk.len - 1];
            } else {
                last[0] = last[1];
                last[1] = chunk.data[0];
            }
            const int written = http->Write(reinterpret_cast<const char*>(chunk.data), chunk.len);
            const size_t chunk_wire_size = ChunkedWireSize(chunk.len);
            ESP_LOGI(TAG, "VISION_WRITE,JPEG_REQUESTED=%u,WIRE_EXPECTED=%u,JPEG_WRITTEN=%d,CHUNK=%u",
                     static_cast<unsigned>(chunk.len),
                     static_cast<unsigned>(chunk_wire_size), written,
                     static_cast<unsigned>(jpeg_chunks + 1));
            heap_caps_free(chunk.data);
            if (written != static_cast<int>(chunk_wire_size)) {
                throw std::runtime_error("Failed to upload vision JPEG chunk");
            }
            jpeg_bytes += chunk.len;
            ++jpeg_chunks;
            if (jpeg_chunks == 1 || (jpeg_chunks % 16) == 0) {
                LogVisionMemory("DURING_JPEG_STREAM");
            }
        }
        if (encoder_thread_.joinable()) encoder_thread_.join();
        if (!encoder_ok.load() || jpeg_bytes == 0) {
            throw std::runtime_error("Failed to encode image to JPEG");
        }
        ESP_LOGI(TAG,
                 "VISION_JPEG,FORMAT=YUV422,WIDTH=%u,HEIGHT=%u,BYTES=%u,CHUNKS=%u,SOI=%02X%02X,EOI=%02X%02X",
                 width, height, static_cast<unsigned>(jpeg_bytes),
                 static_cast<unsigned>(jpeg_chunks), first[0], first[1], last[0], last[1]);
        const std::string footer = "\r\n--" + boundary + "--\r\n";
        const int footer_written = http->Write(footer.c_str(), footer.size());
        const size_t footer_wire_size = ChunkedWireSize(footer.size());
        const int terminator_written = http->Write("", 0);
        ESP_LOGI(TAG, "VISION_WRITE,FOOTER_REQUESTED=%u,WIRE_EXPECTED=%u,FOOTER_WRITTEN=%d,TERMINATOR_EXPECTED=5,TERMINATOR_WRITTEN=%d",
                 static_cast<unsigned>(footer.size()),
                 static_cast<unsigned>(footer_wire_size), footer_written,
                 terminator_written);
        if (footer_written != static_cast<int>(footer_wire_size) || terminator_written != 5) {
            throw std::runtime_error("Failed to finish vision upload");
        }
        if (http->GetStatusCode() != 200) {
            throw std::runtime_error("Failed to upload photo");
        }
        LogVisionMemory("BEFORE_READ_ALL");
        std::string result = http->ReadAll();
        http->Close();
        http_open = false;
        release_queue();
        LogVisionMemory("AFTER_EXPLAIN");
        return result;
    } catch (...) {
        if (http_open) http->Close();
        join_and_drain_encoder();
        release_queue();
        LogVisionMemory("AFTER_EXPLAIN_ERROR");
        throw;
    }
}

#if 0  // Legacy queue/thread implementation kept temporarily for comparison.
std::string Esp32Camera::ExplainLegacy(const std::string &question) {
    std::lock_guard<std::recursive_timed_mutex> lock(camera_mutex_);
    if (explain_url_.empty()) {
        throw std::runtime_error("Image explain URL or token is not set");
    }

    if (current_fb_ == nullptr) {
        throw std::runtime_error("No camera frame captured");
    }

    // Create local JPEG queue
    QueueHandle_t jpeg_queue = xQueueCreate(40, sizeof(JpegChunk));
    if (jpeg_queue == nullptr) {
        ESP_LOGE(TAG, "Failed to create JPEG queue");
        throw std::runtime_error("Failed to create JPEG queue");
    }

    // Start encoding thread
    encoder_thread_ = std::thread([this, jpeg_queue]() {
        int64_t start_time = esp_timer_get_time();
        uint16_t w = current_fb_->width;
        uint16_t h = current_fb_->height;
        v4l2_pix_fmt_t enc_fmt;
        switch (current_fb_->format) {
            case PIXFORMAT_RGB565:
                enc_fmt = V4L2_PIX_FMT_RGB565;
                break;
            case PIXFORMAT_YUV422:
                enc_fmt = V4L2_PIX_FMT_YUYV;  // YUV422 is actually YUYV format
                break;
            case PIXFORMAT_YUV420:
                enc_fmt = V4L2_PIX_FMT_YUV420;
                break;
            case PIXFORMAT_GRAYSCALE:
                enc_fmt = V4L2_PIX_FMT_GREY;
                break;
            case PIXFORMAT_JPEG:
                enc_fmt = V4L2_PIX_FMT_JPEG;
                break;
            case PIXFORMAT_RGB888:
                enc_fmt = V4L2_PIX_FMT_RGB24;
                break;
            default:
                ESP_LOGE(TAG, "Unsupported pixel format: %d", current_fb_->format);
                return;
        }

        // Use encode buffer for RGB565, otherwise use original frame buffer
        uint8_t *jpeg_src_buf = current_fb_->buf;
        size_t jpeg_src_len = current_fb_->len;
        if (current_fb_->format == PIXFORMAT_RGB565 && encode_buf_ != nullptr) {
            jpeg_src_buf = encode_buf_;
            jpeg_src_len = encode_buf_size_;
        }

        bool ok = image_to_jpeg_cb(jpeg_src_buf, jpeg_src_len, w, h, enc_fmt, 96,
            [](void* arg, size_t index, const void* data, size_t len) -> size_t {
                auto jpeg_queue = static_cast<QueueHandle_t>(arg);
                JpegChunk chunk = {.data = nullptr, .len = len};
                if (index == 0 && data != nullptr && len > 0) {
                    chunk.data = (uint8_t*)heap_caps_aligned_alloc(16, len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
                    if (chunk.data == nullptr) {
                        ESP_LOGE(TAG, "Failed to allocate %zu bytes for JPEG chunk", len);
                        chunk.len = 0;
                    } else {
                        memcpy(chunk.data, data, len);
                    }
                } else {
                    chunk.len = 0;  // Sentinel or error
                }
                xQueueSend(jpeg_queue, &chunk, portMAX_DELAY);
                return len;
            }, jpeg_queue);

        if (!ok) {
            JpegChunk chunk = {.data = nullptr, .len = 0};
            xQueueSend(jpeg_queue, &chunk, portMAX_DELAY);
        }
        int64_t end_time = esp_timer_get_time();
        ESP_LOGI(TAG, "JPEG encoding time: %ld ms", int((end_time - start_time) / 1000));
    });

    auto network = Board::GetInstance().GetNetwork();
    auto http = network->CreateHttp(3);
    std::string boundary = "----ESP32_CAMERA_BOUNDARY";

    http->SetHeader("Device-Id", SystemInfo::GetMacAddress().c_str());
    http->SetHeader("Client-Id", Board::GetInstance().GetUuid().c_str());
    if (!explain_token_.empty()) {
        http->SetHeader("Authorization", "Bearer " + explain_token_);
    }
    http->SetHeader("Content-Type", "multipart/form-data; boundary=" + boundary);
    http->SetHeader("Transfer-Encoding", "chunked");
    if (!http->Open("POST", explain_url_)) {
        ESP_LOGE(TAG, "Failed to connect to explain URL");
        encoder_thread_.join();
        JpegChunk chunk;
        while (xQueueReceive(jpeg_queue, &chunk, portMAX_DELAY) == pdPASS) {
            if (chunk.data != nullptr) {
                heap_caps_free(chunk.data);
            } else {
                break;
            }
        }
        vQueueDelete(jpeg_queue);
        throw std::runtime_error("Failed to connect to explain URL");
    }

    {
        std::string question_field;
        question_field += "--" + boundary + "\r\n";
        question_field += "Content-Disposition: form-data; name=\"question\"\r\n";
        question_field += "\r\n";
        question_field += question + "\r\n";
        http->Write(question_field.c_str(), question_field.size());
    }
    {
        std::string file_header;
        file_header += "--" + boundary + "\r\n";
        file_header += "Content-Disposition: form-data; name=\"file\"; filename=\"camera.jpg\"\r\n";
        file_header += "Content-Type: image/jpeg\r\n";
        file_header += "\r\n";
        http->Write(file_header.c_str(), file_header.size());
    }

    size_t total_sent = 0;
    bool saw_terminator = false;
    while (true) {
        JpegChunk chunk;
        if (xQueueReceive(jpeg_queue, &chunk, portMAX_DELAY) != pdPASS) {
            ESP_LOGE(TAG, "Failed to receive JPEG chunk");
            break;
        }
        if (chunk.data == nullptr) {
            saw_terminator = true;
            break;
        }
        http->Write((const char *)chunk.data, chunk.len);
        total_sent += chunk.len;
        heap_caps_free(chunk.data);
    }
    encoder_thread_.join();
    vQueueDelete(jpeg_queue);

    if (!saw_terminator || total_sent == 0) {
        ESP_LOGE(TAG, "JPEG encoder failed or produced empty output");
        throw std::runtime_error("Failed to encode image to JPEG");
    }

    {
        std::string multipart_footer;
        multipart_footer += "\r\n--" + boundary + "--\r\n";
        http->Write(multipart_footer.c_str(), multipart_footer.size());
    }
    http->Write("", 0);

    if (http->GetStatusCode() != 200) {
        ESP_LOGE(TAG, "Failed to upload photo, status code: %d", http->GetStatusCode());
        throw std::runtime_error("Failed to upload photo");
    }

    std::string result = http->ReadAll();
    http->Close();

    size_t remain_stack_size = uxTaskGetStackHighWaterMark(nullptr);
    ESP_LOGI(TAG, "Explain image size=%dx%d, compressed size=%d, remain stack size=%d, question=%s\n%s",
             current_fb_->width, current_fb_->height, (int)total_sent, (int)remain_stack_size, question.c_str(), result.c_str());
    return result;
}
#endif
