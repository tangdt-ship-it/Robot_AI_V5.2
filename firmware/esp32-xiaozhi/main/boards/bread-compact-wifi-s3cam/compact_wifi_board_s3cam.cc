#include "wifi_board.h"
#include "codecs/no_audio_codec.h"
#include "display/lcd_display.h"
#include "system_reset.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "mcp_server.h"
#include "lamp_controller.h"
#include "led/led.h"
#include "esp32_camera.h"
#include "robot_uart.h"
#include "robot/mission_manager.h"
#include "robot/obstacle_assist.h"
#include "robot/teach_route.h"

#include <esp_log.h>
#include <driver/i2c_master.h>
#include <driver/uart.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <driver/spi_common.h>

#if defined(LCD_TYPE_ILI9341_SERIAL)
#include "esp_lcd_ili9341.h"
#endif

#if defined(LCD_TYPE_GC9A01_SERIAL)
#include "esp_lcd_gc9a01.h"
static const gc9a01_lcd_init_cmd_t gc9107_lcd_init_cmds[] = {
    //  {cmd, { data }, data_size, delay_ms}
    {0xfe, (uint8_t[]){0x00}, 0, 0},
    {0xef, (uint8_t[]){0x00}, 0, 0},
    {0xb0, (uint8_t[]){0xc0}, 1, 0},
    {0xb1, (uint8_t[]){0x80}, 1, 0},
    {0xb2, (uint8_t[]){0x27}, 1, 0},
    {0xb3, (uint8_t[]){0x13}, 1, 0},
    {0xb6, (uint8_t[]){0x19}, 1, 0},
    {0xb7, (uint8_t[]){0x05}, 1, 0},
    {0xac, (uint8_t[]){0xc8}, 1, 0},
    {0xab, (uint8_t[]){0x0f}, 1, 0},
    {0x3a, (uint8_t[]){0x05}, 1, 0},
    {0xb4, (uint8_t[]){0x04}, 1, 0},
    {0xa8, (uint8_t[]){0x08}, 1, 0},
    {0xb8, (uint8_t[]){0x08}, 1, 0},
    {0xea, (uint8_t[]){0x02}, 1, 0},
    {0xe8, (uint8_t[]){0x2A}, 1, 0},
    {0xe9, (uint8_t[]){0x47}, 1, 0},
    {0xe7, (uint8_t[]){0x5f}, 1, 0},
    {0xc6, (uint8_t[]){0x21}, 1, 0},
    {0xc7, (uint8_t[]){0x15}, 1, 0},
    {0xf0,
    (uint8_t[]){0x1D, 0x38, 0x09, 0x4D, 0x92, 0x2F, 0x35, 0x52, 0x1E, 0x0C,
                0x04, 0x12, 0x14, 0x1f},
    14, 0},
    {0xf1,
    (uint8_t[]){0x16, 0x40, 0x1C, 0x54, 0xA9, 0x2D, 0x2E, 0x56, 0x10, 0x0D,
                0x0C, 0x1A, 0x14, 0x1E},
    14, 0},
    {0xf4, (uint8_t[]){0x00, 0x00, 0xFF}, 3, 0},
    {0xba, (uint8_t[]){0xFF, 0xFF}, 2, 0},
};
#endif
 
#define TAG "CompactWifiBoardS3Cam"

class CompactWifiBoardS3Cam : public WifiBoard {
private:
 
    Button boot_button_;
    LcdDisplay* display_;
    Esp32Camera* camera_;
    mutable RobotUart robot_uart_;
    MissionManager mission_manager_;
    ObstacleAssist obstacle_assist_;
    TeachRoute teach_route_{&robot_uart_, &mission_manager_};
    volatile bool diagnostic_turn_active_ = false;

    struct DiagnosticTurnRequest {
        CompactWifiBoardS3Cam* board;
        bool left;
        int degrees;
    };

    static void DiagnosticTurnTask(void* context) {
        auto* request = static_cast<DiagnosticTurnRequest*>(context);
        CompactWifiBoardS3Cam* board = request->board;
        const bool left = request->left;
        const int degrees = request->degrees;
        delete request;

        RobotTurnResult result;
        ESP_LOGI(TAG, "ROBOT_DIAG TURN START dir=%s relative=%d speed=10",
                 left ? "left" : "right", degrees);
        const bool completed = board->robot_uart_.SetMode(true, 700) &&
            board->robot_uart_.TurnRelative(left, degrees, 10, result, 13000);
        if (!completed) board->robot_uart_.Stop(700);
        board->robot_uart_.SetMode(false, 700);
        ESP_LOGI(TAG,
                 "ROBOT_DIAG TURN END completed=%d heading=%.2f target=%.2f error=%.2f",
                 completed, result.heading_deg, result.target_deg,
                 result.error_deg);
        board->diagnostic_turn_active_ = false;
        vTaskDelete(nullptr);
    }

    static void DiagnosticConsoleTask(void* context) {
        auto* board = static_cast<CompactWifiBoardS3Cam*>(context);
        char line[64] = {};
        size_t length = 0;
        while (true) {
            // UART0 is owned by ESP-IDF's console VFS. Reading the driver
            // directly bypasses that VFS and does not receive CH340 input on
            // this board, so consume stdin non-blocking instead.
            const int input = getchar();
            if (input == EOF) {
                clearerr(stdin);
                vTaskDelay(pdMS_TO_TICKS(20));
                continue;
            }
            const uint8_t byte = static_cast<uint8_t>(input);
            if (byte == '\r' || byte == '\n') {
                if (length == 0) continue;
                line[length] = '\0';
                length = 0;
                ESP_LOGI(TAG, "ROBOT_DIAG RX: %s", line);

                if (strcmp(line, "camera_capture") == 0) {
                    const int64_t started = esp_timer_get_time();
                    const bool captured = board->camera_ != nullptr &&
                                          board->camera_->Capture();
                    ESP_LOGI(TAG,
                             "CAMERA_DIAG capture=%s elapsed=%ldms screen=%dx%d",
                             captured ? "PASS" : "FAIL",
                             static_cast<long>((esp_timer_get_time() - started) / 1000),
                             board->display_->width(), board->display_->height());
                    continue;
                }
                if (strcmp(line, "camera_dump") == 0) {
                    const bool dumped = board->camera_ != nullptr &&
                                        board->camera_->DumpJpegToConsole();
                    ESP_LOGI(TAG, "CAMERA_DUMP %s", dumped ? "PASS" : "FAIL");
                    continue;
                }
                if (strcmp(line, "camera_describe") == 0) {
                    const int64_t started = esp_timer_get_time();
                    try {
                        if (board->camera_ == nullptr ||
                            !board->camera_->Capture()) {
                            ESP_LOGE(TAG,
                                     "CAMERA_DESCRIBE FAIL capture_failed");
                            continue;
                        }
                        const std::string result = board->camera_->Explain(
                            "Phan tich anh chup phia truoc robot. Tra loi bang "
                            "tieng Viet, ro rang va trung thuc: liet ke tung vat "
                            "the nhin thay, mau sac, vi tri trai/giua/phai va "
                            "gan/xa, huong/cach dat, vat nao nam tren vat nao, "
                            "vat can duong di hay khong, dieu kien anh sang, "
                            "chat luong/lay net cua anh va muc do chac chan. "
                            "Khong suy doan chi tiet khong the quan sat.");
                        ESP_LOGI(TAG,
                                 "CAMERA_DESCRIBE PASS elapsed=%ldms result=%s",
                                 static_cast<long>(
                                     (esp_timer_get_time() - started) / 1000),
                                 result.c_str());
                    } catch (const std::exception& error) {
                        ESP_LOGE(TAG,
                                 "CAMERA_DESCRIBE FAIL elapsed=%ldms error=%s",
                                 static_cast<long>(
                                     (esp_timer_get_time() - started) / 1000),
                                 error.what());
                    }
                    continue;
                }
                if (strcmp(line, "camera_nav") == 0) {
                    CameraNavigationMetrics metrics;
                    const bool analyzed = board->camera_ != nullptr &&
                        board->camera_->AnalyzeNavigation(metrics);
                    ESP_LOGI(TAG,
                             "CAMERA_NAV %s L=%.0f C=%.0f R=%.0f OFFSET=%.2f CONF=%.0f SAMPLES=%u",
                             analyzed ? "PASS" : "FAIL",
                             metrics.left_open_pct, metrics.center_open_pct,
                             metrics.right_open_pct, metrics.corridor_offset,
                             metrics.confidence_pct, metrics.sampled_pixels);
                    continue;
                }
                if (strcmp(line, "robot_obstacle") == 0) {
                    RobotObstacleStatus status;
                    const bool received = board->robot_uart_.GetObstacle(status, 700);
                    ESP_LOGI(TAG,
                             "ROBOT_OBSTACLE %s FRESH=%d ECHO=%d DIST=%.1f RATE=%.1f ZONE=%s LIMITED=%d",
                             received && status.fresh ? "PASS" : "FAIL",
                             status.fresh, status.echo_valid,
                             status.distance_cm, status.approach_rate_cm_s,
                             status.zone, status.limited);
                    continue;
                }
                if (strcmp(line, "robot_odometry") == 0) {
                    RobotOdometry odometry;
                    const bool received = board->robot_uart_.GetOdometry(
                        odometry, 700);
                    const std::string left_ticks =
                        std::to_string(odometry.left_ticks);
                    const std::string right_ticks =
                        std::to_string(odometry.right_ticks);
                    ESP_LOGI(TAG,
                             "ROBOT_ODOMETRY %s DIST=%.1f X=%.1f Y=%.1f H=%.3f LT=%s RT=%s",
                             received ? "PASS" : "FAIL",
                             odometry.distance_mm, odometry.x_mm,
                             odometry.y_mm, odometry.heading_rad,
                             left_ticks.c_str(), right_ticks.c_str());
                    continue;
                }
                if (strcmp(line, "robot_encoder") == 0) {
                    RobotEncoderStatus encoder;
                    const bool received = board->robot_uart_.GetEncoderStatus(
                        encoder, 700);
                    ESP_LOGI(TAG,
                             "ROBOT_ENCODER %s READY=%d HEALTH=%s LV=%.1f RV=%.1f",
                             received ? "PASS" : "FAIL", encoder.ready,
                             encoder.health, encoder.left_velocity_mm_s,
                             encoder.right_velocity_mm_s);
                    continue;
                }
                if (strcmp(line, "scan_obstacle_shadow") == 0) {
                    const bool started = board->mission_manager_.StartShadowScan(35.0f);
                    ESP_LOGI(TAG, "SHADOW_SCAN_START started=%d %s",
                             started,
                             started ? board->mission_manager_.StatusJson().c_str()
                                     : "{\"error\":\"mission_active_or_start_failed\"}");
                    continue;
                }
                if (strcmp(line, "bypass_obstacle_once") == 0) {
                    const bool started = board->mission_manager_.StartBypassOnce();
                    ESP_LOGI(TAG, "BYPASS_ONCE_START started=%d %s",
                             started,
                             started ? board->mission_manager_.StatusJson().c_str()
                                     : "{\"error\":\"mission_active_or_start_failed\"}");
                    continue;
                }
                if (strcmp(line, "navigate_autonomous_12s") == 0) {
                    const bool started =
                        board->mission_manager_.StartAutonomousForward(10, 12,
                                                                       true);
                    ESP_LOGI(TAG, "AUTONOMOUS_12S_START started=%d %s",
                             started,
                             board->mission_manager_.StatusJson().c_str());
                    continue;
                }
                if (strcmp(line, "continue_forward_once") == 0) {
                    RobotObstacleStatus obstacle;
                    CameraNavigationMetrics camera_metrics;
                    const bool obstacle_ok =
                        board->robot_uart_.GetObstacle(obstacle, 700) &&
                        obstacle.fresh &&
                        (!obstacle.echo_valid || obstacle.distance_cm >= 35.0f) &&
                        strcmp(obstacle.zone, "BLOCKED") != 0 &&
                        strcmp(obstacle.zone, "EMERGENCY") != 0;
                    const bool camera_ok = board->camera_ != nullptr &&
                        board->camera_->AnalyzeNavigation(camera_metrics) &&
                        camera_metrics.valid &&
                        camera_metrics.center_open_pct >= 40.0f;
                    const bool started = obstacle_ok && camera_ok &&
                        board->robot_uart_.SetMode(true, 700) &&
                        board->robot_uart_.MoveForward(10, 700);
                    const std::string result =
                        board->CompleteSafeMotion(started, "forward", 10);
                    ESP_LOGI(TAG,
                             "CONTINUE_FORWARD_ONCE obstacle_ok=%d camera_ok=%d %s",
                             obstacle_ok, camera_ok, result.c_str());
                    continue;
                }
                if (strcmp(line, "navigation_status") == 0) {
                    ESP_LOGI(TAG, "NAVIGATION_STATUS %s",
                             board->mission_manager_.StatusJson().c_str());
                    continue;
                }
                if (strcmp(line, "robot_stop") == 0) {
                    const bool stopped = board->robot_uart_.Stop(700);
                    ESP_LOGI(TAG, "ROBOT_DIAG STOP completed=%d", stopped);
                    continue;
                }

                char direction[8] = {};
                int degrees = 0;
                int speed = 0;
                if (sscanf(line, "robot_turn %7s %d %d", direction,
                           &degrees, &speed) != 3 ||
                    (strcmp(direction, "left") != 0 &&
                     strcmp(direction, "right") != 0) ||
                    degrees < 1 || degrees > 30 || speed != 10) {
                    ESP_LOGW(TAG,
                             "ROBOT_DIAG rejected; use robot_turn left|right 1..30 10 or robot_stop");
                    continue;
                }
                if (board->diagnostic_turn_active_) {
                    ESP_LOGW(TAG, "ROBOT_DIAG turn already active");
                    continue;
                }
                auto* request = new DiagnosticTurnRequest{
                    board, strcmp(direction, "left") == 0, degrees};
                board->diagnostic_turn_active_ = true;
                if (xTaskCreatePinnedToCore(DiagnosticTurnTask,
                                            "robot_diag_turn", 4096, request,
                                            2, nullptr, 1) != pdPASS) {
                    board->diagnostic_turn_active_ = false;
                    delete request;
                    ESP_LOGE(TAG, "ROBOT_DIAG cannot start turn task");
                }
                continue;
            }
            if (length < sizeof(line) - 1) {
                line[length++] = static_cast<char>(byte);
            } else {
                length = 0;
            }
        }
    }

    void InitializeDiagnosticConsole() {
        if (xTaskCreatePinnedToCore(DiagnosticConsoleTask,
                                    "robot_diag_console", 4096, this, 1,
                                    nullptr, 1) == pdPASS) {
            ESP_LOGI(TAG,
                     "ROBOT_DIAG console ready: camera_capture; camera_dump; camera_describe; camera_nav; robot_obstacle; robot_odometry; robot_encoder; navigation_status; scan_obstacle_shadow; bypass_obstacle_once; navigate_autonomous_12s; continue_forward_once; robot_turn left|right 1..30 10; robot_stop");
        }
    }

    std::string CompleteSafeMotion(bool started, const char* motion,
                                   int speed) {
        if (!started) {
            robot_uart_.Stop(300);
            return std::string("{\"completed\":false,\"reason\":\"rejected_or_no_uart\"}");
        }
        vTaskDelay(pdMS_TO_TICKS(800));
        RobotState state;
        const bool stopped = robot_uart_.GetState(state, 700) &&
                             !state.moving && state.left == 0 &&
                             state.right == 0;
        if (!stopped) {
            robot_uart_.Stop(300);
        }
        robot_uart_.SetMode(false, 500);
        char result[192];
        snprintf(result, sizeof(result),
                 "{\"completed\":%s,\"motion\":\"%s\",\"speed\":%d,\"duration_ms\":650,\"final_moving\":%s,\"distance_controlled\":false,\"angle_controlled\":false}",
                 stopped ? "true" : "false", motion, speed,
                 stopped ? "false" : "true");
        return std::string(result);
    }

    std::string StartSafeContinuous(bool forward, int speed) {
        const bool started = robot_uart_.SetMode(true, 700) &&
                             robot_uart_.StartContinuous(forward, speed, 700);
        if (!started) {
            robot_uart_.Stop(700);
            robot_uart_.SetMode(false, 700);
        }
        char result[192];
        snprintf(result, sizeof(result),
                 "{\"started\":%s,\"motion\":\"%s\",\"speed\":%d,\"continuous\":true,\"heartbeat_ms\":200,\"lease_timeout_ms\":700,\"requires_stop\":true}",
                 started ? "true" : "false", forward ? "forward" : "backward",
                 speed);
        return std::string(result);
    }

    std::string StartSafeContinuousRotation(bool left, int speed) {
        const bool started = robot_uart_.SetMode(true, 700) &&
            robot_uart_.StartContinuousRotation(left, speed, 700);
        if (!started) {
            robot_uart_.Stop(700);
            robot_uart_.SetMode(false, 700);
        }
        char result[192];
        snprintf(result, sizeof(result),
                 "{\"started\":%s,\"motion\":\"rotate_in_place\",\"direction\":\"%s\",\"speed\":%d,\"continuous\":true,\"heartbeat_ms\":200,\"requires_stop\":true}",
                 started ? "true" : "false", left ? "left" : "right",
                 speed);
        return std::string(result);
    }

    void InitializeRobotTools() {
        auto& mcp_server = McpServer::GetInstance();
        const PropertyList continuous_motion_property({
            Property("speed", kPropertyTypeInteger, 20, 10, 20),
            Property("continuous", kPropertyTypeBoolean, true),
        });
        const PropertyList distance_motion_property({
            Property("distance_mm", kPropertyTypeInteger, 500, 1, 5000),
            Property("forward", kPropertyTypeBoolean, true),
            Property("speed", kPropertyTypeInteger, 15, 10, 20),
        });
        const PropertyList config_speed_property({
            Property("speed", kPropertyTypeInteger, 30, 10, 255),
        });
        const PropertyList enabled_property({
            Property("enabled", kPropertyTypeBoolean),
        });
        const PropertyList navigation_property({
            Property("speed", kPropertyTypeInteger, 15, 10, 20),
            Property("runtime_seconds", kPropertyTypeInteger, 30, 1, 180),
            Property("camera_guidance", kPropertyTypeBoolean, true),
        });
        const PropertyList return_home_property({
            Property("speed", kPropertyTypeInteger, 12, 10, 20),
            Property("camera_guidance", kPropertyTypeBoolean, true),
        });
        // The individual read-only tools below are retained in source for
        // diagnostics history, but are consolidated into get_diagnostics so
        // Xiaozhi sees one clear tool instead of eight near-duplicates.
#if 0
        mcp_server.AddTool(
            "self.robot.get_state",
            "Read STM32 robot mode, fused heading and motor state. This tool never moves the robot.",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                RobotState state;
                if (!robot_uart_.GetState(state, 700)) {
                    return std::string("{\"connected\":false}");
                }
                char json[256];
                snprintf(json, sizeof(json),
                         "{\"connected\":true,\"mode\":\"%s\",\"heading\":%.1f,\"speed\":%d,\"brake\":%s,\"ramp\":%s,\"left\":%d,\"right\":%d,\"moving\":%s,\"compass_ok\":%s,\"ps2_ok\":%s}",
                         state.ai_mode ? "AI" : "MANUAL", state.heading_deg,
                         state.speed,
                         state.brake_enabled ? "true" : "false",
                         state.ramp_enabled ? "true" : "false",
                         state.left, state.right,
                         state.moving ? "true" : "false",
                         state.compass_ok ? "true" : "false",
                         state.ps2_ok ? "true" : "false");
                return std::string(json);
            });
        mcp_server.AddTool(
            "self.robot.get_encoder_status",
            "Đọc trạng thái encoder hai bánh từ STM32, gồm readiness, health và vận tốc thực từng bánh. Công cụ chỉ đọc và không làm robot di chuyển.",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                RobotEncoderStatus encoder;
                const bool ok = robot_uart_.GetEncoderStatus(encoder, 700);
                char json[224];
                snprintf(json, sizeof(json),
                         "{\"ok\":%s,\"ready\":%s,\"health\":\"%s\",\"left_velocity_mm_s\":%.1f,\"right_velocity_mm_s\":%.1f}",
                         ok ? "true" : "false",
                         encoder.ready ? "true" : "false", encoder.health,
                         encoder.left_velocity_mm_s,
                         encoder.right_velocity_mm_s);
                return std::string(json);
            });
        mcp_server.AddTool(
            "self.robot.get_odometry",
            "Đọc odometry thực từ STM32: tổng quãng đường, tọa độ X/Y tương đối, heading hợp nhất và xung encoder. Đây là nguồn vị trí chính của navigation V4.2; tool không làm robot di chuyển.",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                RobotOdometry odom;
                const bool ok = robot_uart_.GetOdometry(odom, 700);
                char json[320];
                snprintf(json, sizeof(json),
                         "{\"ok\":%s,\"distance_mm\":%.1f,\"x_mm\":%.1f,\"y_mm\":%.1f,\"heading_rad\":%.4f,\"left_ticks\":%lld,\"right_ticks\":%lld}",
                         ok ? "true" : "false", odom.distance_mm, odom.x_mm,
                         odom.y_mm, odom.heading_rad,
                         static_cast<long long>(odom.left_ticks),
                         static_cast<long long>(odom.right_ticks));
                return std::string(json);
            });
        mcp_server.AddTool(
            "self.robot.get_imu_status",
            "Đọc MPU6050 từ STM32: trạng thái kết nối/hiệu chuẩn, Gyro Z đã đổi sang hệ trục robot và gia tốc XYZ. Tool chỉ đọc.",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                RobotImuStatus imu;
                const bool ok = robot_uart_.GetImuStatus(imu, 700);
                char json[320];
                snprintf(json, sizeof(json),
                         "{\"ok\":%s,\"ready\":%s,\"calibrated\":%s,\"health\":\"%s\",\"gyro_z_dps\":%.2f,\"accel_g\":{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f}}",
                         ok ? "true" : "false", imu.ready ? "true" : "false",
                         imu.calibrated ? "true" : "false", imu.health,
                         imu.gyro_z_dps, imu.accel_x_g, imu.accel_y_g,
                         imu.accel_z_g);
                return std::string(json);
            });
        mcp_server.AddTool(
            "self.robot.get_fusion_status",
            "Đọc heading fusion STM32 kết hợp Encoder + MPU6050 Gyro Z + Compass, gồm heading, yaw rate, confidence và nguồn cảm biến đang tham gia. Tool chỉ đọc.",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                RobotFusionStatus fusion;
                const bool ok = robot_uart_.GetFusionStatus(fusion, 700);
                char json[320];
                snprintf(json, sizeof(json),
                         "{\"ok\":%s,\"ready\":%s,\"health\":\"%s\",\"heading_deg\":%.2f,\"yaw_rate_dps\":%.2f,\"confidence_pct\":%.1f,\"source\":\"%s\"}",
                         ok ? "true" : "false",
                         fusion.ready ? "true" : "false", fusion.health,
                         fusion.heading_deg, fusion.yaw_rate_dps,
                         fusion.confidence_pct, fusion.source);
                return std::string(json);
            });
        mcp_server.AddTool(
            "self.robot.get_obstacle",
            "Đọc cảm biến siêu âm phía trước trực tiếp từ STM32: khoảng cách, tốc độ tiếp cận và vùng an toàn. Phải dùng công cụ này khi người dùng hỏi về vật cản. Khi BLOCKED hoặc EMERGENCY, hãy thông báo robot đã tự hãm và chỉ đề nghị lùi hoặc xoay để thoát.",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                RobotObstacleStatus status;
                const bool ok = robot_uart_.GetObstacle(status, 700);
                char json[224];
                snprintf(json, sizeof(json),
                         "{\"ok\":%s,\"fresh\":%s,\"echo_valid\":%s,\"distance_cm\":%.1f,\"approach_rate_cm_s\":%.1f,\"zone\":\"%s\",\"motor_limited\":%s}",
                         ok ? "true" : "false",
                         status.fresh ? "true" : "false",
                         status.echo_valid ? "true" : "false",
                         status.distance_cm, status.approach_rate_cm_s,
                         status.zone,
                         status.limited ? "true" : "false");
                return std::string(json);
            });
#endif
        mcp_server.AddTool(
            "self.robot.get_diagnostics",
            "Read robot diagnostics without moving. target=state,encoder,heading,compass,imu,fusion,obstacle,ps2, or all.",
            PropertyList({Property("target", kPropertyTypeString, "all")}),
            [this](const PropertyList& properties) -> ReturnValue {
                const std::string target = properties["target"].value<std::string>();
                const bool all = target == "all";
                std::string json = "{\"target\":\"" + target + "\"";
                if (all || target == "state" || target == "heading") {
                    RobotState s; const bool ok = robot_uart_.GetState(s, 700);
                    json += ",\"state_ok\":" + std::string(ok ? "true" : "false");
                    if (ok) { json += ",\"heading_deg\":" + std::to_string(s.heading_deg) +
                        ",\"moving\":" + std::string(s.moving ? "true" : "false"); }
                }
                if (all || target == "odometry" || target == "position") {
                    RobotOdometry o; const bool ok = robot_uart_.GetOdometry(o, 700);
                    json += ",\"odometry_ok\":" + std::string(ok ? "true" : "false") +
                        ",\"x_mm\":" + std::to_string(o.x_mm) +
                        ",\"y_mm\":" + std::to_string(o.y_mm) +
                        ",\"left_ticks\":" + std::to_string(static_cast<long long>(o.left_ticks)) +
                        ",\"right_ticks\":" + std::to_string(static_cast<long long>(o.right_ticks));
                }
                if (all || target == "encoder") {
                    RobotEncoderStatus e; const bool ok = robot_uart_.GetEncoderStatus(e, 700);
                    json += ",\"encoder_ok\":" + std::string(ok ? "true" : "false") +
                        ",\"encoder_health\":\"" + std::string(e.health) + "\"";
                }
                if (all || target == "compass") {
                    RobotCompassStatus c; const bool ok = robot_uart_.GetCompassStatus(c, 700);
                    json += ",\"compass_ok\":" + std::string(ok ? "true" : "false") +
                        ",\"compass_connected\":" + std::string(c.connected ? "true" : "false") +
                        ",\"compass_heading_deg\":" + std::to_string(c.heading_deg);
                }
                if (all || target == "imu") {
                    RobotImuStatus i; const bool ok = robot_uart_.GetImuStatus(i, 700);
                    json += ",\"imu_ok\":" + std::string(ok ? "true" : "false") +
                        ",\"imu_health\":\"" + std::string(i.health) + "\"";
                }
                if (all || target == "fusion") {
                    RobotFusionStatus f; const bool ok = robot_uart_.GetFusionStatus(f, 700);
                    json += ",\"fusion_ok\":" + std::string(ok ? "true" : "false") +
                        ",\"fusion_heading_deg\":" + std::to_string(f.heading_deg) +
                        ",\"confidence_pct\":" + std::to_string(f.confidence_pct);
                }
                if (all || target == "obstacle") {
                    RobotObstacleStatus o; const bool ok = robot_uart_.GetObstacle(o, 700);
                    json += ",\"obstacle_ok\":" + std::string(ok ? "true" : "false") +
                        ",\"distance_cm\":" + std::to_string(o.distance_cm) +
                        ",\"zone\":\"" + std::string(o.zone) + "\"";
                }
                if (all || target == "ps2") {
                    RobotPs2Status p; const bool ok = robot_uart_.GetPs2Status(p, 700);
                    json += ",\"ps2_ok\":" + std::string(ok ? "true" : "false") +
                        ",\"ps2_state\":\"" + std::string(p.state) + "\"";
                }
                json += "}";
                return json;
            });
        mcp_server.AddTool(
            "self.robot.move_distance",
            "Di chuyển thẳng theo quãng đường encoder thực đo. Dùng khi người dùng yêu cầu tiến hoặc lùi một số mm/cm/m cụ thể. Robot tự dừng khi đạt mục tiêu, khi gặp vật cản, timeout, PS2 override hoặc link lỗi. Không khẳng định đã đi đủ quãng đường nếu completed=false.",
            distance_motion_property,
            [this](const PropertyList& properties) -> ReturnValue {
                // move_distance is an operator-owned, closed-loop motion.
                // Reject it while a mission owns the STM32 motion channel so
                // two callers cannot wait on or overwrite the same terminal
                // DONE/ERROR response.
                if (mission_manager_.IsActive()) {
                    return std::string(
                        "{\"completed\":false,\"error\":\"autonomous_mission_active\"}");
                }
                const int distance_mm = properties["distance_mm"].value<int>();
                const bool forward = properties["forward"].value<bool>();
                const int speed = properties["speed"].value<int>();
                RobotDistanceResult result;
                const bool completed = robot_uart_.SetMode(true, 700) &&
                    robot_uart_.MoveDistance(forward, distance_mm, speed,
                                              result, 31000);
                robot_uart_.SetMode(false, 700);
                const bool pose_synced = mission_manager_.SyncAfterExternalMotion();
                char json[224];
                snprintf(json, sizeof(json),
                         "{\"completed\":%s,\"direction\":\"%s\",\"target_mm\":%.1f,\"travelled_mm\":%.1f,\"pose_synced\":%s}",
                         completed ? "true" : "false",
                         forward ? "forward" : "backward",
                         result.target_mm, result.travelled_mm,
                         pose_synced ? "true" : "false");
                return std::string(json);
            });
        mcp_server.AddTool(
            "self.robot.navigate_autonomously",
            "Thực hiện ngay và ghi nhớ trọn vẹn lệnh tự hành, không hỏi lại người dùng. Robot tiến theo thời lượng yêu cầu; gặp vật cản thì tự dừng, dùng Camera + HC-SR04 chọn phía thoáng, ưu tiên cam kết ngay phía đầu tiên đủ an toàn, lách chéo 35 độ với quãng đường tính theo khoảng cách vật cản, trở lại hướng hành trình và tiếp tục cho đến hết yêu cầu. Nếu tool được gọi lại khi đang tự hành, nhiệm vụ hiện tại được cập nhật/gia hạn thay vì bị hủy. Chỉ STOP, PS2 override, khóa phanh hoặc lỗi cảm biến/link mới được ngắt nhiệm vụ.",
            navigation_property,
            [this](const PropertyList& properties) -> ReturnValue {
                const bool started = mission_manager_.StartAutonomousForward(
                    properties["speed"].value<int>(),
                    properties["runtime_seconds"].value<int>(),
                    properties["camera_guidance"].value<bool>());
                if (!started) {
                    return std::string(
                        "{\"accepted\":false,\"error\":\"incompatible_mission_or_task_start_failed\"}");
                }
                return mission_manager_.StatusJson();
            });
        mcp_server.AddTool(
            "self.robot.set_home",
            "Lưu vị trí hiện tại làm HOME bằng odometry STM32 và xóa breadcrumb cũ. Chỉ dùng khi robot đang đứng yên và không có mission đang chạy.",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                const bool ok = mission_manager_.SetHome();
                if (!ok) return std::string("{\"ok\":false,\"error\":\"mission_active_or_odometry_not_ready\"}");
                return mission_manager_.HomeJson();
            });
        mcp_server.AddTool(
            "self.robot.get_home",
            "Đọc HOME, pose hiện tại và số breadcrumb đã lưu cho chức năng Return Home. Tool không làm robot di chuyển.",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                return mission_manager_.HomeJson();
            });
        mcp_server.AddTool(
            "self.robot.return_home",
            "Cho robot quay về vị trí HOME bằng breadcrumb odometry thực. Robot quay về từng waypoint rồi chạy tiến, vẫn dùng HC-SR04/camera và toàn bộ safety STM32. Chỉ xác nhận đã về khi mission state là return_completed.",
            return_home_property,
            [this](const PropertyList& properties) -> ReturnValue {
                const bool started = mission_manager_.StartReturnHome(
                    properties["speed"].value<int>(),
                    properties["camera_guidance"].value<bool>());
                if (!started) {
                    return std::string("{\"accepted\":false,\"error\":\"home_not_set_or_mission_active\"}");
                }
                return mission_manager_.StatusJson();
            });
        mcp_server.AddTool(
            "self.robot.navigation_map",
            "Lấy bản đồ lưới cục bộ do HC-SR04 và pose encoder + fused heading của STM32 tạo ra. Ký hiệu ?: chưa biết, .: trống, #: vật cản, R: robot. Đây không phải bản đồ SLAM tuyệt đối.",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                return mission_manager_.MapJson();
            });
        mcp_server.AddTool(
            "self.robot.scan_obstacle",
            "Chạy một ưa quét vật cản Shadow Mode: robot dừng, lưu hướng gốc H0, quét thân trái/phải (góc cấu hình) bằng vòng kín fused heading ở tốc độ thấp, đo HC-SR04 và chụp camera mỗi hướng, rồi chạy local path planner để chọn LEFT/RIGHT/NONE. QUAN TRỌNG: robot KHÔNG tự chạy vòng qua vật cản trong Shadow Mode; chỉ trả kết quả kế hoạch. Góc scan mặc định 35 độ (cho phép 10..90).",
            PropertyList({
                Property("scan_angle_deg", kPropertyTypeInteger, 35, 10, 90),
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                const int angle = properties["scan_angle_deg"].value<int>();
                const bool started = mission_manager_.StartShadowScan(
                    static_cast<float>(angle));
                if (!started) {
                    return std::string(
                        "{\"started\":false,\"error\":\"mission_active_or_task_start_failed\"}");
                }
                return mission_manager_.StatusJson();
            });
        mcp_server.AddTool(
            "self.robot.get_obstacle_plan",
            "Đọc kết quả kế hoạch tránh vật cản gần nhất từ Shadow Mode: lựa chọn LEFT/RIGHT/NONE, chi phí trái/phải, độ tin cậy, lý do và dữ liệu quét (HC-SR04 + vision) cho center/left/right. Tool này không làm robot chuyển động.",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                return mission_manager_.PlanJson();
            });
        mcp_server.AddTool(
            "self.robot.get_mission_state",
            "Đọc trạng thái mission/hệ thống điều hướng hiện tại của robot (idle/navigating/scan/shadow...), pose tương đối, trạng thái Shadow Mode, hướng gốc H0, vật cản gần nhất và kết quả quét trái/phải. Tool này không làm robot chuyển động.",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                return mission_manager_.StatusJson();
            });
        mcp_server.AddTool(
            "self.robot.cancel_mission",
            "Hủy mission di chuyển hoặc quét Shadow hiện tại và yêu cầu STM32 dừng ngay. Dùng khi người dùng muốn dừng quét/hủy tự hành hoặc có tình huống không an toàn.",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                mission_manager_.CancelMission();
                const bool stopped = robot_uart_.Stop(700);
                robot_uart_.SetMode(false, 700);
                return std::string(stopped
                    ? "{\"completed\":true,\"moving\":false}"
                    : "{\"completed\":false,\"error\":\"stop_not_verified\"}");
            });
        mcp_server.AddTool(
            "self.robot.get_speed",
            "Đọc tốc độ cấu hình hiện tại trực tiếp từ STM32. Phải dùng công cụ này khi người dùng hỏi tốc độ; không trả lời từ bộ nhớ hội thoại.",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                int speed = 0;
                if (!robot_uart_.GetSpeed(speed, 700)) {
                    return std::string("{\"ok\":false,\"error\":\"uart_or_stm32_rejected\"}");
                }
                char json[64];
                snprintf(json, sizeof(json), "{\"ok\":true,\"speed\":%d}", speed);
                return std::string(json);
            });
        mcp_server.AddTool(
            "self.robot.set_speed",
            "Đặt tốc độ runtime của robot trong dải STM32 cho phép 10 đến 255. Chỉ xác nhận với người dùng sau khi STM32 ACK và đọc lại đúng giá trị. Giai đoạn hiện tại lệnh chuyển động AI ngắn vẫn giới hạn an toàn 10 đến 20.",
            config_speed_property,
            [this](const PropertyList& properties) -> ReturnValue {
                const int requested = properties["speed"].value<int>();
                int actual = 0;
                const bool ok = robot_uart_.SetSpeed(requested, 700) &&
                                robot_uart_.GetSpeed(actual, 700) &&
                                actual == requested;
                char json[96];
                snprintf(json, sizeof(json),
                         "{\"ok\":%s,\"requested\":%d,\"actual\":%d}",
                         ok ? "true" : "false", requested, actual);
                return std::string(json);
            });
        mcp_server.AddTool(
            "self.robot.set_brake",
            "Khóa hoặc nhả hãm động cơ trên STM32. Khi người dùng nói thắng, hãm, phanh, brake hoặc khóa động cơ thì gọi enabled=true; firmware giữ khóa phần cứng ở PWM 251. Khi người dùng nói nhả thắng/nhả hãm thì gọi enabled=false. Chỉ báo thành công sau ACK và đọc lại.",
            enabled_property,
            [this](const PropertyList& properties) -> ReturnValue {
                const bool requested = properties["enabled"].value<bool>();
                bool actual = false;
                const bool ok = robot_uart_.SetBrake(requested, 700) &&
                                robot_uart_.GetBrake(actual, 700) &&
                                actual == requested;
                char json[80];
                snprintf(json, sizeof(json),
                         "{\"ok\":%s,\"enabled\":%s}",
                         ok ? "true" : "false", actual ? "true" : "false");
                return std::string(json);
            });
        mcp_server.AddTool(
            "self.robot.set_ramp",
            "Bật hoặc tắt Ramp runtime trên STM32. STOP an toàn vẫn bỏ qua Ramp. Chỉ báo thành công sau ACK và đọc lại.",
            enabled_property,
            [this](const PropertyList& properties) -> ReturnValue {
                const bool requested = properties["enabled"].value<bool>();
                bool actual = false;
                const bool ok = robot_uart_.SetRamp(requested, 700) &&
                                robot_uart_.GetRamp(actual, 700) &&
                                actual == requested;
                char json[80];
                snprintf(json, sizeof(json),
                         "{\"ok\":%s,\"enabled\":%s}",
                         ok ? "true" : "false", actual ? "true" : "false");
                return std::string(json);
            });
        mcp_server.AddTool(
            "self.robot.get_heading",
            "Đọc góc la bàn mới nhất trực tiếp từ STM32. Phải dùng công cụ này khi người dùng hỏi robot đang ở góc bao nhiêu.",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                float heading = 0.0f;
                const bool ok = robot_uart_.GetHeading(heading, 700);
                char json[80];
                snprintf(json, sizeof(json),
                         "{\"ok\":%s,\"heading_deg\":%.1f}",
                         ok ? "true" : "false", heading);
                return std::string(json);
            });
        mcp_server.AddTool(
            "self.robot.get_compass_status",
            "Đọc kết nối, trạng thái calibration và góc Compass trực tiếp từ STM32.",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                RobotCompassStatus status;
                const bool ok = robot_uart_.GetCompassStatus(status, 700);
                char json[128];
                snprintf(json, sizeof(json),
                         "{\"ok\":%s,\"connected\":%s,\"calibrating\":%s,\"heading_deg\":%.1f}",
                         ok ? "true" : "false",
                         status.connected ? "true" : "false",
                         status.calibrating ? "true" : "false",
                         status.heading_deg);
                return std::string(json);
            });
        mcp_server.AddTool(
            "self.robot.reset",
            "Reset robot reference values without moving. target=heading resets HDG/Compass; target=encoder resets both wheel encoder counters El and Er. Voice: reset la ban, dua HDG ve 0, reset encoder hai banh.",
            PropertyList({Property("target", kPropertyTypeString, "heading")}),
            [this](const PropertyList& properties) -> ReturnValue {
                const auto target = properties["target"].value<std::string>();
                ESP_LOGI(TAG, "MCP_RESET_TARGET=%s", target.c_str());
                if (target == "encoder" || target == "encoders" ||
                    target == "wheel" || target == "wheels") {
                    const bool ok = robot_uart_.ResetEncoders(700);
                    RobotOdometry odometry;
                    const bool verified = ok && robot_uart_.GetOdometry(odometry, 700) &&
                                          odometry.left_ticks == 0 && odometry.right_ticks == 0;
                    char json[128];
                    snprintf(json, sizeof(json),
                             "{\"ok\":%s,\"target\":\"encoder\",\"left_ticks\":%ld,\"right_ticks\":%ld}",
                             verified ? "true" : "false",
                             static_cast<long>(odometry.left_ticks),
                             static_cast<long>(odometry.right_ticks));
                    ESP_LOGI(TAG, "ESP32_TOOL_RESULT=%s target=encoder", verified ? "PASS" : "FAIL");
                    return std::string(json);
                }
                const bool ok = robot_uart_.ResetCompass(2200);
                float heading = 0.0f;
                const bool verified = ok && robot_uart_.GetHeading(heading, 700);
                char json[96];
                snprintf(json, sizeof(json),
                         "{\"ok\":%s,\"target\":\"heading\",\"zero_event\":%s,\"heading_deg\":%.1f}",
                         verified ? "true" : "false", ok ? "true" : "false",
                         heading);
                ESP_LOGI(TAG, "ESP32_TOOL_RESULT=%s target=heading ACK=%s HDG_AFTER=%.1f",
                         verified ? "PASS" : "FAIL", ok ? "1" : "0", heading);
                return std::string(json);
            });
        mcp_server.AddTool(
            "self.robot.get_ps2_status",
            "Đọc trạng thái receiver PS2, software enable, frame freshness và tuổi frame trực tiếp từ STM32. enabled chỉ là trạng thái phần mềm, không phải nguồn điện receiver.",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                RobotPs2Status status;
                const bool ok = robot_uart_.GetPs2Status(status, 700);
                char json[144];
                snprintf(json, sizeof(json),
                         "{\"ok\":%s,\"state\":\"%s\",\"enabled\":%s,\"fresh\":%s,\"age_ms\":%lu}",
                         ok ? "true" : "false", status.state,
                         status.enabled ? "true" : "false",
                         status.fresh ? "true" : "false",
                         static_cast<unsigned long>(status.age_ms));
                return std::string(json);
            });
        mcp_server.AddTool(
            "self.robot.stop",
            "Dừng robot ngay lập tức với ưu tiên cao nhất. Luôn gọi công cụ này khi người dùng nói dừng, dừng lại, đứng lại, stop hoặc robot dừng. Chỉ báo thành công sau DONE,STOP từ STM32.",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                mission_manager_.CancelMission();
                const bool stopped = robot_uart_.Stop(700);
                RobotState state;
                const bool verified = stopped && robot_uart_.GetState(state, 700) &&
                                      !state.moving && state.left == 0 &&
                                      state.right == 0;
                robot_uart_.SetMode(false, 700);
                return std::string(verified
                    ? "{\"completed\":true,\"moving\":false}"
                    : "{\"completed\":false,\"error\":\"stop_not_verified\"}");
            });
        mcp_server.AddTool(
            "self.robot.move_forward",
            "Tiến thẳng liên tục không ngắt quãng. continuous=false chạy một nhịp an toàn; continuous=true chạy liên tục với tự động tránh vật cản bằng Camera + HC-SR04. Gặp vật cản sẽ tự thông báo và kích hoạt chế độ tránh, tìm đường khác rồi tiếp tục hướng ban đầu.",
            continuous_motion_property,
            [this](const PropertyList& properties) -> ReturnValue {
                const int speed = properties["speed"].value<int>();
                const bool continuous = properties["continuous"].value<bool>();
                // continuous=true: Start autonomous forward with obstacle avoidance
                // continuous=false: Single pulse motion
                const bool accepted = mission_manager_.StartAutonomousForward(
                    speed, continuous ? 180U : 1U, true);
                if (!accepted) {
                    return std::string(
                        "{\"accepted\":false,\"error\":\"incompatible_mission_or_task_start_failed\"}");
                }
                return mission_manager_.StatusJson();
            });
        mcp_server.AddTool(
            "self.robot.move_backward",
            "Cho robot lùi. continuous=false chạy một nhịp 650 ms; continuous=true chạy liên tục bằng motion lease và heartbeat cho tới STOP, PS2 override hoặc lỗi link. Không đo quãng đường và không được tuyên bố đã đi chính xác mét/cm.",
            continuous_motion_property,
            [this](const PropertyList& properties) -> ReturnValue {
                if (mission_manager_.IsActive()) {
                    return std::string("{\"completed\":false,\"error\":\"autonomous_mission_active\"}");
                }
                const int speed = properties["speed"].value<int>();
                const bool continuous = properties["continuous"].value<bool>();
                if (continuous) return StartSafeContinuous(false, speed);
                const bool started = robot_uart_.SetMode(true, 700) &&
                                     robot_uart_.MoveBackward(speed, 700);
                return CompleteSafeMotion(started, "backward", speed);
            });
        mcp_server.AddTool(
            "self.robot.turn_left",
            "Cho robot xoay trái tại chỗ. continuous=false xoay một nhịp 650 ms; continuous=true xoay liên tục tới STOP bằng heartbeat. Tuyệt đối không dùng cho yêu cầu có số độ; yêu cầu góc phải dùng self.robot.turn_relative.",
            continuous_motion_property,
            [this](const PropertyList& properties) -> ReturnValue {
                if (mission_manager_.IsActive()) {
                    return std::string("{\"completed\":false,\"error\":\"autonomous_mission_active\"}");
                }
                const int speed = properties["speed"].value<int>();
                const bool continuous = properties["continuous"].value<bool>();
                if (continuous) return StartSafeContinuousRotation(true, speed);
                const bool started = robot_uart_.SetMode(true, 700) &&
                                     robot_uart_.TurnLeft(speed, 700);
                return CompleteSafeMotion(started, "left", speed);
            });
        mcp_server.AddTool(
            "self.robot.turn_right",
            "Cho robot xoay phải tại chỗ. continuous=false xoay một nhịp 650 ms; continuous=true xoay liên tục tới STOP bằng heartbeat. Tuyệt đối không dùng cho yêu cầu có số độ; yêu cầu góc phải dùng self.robot.turn_relative.",
            continuous_motion_property,
            [this](const PropertyList& properties) -> ReturnValue {
                if (mission_manager_.IsActive()) {
                    return std::string("{\"completed\":false,\"error\":\"autonomous_mission_active\"}");
                }
                const int speed = properties["speed"].value<int>();
                const bool continuous = properties["continuous"].value<bool>();
                if (continuous) return StartSafeContinuousRotation(false, speed);
                const bool started = robot_uart_.SetMode(true, 700) &&
                                     robot_uart_.TurnRight(speed, 700);
                return CompleteSafeMotion(started, "right", speed);
            });
        mcp_server.AddTool(
            "self.robot.rotate_continuous",
            "Xoay robot liên tục tại chỗ sang trái hoặc phải cho tới khi người dùng ra lệnh dừng. Heartbeat được firmware gửi tự động. Tool này không nhắm góc; nếu người dùng nói số độ phải dùng self.robot.turn_relative.",
            PropertyList({
                Property("direction", kPropertyTypeString),
                Property("speed", kPropertyTypeInteger, 20, 10, 20),
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                if (mission_manager_.IsActive()) {
                    return std::string("{\"started\":false,\"error\":\"autonomous_mission_active\"}");
                }
                const std::string direction =
                    properties["direction"].value<std::string>();
                const bool left = direction == "left" || direction == "LEFT" ||
                                  direction == "trai" || direction == "trái";
                const bool right = direction == "right" || direction == "RIGHT" ||
                                   direction == "phai" || direction == "phải";
                if (!left && !right) {
                    return std::string("{\"started\":false,\"error\":\"invalid_direction\"}");
                }
                return StartSafeContinuousRotation(
                    left, properties["speed"].value<int>());
            });
        mcp_server.AddTool(
            "self.robot.turn_relative",
            "Yêu cầu STM32 quay tương đối trái hoặc phải bằng vòng kín fused heading (Encoder + MPU6050 + Compass) và chờ DONE. Dùng cho câu có góc như quay trái 45 độ. Chỉ được nói đã quay xong khi completed=true; nếu false phải nói robot chưa hoàn thành góc quay.",
            PropertyList({
                Property("direction", kPropertyTypeString),
                Property("degrees", kPropertyTypeInteger, 1, 180),
                Property("speed", kPropertyTypeInteger, 10, 10, 20),
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                if (mission_manager_.IsActive()) {
                    return std::string("{\"completed\":false,\"error\":\"autonomous_mission_active\"}");
                }
                const std::string direction =
                    properties["direction"].value<std::string>();
                const bool left = direction == "left" || direction == "LEFT" ||
                                  direction == "trai" || direction == "trái";
                const bool right = direction == "right" || direction == "RIGHT" ||
                                   direction == "phai" || direction == "phải";
                if (!left && !right) {
                    return std::string("{\"completed\":false,\"error\":\"invalid_direction\"}");
                }
                const int degrees = properties["degrees"].value<int>();
                const int speed = properties["speed"].value<int>();
                RobotTurnResult turn;
                const bool completed = robot_uart_.SetMode(true, 700) &&
                    robot_uart_.TurnRelative(left, degrees, speed, turn, 13000);
                if (!completed) robot_uart_.Stop(700);
                robot_uart_.SetMode(false, 700);
                char json[256];
                snprintf(json, sizeof(json),
                         "{\"completed\":%s,\"direction\":\"%s\",\"requested_degrees\":%d,\"heading_deg\":%.1f,\"target_deg\":%.1f,\"error_deg\":%.1f,\"compass_closed_loop\":true}",
                         completed ? "true" : "false", left ? "left" : "right",
                         degrees, turn.heading_deg, turn.target_deg,
                         turn.error_deg);
                return std::string(json);
            });
        mcp_server.AddTool(
            "self.robot.turn_to_heading",
            "Yêu cầu STM32 quay tới heading tuyệt đối -180..180 độ bằng vòng kín fused heading (Encoder + MPU6050 + Compass) và chờ DONE. Dùng cho câu như quay về hướng 0 độ. Chỉ xác nhận hoàn tất khi completed=true.",
            PropertyList({
                Property("heading", kPropertyTypeInteger, -180, 180),
                Property("speed", kPropertyTypeInteger, 10, 10, 20),
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                if (mission_manager_.IsActive()) {
                    return std::string("{\"completed\":false,\"error\":\"autonomous_mission_active\"}");
                }
                const int heading = properties["heading"].value<int>();
                const int speed = properties["speed"].value<int>();
                RobotTurnResult turn;
                const bool completed = robot_uart_.SetMode(true, 700) &&
                    robot_uart_.TurnAbsolute(heading, speed, turn, 13000);
                if (!completed) robot_uart_.Stop(700);
                robot_uart_.SetMode(false, 700);
                char json[224];
                snprintf(json, sizeof(json),
                         "{\"completed\":%s,\"requested_heading_deg\":%d,\"heading_deg\":%.1f,\"target_deg\":%.1f,\"error_deg\":%.1f,\"compass_closed_loop\":true}",
                         completed ? "true" : "false", heading,
                         turn.heading_deg, turn.target_deg, turn.error_deg);
                return std::string(json);
            });
    }

    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = DISPLAY_MOSI_PIN;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = DISPLAY_CLK_PIN;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeLcdDisplay() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;
        // 液晶屏控制IO初始化
        ESP_LOGD(TAG, "Install panel IO");
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = DISPLAY_CS_PIN;
        io_config.dc_gpio_num = DISPLAY_DC_PIN;
        io_config.spi_mode = DISPLAY_SPI_MODE;
        io_config.pclk_hz = 40 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io));

        // 初始化液晶屏驱动芯片
        ESP_LOGD(TAG, "Install LCD driver");
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = DISPLAY_RST_PIN;
        panel_config.rgb_ele_order = DISPLAY_RGB_ORDER;
        panel_config.bits_per_pixel = 16;
#if defined(LCD_TYPE_ILI9341_SERIAL)
        ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(panel_io, &panel_config, &panel));
#elif defined(LCD_TYPE_GC9A01_SERIAL)
        ESP_ERROR_CHECK(esp_lcd_new_panel_gc9a01(panel_io, &panel_config, &panel));
        gc9a01_vendor_config_t gc9107_vendor_config = {
            .init_cmds = gc9107_lcd_init_cmds,
            .init_cmds_size = sizeof(gc9107_lcd_init_cmds) / sizeof(gc9a01_lcd_init_cmd_t),
        };        
#else
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel));
#endif
        
        esp_lcd_panel_reset(panel);

        esp_lcd_panel_init(panel);
        esp_lcd_panel_invert_color(panel, DISPLAY_INVERT_COLOR);
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
#ifdef  LCD_TYPE_GC9A01_SERIAL
        panel_config.vendor_config = &gc9107_vendor_config;
#endif
        display_ = new SpiLcdDisplay(panel_io, panel,
                                    DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
    }

    void InitializeCamera() {
        camera_config_t config = {};
        config.pin_d0 = CAMERA_PIN_D0;
        config.pin_d1 = CAMERA_PIN_D1;
        config.pin_d2 = CAMERA_PIN_D2;
        config.pin_d3 = CAMERA_PIN_D3;
        config.pin_d4 = CAMERA_PIN_D4;
        config.pin_d5 = CAMERA_PIN_D5;
        config.pin_d6 = CAMERA_PIN_D6;
        config.pin_d7 = CAMERA_PIN_D7;
        config.pin_xclk = CAMERA_PIN_XCLK;
        config.pin_pclk = CAMERA_PIN_PCLK;
        config.pin_vsync = CAMERA_PIN_VSYNC;
        config.pin_href = CAMERA_PIN_HREF;
        config.pin_sccb_sda = CAMERA_PIN_SIOD;
        config.pin_sccb_scl = CAMERA_PIN_SIOC;
        config.sccb_i2c_port = 0;
        config.pin_pwdn = CAMERA_PIN_PWDN;
        config.pin_reset = CAMERA_PIN_RESET;
        config.xclk_freq_hz = XCLK_FREQ_HZ;
        // This OV2640 carrier produces clean YUYV data. RGB565 captures on the
        // same wiring have repeatable green/magenta corruption even though DMA
        // reports complete frames. The AI JPEG encoder accepts YUYV directly.
        config.pixel_format = PIXFORMAT_YUV422;
        // SVGA is the maximum verified stable 16-bit mode for this camera/DMA
        // wiring. XGA was field-tested and timed out.
        config.frame_size = FRAMESIZE_SVGA;
        config.jpeg_quality = 6;
        config.fb_count = 1;
        config.fb_location = CAMERA_FB_IN_PSRAM;
        config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
        camera_ = new Esp32Camera(config);
        camera_->SetHMirror(false);
        mission_manager_.SetCamera(camera_);
        obstacle_assist_.SetCamera(camera_);
        obstacle_assist_.SetMissionManager(&mission_manager_);
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });
    }

public:
    CompactWifiBoardS3Cam()
        : boot_button_(BOOT_BUTTON_GPIO),
          robot_uart_(ROBOT_UART_RX_PIN, ROBOT_UART_TX_PIN),
          mission_manager_(&robot_uart_),
          obstacle_assist_(&robot_uart_) {
        InitializeSpi();
        InitializeLcdDisplay();
        teach_route_.SetDisplay(display_);
        teach_route_.Begin();
        InitializeButtons();
        InitializeCamera();
        if (robot_uart_.Begin()) {
            robot_uart_.SetObstacleStoppedCallback(&ObstacleAssist::OnStopped,
                                                   &obstacle_assist_);
            teach_route_.StartInputTask();
            InitializeRobotTools();
            InitializeDiagnosticConsole();
        }
        if (DISPLAY_BACKLIGHT_PIN != GPIO_NUM_NC) {
            GetBacklight()->RestoreBrightness();
        }
        
    }

    virtual Led* GetLed() override {
        static NoLed led;
        return &led;
    }

    virtual AudioCodec* GetAudioCodec() override {
#ifdef AUDIO_I2S_METHOD_SIMPLEX
        static NoAudioCodecSimplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_SPK_GPIO_BCLK, AUDIO_I2S_SPK_GPIO_LRCK, AUDIO_I2S_SPK_GPIO_DOUT, AUDIO_I2S_MIC_GPIO_SCK, AUDIO_I2S_MIC_GPIO_WS, AUDIO_I2S_MIC_GPIO_DIN);
#else
        static NoAudioCodecDuplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN);
#endif
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }

    virtual Backlight* GetBacklight() override {
        if (DISPLAY_BACKLIGHT_PIN != GPIO_NUM_NC) {
            static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
            return &backlight;
        }
        return nullptr;
    }

    virtual Camera* GetCamera() override {
        return camera_;
    }

    virtual MissionManager* GetMissionManager() override {
        return &mission_manager_;
    }

    virtual bool IsRobotMotionActive() const override {
        RobotState state;
        if (!robot_uart_.GetState(state, 200)) {
            return false;
        }
        return state.moving || state.ai_mode;
    }
};

DECLARE_BOARD(CompactWifiBoardS3Cam);
