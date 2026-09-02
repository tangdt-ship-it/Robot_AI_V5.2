#ifndef MQTT_PROTOCOL_H
#define MQTT_PROTOCOL_H


#include "protocol.h"
#include <mqtt.h>
#include <udp.h>
#include <cJSON.h>
#include <mbedtls/aes.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <esp_timer.h>

#include <functional>
#include <string>
#include <deque>
#include <mutex>
#include <memory>
#include <atomic>

#define MQTT_PING_INTERVAL_SECONDS 90
#define MQTT_RECONNECT_INTERVAL_MS 60000

#define MQTT_PROTOCOL_SERVER_HELLO_EVENT (1 << 0)

class MqttProtocol : public Protocol {
public:
    MqttProtocol();
    ~MqttProtocol();

    bool Start() override;
    bool SendAudio(std::unique_ptr<AudioStreamPacket> packet) override;
    bool OpenAudioChannel() override;
    void CloseAudioChannel(bool send_goodbye = true) override;
    bool IsAudioChannelOpened() const override;

private:
    // Alive flag for safe scheduled callbacks - set to false in destructor
    std::shared_ptr<std::atomic<bool>> alive_ = std::make_shared<std::atomic<bool>>(true);
    
    EventGroupHandle_t event_group_handle_;

    std::string publish_topic_;

    std::mutex channel_mutex_;
    std::unique_ptr<Mqtt> mqtt_;
    std::unique_ptr<Udp> udp_;
    mbedtls_aes_context aes_ctx_;
    std::string aes_nonce_;
    std::string udp_server_;
    int udp_port_;
    uint32_t local_sequence_;

    // A small bounded jitter buffer keeps a packet that arrives ahead of the
    // expected sequence long enough for normal UDP reordering to resolve.
    // Missing frames are never synthesized here: after the bounded window,
    // playback continues with the next real packet.
    struct PendingAudioPacket {
        uint32_t sequence;
        std::unique_ptr<AudioStreamPacket> packet;
    };
    static constexpr size_t kTargetAudioJitterFrames = 2;
    static constexpr size_t kMaxAudioJitterFrames = 4;
    std::deque<PendingAudioPacket> audio_reorder_buffer_;
    uint32_t expected_remote_sequence_ = 0;
    bool expected_remote_sequence_valid_ = false;
    uint32_t audio_udp_gap_count_ = 0;
    uint32_t audio_late_packet_count_ = 0;
    uint32_t audio_reordered_count_ = 0;
    size_t audio_jitter_max_ = 0;
    int64_t last_audio_gap_log_us_ = 0;
    std::mutex audio_reorder_mutex_;
    std::mutex audio_dispatch_mutex_;
    uint32_t audio_reorder_generation_ = 0;
    esp_timer_handle_t audio_reorder_timer_ = nullptr;
    bool audio_reorder_timer_armed_ = false;
    esp_timer_handle_t reconnect_timer_;

    bool StartMqttClient(bool report_error=false);
    void ParseServerHello(const cJSON* root);
    std::string DecodeHexString(const std::string& hex_string);
    void ResetAudioReorderState();
    void QueueAudioPacket(uint32_t sequence, std::unique_ptr<AudioStreamPacket> packet);
    void DrainAudioReorderBuffer(bool force_gap,
                                 std::deque<std::unique_ptr<AudioStreamPacket>>& ready_packets);
    void DispatchAudioPackets(std::deque<std::unique_ptr<AudioStreamPacket>>& ready_packets,
                              uint32_t generation);
    void LogLateAudioPacket(uint32_t sequence);
    void UpdateAudioReorderTimer();

    bool SendText(const std::string& text) override;
    std::string GetHelloMessage();
};


#endif // MQTT_PROTOCOL_H
