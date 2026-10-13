#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <atomic>
#include <freertos/FreeRTOS.h>

#include "../config/constants.h"

/**
 * WifiManager - the grinder on the home network, as an alternative to Bluetooth
 *
 * Serves a page at http://grinder.local (firmware update, diagnostic report, saved
 * logs, live log) and streams the live log on TCP port WIFI_LOG_PORT. Only one of
 * Bluetooth and WiFi is on at a time; the BT / WiFi menu page switches between them.
 *
 * Threading: everything that touches the WiFi driver or the servers runs in handle(),
 * on the Bluetooth task (Core 1). The UI and the BLE callbacks only raise requests,
 * the same pattern the grind controller uses for UI actions, so a menu tap can never
 * tear the server down in the middle of a request the radio task is serving.
 */
class WifiManager {
public:
    enum class State : uint8_t {
        OFF,
        CONNECTING,      // Joining the network
        CONNECTED,       // Has an address, servers running
        NOT_CONFIGURED,  // Asked to start without a network name stored
    };

    void init();

    // Requests, applied by handle()
    void request_enable();
    void request_disable();
    // From the BLE credentials characteristic. Stored by handle(), off the BLE host task.
    void request_set_credentials(const char* ssid, const char* password);

    // Bluetooth task, every cycle
    void handle();

    // Status, safe from any task
    State get_state() const { return state_.load(); }
    bool is_enabled() const { return state_.load() == State::CONNECTING || state_.load() == State::CONNECTED; }
    bool is_connected() const { return state_.load() == State::CONNECTED; }
    // For the menu toggle: on as soon as it is asked for, before the radio task acts
    bool is_switched_on() const {
        return !disable_requested_.load() && (is_enabled() || enable_requested_.load());
    }
    bool has_credentials() const { return has_credentials_.load(); }
    bool is_connect_slow() const;                   // Connecting for longer than WIFI_CONNECT_SLOW_MS
    void get_ssid(char* out, size_t out_size) const;
    void get_ip(char* out, size_t out_size) const;  // Empty while not connected
    int get_rssi() const { return rssi_.load(); }
    unsigned long get_timeout_remaining_ms() const;

    // An update uploaded over WiFi restarts the grinder; this brings WiFi back up after
    // that restart (once) so the page can confirm the new build. Clears the flag.
    bool take_resume_after_update();

private:
    void start();
    void stop();
    void start_services();
    void register_routes();
    void apply_pending_credentials();
    void service_log_client();
    void touch() { last_activity_ms_ = millis(); }

    // HTTP handlers
    void handle_root();
    void handle_status();
    void handle_report();
    void handle_live();
    void handle_logs_list();
    void handle_log_file();
    void handle_update_upload();
    void handle_update_done();
    void handle_info();
    void handle_sessions_list();
    void handle_session_file();
    void handle_set_network();

    std::atomic<State> state_{State::OFF};
    std::atomic<bool> has_credentials_{false};
    std::atomic<bool> enable_requested_{false};
    std::atomic<bool> disable_requested_{false};
    std::atomic<int> rssi_{0};
    std::atomic<uint32_t> ip_{0};

    // Written by the radio task, read by the UI
    mutable portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
    char ssid_[WIFI_SSID_MAX_LEN + 1] = {};

    // Credentials waiting to be stored (BLE host task -> radio task)
    bool credentials_pending_ = false;
    char pending_ssid_[WIFI_SSID_MAX_LEN + 1] = {};
    char pending_password_[WIFI_PASSWORD_MAX_LEN + 1] = {};

    // Live log copy buffer, shared by the log port and /live - both run on the radio
    // task only, and a member keeps it off that task's stack
    char io_buf_[WIFI_LOG_CHUNK_BYTES];

    WebServer* http_ = nullptr;
    WiFiServer* log_server_ = nullptr;
    WiFiClient log_client_;
    uint32_t log_client_position_ = 0;
    bool services_started_ = false;
    bool routes_registered_ = false;

    unsigned long connect_started_ms_ = 0;
    volatile unsigned long last_activity_ms_ = 0;

    // Firmware update over HTTP
    const char* update_error_ = nullptr;
    bool update_receiving_ = false;
    bool update_finalize_pending_ = false;
    unsigned long update_finalize_at_ms_ = 0;
};

extern WifiManager wifi_manager;
