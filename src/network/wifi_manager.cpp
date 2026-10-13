#include "wifi_manager.h"

#include <ESPmDNS.h>
#include <LittleFS.h>
#include <Preferences.h>
#include <esp_heap_caps.h>
#include <algorithm>
#include <vector>

#include "web_page.h"
#include "../bluetooth/manager.h"
#include "../controllers/grind_controller.h"
#include "../logging/debug_log.h"

extern BluetoothManager g_bluetooth_manager;
extern GrindController grind_controller;

WifiManager wifi_manager;

namespace {

// Saved logs are the only files served, and only by exact name
bool is_log_file_name(const String& name) {
    if (name.length() < 9 || name.length() > 31 || !name.startsWith("log_") || !name.endsWith(".txt")) {
        return false;
    }
    for (size_t i = 0; i < name.length(); i++) {
        char c = name[i];
        if (!(isalnum((unsigned char)c) || c == '_' || c == '.')) {
            return false;
        }
    }
    return true;
}

bool read_credentials(char* ssid, size_t ssid_size, char* password, size_t password_size) {
    ssid[0] = '\0';
    password[0] = '\0';
    Preferences prefs;
    if (!prefs.begin(WIFI_PREFS_NAMESPACE, true)) {
        return false;  // Nothing stored yet
    }
    String stored_ssid = prefs.getString("ssid", "");
    String stored_password = prefs.getString("pass", "");
    prefs.end();
    strncpy(ssid, stored_ssid.c_str(), ssid_size - 1);
    ssid[ssid_size - 1] = '\0';
    strncpy(password, stored_password.c_str(), password_size - 1);
    password[password_size - 1] = '\0';
    return ssid[0] != '\0';
}

void set_resume_flag(bool resume) {
    Preferences prefs;
    if (prefs.begin(WIFI_PREFS_NAMESPACE, false)) {
        if (resume) {
            prefs.putBool("resume", true);
        } else {
            prefs.remove("resume");
        }
        prefs.end();
    }
}

}  // namespace

//------------------------------------------------------------------------------
// Requests and status (any task)
//------------------------------------------------------------------------------

void WifiManager::init() {
    char ssid[WIFI_SSID_MAX_LEN + 1];
    char password[WIFI_PASSWORD_MAX_LEN + 1];
    bool stored = read_credentials(ssid, sizeof(ssid), password, sizeof(password));
    memset(password, 0, sizeof(password));
    has_credentials_.store(stored);
    portENTER_CRITICAL(&mux_);
    strncpy(ssid_, stored ? ssid : "", sizeof(ssid_) - 1);
    portEXIT_CRITICAL(&mux_);
    LOG_BLE("[WIFI] %s\n", stored ? "Network stored - off until switched on" : "No network stored yet");
}

void WifiManager::request_enable() {
    disable_requested_.store(false);
    enable_requested_.store(true);
}

void WifiManager::request_disable() {
    enable_requested_.store(false);
    disable_requested_.store(true);
}

void WifiManager::request_set_credentials(const char* ssid, const char* password) {
    if (!ssid || !password || strlen(ssid) > WIFI_SSID_MAX_LEN || strlen(password) > WIFI_PASSWORD_MAX_LEN) {
        return;
    }
    portENTER_CRITICAL(&mux_);
    strncpy(pending_ssid_, ssid, sizeof(pending_ssid_) - 1);
    pending_ssid_[sizeof(pending_ssid_) - 1] = '\0';
    strncpy(pending_password_, password, sizeof(pending_password_) - 1);
    pending_password_[sizeof(pending_password_) - 1] = '\0';
    credentials_pending_ = true;
    portEXIT_CRITICAL(&mux_);
}

bool WifiManager::is_connect_slow() const {
    return state_.load() == State::CONNECTING && millis() - connect_started_ms_ > WIFI_CONNECT_SLOW_MS;
}

void WifiManager::get_ssid(char* out, size_t out_size) const {
    if (!out || out_size == 0) return;
    portENTER_CRITICAL(&mux_);
    strncpy(out, ssid_, out_size - 1);
    portEXIT_CRITICAL(&mux_);
    out[out_size - 1] = '\0';
}

void WifiManager::get_ip(char* out, size_t out_size) const {
    if (!out || out_size == 0) return;
    out[0] = '\0';
    if (!is_connected()) return;
    uint32_t ip = ip_.load();
    snprintf(out, out_size, "%u.%u.%u.%u", (unsigned)(ip & 0xFF), (unsigned)((ip >> 8) & 0xFF),
             (unsigned)((ip >> 16) & 0xFF), (unsigned)(ip >> 24));
}

unsigned long WifiManager::get_timeout_remaining_ms() const {
    if (!is_enabled()) return 0;
    unsigned long idle = millis() - last_activity_ms_;
    return idle >= WIFI_AUTO_DISABLE_TIMEOUT_MS ? 0 : WIFI_AUTO_DISABLE_TIMEOUT_MS - idle;
}

bool WifiManager::take_resume_after_update() {
    Preferences prefs;
    if (!prefs.begin(WIFI_PREFS_NAMESPACE, true)) {
        return false;
    }
    bool resume = prefs.getBool("resume", false);
    prefs.end();
    if (resume) {
        set_resume_flag(false);  // Once only, whatever happens next
    }
    return resume;
}

//------------------------------------------------------------------------------
// Radio task
//------------------------------------------------------------------------------

void WifiManager::handle() {
    apply_pending_credentials();

    if (disable_requested_.exchange(false) && state_.load() != State::OFF) {
        stop();
    }
    if (enable_requested_.exchange(false) && !is_enabled()) {
        start();
    }
    if (!is_enabled()) {
        return;
    }

    const unsigned long now = millis();
    if (WiFi.status() == WL_CONNECTED) {
        IPAddress address = WiFi.localIP();
        ip_.store((uint32_t)address[0] | ((uint32_t)address[1] << 8) |
                  ((uint32_t)address[2] << 16) | ((uint32_t)address[3] << 24));
        rssi_.store(WiFi.RSSI());
        if (state_.load() != State::CONNECTED) {
            state_.store(State::CONNECTED);
            if (!services_started_) {
                start_services();
            }
        }
    } else if (state_.load() == State::CONNECTED) {
        // The driver reconnects by itself; the servers stay up for when it does
        state_.store(State::CONNECTING);
        connect_started_ms_ = now;
        LOG_BLE("[WIFI] Connection lost - reconnecting\n");
    }

    if (services_started_) {
        http_->handleClient();  // An update upload runs to completion inside this call
        service_log_client();
    }

    // Applied a moment after the reply, so the browser hears the upload succeeded
    // before the grinder restarts
    if (update_finalize_pending_ && (long)(millis() - update_finalize_at_ms_) >= 0) {
        update_finalize_pending_ = false;
        LOG_BLE("[WIFI] Applying update\n");
        set_resume_flag(true);  // Bring WiFi back after the restart, so the page can confirm
        g_bluetooth_manager.report_ota_status("Applying update...");
        if (!g_bluetooth_manager.get_ota_handler().complete_ota()) {  // Restarts on success
            set_resume_flag(false);
            LOG_BLE("[WIFI] Update could not be applied\n");
        }
        touch();
    }

    if (is_enabled() && !g_bluetooth_manager.is_updating() &&
        millis() - last_activity_ms_ > WIFI_AUTO_DISABLE_TIMEOUT_MS) {
        LOG_BLE("[WIFI] %lu min without activity - switching off\n",
                (unsigned long)(WIFI_AUTO_DISABLE_TIMEOUT_MS / 60000));
        stop();
    }
}

void WifiManager::start() {
    char ssid[WIFI_SSID_MAX_LEN + 1];
    char password[WIFI_PASSWORD_MAX_LEN + 1];
    if (!read_credentials(ssid, sizeof(ssid), password, sizeof(password))) {
        has_credentials_.store(false);
        state_.store(State::NOT_CONFIGURED);
        LOG_BLE("[WIFI] No network stored - enter it in the web flasher over Bluetooth\n");
        return;
    }

    // One radio at a time. The menu switches Bluetooth off itself; this covers the
    // restart after a WiFi update and anything else that turns WiFi on.
    if (g_bluetooth_manager.is_enabled()) {
        g_bluetooth_manager.disable();
    }

    LOG_BLE("[WIFI] Joining '%s'\n", ssid);
    WiFi.persistent(false);  // The credentials live in our own namespace, not the driver's
    WiFi.setHostname(WIFI_HOSTNAME);
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    // Modem sleep stays at its default: the Bluetooth stack stays initialised for the
    // whole boot (see BluetoothManager::enable), and the coexistence scheme requires it
    WiFi.begin(ssid, password);
    memset(password, 0, sizeof(password));

    portENTER_CRITICAL(&mux_);
    strncpy(ssid_, ssid, sizeof(ssid_) - 1);
    portEXIT_CRITICAL(&mux_);
    connect_started_ms_ = millis();
    touch();
    state_.store(State::CONNECTING);
}

void WifiManager::stop() {
    if (state_.load() == State::NOT_CONFIGURED) {
        state_.store(State::OFF);
        return;
    }
    // An upload already received but not yet applied holds the grind tasks suspended;
    // release them rather than leave the grinder frozen
    if (update_finalize_pending_) {
        update_finalize_pending_ = false;
        g_bluetooth_manager.get_ota_handler().abort_ota();
    }
    if (services_started_) {
        log_client_.stop();
        log_server_->end();
        http_->stop();
        MDNS.end();
        services_started_ = false;
    }
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    ip_.store(0);
    rssi_.store(0);
    state_.store(State::OFF);
    LOG_BLE("[WIFI] Off\n");
}

void WifiManager::start_services() {
    if (!http_) {
        http_ = new WebServer(WIFI_HTTP_PORT);
    }
    if (!routes_registered_) {
        register_routes();
        routes_registered_ = true;
    }
    if (!log_server_) {
        log_server_ = new WiFiServer(WIFI_LOG_PORT);
    }
    http_->begin();
    log_server_->begin();
    log_server_->setNoDelay(true);
    if (MDNS.begin(WIFI_HOSTNAME)) {
        MDNS.addService("http", "tcp", WIFI_HTTP_PORT);
    } else {
        LOG_BLE("[WIFI] mDNS did not start - use the IP address\n");
    }
    services_started_ = true;

    char ssid[WIFI_SSID_MAX_LEN + 1];
    char ip[16];
    get_ssid(ssid, sizeof(ssid));
    get_ip(ip, sizeof(ip));
    LOG_BLE("[WIFI] On '%s' as %s - http://%s.local, live log on port %d\n",
            ssid, ip, WIFI_HOSTNAME, WIFI_LOG_PORT);
}

void WifiManager::apply_pending_credentials() {
    char ssid[WIFI_SSID_MAX_LEN + 1];
    char password[WIFI_PASSWORD_MAX_LEN + 1];
    portENTER_CRITICAL(&mux_);
    bool pending = credentials_pending_;
    if (pending) {
        memcpy(ssid, pending_ssid_, sizeof(ssid));
        memcpy(password, pending_password_, sizeof(password));
        memset(pending_password_, 0, sizeof(pending_password_));
        credentials_pending_ = false;
    }
    portEXIT_CRITICAL(&mux_);
    if (!pending) {
        return;
    }

    Preferences prefs;
    if (prefs.begin(WIFI_PREFS_NAMESPACE, false)) {
        prefs.putString("ssid", ssid);
        prefs.putString("pass", password);
        prefs.end();
    }
    memset(password, 0, sizeof(password));
    has_credentials_.store(ssid[0] != '\0');
    portENTER_CRITICAL(&mux_);
    strncpy(ssid_, ssid, sizeof(ssid_) - 1);
    portEXIT_CRITICAL(&mux_);
    LOG_BLE("[WIFI] Network set to '%s'\n", ssid);

    if (is_enabled()) {
        stop();
        start();  // Join the new network straight away
    }
}

void WifiManager::service_log_client() {
    if (!log_client_ || !log_client_.connected()) {
        if (log_client_) {
            log_client_.stop();
        }
        WiFiClient incoming = log_server_->accept();
        if (!incoming) {
            return;
        }
        log_client_ = incoming;
        log_client_.setNoDelay(true);
        // Start with what the ring still holds, so the lead-up is visible too
        log_client_position_ = debug_log_live_position() - DEBUG_LIVE_BYTES;
        LOG_BLE("[WIFI] Live log viewer connected\n");
    }

    touch();  // An open viewer keeps WiFi on

    // At most a few KB per cycle, so a slow viewer cannot hold up the radio task
    for (int round = 0; round < 4; round++) {
        uint32_t skipped = 0;
        size_t count = debug_log_live_read(&log_client_position_, io_buf_, sizeof(io_buf_), &skipped);
        if (skipped) {
            char note[64];
            int length = snprintf(note, sizeof(note), "\n[... %lu bytes not shown ...]\n", (unsigned long)skipped);
            log_client_.write(reinterpret_cast<const uint8_t*>(note), length);
        }
        if (count == 0) {
            break;
        }
        if (log_client_.write(reinterpret_cast<const uint8_t*>(io_buf_), count) != count) {
            log_client_.stop();
            break;
        }
    }
}

//------------------------------------------------------------------------------
// HTTP
//------------------------------------------------------------------------------

void WifiManager::register_routes() {
    http_->on("/", HTTP_GET, [this]() { handle_root(); });
    http_->on("/status", HTTP_GET, [this]() { handle_status(); });
    http_->on("/report", HTTP_GET, [this]() { handle_report(); });
    http_->on("/live", HTTP_GET, [this]() { handle_live(); });
    http_->on("/logs", HTTP_GET, [this]() { handle_logs_list(); });
    http_->on("/logs/get", HTTP_GET, [this]() { handle_log_file(); });
    // Raw body (application/octet-stream): handle_update_upload() sees the bytes as they
    // arrive, handle_update_done() answers once the whole body is in
    http_->on("/update", HTTP_POST, [this]() { handle_update_done(); }, [this]() { handle_update_upload(); });
    http_->onNotFound([this]() {
        touch();
        http_->send(404, "text/plain", "Not found");
    });
}

void WifiManager::handle_root() {
    touch();
    http_->send_P(200, "text/html", WIFI_WEB_PAGE);
}

void WifiManager::handle_status() {
    touch();
    char ip[16];
    get_ip(ip, sizeof(ip));
    char json[320];
    snprintf(json, sizeof(json),
             "{\"build\":%d,\"version\":\"%s\",\"git\":\"%s\",\"uptime_s\":%lu,\"ip\":\"%s\",\"rssi\":%d,"
             "\"grinding\":%s,\"updating\":%s,\"heap_internal\":%u}",
             BUILD_NUMBER, BUILD_FIRMWARE_VERSION, get_git_commit_id(), millis() / 1000, ip, get_rssi(),
             grind_controller.is_active() ? "true" : "false",
             g_bluetooth_manager.is_updating() ? "true" : "false",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    http_->sendHeader("Cache-Control", "no-store");
    http_->send(200, "application/json", json);
}

void WifiManager::handle_report() {
    touch();
    if (http_->hasArg("download")) {
        char header[64];
        snprintf(header, sizeof(header), "attachment; filename=\"grinder-diagnostics-build%d.txt\"", BUILD_NUMBER);
        http_->sendHeader("Content-Disposition", header);
    }
    http_->sendHeader("Cache-Control", "no-store");
    http_->setContentLength(CONTENT_LENGTH_UNKNOWN);
    http_->send(200, "text/plain; charset=utf-8", "");
    // Exactly the report Bluetooth sends, written into the response as it is built
    g_bluetooth_manager.write_diagnostic_report([this](const char* chunk) {
        if (chunk && chunk[0]) {
            http_->sendContent(chunk, strlen(chunk));
        }
    });
    http_->sendContent("", 0);  // Ends the chunked response
}

void WifiManager::handle_live() {
    touch();
    // Without a position, start as far back as the ring reaches; read() clamps it
    uint32_t position = http_->hasArg("from") ? (uint32_t)strtoul(http_->arg("from").c_str(), nullptr, 10)
                                              : debug_log_live_position() - DEBUG_LIVE_BYTES;
    String body;
    body.reserve(1024);
    while (body.length() < WIFI_LIVE_RESPONSE_MAX_BYTES) {
        uint32_t skipped = 0;
        size_t count = debug_log_live_read(&position, io_buf_, sizeof(io_buf_), &skipped);
        if (skipped) {
            body += "\n[... ";
            body += String((unsigned long)skipped);
            body += " bytes not shown ...]\n";
        }
        if (count == 0) {
            break;
        }
        body.concat(io_buf_, count);
    }
    http_->sendHeader("X-Next", String((unsigned long)position));
    http_->sendHeader("Cache-Control", "no-store");
    http_->send(200, "text/plain; charset=utf-8", body);
}

void WifiManager::handle_logs_list() {
    touch();
    std::vector<std::pair<String, size_t>> files;
    File dir = LittleFS.open(DEBUG_PERSIST_DIR);
    if (dir && dir.isDirectory()) {
        File file = dir.openNextFile();
        while (file) {
            String name = file.name();
            int slash = name.lastIndexOf('/');
            if (slash >= 0) {
                name = name.substring(slash + 1);
            }
            if (!file.isDirectory() && is_log_file_name(name)) {
                files.emplace_back(name, file.size());
            }
            file.close();
            file = dir.openNextFile();
        }
    }
    // Segment numbers are zero-padded, so name order is age order
    std::sort(files.begin(), files.end(),
              [](const std::pair<String, size_t>& a, const std::pair<String, size_t>& b) { return a.first < b.first; });

    String json = "[";
    for (size_t i = 0; i < files.size(); i++) {
        if (i) json += ",";
        json += "{\"name\":\"" + files[i].first + "\",\"size\":" + String((unsigned long)files[i].second) + "}";
    }
    json += "]";
    http_->sendHeader("Cache-Control", "no-store");
    http_->send(200, "application/json", json);
}

void WifiManager::handle_log_file() {
    touch();
    String name = http_->arg("name");
    if (!is_log_file_name(name)) {
        http_->send(400, "text/plain", "Unknown log file");
        return;
    }
    File file = LittleFS.open(String(DEBUG_PERSIST_DIR) + "/" + name, "r");
    if (!file) {
        http_->send(404, "text/plain", "Log file not found");
        return;
    }
    http_->sendHeader("Content-Disposition", "attachment; filename=\"" + name + "\"");
    http_->streamFile(file, "text/plain");
    file.close();
}

void WifiManager::handle_update_upload() {
    touch();
    HTTPRaw& raw = http_->raw();
    OTAHandler& ota = g_bluetooth_manager.get_ota_handler();

    switch (raw.status) {
        case RAW_START: {
            update_error_ = nullptr;
            update_receiving_ = false;
            const int length = http_->clientContentLength();
            if (grind_controller.is_active()) {
                update_error_ = "A grind is running - update when it has finished";
            } else if (ota.is_ota_active() || update_finalize_pending_) {
                update_error_ = "An update is already in progress";
            } else if (length <= 0) {
                update_error_ = "The upload was empty";
            } else if (!ota.start_ota((uint32_t)length, "", false, "")) {
                // Same delta path and same file as a Bluetooth update from the web flasher
                update_error_ = "The grinder could not start the update";
            } else {
                update_receiving_ = true;
                g_bluetooth_manager.report_ota_status("Receiving update...");
                LOG_BLE("[WIFI] Receiving update, %d KB\n", length / 1024);
            }
            break;
        }
        case RAW_WRITE:
            if (update_receiving_ && !ota.process_data_chunk(raw.buf, raw.currentSize)) {
                update_error_ = "Writing the update failed";
                ota.abort_ota();
                update_receiving_ = false;
            }
            break;
        case RAW_END:
            break;
        case RAW_ABORTED:
            if (update_receiving_) {
                ota.abort_ota();
                update_receiving_ = false;
                update_error_ = "The upload was interrupted";
                LOG_BLE("[WIFI] Update upload interrupted\n");
            }
            break;
    }
}

void WifiManager::handle_update_done() {
    touch();
    if (update_error_) {
        http_->send(409, "text/plain", update_error_);
        update_error_ = nullptr;
        return;
    }
    if (!update_receiving_) {
        http_->send(400, "text/plain", "No update was received");
        return;
    }
    update_receiving_ = false;
    http_->send(200, "text/plain", "Received - applying the update and restarting");
    update_finalize_pending_ = true;
    update_finalize_at_ms_ = millis() + WIFI_OTA_FINALIZE_DELAY_MS;
}
