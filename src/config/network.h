#pragma once

//==============================================================================
// WIFI
//==============================================================================
// WiFi joins the home network for firmware updates, the diagnostic report, saved logs
// and a live log, all from a browser. It shares the radio with Bluetooth and only one
// of the two is on at a time (the BT / WiFi menu page switches them). The network name
// and password are entered in the web flasher and sent once over Bluetooth.
//
// The WiFi driver and TCP/IP stack run on Core 0 at high priority, next to grind
// control - they are compiled into the Arduino core that way. Light traffic such as
// the live log is harmless; an update upload is heavy, so updates are refused while a
// grind runs, and the upload itself suspends the grind tasks like a Bluetooth update.

#define WIFI_HOSTNAME "grinder"                                       // http://grinder.local (mDNS) and the DHCP host name
#define WIFI_AUTO_DISABLE_TIMEOUT_MS (30 * 60 * 1000)                 // Off after this long without any client activity
#define WIFI_CONNECT_SLOW_MS 30000                                    // Status reads "can't connect" after this (it keeps retrying)
#define WIFI_HTTP_PORT 80
#define WIFI_LOG_PORT 23                                              // Raw TCP: every log line as it is written (telnet, nc, PuTTY "raw")
#define WIFI_LOG_CHUNK_BYTES 1024                                     // Live log copied per read
#define WIFI_LIVE_RESPONSE_MAX_BYTES 8192                             // Most log text one /live poll returns
#define WIFI_OTA_FINALIZE_DELAY_MS 500                                // Lets the HTTP reply go out before the update is applied and the device restarts

#define WIFI_PREFS_NAMESPACE "wifi"
#define WIFI_SSID_MAX_LEN 32                                          // 802.11 limit
#define WIFI_PASSWORD_MAX_LEN 63                                      // WPA2 passphrase limit
