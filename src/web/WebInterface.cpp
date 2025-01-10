#include "WebInterface.hpp"

extern "C" {
#include "lwipopts.h"
#include "pico/cyw43_arch.h"
#include "lwip/apps/httpd.h"
}
#include "pico/stdlib.h"
#include "boards/pico_w.h"

#include "cgi.h"
#include "ssi.h"

#include <Arduino.h>

void WebInterfaceSetup(void) {

    if (cyw43_arch_init()) {
        // printf("failed to initialise\n");
        // return 1;
    }
    cyw43_arch_enable_sta_mode();
    // this seems to be the best be can do using the predefined `cyw43_pm_value` macro:
    // cyw43_wifi_pm(&cyw43_state, CYW43_PERFORMANCE_PM);
    // however it doesn't use the `CYW43_NO_POWERSAVE_MODE` value, so we do this instead:
    cyw43_wifi_pm(&cyw43_state, cyw43_pm_value(CYW43_NO_POWERSAVE_MODE, 20, 1, 1, 1));

    Serial.println("WEB- Connecting to WiFi...");
    if (cyw43_arch_wifi_connect_timeout_ms("MEML-CHIME", "chime2024", CYW43_AUTH_WPA2_AES_PSK, 30000)) {
    //if (cyw43_arch_wifi_connect_timeout_ms("MrsWildebeast", "znbiupb45cz9e4f", CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        Serial.println("WEB- Failed to connect.");
        return;
    } else {
        Serial.println("Connected.");

        extern cyw43_t cyw43_state;
        auto ip_addr = cyw43_state.netif[CYW43_ITF_STA].ip_addr.addr;
        Serial.printf("IP Address: %lu.%lu.%lu.%lu\n",
                ip_addr & 0xFF,
                (ip_addr >> 8) & 0xFF,
                (ip_addr >> 16) & 0xFF,
                ip_addr >> 24);
    }
    // turn on LED to signal connected
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);

}


void WebInterfaceRun(void) {

    httpd_init();
    ssi_init();
    cgi_init();
    Serial.println("Http server initialized.");
}