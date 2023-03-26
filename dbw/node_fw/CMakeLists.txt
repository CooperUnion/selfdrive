cmake_minimum_required(VERSION 3.16.0)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Spam as many components as we can at EXCLUDE_COMPONENTS.
# This shaves off quite a bit of build time, but doesn't break anything
# because explicit dependencies between components will override this list.
set(EXCLUDE_COMPONENTS
        freemodbus coap asio mbedtls wpa_supplicant wifi_provisioning spiffs
        openssl nghttp mdns lwip libsodium fatfs expat esp_wifi
        esp_websocket_client esp_netif esp_lcd esp_http_client esp_http_server
        protocomm esp_https_ota esp_eth esp_event esp_tls cbor tinycbor
        esp_local_ctrl mqtt esp-mqtt nghttp2 sdmmc tcp_transport wear_levelling
        jsmn tcpip_adapter unity vfs)

project(node_firmware2)
