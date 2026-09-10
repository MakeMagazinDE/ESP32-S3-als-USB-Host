#include <Arduino.h>
#include <WiFi.h>
#include <time.h>

extern "C" {
#include "esp_err.h"
#include "esp_event.h"
#include "mqtt_client.h"
#include "usb/usb_host.h"
#include "usb/usb_types_ch9.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
}

// =============================================================================
// Benutzerkonfiguration
// =============================================================================

static constexpr char WIFI_SSID[] =
    "xxx";

static constexpr char WIFI_PASSWORD[] =
    "xxx";


static constexpr char MQTT_URI[] =
    "wss://URL/mqtt";

static constexpr char MQTT_USERNAME[] =
    "i-buddy";

static constexpr char MQTT_PASSWORD[] =
    "";

static constexpr char MQTT_TOPIC[] =
    "";

static constexpr char MQTT_TRIGGER[] =
    "";

static constexpr uint32_t HEART_FLASH_TIME_MS = 5000;

static constexpr uint32_t WING_TOGGLE_INTERVAL_MS = 200;
static constexpr uint32_t TURN_TOGGLE_INTERVAL_MS = 500;

static constexpr uint32_t STATUS_INTERVAL_MS = 30000;

static constexpr char ISRG_ROOT_X1_PEM[] = R"EOF(
-----BEGIN CERTIFICATE-----

-----END CERTIFICATE-----
)EOF";

static constexpr uint16_t IBUDDY_VID = 0x1130;
static constexpr uint16_t IBUDDY_PID_1 = 0x0001;
static constexpr uint16_t IBUDDY_PID_2 = 0x0002;

static constexpr uint8_t IBUDDY_INTERFACE = 1;


static constexpr uint8_t IBUDDY_SETUP_REPORT[8] = {
    0x22, 0x09, 0x00, 0x02, 0x01, 0x00, 0x00, 0x00
};

static constexpr uint8_t IBUDDY_HEART_ON  = 0x78;
static constexpr uint8_t IBUDDY_HEART_OFF = 0xF8;

static constexpr uint8_t BIT_HEART = 0x80;
static constexpr uint8_t BIT_BLUE  = 0x40;
static constexpr uint8_t BIT_GREEN = 0x20;
static constexpr uint8_t BIT_RED   = 0x10;

static constexpr uint8_t BIT_WING_UP     = 0x08;
static constexpr uint8_t BIT_WING_DOWN   = 0x04;
static constexpr uint8_t BIT_TURN_LEFT   = 0x02;
static constexpr uint8_t BIT_TURN_RIGHT  = 0x01;

static uint8_t buildBuddyCommand(
    bool heartOn,
    bool blueOn,
    bool greenOn,
    bool redOn,
    uint8_t motionBits
) {
    uint8_t cmd = 0;
    cmd |= heartOn ? 0 : BIT_HEART;
    cmd |= blueOn  ? 0 : BIT_BLUE;
    cmd |= greenOn ? 0 : BIT_GREEN;
    cmd |= redOn   ? 0 : BIT_RED;
    cmd |= motionBits;
    return cmd;
}

static constexpr size_t IBUDDY_REPORT_LENGTH = 8;
static constexpr size_t CONTROL_TRANSFER_LENGTH =
    sizeof(usb_setup_packet_t) + IBUDDY_REPORT_LENGTH;


enum class BuddyCommand : uint8_t {
    FlashHeart
};

static QueueHandle_t buddyCommandQueue = nullptr;

static usb_host_client_handle_t usbClientHandle = nullptr;
static usb_device_handle_t buddyDeviceHandle = nullptr;
static usb_transfer_t* controlTransfer = nullptr;

static volatile bool buddyConnected = false;
static volatile bool controlTransferFinished = false;
static volatile usb_transfer_status_t controlTransferStatus =
    USB_TRANSFER_STATUS_ERROR;

static esp_mqtt_client_handle_t mqttClient = nullptr;
static volatile bool mqttConnected = false;
static volatile bool mqttSubscribed = false;

// Puffer für den seltenen Fall, dass ESP-MQTT eine Nachricht aufteilt.
static constexpr size_t MQTT_RECEIVE_BUFFER_SIZE = 128;
static char mqttReceiveTopic[64] = {};
static char mqttReceiveData[MQTT_RECEIVE_BUFFER_SIZE] = {};
static int mqttExpectedLength = 0;
static int mqttReceivedLength = 0;


static String makeMqttClientId()
{
    const uint64_t chipId = ESP.getEfuseMac();

    char id[48];
    snprintf(
        id,
        sizeof(id),
        "ibuddy-esp32s3-%04X%08X",
        static_cast<uint16_t>(chipId >> 32),
        static_cast<uint32_t>(chipId)
    );

    return String(id);
}

static const char* wifiStatusToString(wl_status_t status)
{
    switch (status) {
        case WL_IDLE_STATUS:
            return "IDLE";

        case WL_NO_SSID_AVAIL:
            return "SSID_NICHT_GEFUNDEN";

        case WL_SCAN_COMPLETED:
            return "SCAN_ABGESCHLOSSEN";

        case WL_CONNECTED:
            return "VERBUNDEN";

        case WL_CONNECT_FAILED:
            return "VERBINDUNG_FEHLGESCHLAGEN";

        case WL_CONNECTION_LOST:
            return "VERBINDUNG_VERLOREN";

        case WL_DISCONNECTED:
            return "GETRENNT";

        default:
            return "UNBEKANNT";
    }
}

static const char* usbTransferStatusToString(usb_transfer_status_t status)
{
    switch (status) {
        case USB_TRANSFER_STATUS_COMPLETED:
            return "COMPLETED";

        case USB_TRANSFER_STATUS_ERROR:
            return "ERROR";

        case USB_TRANSFER_STATUS_TIMED_OUT:
            return "TIMED_OUT";

        case USB_TRANSFER_STATUS_CANCELED:
            return "CANCELED";

        case USB_TRANSFER_STATUS_STALL:
            return "STALL";

        case USB_TRANSFER_STATUS_NO_DEVICE:
            return "NO_DEVICE";

        case USB_TRANSFER_STATUS_OVERFLOW:
            return "OVERFLOW";

        default:
            return "UNKNOWN";
    }
}

static bool connectWiFi(uint32_t timeoutMs = 30000)
{
    if (WiFi.status() == WL_CONNECTED) {
        return true;
    }

    Serial.println();
    Serial.println("[WLAN] Starte Verbindungsaufbau.");
    Serial.printf("[WLAN] SSID: \"%s\"\n", WIFI_SSID);

    WiFi.mode(WIFI_STA);
    WiFi.persistent(false);
    WiFi.setAutoReconnect(true);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    const uint32_t startedAt = millis();
    uint32_t lastStatusOutput = 0;

    while (WiFi.status() != WL_CONNECTED) {
        const uint32_t now = millis();

        if (now - lastStatusOutput >= 1000) {
            lastStatusOutput = now;

            Serial.printf(
                "[WLAN] Warte: Status=%s, vergangen=%lu ms\n",
                wifiStatusToString(WiFi.status()),
                static_cast<unsigned long>(now - startedAt)
            );
        }

        if (now - startedAt >= timeoutMs) {
            Serial.printf(
                "[WLAN] Zeitüberschreitung nach %lu ms; Status=%s\n",
                static_cast<unsigned long>(timeoutMs),
                wifiStatusToString(WiFi.status())
            );
            return false;
        }

        delay(100);
    }

    Serial.println("[WLAN] Verbindung hergestellt.");
    Serial.printf(
        "[WLAN] IP-Adresse: %s\n",
        WiFi.localIP().toString().c_str()
    );
    Serial.printf(
        "[WLAN] Gateway:    %s\n",
        WiFi.gatewayIP().toString().c_str()
    );
    Serial.printf(
        "[WLAN] DNS:        %s\n",
        WiFi.dnsIP().toString().c_str()
    );
    Serial.printf(
        "[WLAN] MAC:        %s\n",
        WiFi.macAddress().c_str()
    );
    Serial.printf("[WLAN] Signal:     %d dBm\n", WiFi.RSSI());

    return true;
}

static bool syncTime(uint32_t timeoutMs = 15000)
{
    Serial.println("[ZEIT] Synchronisiere Systemzeit per NTP...");

    // UTC, keine Sommerzeit-Verschiebung nötig - Zertifikatsprüfung
    // braucht nur eine plausible absolute Zeit, keine Lokalzeit.
    configTime(0, 0, "pool.ntp.org", "time.nist.gov");

    const uint32_t startedAt = millis();
    time_t now = time(nullptr);

    // Vor dem NTP-Sync steht die Uhr auf 1.1.1970. Wir warten, bis ein
    // plausibles Datum (nach dem Jahr 2000) erreicht ist.
    while (now < 946684800) { // 1.1.2000, 00:00 UTC
        if (millis() - startedAt >= timeoutMs) {
            Serial.println("[ZEIT] NTP-Synchronisation fehlgeschlagen (Timeout).");
            return false;
        }

        delay(200);
        now = time(nullptr);
    }

    struct tm timeInfo;
    gmtime_r(&now, &timeInfo);

    char buffer[32];
    strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &timeInfo);

    Serial.printf("[ZEIT] Systemzeit gesetzt (UTC): %s\n", buffer);
    return true;
}

static void controlTransferCallback(usb_transfer_t* transfer)
{
    controlTransferStatus = transfer->status;
    controlTransferFinished = true;
}

static void closeBuddyDevice()
{
    if (buddyDeviceHandle == nullptr) {
        buddyConnected = false;
        return;
    }

    Serial.println("[USB] Schließe i-Buddy-Gerätehandle.");

    const esp_err_t result =
        usb_host_device_close(usbClientHandle, buddyDeviceHandle);

    if (result != ESP_OK) {
        Serial.printf(
            "[USB] usb_host_device_close fehlgeschlagen: %s\n",
            esp_err_to_name(result)
        );
    }

    buddyDeviceHandle = nullptr;
    buddyConnected = false;

    Serial.println("[USB] i-Buddy getrennt.");
}

static void tryOpenBuddy(uint8_t deviceAddress)
{
    if (buddyDeviceHandle != nullptr) {
        Serial.println(
            "[USB] Ein i-Buddy ist bereits geöffnet; weiteres Gerät ignoriert."
        );
        return;
    }

    Serial.printf(
        "[USB] Neues USB-Gerät an Adresse %u erkannt.\n",
        deviceAddress
    );

    usb_device_handle_t candidateHandle = nullptr;

    esp_err_t result = usb_host_device_open(
        usbClientHandle,
        deviceAddress,
        &candidateHandle
    );

    if (result != ESP_OK) {
        Serial.printf(
            "[USB] Gerät %u konnte nicht geöffnet werden: %s\n",
            deviceAddress,
            esp_err_to_name(result)
        );
        return;
    }

    const usb_device_desc_t* deviceDescriptor = nullptr;

    result = usb_host_get_device_descriptor(
        candidateHandle,
        &deviceDescriptor
    );

    if (result != ESP_OK || deviceDescriptor == nullptr) {
        Serial.printf(
            "[USB] Gerätedeskriptor konnte nicht gelesen werden: %s\n",
            esp_err_to_name(result)
        );

        usb_host_device_close(usbClientHandle, candidateHandle);
        return;
    }

    Serial.printf(
        "[USB] Deskriptor: VID=%04X, PID=%04X, Klasse=%02X\n",
        deviceDescriptor->idVendor,
        deviceDescriptor->idProduct,
        deviceDescriptor->bDeviceClass
    );

    const bool supportedBuddy =
    deviceDescriptor->idVendor == IBUDDY_VID &&
    (
        deviceDescriptor->idProduct == IBUDDY_PID_1 ||
        deviceDescriptor->idProduct == IBUDDY_PID_2
    );

if (!supportedBuddy) {
    Serial.println("[USB] Gerät ist kein unterstützter i-Buddy.");
    usb_host_device_close(usbClientHandle, candidateHandle);
    return;
}

Serial.printf(
    "[USB] Unterstützte Buddy-Variante erkannt: VID=%04X PID=%04X\n",
    deviceDescriptor->idVendor,
    deviceDescriptor->idProduct
);

    buddyDeviceHandle = candidateHandle;
    buddyConnected = true;

    Serial.println("[USB] Passender i-Buddy gefunden und geöffnet.");
}

static void usbClientEventCallback(
    const usb_host_client_event_msg_t* eventMessage,
    void* argument
) {
    (void)argument;

    if (eventMessage == nullptr) {
        return;
    }

    switch (eventMessage->event) {
        case USB_HOST_CLIENT_EVENT_NEW_DEV:
            tryOpenBuddy(eventMessage->new_dev.address);
            break;

        case USB_HOST_CLIENT_EVENT_DEV_GONE:
            if (
                buddyDeviceHandle != nullptr &&
                eventMessage->dev_gone.dev_hdl == buddyDeviceHandle
            ) {
                Serial.println("[USB] i-Buddy wurde physisch entfernt.");
                closeBuddyDevice();
            } else {
                Serial.println(
                    "[USB] Ein nicht verwendetes USB-Gerät wurde entfernt."
                );
            }
            break;

        default:
            Serial.printf(
                "[USB] Unbekanntes Client-Ereignis: %d\n",
                static_cast<int>(eventMessage->event)
            );
            break;
    }
}

static bool waitForControlTransfer(uint32_t timeoutMs)
{
    const uint32_t startedAt = millis();

    while (!controlTransferFinished) {
        const esp_err_t result = usb_host_client_handle_events(
            usbClientHandle,
            pdMS_TO_TICKS(20)
        );

        if (result != ESP_OK && result != ESP_ERR_TIMEOUT) {
            Serial.printf(
                "[USB] Fehler beim Bearbeiten der Client-Ereignisse: %s\n",
                esp_err_to_name(result)
            );
            return false;
        }

        if (millis() - startedAt >= timeoutMs) {
            Serial.printf(
                "[USB] Control-Transfer nach %lu ms abgebrochen.\n",
                static_cast<unsigned long>(timeoutMs)
            );
            return false;
        }
    }

    if (controlTransferStatus != USB_TRANSFER_STATUS_COMPLETED) {
        Serial.printf(
            "[USB] Control-Transfer fehlgeschlagen: %s (%d)\n",
            usbTransferStatusToString(controlTransferStatus),
            static_cast<int>(controlTransferStatus)
        );
        return false;
    }

    return true;
}

static bool sendBuddyRawReport(const uint8_t* data8)
{
    if (
        !buddyConnected ||
        buddyDeviceHandle == nullptr ||
        controlTransfer == nullptr
    ) {
        Serial.println(
            "[USB] Report kann nicht gesendet werden: i-Buddy nicht verfügbar."
        );
        return false;
    }

    usb_setup_packet_t setupPacket = {};
    setupPacket.bmRequestType = 0x21;
    setupPacket.bRequest = 0x09;
    setupPacket.wValue = 0x0002;   // korrigiert: war 0x0200
    setupPacket.wIndex = IBUDDY_INTERFACE;
    setupPacket.wLength = IBUDDY_REPORT_LENGTH;

    memcpy(
        controlTransfer->data_buffer,
        &setupPacket,
        sizeof(setupPacket)
    );

    uint8_t* report =
        controlTransfer->data_buffer + sizeof(usb_setup_packet_t);

    memcpy(report, data8, IBUDDY_REPORT_LENGTH);

    Serial.printf(
        "[USB] HID SET_REPORT: Interface=%u, Länge=%u, Daten=",
        IBUDDY_INTERFACE,
        static_cast<unsigned int>(IBUDDY_REPORT_LENGTH)
    );

    for (size_t i = 0; i < IBUDDY_REPORT_LENGTH; ++i) {
        Serial.printf("%02X ", report[i]);
    }

    Serial.println();

    controlTransfer->device_handle = buddyDeviceHandle;
    controlTransfer->bEndpointAddress = 0;
    controlTransfer->callback = controlTransferCallback;
    controlTransfer->context = nullptr;
    controlTransfer->num_bytes = CONTROL_TRANSFER_LENGTH;

    controlTransferFinished = false;
    controlTransferStatus = USB_TRANSFER_STATUS_ERROR;

    const esp_err_t result = usb_host_transfer_submit_control(
        usbClientHandle,
        controlTransfer
    );

    if (result != ESP_OK) {
        Serial.printf(
            "[USB] Control-Transfer konnte nicht gestartet werden: %s\n",
            esp_err_to_name(result)
        );
        return false;
    }

    if (!waitForControlTransfer(1000)) {
        return false;
    }

    Serial.println("[USB] Report erfolgreich übertragen.");
    return true;
}

static bool sendBuddyReport(uint8_t controlCode)
{
    if (!sendBuddyRawReport(IBUDDY_SETUP_REPORT)) {
        Serial.println("[USB] Setup-Report fehlgeschlagen.");
        return false;
    }

    Serial.printf(
        "[USB] Sende i-Buddy-Report, Steuerbyte=0x%02X.\n",
        controlCode
    );

    uint8_t header[8] = {0x55, 0x53, 0x42, 0x43, 0x00, 0x40, 0x02, controlCode};
    return sendBuddyRawReport(header);
}

static void flashBuddyHeart()
{
    Serial.println(
        "[BUDDY] Animation wird gestartet: Herz an, lila Kopf, "
        "Flügelschlag, Links-Rechts-Drehen (5s)."
    );

    if (!buddyConnected) {
        Serial.println(
            "[BUDDY] Abbruch: Der i-Buddy ist nicht angeschlossen."
        );
        return;
    }

    bool wingUp = true;
    bool turnLeft = true;

    uint8_t motion = (wingUp ? BIT_WING_UP : BIT_WING_DOWN) |
                      (turnLeft ? BIT_TURN_LEFT : BIT_TURN_RIGHT);

    uint8_t cmd = buildBuddyCommand(
        /*heartOn=*/true,
        /*blueOn=*/true,
        /*greenOn=*/false,
        /*redOn=*/true,
        motion
    );

    if (!sendBuddyReport(cmd)) {
        Serial.println("[BUDDY] Start der Animation fehlgeschlagen.");
        return;
    }

    const uint32_t startedAt = millis();
    uint32_t lastWingToggle = startedAt;
    uint32_t lastTurnToggle = startedAt;

    while (millis() - startedAt < HEART_FLASH_TIME_MS) {
        const esp_err_t result = usb_host_client_handle_events(
            usbClientHandle,
            pdMS_TO_TICKS(10)
        );

        if (result != ESP_OK && result != ESP_ERR_TIMEOUT) {
            Serial.printf(
                "[USB] Fehler während der Animation: %s\n",
                esp_err_to_name(result)
            );
            break;
        }

        if (!buddyConnected) {
            Serial.println(
                "[BUDDY] i-Buddy wurde während der Animation entfernt."
            );
            return;
        }

        const uint32_t now = millis();
        bool stateChanged = false;

        if (now - lastWingToggle >= WING_TOGGLE_INTERVAL_MS) {
            wingUp = !wingUp;
            lastWingToggle = now;
            stateChanged = true;
        }

        if (now - lastTurnToggle >= TURN_TOGGLE_INTERVAL_MS) {
            turnLeft = !turnLeft;
            lastTurnToggle = now;
            stateChanged = true;
        }

        if (stateChanged) {
            motion = (wingUp ? BIT_WING_UP : BIT_WING_DOWN) |
                     (turnLeft ? BIT_TURN_LEFT : BIT_TURN_RIGHT);

            cmd = buildBuddyCommand(
                /*heartOn=*/true,
                /*blueOn=*/true,
                /*greenOn=*/false,
                /*redOn=*/true,
                motion
            );

            sendBuddyReport(cmd);
        }
    }

    if (!buddyConnected) {
        Serial.println(
            "[BUDDY] i-Buddy wurde während der Animation entfernt."
        );
        return;
    }

    Serial.println("[BUDDY] Animation beendet, schalte alles aus.");

    if (sendBuddyReport(IBUDDY_HEART_OFF)) {
        Serial.println("[BUDDY] Animation abgeschlossen.");
    } else {
        Serial.println("[BUDDY] Ausschalten am Ende fehlgeschlagen.");
    }
}

static void usbHostDaemonTask(void* parameter)
{
    (void)parameter;

    Serial.println("[USB] Host-Daemon-Task läuft.");

    while (true) {
        uint32_t eventFlags = 0;

        const esp_err_t result = usb_host_lib_handle_events(
            portMAX_DELAY,
            &eventFlags
        );

        if (result != ESP_OK) {
            Serial.printf(
                "[USB] Host-Daemon-Fehler: %s\n",
                esp_err_to_name(result)
            );
        }
    }
}

static void usbClientTask(void* parameter)
{
    (void)parameter;

    Serial.println("[USB] Registriere USB-Client.");

    usb_host_client_config_t clientConfig = {};
    clientConfig.is_synchronous = false;
    clientConfig.max_num_event_msg = 5;
    clientConfig.async.client_event_callback = usbClientEventCallback;
    clientConfig.async.callback_arg = nullptr;

    esp_err_t result = usb_host_client_register(
        &clientConfig,
        &usbClientHandle
    );

    if (result != ESP_OK) {
        Serial.printf(
            "[USB] USB-Client-Registrierung fehlgeschlagen: %s\n",
            esp_err_to_name(result)
        );
        vTaskDelete(nullptr);
        return;
    }

    result = usb_host_transfer_alloc(
        CONTROL_TRANSFER_LENGTH,
        0,
        &controlTransfer
    );

    if (result != ESP_OK) {
        Serial.printf(
            "[USB] Transferpuffer konnte nicht angelegt werden: %s\n",
            esp_err_to_name(result)
        );

        usb_host_client_deregister(usbClientHandle);
        usbClientHandle = nullptr;

        vTaskDelete(nullptr);
        return;
    }

    Serial.println(
        "[USB] USB-Client bereit. Der i-Buddy kann angeschlossen werden."
    );

    bool previousBuddyConnected = false;

    while (true) {
        result = usb_host_client_handle_events(
            usbClientHandle,
            pdMS_TO_TICKS(20)
        );

        if (result != ESP_OK && result != ESP_ERR_TIMEOUT) {
            Serial.printf(
                "[USB] Fehler beim Bearbeiten von USB-Ereignissen: %s\n",
                esp_err_to_name(result)
            );
        }

        /*
         * Direkt nach dem Verbinden einen definierten Aus-Zustand setzen.
         */
        if (buddyConnected && !previousBuddyConnected) {
            Serial.println(
                "[BUDDY] Initialisiere i-Buddy im ausgeschalteten Zustand."
            );

            if (!sendBuddyReport(IBUDDY_HEART_OFF)) {
                Serial.println(
                    "[BUDDY] Initialisierung des Aus-Zustands fehlgeschlagen."
                );
            }
        }

        previousBuddyConnected = buddyConnected;

        BuddyCommand command;

        while (
            xQueueReceive(
                buddyCommandQueue,
                &command,
                0
            ) == pdTRUE
        ) {
            switch (command) {
                case BuddyCommand::FlashHeart:
                    flashBuddyHeart();
                    break;
            }
        }
    }
}

static bool startUsbHost()
{
    Serial.println("[USB] Lege Befehlsqueue an.");

    buddyCommandQueue = xQueueCreate(
        8,
        sizeof(BuddyCommand)
    );

    if (buddyCommandQueue == nullptr) {
        Serial.println("[USB] Befehlsqueue konnte nicht angelegt werden.");
        return false;
    }

    usb_host_config_t hostConfig = {};
    hostConfig.skip_phy_setup = false;
    hostConfig.intr_flags = ESP_INTR_FLAG_LEVEL1;

    Serial.println("[USB] Installiere USB-Host-Bibliothek.");

    const esp_err_t result = usb_host_install(&hostConfig);

    if (result != ESP_OK) {
        Serial.printf(
            "[USB] USB-Host konnte nicht installiert werden: %s\n",
            esp_err_to_name(result)
        );
        return false;
    }

    const BaseType_t daemonCreated = xTaskCreatePinnedToCore(
        usbHostDaemonTask,
        "usb-host-daemon",
        4096,
        nullptr,
        2,
        nullptr,
        0
    );

    const BaseType_t clientCreated = xTaskCreatePinnedToCore(
        usbClientTask,
        "usb-ibuddy-client",
        6144,
        nullptr,
        3,
        nullptr,
        0
    );

    if (daemonCreated != pdPASS || clientCreated != pdPASS) {
        Serial.println("[USB] USB-Tasks konnten nicht angelegt werden.");
        return false;
    }

    Serial.println("[USB] USB-Host und Tasks wurden gestartet.");
    return true;
}

static void resetMqttReceiveBuffer()
{
    mqttReceiveTopic[0] = '\0';
    mqttReceiveData[0] = '\0';
    mqttExpectedLength = 0;
    mqttReceivedLength = 0;
}

static void processCompleteMqttMessage()
{
    String topic(mqttReceiveTopic);
    String message(mqttReceiveData);
    message.trim();

    Serial.println("[MQTT] Vollständige Nachricht empfangen:");
    Serial.printf("[MQTT]   Topic:     \"%s\"\n", topic.c_str());
    Serial.printf("[MQTT]   Nachricht: \"%s\"\n", message.c_str());
    Serial.printf("[MQTT]   Länge:     %d Byte\n", mqttReceivedLength);

    if (topic != MQTT_TOPIC) {
        Serial.println("[MQTT] Nachricht ignoriert: anderes Topic.");
        return;
    }

    if (message != MQTT_TRIGGER) {
        Serial.printf(
            "[MQTT] Nachricht ignoriert: erwartet wird \"%s\".\n",
            MQTT_TRIGGER
        );
        return;
    }

    Serial.println("[MQTT] Herz-Trigger erkannt.");

    if (buddyCommandQueue == nullptr) {
        Serial.println(
            "[MQTT] Fehler: USB-Befehlsqueue wurde nicht initialisiert."
        );
        return;
    }

    const BuddyCommand command = BuddyCommand::FlashHeart;

    if (xQueueSend(buddyCommandQueue, &command, 0) == pdTRUE) {
        Serial.println(
            "[MQTT] Blinkbefehl erfolgreich an den USB-Task übergeben."
        );
    } else {
        Serial.println(
            "[MQTT] USB-Befehlsqueue ist voll; Blinkbefehl verworfen."
        );
    }
}

static void handleMqttDataEvent(const esp_mqtt_event_handle_t event)
{
    if (event == nullptr) {
        return;
    }

    Serial.printf(
        "[MQTT] Datenblock: Offset=%d, Block=%d, Gesamt=%d Byte\n",
        event->current_data_offset,
        event->data_len,
        event->total_data_len
    );

    if (event->current_data_offset == 0) {
        resetMqttReceiveBuffer();

        mqttExpectedLength = event->total_data_len;

        if (
            mqttExpectedLength < 0 ||
            mqttExpectedLength >= static_cast<int>(sizeof(mqttReceiveData))
        ) {
            Serial.printf(
                "[MQTT] Nachricht zu groß; maximal %u Byte erlaubt.\n",
                static_cast<unsigned int>(sizeof(mqttReceiveData) - 1)
            );
            resetMqttReceiveBuffer();
            return;
        }

        const int topicLength =
            min(event->topic_len, static_cast<int>(sizeof(mqttReceiveTopic) - 1));

        if (event->topic != nullptr && topicLength > 0) {
            memcpy(mqttReceiveTopic, event->topic, topicLength);
            mqttReceiveTopic[topicLength] = '\0';
        }
    }

    if (
        mqttExpectedLength <= 0 ||
        event->current_data_offset < 0 ||
        event->data_len < 0
    ) {
        Serial.println("[MQTT] Ungültiger oder verworfener Datenblock.");
        return;
    }

    const int targetOffset = event->current_data_offset;
    const int bytesAvailable =
        static_cast<int>(sizeof(mqttReceiveData) - 1) - targetOffset;
    const int bytesToCopy = min(event->data_len, bytesAvailable);

    if (
        event->data != nullptr &&
        bytesToCopy > 0 &&
        targetOffset >= 0
    ) {
        memcpy(
            mqttReceiveData + targetOffset,
            event->data,
            bytesToCopy
        );

        mqttReceivedLength = max(
            mqttReceivedLength,
            targetOffset + bytesToCopy
        );

        mqttReceiveData[mqttReceivedLength] = '\0';
    }

    if (mqttReceivedLength >= mqttExpectedLength) {
        processCompleteMqttMessage();
        resetMqttReceiveBuffer();
    }
}

static void mqttEventHandler(
    void* handlerArgs,
    esp_event_base_t base,
    int32_t eventId,
    void* eventData
) {
    (void)handlerArgs;
    (void)base;

    const esp_mqtt_event_handle_t event =
        static_cast<esp_mqtt_event_handle_t>(eventData);

    switch (static_cast<esp_mqtt_event_id_t>(eventId)) {
        case MQTT_EVENT_BEFORE_CONNECT:
            Serial.println("[MQTT] Verbindungsaufbau wird gestartet.");
            Serial.printf("[MQTT] Ziel: %s\n", MQTT_URI);
            break;

        case MQTT_EVENT_CONNECTED: {
            mqttConnected = true;
            mqttSubscribed = false;

            Serial.println("[MQTT] MQTT-over-WSS-Verbindung hergestellt.");

            const int messageId = esp_mqtt_client_subscribe(
                event->client,
                MQTT_TOPIC,
                0
            );

            if (messageId >= 0) {
                Serial.printf(
                    "[MQTT] Abonnement angefordert: Topic=\"%s\", msg_id=%d\n",
                    MQTT_TOPIC,
                    messageId
                );
            } else {
                Serial.println(
                    "[MQTT] Abonnement konnte nicht angefordert werden."
                );
            }
            break;
        }

        case MQTT_EVENT_DISCONNECTED:
            mqttConnected = false;
            mqttSubscribed = false;
            resetMqttReceiveBuffer();

            Serial.println("[MQTT] Verbindung getrennt.");
            Serial.println("[MQTT] Automatischer Neuaufbau bleibt aktiv.");
            break;

        case MQTT_EVENT_SUBSCRIBED:
            mqttSubscribed = true;

            Serial.printf(
                "[MQTT] Topic erfolgreich abonniert, msg_id=%d.\n",
                event->msg_id
            );
            break;

        case MQTT_EVENT_UNSUBSCRIBED:
            mqttSubscribed = false;

            Serial.printf(
                "[MQTT] Abonnement beendet, msg_id=%d.\n",
                event->msg_id
            );
            break;

        case MQTT_EVENT_PUBLISHED:
            Serial.printf(
                "[MQTT] Veröffentlichung bestätigt, msg_id=%d.\n",
                event->msg_id
            );
            break;

        case MQTT_EVENT_DATA:
            handleMqttDataEvent(event);
            break;

        case MQTT_EVENT_ERROR:
            Serial.println("[MQTT] Verbindungs- oder Protokollfehler.");

            if (event != nullptr && event->error_handle != nullptr) {
                Serial.printf(
                    "[MQTT] Fehlertyp:             %d\n",
                    static_cast<int>(event->error_handle->error_type)
                );
                Serial.printf(
                    "[MQTT] TLS/ESP-Fehler:         0x%X\n",
                    event->error_handle->esp_tls_last_esp_err
                );
                Serial.printf(
                    "[MQTT] TLS-Stackfehler:        0x%X\n",
                    event->error_handle->esp_tls_stack_err
                );
                Serial.printf(
                    "[MQTT] Socket errno:           %d\n",
                    event->error_handle->esp_transport_sock_errno
                );
                Serial.printf(
                    "[MQTT] MQTT-Connect-Rückgabe:  %d\n",
                    event->error_handle->connect_return_code
                );
            }
            break;

        default:
            Serial.printf(
                "[MQTT] Ereignis-ID: %ld\n",
                static_cast<long>(eventId)
            );
            break;
    }
}

static void startMqtt()
{
    if (mqttClient != nullptr) {
        Serial.println("[MQTT] Client ist bereits initialisiert.");
        return;
    }

    static String clientId = makeMqttClientId();

    Serial.println();
    Serial.println("[MQTT] Initialisiere MQTT über Secure WebSocket.");
    Serial.printf("[MQTT] URI:       %s\n", MQTT_URI);
    Serial.printf("[MQTT] Client-ID: %s\n", clientId.c_str());
    Serial.printf("[MQTT] Topic:     %s\n", MQTT_TOPIC);

    esp_mqtt_client_config_t mqttConfig = {};
    mqttConfig.uri = MQTT_URI;
    mqttConfig.client_id = clientId.c_str();
    mqttConfig.keepalive = 30;
    mqttConfig.disable_auto_reconnect = false;

    mqttConfig.cert_pem = ISRG_ROOT_X1_PEM;

    if (strlen(MQTT_USERNAME) > 0) {
        mqttConfig.username = MQTT_USERNAME;
        Serial.println("[MQTT] Benutzeranmeldung ist aktiviert.");
    } else {
        Serial.println("[MQTT] Keine MQTT-Benutzeranmeldung konfiguriert.");
    }

    if (strlen(MQTT_PASSWORD) > 0) {
        mqttConfig.password = MQTT_PASSWORD;
    }

    mqttClient = esp_mqtt_client_init(&mqttConfig);

    if (mqttClient == nullptr) {
        Serial.println("[MQTT] Client konnte nicht initialisiert werden.");
        return;
    }

esp_err_t result = esp_mqtt_client_register_event(
    mqttClient,
    static_cast<esp_mqtt_event_id_t>(ESP_EVENT_ANY_ID),
    mqttEventHandler,
    nullptr
);

    if (result != ESP_OK) {
        Serial.printf(
            "[MQTT] Eventhandler konnte nicht registriert werden: %s\n",
            esp_err_to_name(result)
        );

        esp_mqtt_client_destroy(mqttClient);
        mqttClient = nullptr;
        return;
    }

    result = esp_mqtt_client_start(mqttClient);

    if (result != ESP_OK) {
        Serial.printf(
            "[MQTT] Clientstart fehlgeschlagen: %s\n",
            esp_err_to_name(result)
        );

        esp_mqtt_client_destroy(mqttClient);
        mqttClient = nullptr;
        return;
    }

    Serial.println("[MQTT] Client gestartet.");
}

static void printSystemStatus()
{
    Serial.println();
    Serial.println("[STATUS] Regelmäßiger Systemstatus:");
    Serial.printf(
        "[STATUS]   Laufzeit:       %lu s\n",
        static_cast<unsigned long>(millis() / 1000)
    );
    Serial.printf(
        "[STATUS]   WLAN:           %s\n",
        WiFi.status() == WL_CONNECTED ? "verbunden" : "getrennt"
    );

    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("[STATUS]   WLAN-Signal:    %d dBm\n", WiFi.RSSI());
        Serial.printf(
            "[STATUS]   IP-Adresse:     %s\n",
            WiFi.localIP().toString().c_str()
        );
    }

    Serial.printf(
        "[STATUS]   MQTT:           %s\n",
        mqttConnected ? "verbunden" : "getrennt"
    );
    Serial.printf(
        "[STATUS]   Abonnement:     %s\n",
        mqttSubscribed ? "aktiv" : "nicht aktiv"
    );
    Serial.printf(
        "[STATUS]   i-Buddy:        %s\n",
        buddyConnected ? "angeschlossen" : "nicht angeschlossen"
    );
    Serial.printf(
        "[STATUS]   Freier Heap:    %u Byte\n",
        ESP.getFreeHeap()
    );
}


void setup()
{
    Serial.begin(115200);
    delay(1500);

    Serial.println();
    Serial.println("========================================");
    Serial.println(" ESP32-S3 i-Buddy MQTT über WSS");
    Serial.println("========================================");

    Serial.printf(
        "[SYSTEM] Firmware gebaut: %s %s\n",
        __DATE__,
        __TIME__
    );
    Serial.printf("[SYSTEM] Chipmodell: %s\n", ESP.getChipModel());
    Serial.printf("[SYSTEM] CPU:        %u MHz\n", ESP.getCpuFreqMHz());
    Serial.printf("[SYSTEM] Flash:      %u Byte\n", ESP.getFlashChipSize());
    Serial.printf("[SYSTEM] PSRAM:      %u Byte\n", ESP.getPsramSize());
    Serial.printf("[SYSTEM] Freier Heap:%u Byte\n", ESP.getFreeHeap());

    Serial.println();
    Serial.println("[USB] Starte USB-Host.");

    if (startUsbHost()) {
        Serial.println("[USB] USB-Host erfolgreich gestartet.");
    } else {
        Serial.println("[USB] USB-Host konnte nicht gestartet werden.");
    }

    if (connectWiFi()) {
        syncTime();
        startMqtt();
    } else {
        Serial.println(
            "[WLAN] Erster Verbindungsversuch fehlgeschlagen; "
            "weitere Versuche folgen in loop()."
        );
    }
}

void loop()
{
    static uint32_t lastWiFiAttempt = 0;
    static uint32_t lastStatusOutput = 0;
    static bool previousWiFiConnected = false;

    const uint32_t now = millis();
    const bool wifiConnected = WiFi.status() == WL_CONNECTED;

    if (wifiConnected != previousWiFiConnected) {
        if (wifiConnected) {
            Serial.println("[WLAN] Verbindung ist wieder verfügbar.");
            Serial.printf(
                "[WLAN] IP-Adresse: %s\n",
                WiFi.localIP().toString().c_str()
            );

            if (mqttClient == nullptr) {
                syncTime();
                startMqtt();
            }
        } else {
            Serial.println("[WLAN] WLAN-Verbindung wurde getrennt.");
            mqttConnected = false;
            mqttSubscribed = false;
        }

        previousWiFiConnected = wifiConnected;
    }

    if (!wifiConnected && now - lastWiFiAttempt >= 10000) {
        lastWiFiAttempt = now;

        Serial.println();
        Serial.println(
            "[WLAN] Verbindung fehlt; starte einen neuen Versuch."
        );

        if (connectWiFi(20000) && mqttClient == nullptr) {
            syncTime();
            startMqtt();
        }
    }

    if (now - lastStatusOutput >= STATUS_INTERVAL_MS) {
        lastStatusOutput = now;
        printSystemStatus();
    }

    delay(20);
}