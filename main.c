#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h> // For time functions for timestamp 
#include <sys/time.h> // For gettimeofday

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h" // For Wi-Fi events

#include "driver/adc.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"

#include "esp_spiffs.h"
#include "dirent.h"
#include <sys/stat.h>

// --- New Includes for Wi-Fi and MQTT ---
#include "nvs_flash.h"      // Required for NVS (Non-Volatile Storage) for Wi-Fi config
#include "esp_netif.h"      // Required for network interface
#include "esp_event.h"      // Required for event handling (Wi-Fi, MQTT)
#include "esp_wifi.h"       // Explicitly include for Wi-Fi definitions 
#include "mqtt_client.h"    // Required for MQTT client
#include "cJSON.h"          //For JSON payload creation


// --- ADC Configuration ---
#define ADC_UNIT        ADC_UNIT_1
#define ADC_CHANNEL     ADC_CHANNEL_6
#define ADC_ATTEN       ADC_ATTEN_DB_11
#define ADC_WIDTH       ADC_WIDTH_BIT_12

// --- Sound Detection Thresholds ---
#define LOUD_SOUND_THRESHOLD_DEVIATION 1000
#define ADC_MIDPOINT 2048

// --- Audio Recording Configuration ---
#define SAMPLE_RATE_HZ      100
#define RECORD_DURATION_SEC 3
#define SAMPLES_TO_RECORD   (SAMPLE_RATE_HZ * RECORD_DURATION_SEC)
#define RECORD_BUFFER_SIZE  (SAMPLES_TO_RECORD * sizeof(uint16_t))

// --- SPIFFS Configuration ---
#define BASE_PATH "/spiffs"
#define RECORD_FILE_EXTENSION ".raw"

// --- Wi-Fi Configuration ---
#define WIFI_SSID      "Wokwi-GUEST" // Replace with your Wi-Fi SSID
#define WIFI_PASSWORD  ""            // Replace with your Wi-Fi password if any
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;


// --- MQTT Configuration ---
#define MQTT_BROKER_URI "mqtt://broker.hivemq.com:1883" // Public broker, test.mosquitto.org:1883 or broker.hivemq.com:1883
#define DEVICE_ID "esp32-audio-01" // Unique ID for this device
#define MQTT_TOPIC_ALERTS "esp32/audio_alerts/" DEVICE_ID


// Logger tag for this application
static const char *TAG = "ADC_Monitor";

// --- Global Variables/Flags for State Management ---
static bool sound_detected_flag = false;
static bool recording_in_progress = false;
static uint16_t *audio_record_buffer = NULL;
static int sample_count = 0;
static int recording_counter = 0;
static esp_mqtt_client_handle_t mqtt_client = NULL; // MQTT client handle


// --- Function Prototypes ---
void start_audio_recording();
void stop_audio_recording();
esp_err_t init_spiffs();
void save_audio_to_spiffs(uint16_t *buffer, int num_samples);
void list_spiffs_files();

// --- New Function Prototypes for Wi-Fi and MQTT ---
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
void wifi_init_sta();
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
void mqtt_app_start();


// --- ADC Read Task ---
void adc_read_task(void *pvParameters) {
    printf("--- Entering adc_read_task ---\n");
    audio_record_buffer = (uint16_t *)heap_caps_calloc(SAMPLES_TO_RECORD, sizeof(uint16_t), MALLOC_CAP_DMA);
    if (audio_record_buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate audio record buffer! Stopping task.");
        printf("ERROR: Audio buffer allocation failed!\n");
        vTaskDelete(NULL);
    } else {
        ESP_LOGI(TAG, "Audio record buffer allocated successfully: %d bytes.", RECORD_BUFFER_SIZE);
        printf("Audio buffer allocated.\n");
    }

    adc1_config_width(ADC_WIDTH);
    adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN);
    printf("ADC configured.\n");

    if ((ADC_MIDPOINT + LOUD_SOUND_THRESHOLD_DEVIATION) > 4095 ||
        (ADC_MIDPOINT - LOUD_SOUND_THRESHOLD_DEVIATION) < 0) {
        ESP_LOGW(TAG, "WARNING: LOUD_SOUND_THRESHOLD_DEVIATION might be too extreme for the full ADC range (0-4095).");
    }

    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        uint16_t adc_value = adc1_get_raw(ADC_CHANNEL);
        int deviation = abs((int)adc_value - ADC_MIDPOINT);

        // --- Serial Plotter Output ---
        // printf("%d\n", adc_value);
        // printf("%d,%d\n", adc_value, deviation);

        // --- Sound Detection Logic ---
        if (deviation > LOUD_SOUND_THRESHOLD_DEVIATION) {
            if (!sound_detected_flag) {
                printf("LOUD SOUND DETECTED! (ADC: %d, Dev: %d)\n", adc_value, deviation);
                ESP_LOGI(TAG, "LOUD SOUND DETECTED! (Deviation: %d)", deviation);
                sound_detected_flag = true;
                if (!recording_in_progress) {
                    start_audio_recording();
                }
            }
        } else {
            if (sound_detected_flag) {
                printf("Sound low. (ADC: %d, Dev: %d)\n", adc_value, deviation);
                ESP_LOGI(TAG, "Sound level below threshold.");
            }
            sound_detected_flag = false;
        }

        // --- Audio Recording Logic ---
        if (recording_in_progress) {
            printf("Recording sample %d/%d (ADC: %d)\n", sample_count + 1, SAMPLES_TO_RECORD, adc_value);
            if (sample_count < SAMPLES_TO_RECORD) {
                audio_record_buffer[sample_count] = adc_value;
                sample_count++;
            } else {
                stop_audio_recording();
            }
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000 / SAMPLE_RATE_HZ));
    }
    printf("--- Exiting adc_read_task ---\n");
}

// --- Function to Start Audio Recording ---
void start_audio_recording() {
    printf("--- Entering start_audio_recording ---\n");
    ESP_LOGI(TAG, "Recording event: Starting %d-second audio capture.", RECORD_DURATION_SEC);
    recording_in_progress = true;
    sample_count = 0;
    memset(audio_record_buffer, 0, RECORD_BUFFER_SIZE);
    printf("Audio recording started.\n");
    printf("--- Exiting start_audio_recording ---\n");
}

// --- Function to Stop Audio Recording ---
void stop_audio_recording() {
    printf("--- Entering stop_audio_recording ---\n");
    ESP_LOGI(TAG, "Recording event: Finished %d-second audio capture.", RECORD_DURATION_SEC);
    recording_in_progress = false;
    recording_counter++;

    printf("Calling save_audio_to_spiffs...\n");
    char audio_filename[64];
    snprintf(audio_filename, sizeof(audio_filename), "recording_%d%s", recording_counter, RECORD_FILE_EXTENSION);
    save_audio_to_spiffs(audio_record_buffer, SAMPLES_TO_RECORD);
    printf("save_audio_to_spiffs returned.\n");
    
    list_spiffs_files();

    // --- MQTT Publishing Logic ---
    if (mqtt_client) {
        printf("Preparing MQTT message...\n");
        cJSON *root = cJSON_CreateObject();
        if (root == NULL) {
            ESP_LOGE(TAG, "Failed to create cJSON object");
            printf("ERROR: Failed to create JSON object.\n");
            goto cleanup;
        }

        cJSON_AddStringToObject(root, "event", "sound_detected");
        cJSON_AddStringToObject(root, "device", DEVICE_ID);

        // Get current timestamp
        struct timeval tv;
        gettimeofday(&tv, NULL);
        time_t now = tv.tv_sec;
        struct tm timeinfo;
        char timestamp_str[64];
        localtime_r(&now, &timeinfo);
        strftime(timestamp_str, sizeof(timestamp_str), "%Y-%m-%dT%H:%M:%SZ", &timeinfo); // ISO 8601 format
        cJSON_AddStringToObject(root, "timestamp", timestamp_str);

        // Calculate a simple amplitude (e.g., max absolute deviation from midpoint)
        uint16_t max_dev = 0;
        for (int i = 0; i < SAMPLES_TO_RECORD; i++) {
            uint16_t current_dev = abs((int)audio_record_buffer[i] - ADC_MIDPOINT);
            if (current_dev > max_dev) {
                max_dev = current_dev;
            }
        }
       
        float amplitude = (float)max_dev / ADC_MIDPOINT; 
        cJSON_AddNumberToObject(root, "amplitude", amplitude);

        cJSON_AddStringToObject(root, "audio_filename", audio_filename);

        char *json_string = cJSON_Print(root);
        if (json_string == NULL) {
            ESP_LOGE(TAG, "Failed to print cJSON object to string");
            printf("ERROR: Failed to convert JSON to string.\n");
            goto cleanup;
        }

        int msg_id = esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC_ALERTS, json_string, 0, 1, 0);
        ESP_LOGI(TAG, "MQTT: Published message to %s, msg_id=%d", MQTT_TOPIC_ALERTS, msg_id);
        printf("MQTT: Published alert (msg_id=%d) to topic: %s\nPayload: %s\n", msg_id, MQTT_TOPIC_ALERTS, json_string);

        cJSON_Delete(root);
        free(json_string); 
    } else {
        ESP_LOGW(TAG, "MQTT client not initialized, cannot publish.");
        printf("WARNING: MQTT client not ready, skipping publish.\n");
    }

cleanup:
    printf("--- Exiting stop_audio_recording ---\n");
}

// --- Function to Initialize SPIFFS ---
esp_err_t init_spiffs() {
    printf("--- Entering init_spiffs ---\n");
    ESP_LOGI(TAG, "Initializing SPIFFS");

    esp_vfs_spiffs_conf_t conf = {
      .base_path = BASE_PATH,
      .partition_label = NULL,
      .max_files = 5,
      .format_if_mount_failed = true
    };

    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPIFFS register failed (%s)", esp_err_to_name(ret));
        printf("ERROR: SPIFFS registration failed: %s\n", esp_err_to_name(ret));
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format SPIFFS");
            printf("ERROR: Failed to mount or format SPIFFS\n");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
            printf("ERROR: Failed to find SPIFFS partition (check partitions.csv!)\n");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
            printf("ERROR: Failed to initialize SPIFFS: %s\n", esp_err_to_name(ret));
        }
        printf("--- Exiting init_spiffs with error ---\n");
        return ret;
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info(conf.partition_label, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
        printf("ERROR: Failed to get SPIFFS info: %s\n", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Partition size: total: %d bytes, used: %d bytes", total, used);
        printf("SPIFFS mounted. Total: %zu bytes, Used: %zu bytes\n", total, used);
    }
    printf("--- Exiting init_spiffs successfully ---\n");
    return ret;
}

// --- Function to Save Audio Buffer to SPIFFS ---
void save_audio_to_spiffs(uint16_t *buffer, int num_samples) {
    printf("--- Entering save_audio_to_spiffs ---\n");
    char file_path[64];
    snprintf(file_path, sizeof(file_path), "%s/recording_%d%s", BASE_PATH, recording_counter, RECORD_FILE_EXTENSION);

    ESP_LOGI(TAG, "Saving audio to file: %s", file_path);
    printf("Attempting to save to: %s\n", file_path);

    FILE *f = fopen(file_path, "wb");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing: %s", file_path);
        printf("ERROR: Failed to open file %s for writing!\n", file_path);
        printf("--- Exiting save_audio_to_spiffs with error ---\n");
        return;
    }

    size_t bytes_written = fwrite(buffer, sizeof(uint16_t), num_samples, f);
    fclose(f);

    if (bytes_written == num_samples) {
        ESP_LOGI(TAG, "File saved successfully! Bytes written: %zu", bytes_written * sizeof(uint16_t));
        printf("File saved successfully! Wrote %zu bytes.\n", bytes_written * sizeof(uint16_t));
    } else {
        ESP_LOGE(TAG, "Failed to write all samples to file! Wrote %zu of %d samples.", bytes_written, num_samples);
        printf("ERROR: Partial write to file! Wrote %zu of %d samples.\n", bytes_written, num_samples);
    }
    printf("--- Exiting save_audio_to_spiffs ---\n");
}

// --- Function to List Files in SPIFFS ---
void list_spiffs_files() {
    printf("--- Listing files in %s ---\n", BASE_PATH);
    DIR *dir = opendir(BASE_PATH);
    if (dir == NULL) {
        ESP_LOGE(TAG, "Failed to open directory %s", BASE_PATH);
        printf("ERROR: Failed to open SPIFFS directory!\n");
        return;
    }

    struct dirent *ent;
    while ((ent = readdir(dir)) != NULL) {
        if (ent->d_type == DT_REG) {
            char file_full_path[280];
            snprintf(file_full_path, sizeof(file_full_path), "%s/%s", BASE_PATH, ent->d_name);
            struct stat st;
            if (stat(file_full_path, &st) == 0) {
                printf("  File: %s, Size: %ld bytes\n", ent->d_name, st.st_size);
            } else {
                printf("  File: %s, Size: Unknown (stat failed)\n", ent->d_name);
            }
        }
    }
    closedir(dir);
    printf("--- Finished listing files ---\n");
}

// --- Wi-Fi Event Handler ---
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        printf("Wi-Fi: STA start, connecting...\n");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < 10) { // Max 10 retries
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "Wi-Fi: retry to connect to the AP");
            printf("Wi-Fi: Disconnected, retrying... (%d)\n", s_retry_num);
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            ESP_LOGI(TAG, "Wi-Fi: Connect to the AP failed (Max retries)");
            printf("Wi-Fi: Connection failed after max retries.\n");
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Wi-Fi: Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
        printf("Wi-Fi: Connected with IP: %d.%d.%d.%d\n", IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

// --- Wi-Fi Initialization ---
void wifi_init_sta() {
    printf("--- Entering wifi_init_sta ---\n");
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            // .ssid and .password will be set using strcpy below
            .threshold.authmode = WIFI_AUTH_OPEN, 
        },
    };
    //  way to assign SSID and Password to the array fields
    strcpy((char *)wifi_config.sta.ssid, WIFI_SSID);
    strcpy((char *)wifi_config.sta.password, WIFI_PASSWORD);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Wi-Fi: Initialized, waiting for connection...");
    printf("Wi-Fi: Initialized, connecting to %s...\n", WIFI_SSID);

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Wi-Fi: Connected to AP SSID:%s", WIFI_SSID);
        printf("Wi-Fi: Successfully connected to %s\n", WIFI_SSID);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Wi-Fi: Failed to connect to SSID:%s", WIFI_SSID);
        printf("Wi-Fi: Failed to connect to %s\n", WIFI_SSID);
    } else {
        ESP_LOGE(TAG, "Wi-Fi: UNEXPECTED EVENT");
        printf("Wi-Fi: UNEXPECTED EVENT during connection.\n");
    }

    ESP_LOGI(TAG, "Wi-Fi: Init STA finished.");
    printf("--- Exiting wifi_init_sta ---\n");
}


// --- MQTT Event Handler ---
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    ESP_LOGD(TAG, "MQTT Event dispatched from event loop base=%s, event_id=%" PRIi32, base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
  
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        printf("MQTT: Connected to broker.\n");
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        printf("MQTT: Disconnected from broker.\n");
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        printf("MQTT: Message published, msg_id=%d.\n", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        printf("MQTT: Received data:\n  TOPIC=%.*s\n  DATA=%.*s\n", event->topic_len, event->topic, event->data_len, event->data);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        printf("MQTT: Error detected. errno=%d, transport_err=%d\n", event->error_handle->esp_transport_sock_errno, event->error_handle->esp_transport_sock_errno);
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            ESP_LOGI(TAG, "MQTT: Last error code reported from esp-tls: 0x%x", event->error_handle->esp_tls_stack_err);
        }
        break;
    default:
        ESP_LOGI(TAG, "MQTT: Other event id:%d", event->event_id);
        printf("MQTT: Other event id: %d\n", event->event_id);
        break;
    }
}

// --- MQTT Application Start ---
void mqtt_app_start() {
    printf("--- Entering mqtt_app_start ---\n");
    const esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
        .credentials.client_id = DEVICE_ID, 
    };

    ESP_LOGI(TAG, "MQTT: Initializing client with URI: %s", MQTT_BROKER_URI);
    printf("MQTT: Initializing client for %s with ID %s\n", MQTT_BROKER_URI, DEVICE_ID);
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);
    printf("--- Exiting mqtt_app_start ---\n");
}


// --- Main Application Entry Point ---
void app_main(void) {
    printf("--- Entering app_main ---\n");
    ESP_LOGI(TAG, "Starting ADC Monitor Application");
    printf("It's executed\n");

    // Initialize NVS. Required for Wi-Fi.
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    printf("NVS Flash initialized.\n");

    printf("Calling init_spiffs...\n");
    if (init_spiffs() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPIFFS. Stopping application.");
        printf("ERROR: SPIFFS initialization failed in app_main. Halting.\n");
        return;
    }
    printf("init_spiffs returned successfully.\n");

    printf("Calling wifi_init_sta...\n");
    wifi_init_sta(); // Initialize and connect Wi-Fi
    printf("wifi_init_sta returned.\n");

    printf("Calling mqtt_app_start...\n");
    mqtt_app_start(); // Initialize and start MQTT client
    printf("mqtt_app_start returned.\n");

    printf("Creating adc_read_task...\n");
    xTaskCreate(adc_read_task,
                "ADC_Read_Task",
                4096,
                NULL,
                5,
                NULL);
    printf("adc_read_task created.\n");
    printf("--- Exiting app_main ---\n");
}
