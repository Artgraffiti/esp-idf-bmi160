#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/portable.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/message_buffer.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "mdns.h"

#include "parameter.h"

#include "websocket_server.h"

#define IMU_HOST        SPI2_HOST

static const char *TAG = "MAIN";
static const char *MDNS_HOSTNAME = "esp32";

MessageBufferHandle_t xMessageBufferToClient;
QueueHandle_t xQueueTrans;
spi_device_handle_t spi;

extern "C" {
	void app_main(void);
}


void bmi160(void *pvParameters);

#ifdef __cplusplus
extern "C" {
#endif
void start_wifi(void);
void start_mdns(void);
void start_i2c(void);
int ws_server_start(void);
void udp_trans(void *pvParameters);
void server_task(void *pvParameters);
void client_task(void *pvParameters);
#ifdef __cplusplus
}
#endif

void start_mdns(void)
{
	//initialize mDNS
	ESP_ERROR_CHECK( mdns_init() );
	//set mDNS hostname (required if you want to advertise services)
	ESP_ERROR_CHECK( mdns_hostname_set(MDNS_HOSTNAME) );
	ESP_LOGI(TAG, "mdns hostname set to: [%s]", MDNS_HOSTNAME);

	//initialize service
	ESP_ERROR_CHECK( mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0) );

#if 0
	//set default mDNS instance name
	ESP_ERROR_CHECK( mdns_instance_name_set("ESP32 with mDNS") );
#endif
}

void spi_init() {
    esp_err_t ret;
    ESP_LOGI(TAG, "Initializing bus SPI%d...", IMU_HOST + 1);
    spi_bus_config_t buscfg = {
        .mosi_io_num = CONFIG_GPIO_MOSI,
        .miso_io_num = CONFIG_GPIO_MISO,
        .sclk_io_num = CONFIG_GPIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = SOC_SPI_MAXIMUM_BUFFER_SIZE,
    };
    
    spi_device_interface_config_t devcfg = {
        .mode = 0,                              // SPI mode (CPOL, CPHA) -> (0, 0). p.89 bmi160 ds
        .clock_speed_hz = 10 * 1000 * 1000,     // 10 MHz
        .spics_io_num = CONFIG_GPIO_CS,             // CS pin
        .queue_size = 1,                        // We want to be able to queue 1 transactions at a time
    };

    // Initialize the SPI bus
    ret = spi_bus_initialize(IMU_HOST, &buscfg, SPI_DMA_DISABLED);
    ESP_ERROR_CHECK(ret);

    // Add device to bus
    ret = spi_bus_add_device(IMU_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);
}

void app_main(void)
{
	// esp_log_level_set("IMU", ESP_LOG_NONE);

	// Initialize WiFi
	start_wifi();

	// Initialize mDNS
	start_mdns();

	// Initialize SPI
	spi_init();

#if 0
	// Initialize i2c
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = (gpio_num_t)CONFIG_GPIO_SDA;
	conf.scl_io_num = (gpio_num_t)CONFIG_GPIO_SCL;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = 400000;
	ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
	ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
#endif

	// Create Queue
	xQueueTrans = xQueueCreate(10, sizeof(POSE_t));
	configASSERT( xQueueTrans );

	// Create Message Buffer
	xMessageBufferToClient = xMessageBufferCreate(1024);
	configASSERT( xMessageBufferToClient );

	// Get the local IP address
	esp_netif_ip_info_t ip_info;
	ESP_ERROR_CHECK(esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"), &ip_info));
	char cparam0[64];
	sprintf(cparam0, IPSTR, IP2STR(&ip_info.ip));
	ESP_LOGI(TAG, "cparam0=[%s]", cparam0);

	// Start web socket server
	ws_server_start();

	// Start web server
	xTaskCreate(&server_task, "SERVER", 1024*4, (void *)cparam0, 5, NULL);

	// Start web client
	xTaskCreate(&client_task, "CLIENT", 1024*3, (void *)0x111, 5, NULL);

	// Start imu task
	xTaskCreate(&bmi160, "IMU", 1024*8, NULL, 5, NULL);

	// Start udp task
	xTaskCreate(&udp_trans, "TRANS", 1024*4, NULL, 5, NULL);

	// vTaskDelay(100);
}
