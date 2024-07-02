/* The example of ESP-IDF
 *
 * This sample code is in the public domain.
 */

#include <stdio.h>
#include <inttypes.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/message_buffer.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "cJSON.h"

#include "parameter.h"

extern QueueHandle_t xQueueTrans;
extern MessageBufferHandle_t xMessageBufferToClient;

static const char *TAG = "IMU";

// bmi160 stuff
#include "bmi160.h"
#include "bmi160_defs.h"
//#define __DEBUG__ 1

#define CMD_READ    0x80
#define CMD_WRITE   0x7F

extern spi_device_handle_t spi;

// Source: https://github.com/TKJElectronics/KalmanFilter
#include "Kalman.h"

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead
#define RAD_TO_DEG (180.0/M_PI)
#define DEG_TO_RAD 0.0174533

// Arduino macro
#define micros() (unsigned long) (esp_timer_get_time())
#define delay(ms) esp_rom_delay_us(ms*1000)

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;
	
/* IMU Data */
struct bmi160_dev sensor;

// Accel & Gyro scale factor
float accel_sensitivity;
float gyro_sensitivity;

int8_t user_spi_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *read_data, uint16_t len) {
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));  // Обнуляем структуру транзакции

    uint8_t *tx_buffer = (uint8_t *)calloc(1, sizeof(uint8_t));
    uint8_t *rx_buffer = (uint8_t *)calloc(len + 1, sizeof(uint8_t));

    if (tx_buffer == NULL || rx_buffer == NULL) {
        ESP_LOGE(TAG, "Не удалось выделить память для буферов");
        if (tx_buffer) free(tx_buffer);
        if (rx_buffer) free(rx_buffer);
        return -1;  // Возвращаем ошибку
    }
    
    tx_buffer[0] = reg_addr | CMD_READ;  // Подготовка адреса регистра с командой чтения
    t.length = 8 * (len + 1);  // Длина передачи в битах (адрес регистра + длина данных)
    t.tx_buffer = tx_buffer;   // Указатель на буфер передачи
    t.rx_buffer = rx_buffer;   // Указатель на буфер приема
    t.user = (void*)dev_addr;  // Пользовательские данные (адрес устройства)

    ret = spi_device_polling_transmit(spi, &t);  // Выполняем транзакцию
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Ошибка при чтении данных по SPI: %s", esp_err_to_name(ret));
        free(tx_buffer);
        free(rx_buffer);
        return -1;  // Возвращаем ошибку
    }

    // Копируем полученные данные в буфер read_data (исключая первый байт)
    memcpy(read_data, &rx_buffer[1], len);

    // Освобождаем выделенную память
    free(tx_buffer);
    free(rx_buffer);
    return 0;  // Возвращаем успешное выполнение
}

int8_t user_spi_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *write_data, uint16_t len) {
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));  // Обнуляем структуру транзакции

    uint8_t *tx_buffer = (uint8_t*) calloc(len + 1, sizeof(uint8_t));  // Буфер для передачи данных, включая адрес регистра

    if (tx_buffer == NULL) {
        ESP_LOGE(TAG, "Не удалось выделить память для буфера");
        return -1;  // Возвращаем ошибку
    }

    tx_buffer[0] = reg_addr & CMD_WRITE;  // Первый байт - адрес регистра
    memcpy(&tx_buffer[1], write_data, len);  // Копируем данные для записи после адреса регистра
    t.length = (len + 1) * 8;  // Длина передачи в битах
    t.tx_buffer = tx_buffer;   // Указатель на буфер передачи
    t.user = (void*)dev_addr;  // Пользовательские данные (адрес устройства)

    ret = spi_device_polling_transmit(spi, &t);  // Выполняем транзакцию
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Ошибка при передаче данных по SPI: %s", esp_err_to_name(ret));
        free(tx_buffer);
        return -1;  // Возвращаем ошибку
    }

    // Освобождаем выделенную память
    free(tx_buffer);
    return 0;  // Возвращаем успешное выполнение
}

void user_delay_ms(uint32_t period) {
	esp_rom_delay_us(period*1000);
};

// Get scaled value
void _getMotion6(double *_ax, double *_ay, double *_az, double *_gx, double *_gy, double *_gz) {
	struct bmi160_sensor_data accel;
	struct bmi160_sensor_data gyro;
	int8_t ret = bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_GYRO_SEL), &accel, &gyro, &sensor);
	if (ret != BMI160_OK) {
		ESP_LOGE(TAG, "BMI160 get_sensor_data fail %d", ret);
		vTaskDelete(NULL);
	}
	//printf("%d %d %d - %d %d %d\n", accel.x, accel.y, accel.z, gyro.x, gyro.y, gyro.z);

	// Convert relative to absolute
	*_ax = (double)accel.x / accel_sensitivity;
	*_ay = (double)accel.y / accel_sensitivity;
	*_az = (double)accel.z / accel_sensitivity;
	*_gx = (double)gyro.x / gyro_sensitivity;
	*_gy = (double)gyro.y / gyro_sensitivity;
	*_gz = (double)gyro.z / gyro_sensitivity;
}

void getRollPitch(double accX, double accY, double accZ, double *roll, double *pitch) {
	// atan2 outputs the value of -πto π(radians) - see http://en.wikipedia.org/wiki/Atan2
	// It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
	*roll = atan2(accY, accZ) * RAD_TO_DEG;
	*pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
	*roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
	*pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif
}

void bmi160(void *pvParameters)
{
	sensor.id = 1;
	sensor.intf = BMI160_I2C_INTF;
	sensor.read = user_spi_read;
	sensor.write = user_spi_write;
	sensor.delay_ms = user_delay_ms;
	int8_t ret = bmi160_init(&sensor);
	if (ret == BMI160_OK)
	{
		ESP_LOGI(TAG, "BMI160 initialization success !");
		ESP_LOGI(TAG, "Chip ID 0x%X", sensor.chip_id);
	}
	else
	{
		ESP_LOGE(TAG, "BMI160 initialization fail %d", ret);
		vTaskDelete(NULL);
	}

	// Config Accel
	sensor.accel_cfg.odr = BMI160_ACCEL_ODR_1600HZ;
	sensor.accel_cfg.range = BMI160_ACCEL_RANGE_2G; // -2 --> +2[g]
	sensor.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;
	sensor.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;
	accel_sensitivity = 16384.0; // g

	// Config Gyro
	sensor.gyro_cfg.odr = BMI160_GYRO_ODR_3200HZ;
	//sensor.gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
	sensor.gyro_cfg.range = BMI160_GYRO_RANGE_250_DPS; // -250 --> +250[Deg/Sec]
	sensor.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;
	sensor.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;
	gyro_sensitivity = 131.2; // Deg/Sec

	ret = bmi160_set_sens_conf(&sensor);
	if (ret != BMI160_OK) {
		ESP_LOGE(TAG, "BMI160 set_sens_conf fail %d", ret);
		vTaskDelete(NULL);
	}
	ESP_LOGI(TAG, "bmi160_set_sens_conf");

	// Set Kalman and gyro starting angle
	double ax, ay, az;
	double gx, gy, gz;
	double roll, pitch, yaw; // Roll and pitch are calculated using the accelerometer
	double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

	_getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
	getRollPitch(ax, ay, az, &roll, &pitch);
	kalAngleX = roll;
	kalAngleY = pitch;
	kalmanX.setAngle(roll); // Set starting angle
	kalmanY.setAngle(pitch);
	yaw = 0.0;
	uint32_t timer = micros();

	int elasped = 0;
	bool initialized = false;
	double initial_kalAngleX = 0.0;
	double initial_kalAngleY = 0.0;

	while(1) {
		_getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
		//printf("%f %f %f - %f %f %f\n", ax, ay, az, gx, gy, gz);
		getRollPitch(ax, ay, az, &roll, &pitch);

		double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
		timer = micros();

		/* Roll and pitch estimation */
		double gyroXrate = gx;
		double gyroYrate = gy;

		// yaw
		yaw = yaw + gz * dt;

#ifdef RESTRICT_PITCH
		// This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
		if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
			kalmanX.setAngle(roll);
			kalAngleX = roll;
		} else
			kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
	

		if (abs(kalAngleX) > 90)
			gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
		kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
		// This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
		if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
			kalmanY.setAngle(pitch);
			kalAngleY = pitch;
		} else
			kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

		if (abs(kalAngleY) > 90)
			gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
		kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

		/* Print Data every 10 times */
		if (elasped > 10) {
			// Set the first data
			if (!initialized) {
				initial_kalAngleX = roll;
				initial_kalAngleY = pitch;
				initialized = true;
			}

#if 0
			printf("roll:%f", roll); printf(" ");
			printf("kalAngleX:%f", kalAngleX); printf(" ");
			printf("initial_kalAngleX:%f", initial_kalAngleX); printf(" ");
			printf("kalAngleX-initial_kalAngleX:%f", kalAngleX-initial_kalAngleX); printf(" ");
			printf("\n");

			printf("pitch: %f", pitch); printf(" ");
			printf("kalAngleY:%f", kalAngleY); printf("  ");
			printf("initial_kalAngleY: %f", initial_kalAngleY); printf(" ");
			printf("kalAngleY-initial_kalAngleY: %f", kalAngleY-initial_kalAngleY); printf(" ");
			printf("\n");
#endif

			// Send UDP packet
			float _roll = kalAngleX-initial_kalAngleX;
			float _pitch = kalAngleY-initial_kalAngleY;
			float _yaw = yaw;
			ESP_LOGI(TAG, "roll=%f pitch=%f yaw=%f", _roll, _pitch, _yaw);

			POSE_t pose;
			pose.roll = _roll;
			pose.pitch = _pitch;
			pose.yaw = yaw;
			if (xQueueSend(xQueueTrans, &pose, 100) != pdPASS ) {
				ESP_LOGE(pcTaskGetName(NULL), "xQueueSend fail");
			}

			// Send WEB request
			cJSON *request;
			request = cJSON_CreateObject();
			cJSON_AddStringToObject(request, "id", "data-request");
			cJSON_AddNumberToObject(request, "roll", _roll);
			cJSON_AddNumberToObject(request, "pitch", _pitch);
			cJSON_AddNumberToObject(request, "yaw", _yaw);
			char *my_json_string = cJSON_Print(request);
			ESP_LOGD(TAG, "my_json_string\n%s",my_json_string);
			size_t xBytesSent = xMessageBufferSend(xMessageBufferToClient, my_json_string, strlen(my_json_string), 100);
			if (xBytesSent != strlen(my_json_string)) {
				ESP_LOGE(TAG, "xMessageBufferSend fail");
			}
			cJSON_Delete(request);
			cJSON_free(my_json_string);

			elasped = 0;
		}
	
		elasped++;
		vTaskDelay(1);
	} // end while

	// Never reach here
	vTaskDelete( NULL );
}
