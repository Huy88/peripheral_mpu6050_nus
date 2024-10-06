/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Nordic UART Bridge Service (NUS) sample
 */


#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>


#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <soc.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>

#include <bluetooth/services/nus.h>

#include <dk_buttons_and_leds.h>
#include <zephyr/sys/printk.h>
#include <zephyr/settings/settings.h>

#include <stdio.h>
#include <string.h>

#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#define LOG_MODULE_NAME peripheral_uart
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define STACKSIZE CONFIG_BT_NUS_THREAD_STACK_SIZE
#define PRIORITY 7

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN	(sizeof(DEVICE_NAME) - 1)

#define RUN_STATUS_LED DK_LED1
#define RUN_LED_BLINK_INTERVAL 1000

#define CON_STATUS_LED DK_LED2

#define KEY_PASSKEY_ACCEPT DK_BTN1_MSK
#define KEY_PASSKEY_REJECT DK_BTN2_MSK

#define UART_BUF_SIZE CONFIG_BT_NUS_UART_BUFFER_SIZE
#define UART_WAIT_FOR_BUF_DELAY K_MSEC(50)
#define UART_WAIT_FOR_RX CONFIG_BT_NUS_UART_RX_WAIT_TIME

// MPU6050 I2C地址
#define MPU6050_WHO_AM_I 0x75

// MPU6050寄存器地址
#define MPU6050_RA_TEMP_OUT_H       0x41
#define MPU6050_RA_TEMP_OUT_L       0x42
#define MPU6050_RA_PWR_MGMT_1       0x6B
#define MPU6050_RA_SMPLRT_DIV       0x19
#define MPU6050_RA_GYRO_CONFIG      0x1B
#define MPU6050_RA_ACCEL_CONFIG     0x1C
#define MPU6050_RA_CONFIG           0x1A
#define MPU6050_RA_ACCEL_XOUT_H     0x3B
#define MPU6050_RA_ACCEL_XOUT_L     0x3C
#define MPU6050_RA_ACCEL_YOUT_H     0x3D
#define MPU6050_RA_ACCEL_YOUT_L     0x3E
#define MPU6050_RA_ACCEL_ZOUT_H     0x3F
#define MPU6050_RA_ACCEL_ZOUT_L     0x40
#define MPU6050_RA_GYRO_XOUT_H      0x43
#define MPU6050_RA_GYRO_XOUT_L      0x44
#define MPU6050_RA_GYRO_YOUT_H      0x45
#define MPU6050_RA_GYRO_YOUT_L      0x46
#define MPU6050_RA_GYRO_ZOUT_H      0x47
#define MPU6050_RA_GYRO_ZOUT_L      0x48

static struct bt_gatt_exchange_params exchange_params;
static void exchange_func(struct bt_conn *conn, uint8_t err,
    struct bt_gatt_exchange_params *params)
{
    if (!err) {
        /* According to 3.4.7.1 Handle Value Notification off the ATT protocol.
         * Maximum supported notification is ATT_MTU - 3 */
        uint32_t bt_max_send_len = bt_gatt_get_mtu(conn) - 3;
        printk("max send len is %d", bt_max_send_len);
    }
}

// I2C節點
#define I2C0_NODE DT_NODELABEL(mysensor)
static const struct i2c_dt_spec dev_i2c =I2C_DT_SPEC_GET(I2C0_NODE);
// 將原始加速度數據轉換為重力加速度
float accel_to_g(int16_t accel_raw) {
    return accel_raw / 16384.0;
}

// 將原始陀螺儀數據轉換為每秒度數
float gyro_to_deg_per_sec(int16_t gyro_raw) {
    return gyro_raw / 131.0;
}

void write_to_mpu6050(uint16_t register_address, uint8_t value) {
    int ret;
    uint8_t tx_buf[2];
    tx_buf[0] = register_address;
    tx_buf[1] = value;
    ret = i2c_write_dt(&dev_i2c, tx_buf, sizeof(tx_buf));
    if (ret != 0) {
        printk("Failed to write %d to register %d, error %d\n", value, register_address, ret);
    } else {
        printk("Successfully wrote %d to register %d\n", value, register_address);
    }
}

void read_from_mpu(uint8_t register_address, uint8_t *destination, uint8_t number) {
    int ret;
    ret = i2c_burst_read_dt(&dev_i2c, register_address, destination, number);
    if (ret != 0) {
        printk("Failed to read from register %d\n", register_address);
    }
}

void read_accel(int16_t *pAx, int16_t *pAy, int16_t *pAz) {
    uint8_t buf[6];
    read_from_mpu(MPU6050_RA_ACCEL_XOUT_H, buf, 6);
    *pAx = (buf[0] << 8) | buf[1];
    *pAy = (buf[2] << 8) | buf[3];
    *pAz = (buf[4] << 8) | buf[5];
}

void read_gyro(int16_t *pGx, int16_t *pGy, int16_t *pGz) {
    uint8_t buf[6];
    read_from_mpu(MPU6050_RA_GYRO_XOUT_H, buf, 6);
    *pGx = (buf[0] << 8) | buf[1];
    *pGy = (buf[2] << 8) | buf[3];
    *pGz = (buf[4] << 8) | buf[5];
}

void init_mpu6050(void) {
    write_to_mpu6050(MPU6050_RA_PWR_MGMT_1, 0x00);
    write_to_mpu6050(MPU6050_RA_SMPLRT_DIV, 0x07);
    write_to_mpu6050(MPU6050_RA_CONFIG, 0x06);
    write_to_mpu6050(MPU6050_RA_GYRO_CONFIG, 0x00);
    write_to_mpu6050(MPU6050_RA_ACCEL_CONFIG, 0x00);
}

bool Who_am_i(void) {
    int ret;
    uint8_t who_am_i;
    read_from_mpu(MPU6050_WHO_AM_I, &who_am_i, 1);
    if (ret != 0) {
        printk("Failed to write/read I2C device address %x at Reg. %x \n", dev_i2c.addr, MPU6050_WHO_AM_I);
    }
    if (who_am_i == 0x70) {
        printk("True Device");
        return true;
    } else {
        printk("Who_am_i: %d (0x%x)", who_am_i, who_am_i);
        return false;
    }
}



static K_SEM_DEFINE(ble_init_ok, 0, 1);

static struct bt_conn *current_conn;
static struct bt_conn *auth_conn;

struct uart_data_t {
	void *fifo_reserved;
	uint8_t data[UART_BUF_SIZE];
	uint16_t len;
};

static K_FIFO_DEFINE(fifo_uart_tx_data);
static K_FIFO_DEFINE(fifo_uart_rx_data);

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (err) {
		LOG_ERR("Connection failed (err %u)", err);
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Connected %s", addr);
	current_conn = bt_conn_ref(conn);
	
	int rc;

	/* maximize ATT MTU at peer side (CONFIG_BT_L2CAP_TX_MTU)*/
    exchange_params.func = exchange_func;
	LOG_INF("sending ATT MTU to peer..");
	rc = bt_gatt_exchange_mtu(current_conn, &exchange_params);
	if (rc) {
		LOG_ERR("failed to negotiate maximum mtu with peer [%d]", rc);
	}

	dk_set_led_on(CON_STATUS_LED);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Disconnected: %s (reason %u)", addr, reason);

	if (auth_conn) {
		bt_conn_unref(auth_conn);
		auth_conn = NULL;
	}

	if (current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
		dk_set_led_off(CON_STATUS_LED);
	}
}


BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected    = connected,
	.disconnected = disconnected,
};

static void bt_receive_cb(struct bt_conn *conn, const uint8_t *const data,
			  uint16_t len)
{
	int err;
	char addr[BT_ADDR_LE_STR_LEN] = {0};

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, ARRAY_SIZE(addr));

	LOG_INF("Received data from: %s", addr);

}

static struct bt_nus_cb nus_cb = {
	.received = bt_receive_cb,
};

void error(void)
{
	dk_set_leds_state(DK_ALL_LEDS_MSK, DK_NO_LEDS_MSK);

	while (true) {
		/* Spin for ever */
		k_sleep(K_MSEC(1000));
	}
}


static void configure_gpio(void)
{
	int err;

	err = dk_leds_init();
	if (err) {
		LOG_ERR("Cannot init LEDs (err: %d)", err);
	}
}

int main(void)
{
	int blink_status = 0;
	int err = 0;
	if (!device_is_ready(dev_i2c.bus)) {
		printk("I2C bus %s is not ready!\n", dev_i2c.bus->name);
		return -1;
	}
	if(!Who_am_i())
	{
		printk("Not mpu6050\n");
	}
	init_mpu6050();
	printk("Initialized mpu6050!!\n");
	configure_gpio();

	err = bt_enable(NULL);
	if (err) {
		error();
	}

	LOG_INF("Bluetooth initialized");

	k_sem_give(&ble_init_ok);

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	err = bt_nus_init(&nus_cb);
	if (err) {
		LOG_ERR("Failed to initialize UART service (err: %d)", err);
		return 0;
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd,
			      ARRAY_SIZE(sd));
	if (err) {
		LOG_ERR("Advertising failed to start (err %d)", err);
		return 0;
	}

	for (;;) {
		dk_set_led(RUN_STATUS_LED, (++blink_status) % 2);
		k_sleep(K_MSEC(RUN_LED_BLINK_INTERVAL));
	}
}

void ble_write_thread(void)
{
	/* Don't go any further until BLE is initialized */
	k_sem_take(&ble_init_ok, K_FOREVER);
	struct uart_data_t nus_data = {
		.len = 0,
	};

	for (;;) {
		int16_t accel[3]; // x, y, z 加速度
		int16_t gyro[3];  // x, y, z 角速度

		// 從 MPU6050 讀取數據
		read_gyro(&gyro[0],&gyro[1],&gyro[2]);
		printk("Gx=%d",gyro[0]);
		read_accel(&accel[0],&accel[1],&accel[2]);

		char message[100];
		int message_len;
		// 將加速度和陀螺儀數據格式化
		message_len = snprintf(message, sizeof(message), 
				"AX=%dAY=%dAZ=%dGX=%dGY=%dGZ=%d\n", 
				accel[0], accel[1], accel[2], 
				gyro[0], gyro[1], gyro[2]);


		// 透過 BLE 傳送數據
		if (bt_nus_send(current_conn, message, message_len)) {
			LOG_WRN("Failed to send data over BLE connection");
		}
		k_msleep(100);
	}
}

K_THREAD_DEFINE(ble_write_thread_id, STACKSIZE, ble_write_thread, NULL, NULL,
		NULL, PRIORITY, 0, 0);
