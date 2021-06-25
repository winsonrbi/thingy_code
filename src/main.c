/* main.c - Application main entry point */

/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdbool.h>
#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <sys/printk.h>
#include <sys/byteorder.h>
#include <zephyr.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <bluetooth/services/bas.h>

/*From sensor -Winson*/
#include <device.h>
#include<drivers/sensor.h>
#include <drivers/sensor/ccs811.h>
#include <stdio.h>
#include <sys/util.h>

#define SENSOR_1_NAME				"Temperature Sensor 1"
#define SENSOR_2_NAME				"Temperature Sensor 2"
#define SENSOR_3_NAME				"Humidity Sensor"
#define SENSOR_4_NAME				"Air Quality Sensor"
/* Sensor Internal Update Interval [seconds] */
#define SENSOR_1_UPDATE_IVAL			5
#define SENSOR_2_UPDATE_IVAL			12
#define SENSOR_3_UPDATE_IVAL			60
#define SENSOR_4_UPDATE_IVAL			30
#define SENSOR_5_UPDATE_IVAL			15

/* ESS error definitions */
#define ESS_ERR_WRITE_REJECT			0x80
#define ESS_ERR_COND_NOT_SUPP			0x81

/* ESS Trigger Setting conditions */
#define ESS_TRIGGER_INACTIVE			0x00
#define ESS_FIXED_TIME_INTERVAL			0x01
#define ESS_NO_LESS_THAN_SPECIFIED_TIME		0x02
#define ESS_VALUE_CHANGED			0x03
#define ESS_LESS_THAN_REF_VALUE			0x04
#define ESS_LESS_OR_EQUAL_TO_REF_VALUE		0x05
#define ESS_GREATER_THAN_REF_VALUE		0x06
#define ESS_GREATER_OR_EQUAL_TO_REF_VALUE	0x07
#define ESS_EQUAL_TO_REF_VALUE			0x08
#define ESS_NOT_EQUAL_TO_REF_VALUE		0x09

/*** use uuid's defined here https://nordicsemiconductor.github.io/Nordic-Thingy52-FW/documentation/firmware_architecture.html */
static const struct bt_uuid_128 env_uuid = BT_UUID_INIT_128(
		BT_UUID_128_ENCODE(0xEF680200,0x9B35,0x4933,0x9B10,0x52FFA9740042));

static const struct bt_uuid_128 gas_uuid = BT_UUID_INIT_128(
		BT_UUID_128_ENCODE(0xEF680204,0x9B35,0x4933,0x9B10,0x52FFA9740042));

static const struct bt_uuid_128 pressure_uuid = BT_UUID_INIT_128(
		BT_UUID_128_ENCODE(0xEF680202,0x9B35,0x4933,0x9B10,0x52FFA9740042));

static const struct bt_uuid_128 temp_uuid = BT_UUID_INIT_128(
		BT_UUID_128_ENCODE(0xEF680201,0x9B35,0x4933,0x9B10,0x52FFA9740042));

static const struct bt_uuid_128 humidity_uuid = BT_UUID_INIT_128(
		BT_UUID_128_ENCODE(0xEF680203,0x9B35,0x4933,0x9B10,0x52FFA9740042));
/* Environmental Sensing Service Declaration */

struct es_measurement {
	uint16_t flags; /* Reserved for Future Use */
	uint8_t sampling_func;
	uint32_t meas_period;
	uint32_t update_interval;
	uint8_t application;
	uint8_t meas_uncertainty;
};

struct temperature_sensor {
	int16_t temp_value;

	/* Valid Range */
	int16_t lower_limit;
	int16_t upper_limit;

	/* ES trigger setting - Value Notification condition */
	uint8_t condition;
	union {
		uint32_t seconds;
		int16_t ref_val; /* Reference temperature */
	};

	struct es_measurement meas;
};

struct humidity_sensor {
	int16_t humid_value;

	struct es_measurement meas;
};

struct air_quality_values {
	uint16_t eco2;
	uint16_t tvoc;
}__packed;

struct air_quality_sensor {
	struct air_quality_values values;
	uint8_t condition;
	union {
		uint32_t seconds;
		int16_t ref_val; /* Not actually used  */
	};

	struct es_measurement meas;
};

struct pressure_values {
	int32_t integer;
	uint8_t decimal;
}__packed;

struct pressure_sensor {
	struct pressure_values values;
	uint8_t condition;

	union {
		uint32_t seconds;
		int16_t ref_val; /* Reference value is not acutally used*/
	};

	struct es_measurement meas;
}
/*** read call backs, used to define how the data should be read for the BT_GATT_CHARACTERISTICS macros */
static ssize_t read_u16(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			void *buf, uint16_t len, uint16_t offset)
{
	const uint16_t *u16 = attr->user_data;
	uint16_t value = sys_cpu_to_le16(*u16);

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &value,
				 sizeof(value));
}

static ssize_t read_air_quality_values(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			void *buf, uint16_t len, uint16_t offset)
{
	const struct air_quality_values *values = attr->user_data;
	uint16_t values_to_send[2];
	values_to_send[0] = sys_cpu_to_le16(values->eco2);
	values_to_send[1] = sys_cpu_to_le16(values->tvoc);

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &values_to_send,
				 sizeof(values_to_send));
}

static ssize_t read_pressure_values(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			void *buf, uint16_t len, uint16_t offset)
{
	const struct pressure_values *values = attr->user_data;
	struct pressure_values values_to_send;
	values_to_send.integer = sys_cpu_to_le32(values->integer);
	values_to_send.decimal = values->decimal;
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &values_to_send,
				 sizeof(values_to_send));
}
/*** Initialize sensors */
/*** these bools are used to only fetch data when notify is enabled*/
static bool notify_temp;
static bool notify_gas;
static struct temperature_sensor sensor_1 = {
		.temp_value = 1200,
		.lower_limit = -10000,
		.upper_limit = 10000,
		.condition = ESS_VALUE_CHANGED,
		.meas.sampling_func = 0x00,
		.meas.meas_period = 0x01,
		.meas.update_interval = SENSOR_1_UPDATE_IVAL,
		.meas.application = 0x1c,
		.meas.meas_uncertainty = 0x04,
};

static struct temperature_sensor sensor_2 = {
		.temp_value = 1800,
		.lower_limit = -1000,
		.upper_limit = 5000,
		.condition = ESS_VALUE_CHANGED,
		.meas.sampling_func = 0x00,
		.meas.meas_period = 0x01,
		.meas.update_interval = SENSOR_2_UPDATE_IVAL,
		.meas.application = 0x1b,
		.meas.meas_uncertainty = 0x04,
};

static struct humidity_sensor sensor_3 = {
		.humid_value = 6233,
		.meas.sampling_func = 0x02,
		.meas.meas_period = 0x0e10,
		.meas.update_interval = SENSOR_3_UPDATE_IVAL,
		.meas.application = 0x1c,
		.meas.meas_uncertainty = 0x01,
};

static struct air_quality_sensor sensor_4 = {
		.values.eco2 = 420,
		.values.tvoc = 420,
		.condition = ESS_VALUE_CHANGED,
		.meas.sampling_func = 0x00,
		.meas.meas_period = 0x01,
		.meas.update_interval = SENSOR_4_UPDATE_IVAL,
		.meas.application = 0x1b,
		.meas.meas_uncertainty = 0x04,
};

static struct pressure_sensor sensor_5 = {
		.values.integer = 0,
		.valeus.decimal = 0,
		.condition = ESS_VALUE_CHANGED,
		.meas.sampling_func = 0x00,
		.meas.meas_period = 0x01,
		.meas.update_interval = SENSOR_5_UPDATE_IVAL,
		.meas.application = 0x1b,
		.meas.meas_uncertainty = 0x04,
		
};

/*** Client characteristic configuration */
static void temp_ccc_cfg_changed(const struct bt_gatt_attr *attr,
				 uint16_t value)
{
	notify_temp = value == BT_GATT_CCC_NOTIFY;
}

static void gas_ccc_cfg_changed(const struct bt_gatt_attr *attr,
				 uint16_t value)
{
	notify_gas = value == BT_GATT_CCC_NOTIFY;
}
static void pressure_ccc_cfg_changed(const struct bt_gatt_attr *attr,
				 uint16_t value)
{
	notify_pressure = value == BT_GATT_CCC_NOTIFY;
}

struct read_es_measurement_rp {
	uint16_t flags; /* Reserved for Future Use */
	uint8_t sampling_function;
	uint8_t measurement_period[3];
	uint8_t update_interval[3];
	uint8_t application;
	uint8_t measurement_uncertainty;
} __packed;

static ssize_t read_es_measurement(struct bt_conn *conn,
				   const struct bt_gatt_attr *attr, void *buf,
				   uint16_t len, uint16_t offset)
{
	const struct es_measurement *value = attr->user_data;
	struct read_es_measurement_rp rsp;

	rsp.flags = sys_cpu_to_le16(value->flags);
	rsp.sampling_function = value->sampling_func;
	sys_put_le24(value->meas_period, rsp.measurement_period);
	sys_put_le24(value->update_interval, rsp.update_interval);
	rsp.application = value->application;
	rsp.measurement_uncertainty = value->meas_uncertainty;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &rsp,
				 sizeof(rsp));
}

static ssize_t read_temp_valid_range(struct bt_conn *conn,
				     const struct bt_gatt_attr *attr, void *buf,
				     uint16_t len, uint16_t offset)
{
	const struct temperature_sensor *sensor = attr->user_data;
	uint16_t tmp[] = {sys_cpu_to_le16(sensor->lower_limit),
			  sys_cpu_to_le16(sensor->upper_limit)};

	return bt_gatt_attr_read(conn, attr, buf, len, offset, tmp,
				 sizeof(tmp));
}

struct es_trigger_setting_seconds {
	uint8_t condition;
	uint8_t sec[3];
} __packed;

struct es_trigger_setting_reference {
	uint8_t condition;
	int16_t ref_val;
} __packed;

static ssize_t read_temp_trigger_setting(struct bt_conn *conn,
					 const struct bt_gatt_attr *attr,
					 void *buf, uint16_t len,
					 uint16_t offset)
{
	const struct temperature_sensor *sensor = attr->user_data;

	switch (sensor->condition) {
	/* Operand N/A */
	case ESS_TRIGGER_INACTIVE:
		__fallthrough;
	case ESS_VALUE_CHANGED:
		return bt_gatt_attr_read(conn, attr, buf, len, offset,
					 &sensor->condition,
					 sizeof(sensor->condition));
	/* Seconds */
	case ESS_FIXED_TIME_INTERVAL:
		__fallthrough;
	case ESS_NO_LESS_THAN_SPECIFIED_TIME: {
			struct es_trigger_setting_seconds rp;

			rp.condition = sensor->condition;
			sys_put_le24(sensor->seconds, rp.sec);

			return bt_gatt_attr_read(conn, attr, buf, len, offset,
						 &rp, sizeof(rp));
		}
	/* Reference temperature */
	default: {
			struct es_trigger_setting_reference rp;

			rp.condition = sensor->condition;
			rp.ref_val = sys_cpu_to_le16(sensor->ref_val);

			return bt_gatt_attr_read(conn, attr, buf, len, offset,
						 &rp, sizeof(rp));
		}
	}
}
/*** determines which condition we notify on*/
static bool check_condition(uint8_t condition, int16_t old_val, int16_t new_val,
			    int16_t ref_val)
{
	switch (condition) {
	case ESS_TRIGGER_INACTIVE:
		return false;
	case ESS_FIXED_TIME_INTERVAL:
	case ESS_NO_LESS_THAN_SPECIFIED_TIME:
		/* TODO: Check time requirements */
		return false;
	case ESS_VALUE_CHANGED:
		return new_val != old_val;
	case ESS_LESS_THAN_REF_VALUE:
		return new_val < ref_val;
	case ESS_LESS_OR_EQUAL_TO_REF_VALUE:
		return new_val <= ref_val;
	case ESS_GREATER_THAN_REF_VALUE:
		return new_val > ref_val;
	case ESS_GREATER_OR_EQUAL_TO_REF_VALUE:
		return new_val >= ref_val;
	case ESS_EQUAL_TO_REF_VALUE:
		return new_val == ref_val;
	case ESS_NOT_EQUAL_TO_REF_VALUE:
		return new_val != ref_val;
	default:
		return false;
	}
}

static bool check_condition_int32(uint8_t condition, int32_t old_val, int32_t new_val,
			    int32_t ref_val)
{
	switch (condition) {
	case ESS_TRIGGER_INACTIVE:
		return false;
	case ESS_FIXED_TIME_INTERVAL:
	case ESS_NO_LESS_THAN_SPECIFIED_TIME:
		/* TODO: Check time requirements */
		return false;
	case ESS_VALUE_CHANGED:
		return new_val != old_val;
	case ESS_LESS_THAN_REF_VALUE:
		return new_val < ref_val;
	case ESS_LESS_OR_EQUAL_TO_REF_VALUE:
		return new_val <= ref_val;
	case ESS_GREATER_THAN_REF_VALUE:
		return new_val > ref_val;
	case ESS_GREATER_OR_EQUAL_TO_REF_VALUE:
		return new_val >= ref_val;
	case ESS_EQUAL_TO_REF_VALUE:
		return new_val == ref_val;
	case ESS_NOT_EQUAL_TO_REF_VALUE:
		return new_val != ref_val;
	default:
		return false;
	}
}
static bool check_condition_uint8(uint8_t condition, uint8_t old_val, uint8_t new_val,
			    int16_t ref_val)
{
	switch (condition) {
	case ESS_TRIGGER_INACTIVE:
		return false;
	case ESS_FIXED_TIME_INTERVAL:
	case ESS_NO_LESS_THAN_SPECIFIED_TIME:
		/* TODO: Check time requirements */
		return false;
	case ESS_VALUE_CHANGED:
		return new_val != old_val;
	case ESS_LESS_THAN_REF_VALUE:
		return new_val < ref_val;
	case ESS_LESS_OR_EQUAL_TO_REF_VALUE:
		return new_val <= ref_val;
	case ESS_GREATER_THAN_REF_VALUE:
		return new_val > ref_val;
	case ESS_GREATER_OR_EQUAL_TO_REF_VALUE:
		return new_val >= ref_val;
	case ESS_EQUAL_TO_REF_VALUE:
		return new_val == ref_val;
	case ESS_NOT_EQUAL_TO_REF_VALUE:
		return new_val != ref_val;
	default:
		return false;
	}
}
/*** update_ functions are used for notify */
static void update_temperature(struct bt_conn *conn,
			       const struct bt_gatt_attr *chrc, int16_t value,
			       struct temperature_sensor *sensor)
{
	bool notify = check_condition(sensor->condition,
				      sensor->temp_value, value,
				      sensor->ref_val);

	/* Update temperature value */
	sensor->temp_value = value;

	/* Trigger notification if conditions are met */
	if (notify) {
		value = sys_cpu_to_le16(sensor->temp_value);

		bt_gatt_notify(conn, chrc, &value, sizeof(value));
	}
}

static void update_gas(	struct bt_conn *conn,
		      	const  struct bt_gatt_attr *chrc, int16_t co2, int16_t tvoc,
			struct air_quality_sensor *sensor)
{
	bool notify_co2 = check_condition(sensor->condition,
					  sensor->values.eco2, co2,
					  sensor->ref_val);
	bool notify_tvoc = check_condition(sensor->condition,
					  sensor->values.tvoc, tvoc,
					  sensor->ref_val);
	sensor->values.eco2 = co2;
	sensor->values.tvoc = tvoc;
	if(notify_co2 || notify_tvoc) {
		uint16_t values_to_send[2];
		values_to_send[0] = sys_cpu_to_le16(sensor->values.eco2);
		values_to_send[1] = sys_cpu_to_le16(sensor->values.tvoc);
		bt_gatt_notify(conn, chrc, &values_to_send, sizeof(values_to_send));
	}

}

static void update_pressure(struct bt_conn *conn,
		      	const  struct bt_gatt_attr *chrc, int32_t integer, uint8_t decimal,
			struct pressure_sensor *sensor)
{
	bool notify_integer = check_condition_int32(sensor->condition,
					  sensor->values.integer, integer,
					  sensor->ref_val);
	bool notify_decimal = check_condition_uint8(sensor->condition,
					  sensor->values.decimal, decimal,
					  sensor->ref_val);
	sensor->values.integer = integer;
	sensor->values.decimal = decimal;
	if(notify_co2 || notify_tvoc) {
		struct pressure_values values_to_send;
		values_to_send.integer = sys_cpu_to_le32(sensor->values.integer);
		values_to_send.decimal = sensor->values.decimal;
		bt_gatt_notify(conn, chrc, &values_to_send, sizeof(values_to_send));
	}

}

BT_GATT_SERVICE_DEFINE(ess_svc,
	BT_GATT_PRIMARY_SERVICE(&env_uuid.uuid),

	/* Temperature Sensor 1 */
	BT_GATT_CHARACTERISTIC(&temp_uuid.uuid,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ,
			       read_u16, NULL, &sensor_1.temp_value),
	BT_GATT_DESCRIPTOR(BT_UUID_ES_MEASUREMENT, BT_GATT_PERM_READ,
			   read_es_measurement, NULL, &sensor_1.meas),
	BT_GATT_CUD(SENSOR_1_NAME, BT_GATT_PERM_READ),
	BT_GATT_DESCRIPTOR(BT_UUID_VALID_RANGE, BT_GATT_PERM_READ,
			   read_temp_valid_range, NULL, &sensor_1),
	BT_GATT_DESCRIPTOR(BT_UUID_ES_TRIGGER_SETTING,
			   BT_GATT_PERM_READ, read_temp_trigger_setting,
			   NULL, &sensor_1),
	BT_GATT_CCC(temp_ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

	/* Temperature Sensor 2 */
	BT_GATT_CHARACTERISTIC(&temp_uuid.uuid,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ,
			       read_u16, NULL, &sensor_2.temp_value),
	BT_GATT_DESCRIPTOR(BT_UUID_ES_MEASUREMENT, BT_GATT_PERM_READ,
			   read_es_measurement, NULL, &sensor_2.meas),
	BT_GATT_CUD(SENSOR_2_NAME, BT_GATT_PERM_READ),
	BT_GATT_DESCRIPTOR(BT_UUID_VALID_RANGE, BT_GATT_PERM_READ,
			   read_temp_valid_range, NULL, &sensor_2),
	BT_GATT_DESCRIPTOR(BT_UUID_ES_TRIGGER_SETTING,
			   BT_GATT_PERM_READ, read_temp_trigger_setting,
			   NULL, &sensor_2),
	BT_GATT_CCC(temp_ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

	/* Humidity Sensor */
	BT_GATT_CHARACTERISTIC(&humidity_uuid.uuid, BT_GATT_CHRC_READ,
			       BT_GATT_PERM_READ,
			       read_u16, NULL, &sensor_3.humid_value),
	BT_GATT_CUD(SENSOR_3_NAME, BT_GATT_PERM_READ),
	BT_GATT_DESCRIPTOR(BT_UUID_ES_MEASUREMENT, BT_GATT_PERM_READ,
			   read_es_measurement, NULL, &sensor_3.meas),
	/* Air Quality Sensor */
	BT_GATT_CHARACTERISTIC(&gas_uuid.uuid, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ,
			       read_air_quality_values, NULL, &sensor_4.values),
	BT_GATT_CUD(SENSOR_4_NAME, BT_GATT_PERM_READ),
	BT_GATT_DESCRIPTOR(BT_UUID_ES_MEASUREMENT, BT_GATT_PERM_READ,
			   read_es_measurement, NULL, &sensor_4.meas),
	BT_GATT_CCC(gas_ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	/* Pressure Sensor */
	BT_GATT_CHARACTERISTIC(&pressure_uuid.uuid, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ,
			       read_pressure_values, NULL, &sensor_4.values),
	BT_GATT_CUD(SENSOR_5_NAME, BT_GATT_PERM_READ),
	BT_GATT_DESCRIPTOR(BT_UUID_ES_MEASUREMENT, BT_GATT_PERM_READ,
			   read_es_measurement, NULL, &sensor_5.meas),
	BT_GATT_CCC(pressure_ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

static void ess_simulate(const struct device *hts221, const struct device *ccs811)
{
	/*** Sensor Code -Winson */
	//HTS221 temp and humidity sensor
	struct sensor_value temp, hum;
	//CCS811 gas sensor
	struct sensor_value co2, tvoc, voltage, current;

	if (sensor_sample_fetch(hts221) < 0) {
		printf("Sensor sample update error\n");
		return;
	}

	if (sensor_channel_get(hts221, SENSOR_CHAN_AMBIENT_TEMP, &temp) < 0) {
		printf("Cannot read HTS221 temperature channel\n");
		return;
	}

	if (sensor_channel_get(hts221, SENSOR_CHAN_HUMIDITY, &hum) < 0) {
		printf("Cannot read HTS221 humidity channel\n");
		return;
	}
	update_temperature(NULL, &ess_svc.attrs[2], (int) (sensor_value_to_double(&temp) * 100), &sensor_1);
	update_temperature(NULL, &ess_svc.attrs[9], (int) (sensor_value_to_double(&temp) * 100), &sensor_2);
	sensor_3.humid_value = (int) (sensor_value_to_double(&hum) * 100);

	if (sensor_sample_fetch(ccs811) < 0) {
		printf("Sensor sample update error\n");
		return;
	}

	if (sensor_channel_get(ccs811, SENSOR_CHAN_CO2, &co2) < 0) {
		printf("Cannot read CCS811 co2 channel\n");
		return;
	}

	if (sensor_channel_get(ccs811, SENSOR_CHAN_VOC, &tvoc) < 0) {
		printf("Cannot read HTS221 voc channel\n");
		return;
	}
	printf("CCS811 gas sensor value: %d\n",(int) (sensor_value_to_double(&co2)));
	update_gas(NULL, &ess_svc.attrs[20], (int)(sensor_value_to_double(&co2)), (int) (sensor_value_to_double(&tvoc)), &sensor_4); 

	
}

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_GAP_APPEARANCE, 0x00, 0x03),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL,
		      BT_UUID_16_ENCODE(BT_UUID_ESS_VAL),
		      BT_UUID_16_ENCODE(BT_UUID_BAS_VAL)),
};

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		printk("Connection failed (err 0x%02x)\n", err);
	} else {
		printk("Connected\n");
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("Disconnected (reason 0x%02x)\n", reason);
}

static struct bt_conn_cb conn_callbacks = {
	.connected = connected,
	.disconnected = disconnected,
};

static void bt_ready(void)
{
	int err;

	printk("Bluetooth initialized\n");

	err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}

	printk("Advertising successfully started\n");
}

static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Passkey for %s: %06u\n", addr, passkey);
}

static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Pairing cancelled: %s\n", addr);
}

static struct bt_conn_auth_cb auth_cb_display = {
	.passkey_display = auth_passkey_display,
	.passkey_entry = NULL,
	.cancel = auth_cancel,
};

static void bas_notify(void)
{
	uint8_t battery_level = bt_bas_get_battery_level();

	battery_level--;

	if (!battery_level) {
		battery_level = 100U;
	}

	bt_bas_set_battery_level(battery_level);
}
/*** struct for sensor -Winson*/
static void hts221_handler(const struct device *dev,
			   struct sensor_trigger *trig)
{
	process_sample(dev);
}
/*** -Winson */

void main(void)
{
	int err;
	printf("Started Custom Bluetooth Service\n");
	printk("KERNEL: Started Custom Bluetooth Service\n");
	/*** Sensor code -Winson */
	const struct device *hts221 = device_get_binding("HTS221");
	const struct device *ccs811 = device_get_binding("CCS811");
	if (hts221 == NULL) {
		printf("Could not get HTS221 device\n");
		return;
	}

	if (ccs811 == NULL) {
		printf("Could not get CCS811 device\n");
		return;
	}

	if (IS_ENABLED(CONFIG_HTS221_TRIGGER)) {
		struct sensor_trigger trig = {
			.type = SENSOR_TRIG_DATA_READY,
			.chan = SENSOR_CHAN_ALL,
		};
		if (sensor_trigger_set(hts221, &trig, hts221_handler) < 0) {
			printf("Cannot configure trigger\n");
			return;
		};
	}
	/*** End of sensor code -Winson */


	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	bt_ready();

	bt_conn_cb_register(&conn_callbacks);
	bt_conn_auth_cb_register(&auth_cb_display);

	while (!IS_ENABLED(CONFIG_HTS221_TRIGGER)) {
		k_sleep(K_SECONDS(1));

		/* Temperature simulation */
		ess_simulate(hts221, ccs811);

		/* Battery level simulation */
		bas_notify();
	}
}
