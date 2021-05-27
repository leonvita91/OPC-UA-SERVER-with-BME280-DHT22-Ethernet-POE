#include "open62541.h"
#include "model.h"
#include "DHT22.h"
#include "driver/gpio.h"
#include "bme280.h"
#include "bme280.c"
#include "driver/i2c.h"





#define TAG_BME280 "BME280"
#define I2C_MASTER_ACK 0
#define I2C_MASTER_NACK 1






s8 BME280_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 iError = BME280_INIT_VALUE;

	esp_err_t espRc;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);

	i2c_master_write_byte(cmd, reg_addr, true);
	i2c_master_write(cmd, reg_data, cnt, true);
	i2c_master_stop(cmd);

	espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
	if (espRc == ESP_OK) {
		iError = SUCCESS;
	} else {
		iError = FAIL;
	}
	i2c_cmd_link_delete(cmd);

	return (s8)iError;
}

s8 BME280_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 iError = BME280_INIT_VALUE;
	esp_err_t espRc;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd, reg_addr, true);

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);

	if (cnt > 1) {
		i2c_master_read(cmd, reg_data, cnt-1, I2C_MASTER_ACK);
	}
	i2c_master_read_byte(cmd, reg_data+cnt-1, I2C_MASTER_NACK);
	i2c_master_stop(cmd);

	espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
	if (espRc == ESP_OK) {
		iError = SUCCESS;
	} else {
		iError = FAIL;
	}

	i2c_cmd_link_delete(cmd);

	return (s8)iError;
}

void BME280_delay_msek(u32 msek)
{
	vTaskDelay(msek/portTICK_PERIOD_MS);
}

struct bme280_t bme280 = {
		.bus_write = BME280_I2C_bus_write,
		.bus_read = BME280_I2C_bus_read,
		.dev_addr = BME280_I2C_ADDRESS2,
		.delay_msec = BME280_delay_msek
	};


float ReadTemperatureBME(){
    ESP_LOGI(TAG_BME280, "ReadTemperatureBME start");
    s32 com_rslt;
	s32 v_uncomp_pressure_s32;
	s32 v_uncomp_temperature_s32;
	s32 v_uncomp_humidity_s32;

    com_rslt = bme280_init(&bme280);

    com_rslt += bme280_set_oversamp_pressure(BME280_OVERSAMP_16X);
	com_rslt += bme280_set_oversamp_temperature(BME280_OVERSAMP_2X);
	com_rslt += bme280_set_oversamp_humidity(BME280_OVERSAMP_1X);

	com_rslt += bme280_set_standby_durn(BME280_STANDBY_TIME_1_MS);
	com_rslt += bme280_set_filter(BME280_FILTER_COEFF_16);

	com_rslt += bme280_set_power_mode(BME280_NORMAL_MODE);

    com_rslt = bme280_read_uncomp_pressure_temperature_humidity(
				&v_uncomp_pressure_s32, &v_uncomp_temperature_s32, &v_uncomp_humidity_s32);

			if (com_rslt == SUCCESS) {
				ESP_LOGI(TAG_BME280, "%.2f degC / %.3f hPa / %.3f %%",
					bme280_compensate_temperature_double(v_uncomp_temperature_s32),
					bme280_compensate_pressure_double(v_uncomp_pressure_s32)/100, // Pa -> hPa
					bme280_compensate_humidity_double(v_uncomp_humidity_s32));
			} else {
				ESP_LOGE(TAG_BME280, "measure error. code: %d", com_rslt);
			}

    ESP_LOGI(TAG_BME280, "ReadTemperatureBME End");
	return bme280_compensate_temperature_double(v_uncomp_temperature_s32);
}

float ReadHumidityBME(){
	ESP_LOGI(TAG_BME280, "ReadHumidityBME start");

    s32 com_rslt;
        s32 v_uncomp_pressure_s32;
        s32 v_uncomp_temperature_s32;
        s32 v_uncomp_humidity_s32;

    com_rslt = bme280_init(&bme280);

    com_rslt += bme280_set_oversamp_pressure(BME280_OVERSAMP_16X);
        com_rslt += bme280_set_oversamp_temperature(BME280_OVERSAMP_2X);
        com_rslt += bme280_set_oversamp_humidity(BME280_OVERSAMP_1X);

        com_rslt += bme280_set_standby_durn(BME280_STANDBY_TIME_1_MS);
        com_rslt += bme280_set_filter(BME280_FILTER_COEFF_16);

        com_rslt += bme280_set_power_mode(BME280_NORMAL_MODE);

    com_rslt = bme280_read_uncomp_pressure_temperature_humidity(
                                &v_uncomp_pressure_s32, &v_uncomp_temperature_s32, &v_uncomp_humidity_s32);

                        if (com_rslt == SUCCESS) {
                                ESP_LOGI(TAG_BME280, "%.2f degC / %.3f hPa / %.3f %%",
                                        bme280_compensate_temperature_double(v_uncomp_temperature_s32),
                                        bme280_compensate_pressure_double(v_uncomp_pressure_s32)/100, // Pa -> hPa
                                        bme280_compensate_humidity_double(v_uncomp_humidity_s32));
                        } else {
                                ESP_LOGE(TAG_BME280, "measure error. code: %d", com_rslt);
                        }

    ESP_LOGI(TAG_BME280, "ReadHumidityBME End");
        return bme280_compensate_humidity_double(v_uncomp_humidity_s32);
}



float ReadPressureBME(){
	ESP_LOGI(TAG_BME280, "ReadPressureBME start");

    s32 com_rslt;
        s32 v_uncomp_pressure_s32;
        s32 v_uncomp_temperature_s32;
        s32 v_uncomp_humidity_s32;

    com_rslt = bme280_init(&bme280);

    com_rslt += bme280_set_oversamp_pressure(BME280_OVERSAMP_16X);
        com_rslt += bme280_set_oversamp_temperature(BME280_OVERSAMP_2X);
        com_rslt += bme280_set_oversamp_humidity(BME280_OVERSAMP_1X);

        com_rslt += bme280_set_standby_durn(BME280_STANDBY_TIME_1_MS);
        com_rslt += bme280_set_filter(BME280_FILTER_COEFF_16);

        com_rslt += bme280_set_power_mode(BME280_NORMAL_MODE);

    com_rslt = bme280_read_uncomp_pressure_temperature_humidity(
                                &v_uncomp_pressure_s32, &v_uncomp_temperature_s32, &v_uncomp_humidity_s32);

                        if (com_rslt == SUCCESS) {
                                ESP_LOGI(TAG_BME280, "%.2f degC / %.3f hPa / %.3f %%",
                                        bme280_compensate_temperature_double(v_uncomp_temperature_s32),
                                        bme280_compensate_pressure_double(v_uncomp_pressure_s32)/100, // Pa -> hPa
                                        bme280_compensate_humidity_double(v_uncomp_humidity_s32));
                        } else {
                                ESP_LOGE(TAG_BME280, "measure error. code: %d", com_rslt);
                        }

    ESP_LOGI(TAG_BME280, "ReadPressureBME End");
        return bme280_compensate_pressure_double(v_uncomp_pressure_s32);
}






/* DHT22 */

UA_StatusCode
readCurrentTemperatureDHT(UA_Server *server,
                const UA_NodeId *sessionId, void *sessionContext,
                const UA_NodeId *nodeId, void *nodeContext,
                UA_Boolean sourceTimeStamp, const UA_NumericRange *range,
                UA_DataValue *dataValue) {
    UA_Float temperatureDHT = ReadTemperature(DHT22_GPIO);
    UA_Variant_setScalarCopy(&dataValue->value, &temperatureDHT,
                             &UA_TYPES[UA_TYPES_FLOAT]);
    dataValue->hasValue = true;
    return UA_STATUSCODE_GOOD;
}


void
addCurrentTemperatureDHTDataSourceVariable(UA_Server *server) {
    UA_VariableAttributes attr = UA_VariableAttributes_default;
    attr.displayName = UA_LOCALIZEDTEXT("en-US", "temperature_dht");
    attr.dataType = UA_TYPES[UA_TYPES_FLOAT].typeId;
    attr.accessLevel = UA_ACCESSLEVELMASK_READ;

    UA_NodeId currentNodeId = UA_NODEID_STRING(1, "temperature");
    UA_QualifiedName currentName = UA_QUALIFIEDNAME(1, "Ambient Temperature");
    UA_NodeId parentNodeId = UA_NODEID_NUMERIC(0, UA_NS0ID_OBJECTSFOLDER);
    UA_NodeId parentReferenceNodeId = UA_NODEID_NUMERIC(0, UA_NS0ID_ORGANIZES);
    UA_NodeId variableTypeNodeId = UA_NODEID_NUMERIC(0, UA_NS0ID_BASEDATAVARIABLETYPE);

    UA_DataSource timeDataSource;
    timeDataSource.read = readCurrentTemperatureDHT;
    UA_Server_addDataSourceVariableNode(server, currentNodeId, parentNodeId,
                                        parentReferenceNodeId, currentName,
                                        variableTypeNodeId, attr,
                                        timeDataSource, NULL, NULL);
}



/* BME280 Temp  */



UA_StatusCode
readCurrentTemperatureBME(UA_Server *server,
                const UA_NodeId *sessionId, void *sessionContext,
                const UA_NodeId *nodeId, void *nodeContext,
                UA_Boolean sourceTimeStamp, const UA_NumericRange *range,
                UA_DataValue *dataValue) {
    UA_Float temperature = ReadTemperatureBME();
    UA_Variant_setScalarCopy(&dataValue->value, &temperature,&UA_TYPES[UA_TYPES_FLOAT]); 
    dataValue->hasValue = true;
    return UA_STATUSCODE_GOOD;
}


void
addCurrentTemperatureDataSourceVariableBME(UA_Server *server) {
    UA_VariableAttributes attr = UA_VariableAttributes_default;
    attr.displayName = UA_LOCALIZEDTEXT("en-US", "temperature_bme");
    attr.dataType = UA_TYPES[UA_TYPES_FLOAT].typeId;
    attr.accessLevel = UA_ACCESSLEVELMASK_READ;

    UA_NodeId currentNodeId = UA_NODEID_STRING(1, "temperature_bme");
    UA_QualifiedName currentName = UA_QUALIFIEDNAME(1, "Ambient Temperature");
    UA_NodeId parentNodeId = UA_NODEID_NUMERIC(0, UA_NS0ID_OBJECTSFOLDER);
    UA_NodeId parentReferenceNodeId = UA_NODEID_NUMERIC(0, UA_NS0ID_ORGANIZES);
    UA_NodeId variableTypeNodeId = UA_NODEID_NUMERIC(0, UA_NS0ID_BASEDATAVARIABLETYPE);

    UA_DataSource timeDataSource;
    timeDataSource.read = readCurrentTemperatureBME;
    UA_Server_addDataSourceVariableNode(server, currentNodeId, parentNodeId,
                                        parentReferenceNodeId, currentName,
                                        variableTypeNodeId, attr,
                                        timeDataSource, NULL, NULL);
}

/* BME280 Humidity*/


UA_StatusCode
readCurrentHumidityBME(UA_Server *server,
                const UA_NodeId *sessionId, void *sessionContext,
                const UA_NodeId *nodeId, void *nodeContext,
                UA_Boolean sourceTimeStamp, const UA_NumericRange *range,
                UA_DataValue *dataValue) {
    UA_Float humidity = ReadHumidityBME();
    UA_Variant_setScalarCopy(&dataValue->value, &humidity,&UA_TYPES[UA_TYPES_FLOAT]);
    dataValue->hasValue = true;
    return UA_STATUSCODE_GOOD;
}


void
addCurrentHumidityDataSourceVariableBME(UA_Server *server) {
    UA_VariableAttributes attr = UA_VariableAttributes_default;
    attr.displayName = UA_LOCALIZEDTEXT("en-US", "humidity_bme");
    attr.dataType = UA_TYPES[UA_TYPES_FLOAT].typeId;
    attr.accessLevel = UA_ACCESSLEVELMASK_READ;

    UA_NodeId currentNodeId = UA_NODEID_STRING(1, "humidity_bme");
    UA_QualifiedName currentName = UA_QUALIFIEDNAME(1, "Ambient humidity");
    UA_NodeId parentNodeId = UA_NODEID_NUMERIC(0, UA_NS0ID_OBJECTSFOLDER);
    UA_NodeId parentReferenceNodeId = UA_NODEID_NUMERIC(0, UA_NS0ID_ORGANIZES);
    UA_NodeId variableTypeNodeId = UA_NODEID_NUMERIC(0, UA_NS0ID_BASEDATAVARIABLETYPE);

    UA_DataSource timeDataSource;
    timeDataSource.read = readCurrentHumidityBME;
    UA_Server_addDataSourceVariableNode(server, currentNodeId, parentNodeId,
                                        parentReferenceNodeId, currentName,
                                        variableTypeNodeId, attr,
                                        timeDataSource, NULL, NULL);
}

/* BME280 PRESSURE*/


UA_StatusCode
readCurrentPressureBME(UA_Server *server,
                const UA_NodeId *sessionId, void *sessionContext,
                const UA_NodeId *nodeId, void *nodeContext,
                UA_Boolean sourceTimeStamp, const UA_NumericRange *range,
                UA_DataValue *dataValue) {
    UA_Float pressure = ReadPressureBME();
    UA_Variant_setScalarCopy(&dataValue->value, &pressure,&UA_TYPES[UA_TYPES_FLOAT]);
    dataValue->hasValue = true;
    return UA_STATUSCODE_GOOD;
}


void
addCurrentPressureDataSourceVariableBME(UA_Server *server) {
    UA_VariableAttributes attr = UA_VariableAttributes_default;
    attr.displayName = UA_LOCALIZEDTEXT("en-US", "pressure_bme");
    attr.dataType = UA_TYPES[UA_TYPES_FLOAT].typeId;
    attr.accessLevel = UA_ACCESSLEVELMASK_READ;
    UA_NodeId currentNodeId = UA_NODEID_STRING(1, "pressure_bme");
    UA_QualifiedName currentName = UA_QUALIFIEDNAME(1, "Ambient pressure");
    UA_NodeId parentNodeId = UA_NODEID_NUMERIC(0, UA_NS0ID_OBJECTSFOLDER);
    UA_NodeId parentReferenceNodeId = UA_NODEID_NUMERIC(0, UA_NS0ID_ORGANIZES);
    UA_NodeId variableTypeNodeId = UA_NODEID_NUMERIC(0, UA_NS0ID_BASEDATAVARIABLETYPE);

    UA_DataSource timeDataSource;
    timeDataSource.read = readCurrentPressureBME;
    UA_Server_addDataSourceVariableNode(server, currentNodeId, parentNodeId,
                                        parentReferenceNodeId, currentName,
                                        variableTypeNodeId, attr,
                                        timeDataSource, NULL, NULL);
}


///////////////////////////////


float ReadHeapESP(){
    ESP_LOGI(TAG_BME280, "Read Heap ESP32 start");
	ESP_LOGI(TAG_BME280, "Heap Left : %d", xPortGetFreeHeapSize());
	return xPortGetFreeHeapSize() / 1.0;
}




UA_StatusCode
readCurrentHeapESP(UA_Server *server,
                const UA_NodeId *sessionId, void *sessionContext,
                const UA_NodeId *nodeId, void *nodeContext,
                UA_Boolean sourceTimeStamp, const UA_NumericRange *range,
                UA_DataValue *dataValue) {
    UA_Float heap = xPortGetFreeHeapSize();
    UA_Variant_setScalarCopy(&dataValue->value, &heap,&UA_TYPES[UA_TYPES_FLOAT]); 
    dataValue->hasValue = true;
    return UA_STATUSCODE_GOOD;
}


void
addCurrentDataSourceVariableHeapESP(UA_Server *server) {
    UA_VariableAttributes attr = UA_VariableAttributes_default;
    attr.displayName = UA_LOCALIZEDTEXT("en-US", "heap_memory");
    attr.dataType = UA_TYPES[UA_TYPES_FLOAT].typeId;
    attr.accessLevel = UA_ACCESSLEVELMASK_READ;

    UA_NodeId currentNodeId = UA_NODEID_STRING(1, "heap_esp32");
    UA_QualifiedName currentName = UA_QUALIFIEDNAME(1, "Ambient Heap");
    UA_NodeId parentNodeId = UA_NODEID_NUMERIC(0, UA_NS0ID_OBJECTSFOLDER);
    UA_NodeId parentReferenceNodeId = UA_NODEID_NUMERIC(0, UA_NS0ID_ORGANIZES);
    UA_NodeId variableTypeNodeId = UA_NODEID_NUMERIC(0, UA_NS0ID_BASEDATAVARIABLETYPE);

    UA_DataSource timeDataSource;
    timeDataSource.read = readCurrentHeapESP;
    UA_Server_addDataSourceVariableNode(server, currentNodeId, parentNodeId,
                                        parentReferenceNodeId, currentName,
                                        variableTypeNodeId, attr,
                                        timeDataSource, NULL, NULL);
}









