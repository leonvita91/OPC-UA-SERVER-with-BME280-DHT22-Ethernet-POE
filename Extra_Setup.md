## Add Nodes to OPC-UA

##  **Simple Example To add node bme280 temperature**


1. First go to this Dirctory ~/esp/esp-idf/examples/opc-ua-with-bme280/components/model 
2. Here you will find  {+CMakeLists.txt+}, {+DHT22.c +}, {+ model.c+} we need **model.c and include file**
3. go edit the model.c first with any editor you have in my case vi model.c  

```c 
//inside you should add the function that called the driver BME280 of your sensor. in this case we added 
#include"bme280.h"
#include"bme280.c"

and we called this function from driver which in BME280 case this .
```

```c
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
```


4. after adding the function for driver you should call READ Function in order to read temperature for bme280.

```c
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
        return bme280_compensate_temperature_double(v_uncomp_{+temperature+}_s32);
}
```
* You can edit this 3 lines for examples from temperature to humidity
* {+ float ReadHumidityBME(){ +}
* {+ ESP_LOGI(TAG_BME280, "ReadHumidityBME start");+}       
* {+return bme280_compensate_temperature_double(v_uncomp_humidity_s32);+}


5. Now you should add the OPC-UA status and variable,nodeID,DisplayName 
```c
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
```
**NOTE** Also you can change from temperature to humidity in case you want to edit.

---------------------------------------------------------------------------------------------
6. You have to copy this line `addCurrentTemperatureDataSourceVariableBME(UA_Server *server)`
7. save and exit.
8. go to include file and edit model.h which is the header file.
9. Here you add the UA_StatusCode to make UPC-UA able to read your Variable,nodeID and call the addCurrentTemperatureDataSourceVariableBME Function.

```c
UA_StatusCode
readCurrentTemperatureBME(UA_Server *server,
                const UA_NodeId *sessionId, void *sessionContext,
                const UA_NodeId *nodeId, void *nodeContext,
                UA_Boolean sourceTimeStamp, const UA_NumericRange *range,
                UA_DataValue *dataValue);

void
addCurrentTemperatureDataSourceVariableBME(UA_Server *server);
```

10. Last thing :) copy the VariableName which is here `addCurrentTemperatureDataSourceVariableBME`  {+SAVE&EXIT+}
11. go back to opcua-esp32.c which in this Dirctory `cd ~/esp/esp-idf/examples/opc-ua-with-bme280/main` and edit opcua-esp32.c
12. add your Variable next to those Variables
* {+addCurrentTemperatureDataSourceVariableBME+}
```c
/* Add Information Model Objects Here */
    addCurrentTemperatureDHTDataSourceVariable(server);
    addCurrentTemperatureDataSourceVariableBME(server);
    addCurrentHumidityDataSourceVariableBME(server);
    addCurrentPressureDataSourceVariableBME(server);
    addCurrentDataSourceVariableHeapESP(server);
```

## Done Adding NODE
------------------------------------

## Editing OPC-UA buffer.

* If you want to edit the OPC-UA buffer to make it more or less.
* **NOTE** {-Keep in mind more node more heap memory-}
* First to edit go to `cd ~/esp/esp-idf/examples/opc-ua-with-bme280/main` open opcua-esp32.c
* in this lines you can make the buffer more or less 
```c
//BufferSize's got to be decreased due to latest refactorings in open62541 v1.2rc.
    UA_Int32 sendBufferSize = 10000; // 16384;
    UA_Int32 recvBufferSize = 10000; //16384;
```
**NOTE** {-it is very important to add the sum of your buffer in opcua complete buffer in this line code-}

```c
xTaskCreatePinnedToCore(opcua_task, "opcua_task", 24000, NULL, 10, NULL, 0);
```
**for example** 
* The sum of {+10000+10000 =20000+} also its better if you make a bit more than sum like here {+24000+} in order to avoid {-Overflow Stack -}

## Done Editing OPC-UA buffer

---------------------------------

## Editing idf.py menuconfig settings.

* go to project file `cd ~/esp/esp-idf/examples/opc-ua-with-bme280`
* write `idf.py menuconfig`
* to change the network settings go to {+Connection Configuration+} inside are static IP or DHCP ,WIFI and Ethernet Connection.
* to change the ethernet adapter settings go to {+Comonent Config+} than {+Ethernet+} inside you can change pin or modes.

**NOTE:**
       * We had used light wights {+LWIP+} .
       * We **didnt** used **SPI** beucase the Ethernet adapter not supported. 
       * Without **SPI** we cant access the external memory.
       * This is not only the settings there are alot inside the idf.py menuconfig but this is a simple version of hug settings .

## Done Editing idf.py menuconfig settings.
-------------------------------------------------------------------------------------------------------------------------------------




