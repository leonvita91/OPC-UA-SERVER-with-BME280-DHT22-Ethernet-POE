#include "open62541.h"

/* GPIO Numbers */
// #define BLINK_GPIO 2
#define DHT22_GPIO 4
#define RELAY_0_GPIO 32
#define RELAY_1_GPIO 33
// #define RELAY_2_GPIO 26
// #define RELAY_3_GPIO 27

#define SDA_PIN GPIO_NUM_13
#define SCL_PIN GPIO_NUM_16



// /*  Method */
// UA_StatusCode
// ledProcessCallBack(UA_Server *server,
//                    const UA_NodeId *sessionId, void *sessionHandle,
//                    const UA_NodeId *methodId, void *methodContext,
//                    const UA_NodeId *objectId, void *objectContext,
//                    size_t inputSize, const UA_Variant *input,
//                    size_t outputSize, UA_Variant *output);

// void
// addLEDMethod(UA_Server *server);

/* DHT22 */
UA_StatusCode
readCurrentTemperatureDHT(UA_Server *server,
                const UA_NodeId *sessionId, void *sessionContext,
                const UA_NodeId *nodeId, void *nodeContext,
                UA_Boolean sourceTimeStamp, const UA_NumericRange *range,
                UA_DataValue *dataValue);

void
addCurrentTemperatureDHTDataSourceVariable(UA_Server *server);

/* BME280 temp */
UA_StatusCode
readCurrentTemperatureBME(UA_Server *server,
                const UA_NodeId *sessionId, void *sessionContext,
                const UA_NodeId *nodeId, void *nodeContext,
                UA_Boolean sourceTimeStamp, const UA_NumericRange *range,
                UA_DataValue *dataValue);

void
addCurrentTemperatureDataSourceVariableBME(UA_Server *server);

/* BME280 Humidity*/
UA_StatusCode
readCurrentHumidityBME(UA_Server *server,
                const UA_NodeId *sessionId, void *sessionContext,
                const UA_NodeId *nodeId, void *nodeContext,
                UA_Boolean sourceTimeStamp, const UA_NumericRange *range,
                UA_DataValue *dataValue);

void
addCurrentHumidityDataSourceVariableBME(UA_Server *server);

/* BME280 Pressure*/
UA_StatusCode
readCurrentPressureBME(UA_Server *server,
                const UA_NodeId *sessionId, void *sessionContext,
                const UA_NodeId *nodeId, void *nodeContext,
                UA_Boolean sourceTimeStamp, const UA_NumericRange *range,
                UA_DataValue *dataValue);

void
addCurrentPressureDataSourceVariableBME(UA_Server *server);


/*HEAP MEOMRY ESP32*/

UA_StatusCode
readCurrentHeapESP(UA_Server *server,
                const UA_NodeId *sessionId, void *sessionContext,
                const UA_NodeId *nodeId, void *nodeContext,
                UA_Boolean sourceTimeStamp, const UA_NumericRange *range,
                UA_DataValue *dataValue);

void
addCurrentDataSourceVariableHeapESP(UA_Server *server);



