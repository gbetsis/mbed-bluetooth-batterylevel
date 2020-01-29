#include <events/mbed_events.h>
#include <mbed.h>
#include "ble/BLE.h"
#include "ble/Gap.h"
#include "ble/services/BatteryService.h"
#include "max32630fthr.h"

// ADC Vref volts (this is fixed for MAX32630)
#define VREF 1.2f
#define VMAX 4.200
#define VMIN 3.000

MAX32630FTHR pegasus(MAX32630FTHR::VIO_3V3);

AnalogIn batMonitor(AIN_0);

//Setup I2C bus
I2C i2cBus(P5_7, P6_0);     //Actually i2c2

//Setup PMIC
MAX14690 pmic(&i2cBus);

DigitalOut led1(LED1, 1);

const static char     DEVICE_NAME[] = "MAX32630FTHR";
static const uint16_t uuid16_list[] = {GattService::UUID_BATTERY_SERVICE};

int batteryLevel = 0;
static BatteryService* batteryServicePtr;

static EventQueue eventQueue(/* event count */ 16 * EVENTS_EVENT_SIZE);

void disconnectionCallback(const Gap::DisconnectionCallbackParams_t *params)
{
    BLE::Instance().gap().startAdvertising();
}

void updateSensorValue() {
    float curVoltage;
    curVoltage = batMonitor.read()*4.0f*VREF;
    batteryLevel = (int) ((curVoltage - (VMIN*1.03))*100 / ((VMAX*0.98) - (VMIN*1.03)));
    if (batteryLevel > 100) batteryLevel = 100;
    if (batteryLevel < 0)   batteryLevel = 0;
        
    batteryServicePtr->updateBatteryLevel(batteryLevel);
}

void blinkCallback(void)
{
    led1 = !led1; /* Do blinky on LED1 while we're waiting for BLE events */

    BLE &ble = BLE::Instance();
    if (ble.gap().getState().connected) {
        eventQueue.call(updateSensorValue);
    }
}

/**
 * This function is called when the ble initialization process has failled
 */
void onBleInitError(BLE &ble, ble_error_t error)
{
    /* Initialization error handling should go here */
}

/**
 * Callback triggered when the ble initialization process has finished
 */
void bleInitComplete(BLE::InitializationCompleteCallbackContext *params)
{
    BLE&        ble   = params->ble;
    ble_error_t error = params->error;

    if (error != BLE_ERROR_NONE) {
        /* In case of error, forward the error handling to onBleInitError */
        onBleInitError(ble, error);
        return;
    }

    /* Ensure that it is the default instance of BLE */
    if(ble.getInstanceID() != BLE::DEFAULT_INSTANCE) {
        return;
    }

    ble.gap().onDisconnection(disconnectionCallback);

    /* Setup primary service */
    batteryServicePtr = new BatteryService(ble, batteryLevel);

    /* Setup advertising */
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::BREDR_NOT_SUPPORTED | GapAdvertisingData::LE_GENERAL_DISCOVERABLE);
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LIST_16BIT_SERVICE_IDS, (uint8_t *) uuid16_list, sizeof(uuid16_list));
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LOCAL_NAME, (uint8_t *) DEVICE_NAME, sizeof(DEVICE_NAME));
    ble.gap().setAdvertisingType(GapAdvertisingParams::ADV_CONNECTABLE_UNDIRECTED);
    ble.gap().setAdvertisingInterval(1000); /* 1000ms */
    ble.gap().startAdvertising();

    printf("Started Advertising...\r\n");
}

void scheduleBleEventsProcessing(BLE::OnEventsToProcessCallbackContext* context) {
    BLE &ble = BLE::Instance();
    eventQueue.call(Callback<void()>(&ble, &BLE::processEvents));
}

int main()
{
    pegasus.max14690.monSet(pegasus.max14690.MON_BAT, pegasus.max14690.MON_DIV4);
    pmic.monCfg = MAX14690::MON_BAT;

    eventQueue.call_every(500, blinkCallback);

    BLE &ble = BLE::Instance();
    ble.onEventsToProcess(scheduleBleEventsProcessing);
    ble.init(bleInitComplete);

    eventQueue.dispatch_forever();

    return 0;
}