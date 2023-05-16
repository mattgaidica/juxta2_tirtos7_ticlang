/******************************************************************************

 @file  multi_role.c

 @brief This file contains the multi_role sample application for use
 with the CC2650 Bluetooth Low Energy Protocol Stack.

 Group: WCS, BTS
 Target Device: cc13xx_cc26xx
 2d Author: Matt Gaidica

 *****************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <unistd.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/sysbios/knl/Task.h>

#include <time.h>
#include <ti/sysbios/hal/Seconds.h>

#if (!(defined FREERTOS) && !(defined __TI_COMPILER_VERSION__)) && !(defined(__clang__))
#include <intrinsics.h>
#endif

#include <ti/drivers/ADC.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/NVS.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>
#include <driverlib/osc.h> // only for modifying osc
//#include <ti/drivers/UART2.h>

#include <icall.h>
#include <util.h>
#include <bcomdef.h>
/* This Header file contains all BLE API and icall structure definition */
#include <icall_ble_api.h>

#include <devinfoservice.h>
#include <juxta_gatt_service.h>
//#include <clock_gatt_service.h>
#include <ti_drivers_config.h>

#include <ti_ble_config.h>
#include <multi_role.h>
#include <lsm303agr_reg.h>
#include <SPI_NAND.h>
#include <Serialize.h>

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
static uint8 JUXTA_VERSION[DEVINFO_STR_ATTR_LEN + 1] = "v230502";
#define INT_THRESHOLD_MG    1000
#define INT_THRESHOLD_XL1   0x06
#define INT_DURATION_XL     0
#define INT_DURATION_6D     0 // N/ODR
#define INT_THRESHOLD_XL2   0x08 // 0x21 = ~45-degree angle

#define DUMP_RESET_KEY      0x00
#define LOGS_DUMP_KEY       0x11
#define META_DUMP_KEY       0x22
#define JUXTA_BASE_LOGS     0 // put logs here in NAND
#define JUXTA_BASE_META     0x200000 // put meta here in NAND
#define VDROPOUT            2.8 // LM3673 >= 2.7V, ADP2108 >= 2.3V
static const uint16_t IS_NOT_REQUESTING_TIME = 0x0000;
static const uint16_t IS_REQUESTING_TIME = 0xFFFF;

// Juxta NVS
#define JUXTA_LOG_ENTRY_SIZE            13
#define JUXTA_META_ENTRY_SIZE           11

typedef enum
{
    JUXTA_CONFIG_OFFSET_LOGCOUNT,
    JUXTA_CONFIG_OFFSET_LOGADDR,
    JUXTA_CONFIG_OFFSET_METACOUNT,
    JUXTA_CONFIG_OFFSET_METAADDR,
    JUXTA_CONFIG_OFFSET_MODE,
    JUXTA_CONFIG_OFFSET_SUBJECT0,
    JUXTA_CONFIG_OFFSET_SUBJECT1,
    JUXTA_CONFIG_OFFSET_SUBJECT2,
    JUXTA_CONFIG_OFFSET_SUBJECT3,
    JUXTA_CONFIG_OFFSET_NUMEL // leave at end
} juxtaConfigOffsets_t;

// Timing
#define JUXTA_LED_TIMEOUT_PERIOD        50 // ms
#define LSM303AGR_BOOT_TIME             5 // ms
#define SPI_HALF_PERIOD                 1 // 1us, Fs = 500kHz
#define JUXTA_1HZ_PERIOD                1000 // ms
#define JUXTA_SUBHZ_PERIOD              1000 * 60 * 5 // (1min) * n
#define JUXTA_STARTUP_TIMEOUT           100 // ms
#define JUXTA_6D_SHAKE_TO_WAKE_AFTER    10 // s
#define JUXTA_CENTRAL_TIMEOUT           5000 // ms
#define JUXTA_CONN_ATTEMPT_TIMEOUT      4000 // ms, should be >> advertising interval
#define REQUEST_TIME_EVERY              86400 // s
#define JUXTA_INT_TIMEOUT               1000 * 60 // ms

// Application events
#define MR_EVT_CHAR_CHANGE         1
#define MR_EVT_ADV_REPORT          2
#define MR_EVT_SCAN_ENABLED        3
#define MR_EVT_SCAN_DISABLED       4
#define MR_EVT_SVC_DISC            5
#define MR_EVT_ADV                 6
#define MR_EVT_PAIRING_STATE       7
#define MR_EVT_PASSCODE_NEEDED     8
#define MR_EVT_SEND_PARAM_UPDATE   9
#define MR_EVT_READ_RPA            10
#define MR_EVT_INSUFFICIENT_MEM    11
#define JUXTA_EVT_LED_TIMEOUT      12
#define JUXTA_TIME_UPDATED         13
#define JUXTA_EVT_1HZ              14
#define JUXTA_EVT_SUBHZ            15
#define JUXTA_EVT_INTERVAL_MODE    16
#define JUXTA_EVT_CONNECTED        17
#define JUXTA_EVT_DISCONNECTED     18
#define JUXTA_EVT_INT_XL1          19
#define JUXTA_EVT_INT_XL2          20
#define JUXTA_EVT_INT_MG           21
#define JUXTA_EVT_STARTUP          22
#define JUXTA_EVT_INT_XL1_TIMEOUT  23
#define JUXTA_EVT_INT_MG_TIMEOUT   24
#define JUXTA_EVT_CENTRAL_TIMEOUT  25
#define JUXTA_EVT_WRITE_GATT       26
#define JUXTA_EVT_TIME_UPDATE      27

// Internal Events for RTOS application
#define MR_ICALL_EVT                         ICALL_MSG_EVENT_ID // Event_Id_31
#define MR_QUEUE_EVT                         UTIL_QUEUE_EVENT_ID // Event_Id_30

#define MR_ALL_EVENTS                        (MR_ICALL_EVT           | \
                                              MR_QUEUE_EVT)

// Supervision timeout conversion rate to miliseconds
#define CONN_TIMEOUT_MS_CONVERSION            10

// Task configuration
#define MR_TASK_PRIORITY                     1
#ifndef MR_TASK_STACK_SIZE
#define MR_TASK_STACK_SIZE                   1024
#endif

// Discovery states
typedef enum
{
    BLE_DISC_STATE_IDLE,                // Idle
    BLE_DISC_STATE_MTU,                 // Exchange ATT MTU size
    BLE_DISC_STATE_SVC,                 // Service discovery
    BLE_DISC_STATE_CHAR                 // Characteristic discovery
} discState_t;

// address string length is an ascii character for each digit +
#define MR_ADDR_STR_SIZE     15
#define CONNINDEX_INVALID  0xFF

// Spin if the expression is not true
#define MULTIROLE_ASSERT(expr) if (!(expr)) multi_role_spin();

/*********************************************************************
 * TYPEDEFS
 */

// App event passed from profiles.
typedef struct
{
    uint8_t event;    // event type
    void *pData;   // event data pointer
} mrEvt_t;

// Container to store paring state info when passing from gapbondmgr callback
// to app event. See the pfnPairStateCB_t documentation from the gapbondmgr.h
// header file for more information on each parameter.
typedef struct
{
    uint8_t state;
    uint16_t connHandle;
    uint8_t status;
} mrPairStateData_t;

// Container to store passcode data when passing from gapbondmgr callback
// to app event. See the pfnPasscodeCB_t documentation from the gapbondmgr.h
// header file for more information on each parameter.
typedef struct
{
    uint8_t deviceAddr[B_ADDR_LEN];
    uint16_t connHandle;
    uint8_t uiInputs;
    uint8_t uiOutputs;
    uint32_t numComparison;
} mrPasscodeData_t;

// Scanned device information record
typedef struct
{
    uint8_t addrType;         // Peer Device's Address Type
    uint8_t addr[B_ADDR_LEN]; // Peer Device Address
    int8_t rssi;
    uint16_t requestTime;
} scanRec_t;

// Container to store information from clock expiration using a flexible array
// since data is not always needed
typedef struct
{
    uint8_t event;
    uint8_t data[];
} mrClockEventData_t;

// Container to store advertising event data when passing from advertising
// callback to app event. See the respective event in GapAdvScan_Event_IDs
// in gap_advertiser.h for the type that pBuf should be cast to.
typedef struct
{
    uint32_t event;
    void *pBuf;
} mrGapAdvEventData_t;

// List element for parameter update and PHY command status lists
typedef struct
{
    List_Elem elem;
    uint16_t connHandle;
} mrConnHandleEntry_t;

// Connected device information
typedef struct
{
    uint16_t connHandle;           // Connection Handle
    mrClockEventData_t *pParamUpdateEventData; // pointer to paramUpdateEventData
    uint16_t charHandle;           // Characteristic Handle
    uint8_t addr[B_ADDR_LEN];     // Peer Device Address
    Clock_Struct *pUpdateClock;         // pointer to clock struct
    uint8_t discState;            // Per connection discovery state
    uint8_t role; // GAP_PROFILE_PERIPHERAL | GAP_PROFILE_CENTRAL
} mrConnRec_t;

/*********************************************************************
 * LOCAL VARIABLES
 */

#define APP_EVT_EVENT_MAX  0x13
char *appEventStrings[] = { "APP_ZERO             ", "APP_CHAR_CHANGE      ",
                            "APP_KEY_CHANGE       ", "APP_ADV_REPORT       ",
                            "APP_SCAN_ENABLED     ", "APP_SCAN_DISABLED    ",
                            "APP_SVC_DISC         ", "APP_ADV              ",
                            "APP_PAIRING_STATE    ", "APP_SEND_PARAM_UPDATE",
                            "APP_PERIODIC         ", "APP_READ_RPA         ",
                            "APP_INSUFFICIENT_MEM ", };

/*********************************************************************
 * LOCAL VARIABLES
 */

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Event globally used to post local events and pend on system and
// local events.
static ICall_SyncHandle syncEvent;

// Clock instance for RPA read events.
static Clock_Struct clkRpaRead;

// Memory to pass RPA read event ID to clock handler
mrClockEventData_t argRpaRead = { .event = MR_EVT_READ_RPA };
// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

// Task configuration
Task_Struct mrTask;

#if defined __TI_COMPILER_VERSION__
#pragma DATA_ALIGN(mrTaskStack, 8)
#else
#pragma data_alignment=8
#endif
uint8_t mrTaskStack[MR_TASK_STACK_SIZE];

// Maximim PDU size (default = 27 octets)
static uint16 mrMaxPduSize;

#if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
// Number of scan results filtered by Service UUID
static uint8_t numScanRes = 0;

// Scan results filtered by Service UUID
static scanRec_t scanList[DEFAULT_MAX_SCAN_RES];
#endif // DEFAULT_DEV_DISC_BY_SVC_UUID

// Discovered service start and end handle
static uint16_t svcStartHdl = 0;
static uint16_t svcEndHdl = 0;

// Value to write
static uint8_t charVal = 0;

// Number of connected devices
static uint8_t numConn = 0;

// Connection handle of current connection
static uint16_t mrConnHandle = LINKDB_CONNHANDLE_INVALID;

// List to store connection handles for queued param updates
static List_List paramUpdateList;

// Per-handle connection info
static mrConnRec_t connList[MAX_NUM_BLE_CONNS];

// Advertising handles
static uint8 advHandle;

// Address mode
static GAP_Addr_Modes_t addrMode = DEFAULT_ADDRESS_MODE;

// Current Random Private Address
static uint8 rpa[B_ADDR_LEN] = { 0 };

// Initiating PHY
static uint8_t mrInitPhy = INIT_PHY_1M;

// JUXTA
static Clock_Struct clkJuxtaLEDTimeout;
mrClockEventData_t argJuxtaLEDTimeout = { .event = JUXTA_EVT_LED_TIMEOUT };
static Clock_Struct clkJuxta1Hz;
mrClockEventData_t argJuxta1Hz = { .event = JUXTA_EVT_1HZ };
static Clock_Struct clkJuxtaSubHz;
mrClockEventData_t argJuxtaSubHz = { .event = JUXTA_EVT_SUBHZ };
static Clock_Struct clkJuxtaIntervalMode;
mrClockEventData_t argJuxtaIntervalMode = { .event = JUXTA_EVT_INTERVAL_MODE };
static Clock_Struct clkJuxtaStartup;
mrClockEventData_t argJuxtaStartup = { .event = JUXTA_EVT_STARTUP };
static Clock_Struct clkJuxtaXLIntTimeout;
mrClockEventData_t argJuxtaXLIntTimeout = { .event = JUXTA_EVT_INT_XL1_TIMEOUT };
static Clock_Struct clkJuxtaMGIntTimeout;
mrClockEventData_t argJuxtaMGIntTimeout = { .event = JUXTA_EVT_INT_MG_TIMEOUT };
static Clock_Struct clkJuxtaCentralTimeout;
mrClockEventData_t argJuxtaCentralTimeout =
        { .event = JUXTA_EVT_CENTRAL_TIMEOUT };
static Clock_Struct clkJuxtaTimeUpdateTimeout;
mrClockEventData_t argJuxtaTimeUpdateTimeout =
        { .event = JUXTA_EVT_TIME_UPDATE };

typedef enum
{
    JUXTA_DATATYPE_XL,
    JUXTA_DATATYPE_MG,
    JUXTA_DATATYPE_CONN,
    JUXTA_DATATYPE_VBATT,
    JUXTA_DATATYPE_TEMP,
    JUXTA_DATATYPE_MODE,
    JUXTA_DATATYPE_TIME_SYNC,
    JUXTA_DATATYPE_IS_BASE,
    JUXTA_DATATYPE_RESET_LOGS,
    JUXTA_DATATYPE_RESET_META
} juxtaDatatypes_t;

typedef enum
{
    JUXTA_OPT_MODE,
    JUXTA_OPT_BASE,
    JUXTA_OPT_ADV_EVERY_0,
    JUXTA_OPT_ADV_EVERY_1,
    JUXTA_OPT_SCAN_EVERY_0,
    JUXTA_OPT_SCAN_EVERY_1
} juxtaOptions_t;

static uint32_t lastXLIntTime, lastMGIntTime;

typedef enum
{
    JUXTA_MODE_SHELF, JUXTA_MODE_INTERVAL,
//    JUXTA_MODE_MOTION,
//    JUXTA_MODE_BASE,
    JUXTA_MODE_NUMEL // leave at end
} juxtaMode_t;

static uint8_t juxtaMode;
static bool scanInitDone = false;
static bool advertInitDone = false;
static uint32_t localTime = 0;
static uint8_t nScanRes = 0;

static uint32_t juxtaAdvInterval; // ms * 1.6
static uint32_t juxtaAdvInterval_table[4] = { 1000 * 1.6, 2000 * 1.6, 5000
                                                      * 1.6,
                                              10000 * 1.6 };
static uint32_t juxtaScanInterval;
static uint32_t juxtaScanInterval_table[4] = { 10 * 1000, 20 * 1000, 30 * 1000,
                                               60 * 1000 };

typedef enum
{
    JUXTA_SCAN_DISABLE, JUXTA_SCAN_ONCE
} juxtaScanModes_t;

typedef enum
{
    JUXTA_ADV_DISABLE, JUXTA_ADV_ONCE, JUXTA_ADV_FOREVER
} juxtaAdvModes_t;

static bool isConnected = false;
static bool isBase; // set at init
static uint32_t lastTimeUpdate = 0;

NVS_Handle nvsConfigHandle;
NVS_Attrs regionAttrs;
static uint32_t nvsConfigBuffer[JUXTA_CONFIG_OFFSET_NUMEL];

static int16_t data_raw_acceleration[3];
static int16_t data_raw_magnetic[3];
static int16_t data_raw_temperature;
static float acceleration_mg[3];
static float magnetic_mG[3];
static float temperature_degC = 0;
static uint8_t whoamI, rst;

typedef struct
{
    uint_least8_t CS_PIN;
    uint_least8_t CLK_PIN;
    uint_least8_t DIO_PIN;
} sensbus_t;

static sensbus_t xl_bus = { CS_XL, SPC, SDIO };
static sensbus_t mag_bus = { CS_MAG, SPC, SDIO };
stmdev_ctx_t dev_ctx_xl;
stmdev_ctx_t dev_ctx_mg;

GPIO_PinConfig sdioPinConfigs[2] = { GPIO_CFG_OUTPUT_INTERNAL
                                             | GPIO_CFG_OUT_STR_LOW
                                             | GPIO_CFG_OUT_LOW, /* OUTPUT */
                                     GPIO_CFG_INPUT_INTERNAL
                                             | GPIO_CFG_IN_INT_NONE
                                             | GPIO_CFG_PULL_UP_INTERNAL, /* INPUT */
};

GPIO_PinConfig basePinConfig[1] = { GPIO_CFG_INPUT_INTERNAL
        | GPIO_CFG_IN_INT_NONE | GPIO_CFG_PULL_NONE_INTERNAL, /* INPUT */
};

uint8_t dataBuffer[JUXTAPROFILE_DATA_LEN] = { 0 };

ADC_Handle adc_vBatt;
ADC_Params adcParams_vBatt;
int_fast16_t adcRes;
static uint16_t data_raw_voltage;
static uint32_t voltage_uv = 0;
static float vbatt = 0.0;
static bool has_NAND;
static uint8_t xl_ref_buff;

// addresses for NAND memory, tracked in NAND_Write()
static uint32_t logAddr, logRecoveryAddr, metaAddr, metaRecoveryAddr; // recall from NVS
// counts track log/metaAddr but are atomic
static uint32_t logCount, logRecoveryCount, metaCount, metaRecoveryCount;
// volatile buffers for collection
static uint8_t logBuffer[PAGE_SIZE], metaBuffer[PAGE_SIZE]; // 2176, PAGE_DATA_SIZE = 2048, use for R/W
// entries = single units of collected data
static uint8_t logEntry[JUXTA_LOG_ENTRY_SIZE];
static uint8_t metaEntry[JUXTA_META_ENTRY_SIZE];
// variables used in the data dump process
static uint32_t dumpCount, dumpAddr = 0;
static uint8_t dumpResetFlag = 1;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void multi_role_init(void);
static void multi_role_scanInit(void);
static void multi_role_advertInit(void);
static void multi_role_taskFxn(UArg a0, UArg a1);

static uint8_t multi_role_processStackMsg(ICall_Hdr *pMsg);
static uint8_t multi_role_processGATTMsg(gattMsgEvent_t *pMsg);
static void multi_role_processAppMsg(mrEvt_t *pMsg);
static void multi_role_processCharValueChangeEvt(uint8_t paramID);
static void multi_role_processGATTDiscEvent(gattMsgEvent_t *pMsg);
static void multi_role_processPasscode(mrPasscodeData_t *pData);
static void multi_role_processPairState(mrPairStateData_t *pairingEvent);
static void multi_role_processGapMsg(gapEventHdr_t *pMsg);
static void multi_role_processParamUpdate(uint16_t connHandle);
static void multi_role_processAdvEvent(mrGapAdvEventData_t *pEventData);

static void multi_role_charValueChangeCB(uint8_t paramID);
static status_t multi_role_enqueueMsg(uint8_t event, void *pData);
static uint16_t multi_role_getConnIndex(uint16_t connHandle);
static uint8_t multi_role_addConnInfo(uint16_t connHandle, uint8_t *pAddr,
                                      uint8_t role);
static void multi_role_clockHandler(UArg arg);
static uint8_t multi_role_clearConnListEntry(uint16_t connHandle);
#if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
static void multi_role_addScanInfo(uint8_t *pAddr, uint8_t addrType,
                                   int8_t rssi, uint16_t requestTime);
static bool multi_role_findSvcUuid(uint16_t uuid, uint8_t *pData,
                                   uint16_t dataLen);
#endif // DEFAULT_DEV_DISC_BY_SVC_UUID
static uint8_t multi_role_removeConnInfo(uint16_t connHandle);

static void multi_role_startSvcDiscovery(void);
#ifndef Display_DISABLE_ALL
static char* multi_role_getConnAddrStr(uint16_t connHandle);
#endif
static void multi_role_advCB(uint32_t event, void *pBuf, uintptr_t arg);
static void multi_role_scanCB(uint32_t evt, void *msg, uintptr_t arg);
static void multi_role_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                  uint8_t uiInputs, uint8_t uiOutputs,
                                  uint32_t numComparison);
static void multi_role_pairStateCB(uint16_t connHandle, uint8_t state,
                                   uint8_t status);
static void multi_role_updateRPA(void);
bool multi_role_doConnect(uint8_t index);
bool multi_role_doGattRead(void);
bool multi_role_doGattWrite(void);
bool multi_role_doDisconnect(void);

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
static void platform_delay(uint32_t ms);
static void shutdownLEDs(void);
static void timeoutLED(uint8_t index);
static void loadConfigs(void);
static void saveConfigs(void);
static void logScan(void);
static void logMetaData(uint8_t dataType, float data, uint32_t lastTime);
static void doScan(juxtaScanModes_t scanMode);
static void doAdvertise(juxtaAdvModes_t advMode);
ReturnType NAND_Write(uAddrType *addr, uint8_t *buffer, uint8_t *data,
                      uint16_t len);
static void setXL(void);
static void setMag(void);
static void setTemp(void);
void intXL1(uint_least8_t index); // not static for SysConfig
void intXL2(uint_least8_t index); // not static for SysConfig
void intMG(uint_least8_t index); // not static for SysConfig
static void clearIntXL1(void);
static void clearIntMG(void);
static void clearIntXL1(void);
static void clearIntMG(void);
static void setTemp(void);
static void setVoltage(void);
static uint8_t get_bit_value(const uint8_t *data, uint16_t n);
static void set_bit_value(uint8_t *data, uint16_t n, uint8_t bit_value,
                          uint16_t len);
static void setMetaRecovery(uint32_t recoveryAddr, uint32_t recoveryCount,
                            ReturnType ret);
static void setLogRecovery(uint32_t recoveryAddr, uint32_t recoveryCount,
                           ReturnType ret);
static ReturnType dumpData(uint32_t *addr, uint8_t *buffer, uint32_t BASE);

/*********************************************************************
 * EXTERN FUNCTIONS
 */
extern void AssertHandler(uint8 assertCause, uint8 assertSubcause);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// Simple GATT Profile Callbacks
static simpleProfileCBs_t multi_role_simpleProfileCBs = {
        multi_role_charValueChangeCB // Characteristic value change callback
        };

// GAP Bond Manager Callbacks
static gapBondCBs_t multi_role_BondMgrCBs = { multi_role_passcodeCB, // Passcode callback
        multi_role_pairStateCB                  // Pairing state callback
        };

/*********************************************************************
 * PUBLIC FUNCTIONS
 */
static void juxtaTryTimeUpdate(void)
{
    for (uint8_t index = 0; index < DEFAULT_MAX_SCAN_RES; index++)
    {
        if (scanList[index].requestTime == IS_REQUESTING_TIME)
        {
            scanList[index].requestTime = IS_NOT_REQUESTING_TIME; // this will reset with every scan anyways
            multi_role_doConnect(index); // does GapInit_connect(scanList[index])...)
            Util_startClock(&clkJuxtaTimeUpdateTimeout);
            break;
        }
    }
}

static bool juxtaSetAdvData(uint8_t *pData, uint8_t dataLen, void *pValue,
                            uint8_t adType)
{
    uint8_t *pEnd;
    uint8_t len;
    uint8_t dataType;
    pEnd = pData + dataLen - 1;

// While end of data not reached
    while (pData < pEnd)
    {
        // Get length of next AD item
        len = *pData++;
        if (len > 0)
        {
            dataType = *pData;
            if (dataType == adType) // match?
            {
                pData++;
                len--;
                memcpy(pData, pValue, len);
                return true;
            }
            else
            {
                // Go to next item
                pData += len;
            }
        }
    }
    return false;
}

static bool juxtaGetAdvData(uint8_t *pData, uint8_t dataLen, void *pValue,
                            uint8_t adType)
{
    uint8_t *pEnd;
    uint8_t len;
    uint8_t dataType;
    pEnd = pData + dataLen - 1;

// While end of data not reached
    while (pData < pEnd)
    {
        // Get length of next AD item
        len = *pData++;
        if (len > 0)
        {
            dataType = *pData;
            if (dataType == adType) // match?
            {
                pData++;
                len--;
                memcpy(pValue, pData, len);
                return true;
            }
            else
            {
                // Go to next item
                pData += len;
            }
        }
    }
    return false;
}

static void juxtaRequestTime(bool doRequest)
{
    if (doRequest)
    {
        juxtaSetAdvData(advData1, sizeof(advData1), (void*) &IS_REQUESTING_TIME,
        GAP_ADTYPE_APPEARANCE);
    }
    else
    {
        juxtaSetAdvData(advData1, sizeof(advData1),
                        (void*) &IS_NOT_REQUESTING_TIME,
                        GAP_ADTYPE_APPEARANCE);
    }
    doAdvertise(JUXTA_ADV_DISABLE);
    GapAdv_loadByHandle(advHandle, GAP_ADV_DATA_TYPE_ADV, sizeof(advData1),
                        advData1);
    doAdvertise(JUXTA_ADV_FOREVER);
}

// could also make juxtaOptions a global
static uint8_t getJuxtaOptions(void)
{
    uint8_t juxtaOptions = 0, k;
    juxtaOptions |= 0b00000001 & juxtaMode;
    if (isBase)
    {
        juxtaOptions |= 0b00000010;
    }
// lookup tables
    for (k = 0; k < sizeof(juxtaAdvInterval_table); k++)
    {
        if (juxtaAdvInterval == juxtaAdvInterval_table[k])
        {
            juxtaOptions |= 0b00001100 & (k << JUXTA_OPT_ADV_EVERY_0);
            break;
        }
    }
    for (k = 0; k < sizeof(juxtaScanInterval_table); k++)
    {
        if (juxtaScanInterval == juxtaScanInterval_table[k])
        {
            juxtaOptions |= 0b00110000 & (k << JUXTA_OPT_SCAN_EVERY_0);
            break;
        }
    }
    simpleProfile_SetParameter(JUXTAPROFILE_ADVMODE,
    JUXTAPROFILE_ADVMODE_LEN,
                               &juxtaOptions);
    return juxtaOptions;
}

static void setJuxtaOptions(uint8_t juxtaOptions)
{
    uint8_t k; // index
    juxtaMode = 0b00000001 & juxtaOptions;
// 0b00000010 = isBase, read-only
    k = (0b00001100 & juxtaOptions) >> JUXTA_OPT_ADV_EVERY_0;
    juxtaAdvInterval = juxtaAdvInterval_table[k];

    k = (0b00110000 & juxtaOptions) >> JUXTA_OPT_SCAN_EVERY_0;
    juxtaScanInterval = juxtaScanInterval_table[k];
}

// could test status, but not sure what to do with it
static void logMetaData(uint8_t dataType, float data, uint32_t lastTime)
{
    if (!has_NAND)
        return;

    uint8_t iEntry = 0;
    uint8_t uuid[ATT_BT_UUID_SIZE] = { LO_UINT16(JUXTAPROFILE_SERV_UUID),
                                       HI_UINT16(JUXTAPROFILE_SERV_UUID) };

    uint32_t tempAddr = metaAddr;
    uint32_t tempCount = metaCount;
    memcpy(metaEntry, uuid, sizeof(uuid));
    iEntry += sizeof(uuid); // int16
    memcpy(metaEntry + iEntry, &dataType, sizeof(dataType));
    iEntry += sizeof(dataType); // uint8_t
    memcpy(metaEntry + iEntry, &data, sizeof(data)); // uint32
    iEntry += sizeof(data); // uint32_t
    memcpy(metaEntry + iEntry, &lastTime, sizeof(lastTime)); // uint32
    ReturnType ret = NAND_Write(&metaAddr, metaBuffer, metaEntry,
    JUXTA_META_ENTRY_SIZE);
    metaCount++;
    simpleProfile_SetParameter(JUXTAPROFILE_METACOUNT,
    JUXTAPROFILE_METACOUNT_LEN,
                               &metaCount);
    // if a page was written, commit these (new metaCount is only in buffer)
    setMetaRecovery(tempAddr, tempCount, ret);
}

static void doScan(juxtaScanModes_t scanMode)
{
    if (scanMode == JUXTA_SCAN_ONCE)
    {
        GapScan_enable(0, DEFAULT_SCAN_DURATION, DEFAULT_MAX_SCAN_RES);
    }
    else
    {
        GapScan_disable();
    }
}

static void doAdvertise(juxtaAdvModes_t advMode)
{
    if (advMode == JUXTA_ADV_ONCE)
    {
        GapAdv_enable(advHandle, GAP_ADV_ENABLE_OPTIONS_USE_DURATION,
                      DEFAULT_SCAN_DURATION);
    }
    else if (advMode == JUXTA_ADV_FOREVER)
    {
        GapAdv_enable(advHandle, GAP_ADV_ENABLE_OPTIONS_USE_MAX, 0);
    }
    else
    {
        GapAdv_disable(advHandle);
    }
}

ReturnType NAND_Write(uAddrType *addr, uint8_t *buffer, uint8_t *data,
                      uint16_t len)
{
    ReturnType ret = Flash_ProgramFailed;
    uint32_t nvsAddr = *addr;
    uint32_t MAX_MEM_LOC = FLASH_SIZE; // meta memory goes to end
    uint16_t i;

    if (*addr < JUXTA_BASE_META) // it must be logs
    {
        MAX_MEM_LOC = JUXTA_BASE_META;
    }
    if (*addr + len >= MAX_MEM_LOC)
    {
        ret = Flash_MemoryOverflow;
        return ret;
    }

    for (i = 0; i < len; i++)
    {
        memcpy(buffer + ADDRESS_2_COL(*addr), data + i, sizeof(uint8_t)); // add byte to buffer
// check if area needs to be erased
        if (ADDRESS_2_PAGE(*addr) == 0 && ADDRESS_2_COL(*addr) == 0) // 0th page in block, 0th col
        {
            ret = FlashBlockErase(*addr); // wipe next 64 pages
            if (ret != Flash_Success)
            {
                return ret; // return early, !! but then what?
            }
            ret = Flash_ProgramFailed; // reset, return hereon should reflect FlashPageProgram()
        }
        *addr += 1;
        if (ADDRESS_2_COL(*addr) == PAGE_DATA_SIZE) // end of page/last col: write it
        {
            (*addr) &= 0xFFFFF000; // force write to 0th column index
            ret = FlashPageProgram(*addr, buffer, PAGE_DATA_SIZE); // write page
            (*addr) += 0x00001000; // increment page (2 * PAGE_DATA_SIZE)
        }
    }
    return ret;
}

static void setXL(void)
{
    /* Read output only if new value is available */
    lsm303agr_reg_t reg;
    lsm303agr_xl_status_get(&dev_ctx_xl, &reg.status_reg_a);

    if (reg.status_reg_a.zyxda)
    {
        /* Read accelerometer data */
        lsm303agr_acceleration_raw_get(&dev_ctx_xl, data_raw_acceleration);
        acceleration_mg[0] = lsm303agr_from_fs_2g_hr_to_mg(
                data_raw_acceleration[0]);
        acceleration_mg[1] = lsm303agr_from_fs_2g_hr_to_mg(
                data_raw_acceleration[1]);
        acceleration_mg[2] = lsm303agr_from_fs_2g_hr_to_mg(
                data_raw_acceleration[2]);
    }
}

static void setMag(void)
{
    /* Read output only if new value is available */
    lsm303agr_reg_t reg;
    lsm303agr_mag_status_get(&dev_ctx_mg, &reg.status_reg_m);

    if (reg.status_reg_m.zyxda)
    {
        /* Read magnetic field data */
        lsm303agr_magnetic_raw_get(&dev_ctx_mg, data_raw_magnetic);
        magnetic_mG[0] = lsm303agr_from_lsb_to_mgauss(data_raw_magnetic[0]);
        magnetic_mG[1] = lsm303agr_from_lsb_to_mgauss(data_raw_magnetic[1]);
        magnetic_mG[2] = lsm303agr_from_lsb_to_mgauss(data_raw_magnetic[2]);
    }
}

// setup in SysConfig
void intMG(uint_least8_t index)
{
    multi_role_enqueueMsg(JUXTA_EVT_INT_MG, NULL);
}

static void clearIntMG(void)
{
    lsm303agr_int_source_reg_m_t int_source_reg_m;
    GPIO_clearInt(INT_MAG);
    do
    {
        lsm303agr_mag_int_gen_source_get(&dev_ctx_mg, &int_source_reg_m);
    }
    while (!GPIO_read(INT_MAG));
}

// setup in SysConfig
void intXL1(uint_least8_t index)
{
    multi_role_enqueueMsg(JUXTA_EVT_INT_XL1, NULL);
}

static void clearIntXL1(void)
{
    lsm303agr_int1_src_a_t int1_src_reg_a;
    GPIO_clearInt(INT_XL_1);
    do
    {
        lsm303agr_xl_int1_gen_source_get(&dev_ctx_xl, &int1_src_reg_a);
    }
    while (!GPIO_read(INT_XL_1));
}

// setup in SysConfig
void intXL2(uint_least8_t index)
{
    multi_role_enqueueMsg(JUXTA_EVT_INT_XL2, NULL);
}

static void clearIntXL2(void)
{
    lsm303agr_int2_src_a_t int2_src_reg_a;
    GPIO_clearInt(INT_XL_2);
    do
    {
        lsm303agr_xl_int2_gen_source_get(&dev_ctx_xl, &int2_src_reg_a);
    }
    while (!GPIO_read(INT_XL_2));
}

static void setTemp(void)
{
    /* Read output only if new value is available */
    lsm303agr_reg_t reg;
    lsm303agr_temp_data_ready_get(&dev_ctx_xl, &reg.byte);

    if (reg.byte)
    {
        /* Read temperature data */
        lsm303agr_temperature_raw_get(&dev_ctx_xl, &data_raw_temperature);
        temperature_degC = lsm303agr_from_lsb_hr_to_celsius(
                data_raw_temperature);
        simpleProfile_SetParameter(JUXTAPROFILE_TEMP,
        JUXTAPROFILE_TEMP_LEN,
                                   &temperature_degC);
    }
}

static void setVoltage(void)
{
    ADC_convert(adc_vBatt, &data_raw_voltage);
    voltage_uv = (ADC_convertToMicroVolts(adc_vBatt, data_raw_voltage) * 250)
            / 100;
    vbatt = (float) voltage_uv / 1e6;
    simpleProfile_SetParameter(JUXTAPROFILE_VBATT, JUXTAPROFILE_VBATT_LEN,
                               &voltage_uv);
}

static uint8_t get_bit_value(const uint8_t *data, uint16_t n)
{
    uint8_t byte_index = n / 8; // calculate the byte index
    uint8_t bit_index = n % 8; // calculate the bit index
    uint8_t byte = *(data + byte_index); // get the byte at the byte index

// use bitwise operators to get the bit value
    uint8_t mask = 1 << bit_index;
    uint8_t bit_value = (byte & mask) >> bit_index;

    return bit_value;
}

// introduced len
static void set_bit_value(uint8_t *data, uint16_t n, uint8_t bit_value,
                          uint16_t len)
{
    uint8_t byte_index = len - (n / 8) - 1; // calculate the byte index
    uint8_t bit_index = n % 8; // calculate the bit index
    uint8_t byte = *(data + byte_index); // get the byte at the byte index

// use bitwise operators to set the bit value
    uint8_t mask = 1 << bit_index;
    byte &= ~mask; // clear the bit at the bit index
    byte |= (bit_value << bit_index); // set the bit to the specified value
    *(data + byte_index) = byte; // store the modified byte back into the data array
}

// for 3-wire SPI
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{

    sensbus_t *sensbus = (sensbus_t*) handle;
    uint8_t i;
    reg |= 0x80; // set read bit
    if (sensbus->CS_PIN == CS_XL)
    {
        reg |= 0x40; // auto-increment r/w
    }

//    GPIO_setConfig(sensbus->DIO_PIN, sdioPinConfigs[0]); // !! OUTPUT
    GPIO_write(sensbus->CLK_PIN, 1); // start high
    GPIO_write(sensbus->CS_PIN, 0); // active low
    usleep(1); // CS pin
// write addr
    for (i = 0; i < 8; i++)
    {
        GPIO_write(sensbus->CLK_PIN, 0); // clock down
        uint8_t bit_value = ((reg >> (7 - i)) & 1);
        GPIO_write(sensbus->DIO_PIN, bit_value); // addr, MSB

        usleep(SPI_HALF_PERIOD); // delay half period
        GPIO_write(sensbus->CLK_PIN, 1); // clock up
        usleep(SPI_HALF_PERIOD); // delay again
    }
// read data len
    GPIO_setConfig(sensbus->DIO_PIN, sdioPinConfigs[1]); // set as input
    for (i = 0; i < (8 * len); i++)
    {
        GPIO_write(sensbus->CLK_PIN, 0); // clock down
        usleep(SPI_HALF_PERIOD); // delay half period
        GPIO_write(sensbus->CLK_PIN, 1); // clock up
        uint8_t bitValue = GPIO_read(sensbus->DIO_PIN);
        uint8_t bitPos = (8 * len - 1) - i;
        set_bit_value(bufp, bitPos, bitValue, len);
        usleep(SPI_HALF_PERIOD); // delay again
    }
    GPIO_write(sensbus->CS_PIN, 1); // de-activate
    GPIO_setConfig(sensbus->DIO_PIN, sdioPinConfigs[0]); // set as output
    return 0;
}
// for 3-wire SPI
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len)
{
    sensbus_t *sensbus = (sensbus_t*) handle;
    uint8_t i;
    if (sensbus->CS_PIN == CS_XL)
    {
        reg |= 0x40; // auto-increment r/w
    }
    GPIO_setConfig(sensbus->DIO_PIN, sdioPinConfigs[0]); // set as output
    GPIO_write(sensbus->CLK_PIN, 1); // start high
    GPIO_write(sensbus->CS_PIN, 0); // active low
    usleep(1); // CS pin
// write addr
    for (i = 0; i < 8; i++)
    {
        GPIO_write(sensbus->CLK_PIN, 0); // clock down
        GPIO_write(sensbus->DIO_PIN, (reg >> (7 - i)) & 1); // addr, MSB
        usleep(SPI_HALF_PERIOD); // delay half period
        GPIO_write(sensbus->CLK_PIN, 1); // clock up
        usleep(SPI_HALF_PERIOD); // delay again
    }
// write data len
    for (i = 0; i < (8 * len); i++)
    {
        GPIO_write(sensbus->CLK_PIN, 0); // clock down
        GPIO_write(sensbus->DIO_PIN, get_bit_value(bufp, (8 * len - 1) - i));
        usleep(SPI_HALF_PERIOD); // delay half period
        GPIO_write(sensbus->CLK_PIN, 1); // clock up
        usleep(SPI_HALF_PERIOD); // delay again
    }
    GPIO_write(sensbus->CS_PIN, 1); // de-activate
//    GPIO_setConfig(sensbus->DIO_PIN, sdioPinConfigs[1]); // !! INPUT
    return 0;
}

static void platform_delay(uint32_t ms)
{
    usleep(ms * 1000);
}

static void timeoutLED(uint8_t index)
{
    if (index == LED1)
    {
        GPIO_write(index, 1);
        // restart ensures the clock is not started twice (avoid memory issues)
        Util_restartClock(&clkJuxtaLEDTimeout, JUXTA_LED_TIMEOUT_PERIOD);
    }
}

static void shutdownLEDs(void)
{
    GPIO_write(LED1, 0);
}

static ReturnType dumpData(uint32_t *addr, uint8_t *buffer, uint32_t BASE)
{
    ReturnType ret = Flash_ProgramFailed;
    if (!has_NAND)
        return ret;

    if (dumpAddr < *addr)
    {
// at start: write the existing buffer to NAND, fill remaining page with 0xFF
        if (dumpAddr == BASE && dumpCount == 0)
        {
            uint32_t restoreAddr = *addr;
            uint8_t none = 0xFF;
            while (1)
            {
                ret = NAND_Write(addr, buffer, &none, 1);
                if (ret == Flash_Success)
                    break;
            }
            *addr = restoreAddr; // put addr back to where it was
        }
// logDumpCount increments by 128 (JUXTAPROFILE_DATA_LEN): goes to zero every page
        if (dumpCount == 0)
        {
            FlashPageRead(dumpAddr, buffer);
        }
        memcpy(dataBuffer, buffer + (JUXTAPROFILE_DATA_LEN * dumpCount),
        JUXTAPROFILE_DATA_LEN);
        simpleProfile_SetParameter(JUXTAPROFILE_DATA,
        JUXTAPROFILE_DATA_LEN,
                                   dataBuffer);
        dumpCount++; // should iterate 16 times for each page
        if (dumpCount == PAGE_DATA_SIZE / JUXTAPROFILE_DATA_LEN)
        {
            dumpCount = 0; // reset to read page next time
            dumpAddr += 0x1000; // this increments until logAddr
        }
    }
    return ret;
}

static void setLogRecovery(uint32_t recoveryAddr, uint32_t recoveryCount,
                           ReturnType ret)
{
    if (ret == Flash_Success)
    {
        logRecoveryAddr = recoveryAddr;
        logRecoveryCount = recoveryCount;
        saveConfigs();
    }
}

static void setMetaRecovery(uint32_t recoveryAddr, uint32_t recoveryCount,
                            ReturnType ret)
{
    if (ret == Flash_Success)
    {
        metaRecoveryAddr = recoveryAddr;
        metaRecoveryCount = recoveryCount;
        saveConfigs();
    }
}

static void saveConfigs(void)
{
    NVS_getAttrs(nvsConfigHandle, &regionAttrs);
    NVS_erase(nvsConfigHandle, 0, regionAttrs.sectorSize); // erase all each time
// nvsConfigBuffer is uint32 (so is pointer that increments)
    nvsConfigBuffer[JUXTA_CONFIG_OFFSET_LOGCOUNT] = logRecoveryCount;
    nvsConfigBuffer[JUXTA_CONFIG_OFFSET_LOGADDR] = logRecoveryAddr;
    nvsConfigBuffer[JUXTA_CONFIG_OFFSET_METACOUNT] = metaRecoveryCount;
    nvsConfigBuffer[JUXTA_CONFIG_OFFSET_METAADDR] = metaRecoveryAddr;
    uint8_t juxtaOptions = getJuxtaOptions();
    nvsConfigBuffer[JUXTA_CONFIG_OFFSET_MODE] = (uint32_t) juxtaOptions;
// convert from uint8 arr
    memcpy(&nvsConfigBuffer[JUXTA_CONFIG_OFFSET_SUBJECT0],
           &juxtaProfile_subject, sizeof(juxtaProfile_subject));
    NVS_write(nvsConfigHandle, 0, (void*) nvsConfigBuffer,
              sizeof(nvsConfigBuffer),
              NVS_WRITE_POST_VERIFY);
}

static void loadConfigs(void)
{

    NVS_read(nvsConfigHandle, 0, (void*) nvsConfigBuffer,
             sizeof(nvsConfigBuffer));
    logRecoveryCount = nvsConfigBuffer[JUXTA_CONFIG_OFFSET_LOGCOUNT];
    logRecoveryAddr = nvsConfigBuffer[JUXTA_CONFIG_OFFSET_LOGADDR];
    metaRecoveryCount = nvsConfigBuffer[JUXTA_CONFIG_OFFSET_METACOUNT];
    metaRecoveryAddr = nvsConfigBuffer[JUXTA_CONFIG_OFFSET_METAADDR];
    uint8_t juxtaOptions = (uint8_t) nvsConfigBuffer[JUXTA_CONFIG_OFFSET_MODE];
    setJuxtaOptions(juxtaOptions);
// !! override defaults
    if (isBase)
    {
        juxtaAdvInterval = juxtaAdvInterval_table[0];
        juxtaScanInterval = juxtaScanInterval_table[0];
    }
    else
    {
        juxtaAdvInterval = juxtaAdvInterval_table[1];
        juxtaScanInterval = juxtaScanInterval_table[2];
    }

// convert back to uint8 arr
    memcpy(&juxtaProfile_subject,
           &nvsConfigBuffer[JUXTA_CONFIG_OFFSET_SUBJECT0],
           sizeof(juxtaProfile_subject));

    uint32_t readAddr;
    if (logRecoveryAddr < JUXTA_BASE_META)
    {
        logCount = logRecoveryCount;
        logAddr = logRecoveryAddr;
        readAddr = logRecoveryAddr;
        readAddr &= 0xFFFFF000; // read from 0th column
        FlashPageRead(readAddr, logBuffer);
    }
    else
    {
        logRecoveryCount = 0;
        logCount = 0;
        logRecoveryAddr = 0;
        logAddr = 0;
    }

    if (metaRecoveryAddr >= JUXTA_BASE_META && metaRecoveryAddr < FLASH_SIZE)
    {
        metaCount = metaRecoveryCount;
        metaAddr = metaRecoveryAddr;
        readAddr = metaRecoveryAddr;
        readAddr &= 0xFFFFF000; // read from 0th column
        FlashPageRead(readAddr, metaBuffer);
    }
    else
    {
        metaRecoveryCount = 0;
        metaCount = 0;
        metaRecoveryAddr = JUXTA_BASE_META;
        metaAddr = JUXTA_BASE_META;
    }
    saveConfigs(); // re-save
}

static void logScan(void) // called after MR_EVT_ADV_REPORT -> multi_role_addScanInfo()
{
    if (!has_NAND)
        return;

    uint8_t i;
    if (numScanRes > 0)
    {
        localTime = Seconds_get();
        for (i = 0; i < numScanRes; i++)
        {
            uint32_t tempAddr = logAddr;
            uint32_t tempCount = logCount;
            uint8_t iEntry = 0;
            // load buffer
            uint8_t uuid[ATT_BT_UUID_SIZE] =
                    { LO_UINT16(JUXTAPROFILE_SERV_UUID), HI_UINT16(
                            JUXTAPROFILE_SERV_UUID) };

            memcpy(logEntry, uuid, sizeof(uuid));
            iEntry += sizeof(uuid);
            memcpy(logEntry + iEntry, scanList[i].addr, B_ADDR_LEN);
            iEntry += B_ADDR_LEN;
            memcpy(logEntry + iEntry, &scanList[i].rssi,
                   sizeof(scanList[i].rssi));
            iEntry += sizeof(scanList[i].rssi);
            memcpy(logEntry + iEntry, &localTime, sizeof(localTime));
            ReturnType ret = NAND_Write(&logAddr, logBuffer, logEntry,
            JUXTA_LOG_ENTRY_SIZE);
            logCount++;
            setLogRecovery(tempAddr, tempCount, ret); // if a page was written, commit these
        }
        simpleProfile_SetParameter(JUXTAPROFILE_LOGCOUNT,
        JUXTAPROFILE_LOGCOUNT_LEN,
                                   &logCount);
        timeoutLED(LED1);
//        numScanRes = 0; do this in MR_EVT_SCAN_DISABLED
    }
}

static void axyInit(void)
{
    dev_ctx_xl.write_reg = platform_write;
    dev_ctx_xl.read_reg = platform_read;
    dev_ctx_xl.handle = (void*) &xl_bus;
    dev_ctx_mg.write_reg = platform_write;
    dev_ctx_mg.read_reg = platform_read;
    dev_ctx_mg.handle = (void*) &mag_bus;
    /* Wait sensor boot time */
    platform_delay(LSM303AGR_BOOT_TIME);
    /* Check device ID */
    lsm303agr_xl_spi_mode_set(&dev_ctx_xl, LSM303AGR_SPI_3_WIRE);
    whoamI = 0;
    do
    {
        lsm303agr_xl_device_id_get(&dev_ctx_xl, &whoamI);
    }
    while (whoamI != LSM303AGR_ID_XL);

    /* Restore default configuration for magnetometer */
    lsm303agr_mag_reset_set(&dev_ctx_mg, PROPERTY_ENABLE);
    do
    {
        lsm303agr_mag_reset_get(&dev_ctx_mg, &rst);
    }
    while (rst);
    lsm303agr_mag_i2c_interface_set(&dev_ctx_mg, LSM303AGR_I2C_DISABLE);
    whoamI = 0;
    do
    {
        lsm303agr_mag_device_id_get(&dev_ctx_mg, &whoamI);
    }
    while (whoamI != LSM303AGR_ID_MG);

    lsm303agr_xl_block_data_update_set(&dev_ctx_xl, PROPERTY_ENABLE);
    lsm303agr_xl_data_rate_set(&dev_ctx_xl, LSM303AGR_XL_ODR_10Hz); // LSM303AGR_XL_POWER_DOWN
    lsm303agr_xl_full_scale_set(&dev_ctx_xl, LSM303AGR_2g);
    lsm303agr_temperature_meas_set(&dev_ctx_xl, LSM303AGR_TEMP_ENABLE); // LSM303AGR_TEMP_ENABLE
    lsm303agr_xl_operating_mode_set(&dev_ctx_xl, LSM303AGR_HR_12bit);

    // INT1 config see AN4825 pg.37
    lsm303agr_xl_high_pass_int_conf_set(&dev_ctx_xl, LSM303AGR_ON_INT1_GEN); // CTRL_REG2_A
    lsm303agr_xl_high_pass_mode_set(&dev_ctx_xl, LSM303AGR_NORMAL_WITH_RST); // CTRL_REG2_A - LSM303AGR_AUTORST_ON_INT
    lsm303agr_xl_high_pass_on_outputs_set(&dev_ctx_xl,
    PROPERTY_ENABLE); // CTRL_REG2_A
    lsm303agr_xl_high_pass_bandwidth_set(&dev_ctx_xl, LSM303AGR_LIGHT); // CTRL_REG2_A
    lsm303agr_ctrl_reg3_a_t ctrl_reg3 = { 0 };
    ctrl_reg3.i1_aoi1 = 1;
    lsm303agr_xl_pin_int1_config_set(&dev_ctx_xl, &ctrl_reg3); // CTRL_REG3_A
    lsm303agr_xl_int1pin_notification_mode_set(&dev_ctx_xl,
                                               LSM303AGR_INT1_LATCHED); // CTRL_REG5_A
    lsm303agr_xl_int1_gen_threshold_set(&dev_ctx_xl,
    INT_THRESHOLD_XL1); // INT1_THS_A
    lsm303agr_xl_int1_gen_duration_set(&dev_ctx_xl,
    INT_DURATION_XL); // INT1_DURATION_A
    lsm303agr_xl_filter_reference_get(&dev_ctx_xl, &xl_ref_buff); // read/set ref: REFERENCE_A

    lsm303agr_int1_cfg_a_t int1_cfg_a = { 0 }; // OR (not AND) the events
    int1_cfg_a.xhie = 1; // only use high events
    int1_cfg_a.yhie = 1;
    int1_cfg_a.zhie = 1;
    lsm303agr_xl_int1_gen_conf_set(&dev_ctx_xl, &int1_cfg_a); // INT1_CFG_A
    lsm303agr_int1_src_a_t int1_src_reg_a;

    // INT2 config
    lsm303agr_int2_cfg_a_t int2_cfg_a = { 0 };
    int2_cfg_a.aoi = 1; // 6-direction position recognition
    int2_cfg_a._6d = 1;
    int2_cfg_a.zhie = 1; // XL facing sky
    lsm303agr_xl_int2_gen_conf_set(&dev_ctx_xl, &int2_cfg_a);
    lsm303agr_xl_int2pin_notification_mode_set(&dev_ctx_xl,
                                               LSM303AGR_INT2_LATCHED);
    lsm303agr_xl_int2_gen_threshold_set(&dev_ctx_xl,
    INT_THRESHOLD_XL2); // INT1_THS_A
    lsm303agr_xl_int2_gen_duration_set(&dev_ctx_xl, INT_DURATION_6D); // INT2_DURATION_A
    lsm303agr_ctrl_reg6_a_t ctrl_reg6_a = { 0 };
    ctrl_reg6_a.h_lactive = 1; // interrupt(s?) active low
    ctrl_reg6_a.i2_int2 = 1; // enables interrupt 2 function on INT2 pin
    lsm303agr_xl_pin_int2_config_set(&dev_ctx_xl, &ctrl_reg6_a); // CTRL_REG6_A

    // MG SECTION
    lsm303agr_mag_block_data_update_set(&dev_ctx_mg, PROPERTY_ENABLE);
    lsm303agr_mag_data_rate_set(&dev_ctx_mg, LSM303AGR_MG_ODR_10Hz);
    lsm303agr_mag_power_mode_set(&dev_ctx_mg, LSM303AGR_LOW_POWER);
    /* Set / Reset magnetic sensor mode */
    lsm303agr_mag_set_rst_mode_set(&dev_ctx_mg,
                                   LSM303AGR_SET_SENS_ONLY_AT_POWER_ON);
    // LSM303AGR_POWER_DOWN, LSM303AGR_SINGLE_TRIGGER, LSM303AGR_CONTINUOUS_MODE
    lsm303agr_mag_operating_mode_set(&dev_ctx_mg, LSM303AGR_POWER_DOWN);
    //    lsm303agr_mag_drdy_on_pin_set(&dev_ctx_mg, PROPERTY_ENABLE);
    //    lsm303agr_mag_int_on_pin_set(&dev_ctx_mg, PROPERTY_ENABLE);
    //// setup mg interrupt
    //    lsm303agr_mag_int_gen_treshold_set(&dev_ctx_mg, INT_THRESHOLD_MG);
    //    lsm303agr_int_crtl_reg_m_t int_ctrl_reg = { 0 };
    //    int_ctrl_reg.iel = 1; // latch
    //    int_ctrl_reg.ien = 1; // enable int
    //    int_ctrl_reg.xien = 1; // axis enabled
    //    int_ctrl_reg.yien = 1; // axis enabled
    //    int_ctrl_reg.zien = 1; // axis enabled
    //    lsm303agr_mag_int_gen_conf_set(&dev_ctx_mg, &int_ctrl_reg);
}

// *** TI CODE ***
static void multi_role_spin(void)
{
    volatile uint8_t x = 0;

    while (1)
    {
        x++;
    }
}

void multi_role_createTask(void)
{
    Task_Params taskParams;
// Configure task
    Task_Params_init(&taskParams);
    taskParams.stack = mrTaskStack;
    taskParams.stackSize = MR_TASK_STACK_SIZE;
    taskParams.priority = MR_TASK_PRIORITY;
    Task_construct(&mrTask, multi_role_taskFxn, &taskParams, NULL);
}

static void multi_role_init(void)
{
    GPIO_write(LED1, 1);

// https://dev.ti.com/tirex/explore/node?node=A__AFqGfTyW2A4kyl8oykWk8Q__com.ti.SIMPLELINK_CC13XX_CC26XX_SDK__BSEc4rl__6.20.00.29
    Seconds_set(0);

    isBase = !GPIO_read(BASE_GPIO);
    GPIO_setConfig(BASE_GPIO, basePinConfig[0]); // remove pull-up to decrease power consumption

    ADC_init();
    ADC_Params_init(&adcParams_vBatt);
    adc_vBatt = ADC_open(CONFIG_ADC_VBATT, &adcParams_vBatt);
    setVoltage();
// danger zone, undefined behavior, just blink
    if (vbatt < VDROPOUT)
    {
        while (1)
        {
            GPIO_toggle(LED1);
            usleep(50000);
        }
    }

// swap address to include BLE MAC address (unique for each device)
    uint64_t bleAddress = *((uint64_t*) (FCFG1_BASE + FCFG1_O_MAC_BLE_0))
            & 0xFFFFFFFFFFFF;
    uint8_t bleArr[6], i;
    for (i = 0; i < sizeof(bleArr); i++)
    {
        bleArr[i] = bleAddress >> 8 * i;
    }
    char *pStrAddr = Util_convertBdAddr2Str(bleArr); // size 13, includes 1 end char
    memset(attDeviceName + 1, 0, GAP_DEVICE_NAME_LEN - 1); // clean
    memcpy(attDeviceName + 1, pStrAddr, 12); // include J at start
    juxtaSetAdvData(advData1, sizeof(advData1), (void*) attDeviceName,
    GAP_ADTYPE_LOCAL_NAME_COMPLETE);

    NVS_init();
    nvsConfigHandle = NVS_open(NVS_JUXTA_CONFIG, NULL);
// note: this has to be in 3-wire mode so CS is manually controlled
    SPI_init();
    SPI_Params_init(&spiParams);
    SPI_MEM_handle = SPI_open(SPI_MEM_CONFIG, &spiParams);
    has_NAND = NAND_Init();
    loadConfigs(); // comes after NAND to fill recovery buffer

    // try to avoid rare temperature bug
    while (true)
    {
        axyInit();
//        setTemp();
//        if (temperature_degC > 0 && temperature_degC < 40) {
//            break;
//        }
        break;
    }

    BLE_LOG_INT_TIME(0, BLE_LOG_MODULE_APP, "APP : ---- init ", MR_TASK_PRIORITY);
// ******************************************************************
// N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
// ******************************************************************
// Register the current thread as an ICall dispatcher application
// so that the application can send and receive messages.
    ICall_registerApp(&selfEntity, &syncEvent);

//  Display_printf(dispHandle, MR_ROW_SEPARATOR, 0, "====================");

// Create an RTOS queue for message from profile to be sent to app.
    appMsgQueue = Util_constructQueue(&appMsg);

// Initialize Connection List
    multi_role_clearConnListEntry(LINKDB_CONNHANDLE_ALL);

// Set the Device Name characteristic in the GAP GATT Service
    GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN,
                     (void* )attDeviceName);

// Configure GAP
    {
        uint16_t paramUpdateDecision = DEFAULT_PARAM_UPDATE_REQ_DECISION;
// Pass all parameter update requests to the app for it to decide
        GAP_SetParamValue(GAP_PARAM_LINK_UPDATE_DECISION, paramUpdateDecision);
    }

// Set default values for Data Length Extension
// Extended Data Length Feature is already enabled by default
    {
// Set initial values to maximum, RX is set to max. by default(251 octets, 2120us)
// Some brand smartphone is essentially needing 251/2120, so we set them here.
#define APP_SUGGESTED_PDU_SIZE 251 //default is 27 octets(TX)
#define APP_SUGGESTED_TX_TIME 2120 //default is 328us(TX)

        HCI_LE_WriteSuggestedDefaultDataLenCmd(APP_SUGGESTED_PDU_SIZE,
                                               APP_SUGGESTED_TX_TIME);
    }
// Initialize GATT Client, used by GAPBondMgr to look for RPAO characteristic for network privacy
    GATT_InitClient();

// Register to receive incoming ATT Indications/Notifications
    GATT_RegisterForInd(selfEntity);

// Setup the GAP Bond Manager
    setBondManagerParameters();

// Initialize GATT attributes
    GGS_AddService(GAP_SERVICE);// GAP
    GATTServApp_AddService(GATT_ALL_SERVICES);   // GATT attributes
    DevInfo_AddService();                      // Device Information Service
    simpleProfile_AddService(GATT_ALL_SERVICES); // Simple GATT Profile

// Setup the SimpleProfile Characteristic Values
    {
        simpleProfile_SetParameter(JUXTAPROFILE_LOGCOUNT,
        JUXTAPROFILE_LOGCOUNT_LEN,
                                   &logCount);
        simpleProfile_SetParameter(JUXTAPROFILE_METACOUNT,
        JUXTAPROFILE_METACOUNT_LEN,
                                   &metaCount);
        simpleProfile_SetParameter(JUXTAPROFILE_LOCALTIME,
        JUXTAPROFILE_LOCALTIME_LEN,
                                   &localTime);
        simpleProfile_SetParameter(JUXTAPROFILE_VBATT,
        JUXTAPROFILE_VBATT_LEN,
                                   &voltage_uv);
        simpleProfile_SetParameter(JUXTAPROFILE_TEMP,
        JUXTAPROFILE_TEMP_LEN,
                                   &temperature_degC);
        // JUXTAPROFILE_ADVMODE is filled with juxtaOptions in getJuxtaOptions()
        simpleProfile_SetParameter(JUXTAPROFILE_DATA,
        JUXTAPROFILE_DATA_LEN,
                                   dataBuffer);
// no need to set CHAR8 (COMMAND), write-only
    }

// Register callback with SimpleGATTprofile
    simpleProfile_RegisterAppCBs(&multi_role_simpleProfileCBs);

// Start Bond Manager and register callback
    VOID GAPBondMgr_Register(&multi_role_BondMgrCBs);

// Register with GAP for HCI/Host messages
    GAP_RegisterForMsgs(selfEntity);

// Register for GATT local events and ATT Responses pending for transmission
    GATT_RegisterForMsgs(selfEntity);

    BLE_LOG_INT_TIME(0, BLE_LOG_MODULE_APP, "APP : ---- call GAP_DeviceInit", GAP_PROFILE_PERIPHERAL | GAP_PROFILE_CENTRAL);
//Initialize GAP layer for Peripheral and Central role and register to receive GAP events
    GAP_DeviceInit(GAP_PROFILE_PERIPHERAL | GAP_PROFILE_CENTRAL, selfEntity,
                   addrMode, &pRandomAddress);

// connection periodic
    Util_constructClock(&clkJuxta1Hz, multi_role_clockHandler, 0,
    JUXTA_1HZ_PERIOD,
                        false, (UArg) &argJuxta1Hz);
// time keeper/save function
    Util_constructClock(&clkJuxtaSubHz, multi_role_clockHandler,
    JUXTA_SUBHZ_PERIOD,
                        JUXTA_SUBHZ_PERIOD,
                        true,
                        (UArg) &argJuxtaSubHz);
// simple LED timeout/turnoff (blink)
    Util_constructClock(&clkJuxtaLEDTimeout, multi_role_clockHandler,
    JUXTA_LED_TIMEOUT_PERIOD,
                        0, false, (UArg) &argJuxtaLEDTimeout);
// one-shot, add period later
    Util_constructClock(&clkJuxtaIntervalMode, multi_role_clockHandler, 0, 0,
    false,
                        (UArg) &argJuxtaIntervalMode);
// one-shot to yoke XL logging sampling rate
    Util_constructClock(&clkJuxtaXLIntTimeout, multi_role_clockHandler,
    JUXTA_INT_TIMEOUT,
                        0,
                        false,
                        (UArg) &argJuxtaXLIntTimeout);
// one-shot to yoke MG logging sampling rate
    Util_constructClock(&clkJuxtaMGIntTimeout, multi_role_clockHandler,
    JUXTA_INT_TIMEOUT,
                        0,
                        false,
                        (UArg) &argJuxtaMGIntTimeout);
    Util_constructClock(&clkJuxtaCentralTimeout, multi_role_clockHandler,
    JUXTA_CENTRAL_TIMEOUT,
                        0,
                        false,
                        (UArg) &argJuxtaCentralTimeout);
    Util_constructClock(&clkJuxtaTimeUpdateTimeout, multi_role_clockHandler,
    JUXTA_CONN_ATTEMPT_TIMEOUT,
                        0,
                        false,
                        (UArg) &argJuxtaTimeUpdateTimeout);

// check status of scan/adv init
    Util_constructClock(&clkJuxtaStartup, multi_role_clockHandler,
    JUXTA_STARTUP_TIMEOUT,
                        JUXTA_STARTUP_TIMEOUT, true, (UArg) &argJuxtaStartup);
// interrupts and mode are handled at JUXTA_EVT_STARTUP -> JUXTA_EVT_DISCONNECTED
    if (isBase)
    {
        HCI_EXT_SetTxPowerCmd(HCI_EXT_TX_POWER_5_DBM);
    }
    logMetaData(JUXTA_DATATYPE_MODE, juxtaMode, localTime);
    logMetaData(JUXTA_DATATYPE_IS_BASE, isBase, localTime);
    juxtaRequestTime(true); // do after advertise is setup, for all devices

    shutdownLEDs();
}

static void multi_role_taskFxn(UArg a0, UArg a1)
{
// Initialize application
    multi_role_init();

// Application main loop
    for (;;)
    {
        uint32_t events;
// Waits for an event to be posted associated with the calling thread.
// Note that an event associated with a thread is posted when a
// message is queued to the message receive queue of the thread

        events = Event_pend(syncEvent, Event_Id_NONE, MR_ALL_EVENTS,
        ICALL_TIMEOUT_FOREVER); // event_31 + event_30

        if (events)
        {
            ICall_EntityID dest;
            ICall_ServiceEnum src;
            ICall_HciExtEvt *pMsg = NULL;

            if (ICall_fetchServiceMsg(&src, &dest,
                                      (void**) &pMsg) == ICALL_ERRNO_SUCCESS)
            {
                uint8_t safeToDealloc = TRUE;

                if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
                {
                    ICall_Stack_Event *pEvt = (ICall_Stack_Event*) pMsg;

                    // Check for BLE stack events first
                    if (pEvt->signature != 0xffff)
                    {
                        // Process inter-task message
                        safeToDealloc = multi_role_processStackMsg(
                                (ICall_Hdr*) pMsg);
                    }
                }

                if (pMsg && safeToDealloc)
                {
                    ICall_freeMsg(pMsg);
                }
            }

// If RTOS queue is not empty, process app message.
            if (events & MR_QUEUE_EVT)
            {
                while (!Queue_empty(appMsgQueue))
                {
                    mrEvt_t *pMsg = (mrEvt_t*) Util_dequeueMsg(appMsgQueue);
                    if (pMsg)
                    {
                        // Process message.
                        multi_role_processAppMsg(pMsg);

                        // Free the space from the message.
                        ICall_free(pMsg);
                    }
                }
            }
        }
    }
}

static uint8_t multi_role_processStackMsg(ICall_Hdr *pMsg)
{
    uint8_t safeToDealloc = TRUE;

    BLE_LOG_INT_INT(0, BLE_LOG_MODULE_APP, "APP : Stack msg status=%d, event=0x%x\n", pMsg->status, pMsg->event);

    switch (pMsg->event)
    {
    case GAP_MSG_EVENT:
//multi_role_processRoleEvent((gapMultiRoleEvent_t *)pMsg);
        multi_role_processGapMsg((gapEventHdr_t*) pMsg);
        break;

    case GATT_MSG_EVENT:
// Process GATT message
        safeToDealloc = multi_role_processGATTMsg((gattMsgEvent_t*) pMsg);
        break;

    case HCI_GAP_EVENT_EVENT:
    {
// Process HCI message
        switch (pMsg->status)
        {
        case HCI_COMMAND_COMPLETE_EVENT_CODE:
            break;

        case HCI_BLE_HARDWARE_ERROR_EVENT_CODE:
            AssertHandler(HAL_ASSERT_CAUSE_HARDWARE_ERROR, 0);
            break;

        default:
            break;
        }

        break;
    }

    case L2CAP_SIGNAL_EVENT:
// place holder for L2CAP Connection Parameter Reply
        break;

    default:
// Do nothing
        break;
    }

    return (safeToDealloc);
}

/*********************************************************************
 * @fn      multi_role_processGapMsg
 *
 * @brief   GAP message processing function.
 *
 * @param   pMsg - pointer to event message structure
 *
 * @return  none
 */
static void multi_role_processGapMsg(gapEventHdr_t *pMsg)
{
    switch (pMsg->opcode)
    {
    case GAP_DEVICE_INIT_DONE_EVENT:
    {
        gapDeviceInitDoneEvent_t *pPkt = (gapDeviceInitDoneEvent_t*) pMsg;

        BLE_LOG_INT_TIME(0, BLE_LOG_MODULE_APP, "APP : ---- got GAP_DEVICE_INIT_DONE_EVENT", 0);
        if (pPkt->hdr.status == SUCCESS)
        {
// Store the system ID
            uint8_t systemId[DEVINFO_SYSTEM_ID_LEN];

// use 6 bytes of device address for 8 bytes of system ID value
            systemId[0] = pPkt->devAddr[0];
            systemId[1] = pPkt->devAddr[1];
            systemId[2] = pPkt->devAddr[2];

// set middle bytes to zero
            systemId[4] = 0x00;
            systemId[3] = 0x00;

// shift three bytes up
            systemId[7] = pPkt->devAddr[5];
            systemId[6] = pPkt->devAddr[4];
            systemId[5] = pPkt->devAddr[3];

// Set Device Info Service Parameter
            DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN,
                                 systemId);
            DevInfo_SetParameter(DEVINFO_SOFTWARE_REV, DEVINFO_STR_ATTR_LEN,
                                 JUXTA_VERSION);
            // could utilize DEVINFO_SOFTWARE_REV here

            BLE_LOG_INT_INT(0, BLE_LOG_MODULE_APP, "APP : ---- start advert %d,%d\n", 0, 0);
//Setup and start advertising
            multi_role_advertInit();
        }
//Setup scanning
        multi_role_scanInit();
        mrMaxPduSize = pPkt->dataPktLen;
        break;
    }

    case GAP_CONNECTING_CANCELLED_EVENT:
    {
        break;
    }

    case GAP_LINK_ESTABLISHED_EVENT:
    {
        uint16_t connHandle = ((gapEstLinkReqEvent_t*) pMsg)->connectionHandle;
        uint8_t role = ((gapEstLinkReqEvent_t*) pMsg)->connRole;
        uint8_t *pAddr = ((gapEstLinkReqEvent_t*) pMsg)->devAddr;
        uint8_t connIndex;
//        uint8_t *pStrAddr;

// Add this connection info to the list
        connIndex = multi_role_addConnInfo(connHandle, pAddr, role);
// connIndex cannot be equal to or greater than MAX_NUM_BLE_CONNS
        MULTIROLE_ASSERT(connIndex < MAX_NUM_BLE_CONNS);
        connList[connIndex].charHandle = 0;

        // JUXTA
        multi_role_enqueueMsg(JUXTA_EVT_CONNECTED, NULL); // clocks here
        break;
    }

    case GAP_LINK_TERMINATED_EVENT:
    {
        uint16_t connHandle =
                ((gapTerminateLinkEvent_t*) pMsg)->connectionHandle;
        uint8_t connIndex;
//        uint8_t *pStrAddr;

        BLE_LOG_INT_STR(0, BLE_LOG_MODULE_APP, "APP : GAP msg: status=%d, opcode=%s\n", 0, "GAP_LINK_TERMINATED_EVENT");
// Mark this connection deleted in the connected device list.
        connIndex = multi_role_removeConnInfo(connHandle);

// connIndex cannot be equal to or greater than MAX_NUM_BLE_CONNS
        MULTIROLE_ASSERT(connIndex < MAX_NUM_BLE_CONNS);
//        pStrAddr = (uint8_t*) Util_convertBdAddr2Str(connList[connIndex].addr);
// If no active connections
        if (numConn == 0)
        {
            // keep advertising if MAX_NUM_BLE_CONNS > 1
            // isConnected = false; // if MAX_NUM_BLE_CONNS > 1
        }

        // JUXTA
        multi_role_enqueueMsg(JUXTA_EVT_DISCONNECTED, NULL); // clocks here
        break;
    }

    case GAP_UPDATE_LINK_PARAM_REQ_EVENT:
    {
        gapUpdateLinkParamReqReply_t rsp;
        gapUpdateLinkParamReqEvent_t *pReq =
                (gapUpdateLinkParamReqEvent_t*) pMsg;

        rsp.connectionHandle = pReq->req.connectionHandle;
        rsp.signalIdentifier = pReq->req.signalIdentifier;

// Only accept connection intervals with slave latency of 0
// This is just an example of how the application can send a response
        if (pReq->req.connLatency == 0)
        {
            rsp.intervalMin = pReq->req.intervalMin;
            rsp.intervalMax = pReq->req.intervalMax;
            rsp.connLatency = pReq->req.connLatency;
            rsp.connTimeout = pReq->req.connTimeout;
            rsp.accepted = TRUE;
        }
        else
        {
            rsp.accepted = FALSE;
        }

// Send Reply
        VOID GAP_UpdateLinkParamReqReply(&rsp);

        break;
    }

    case GAP_LINK_PARAM_UPDATE_EVENT:
    {
        gapLinkUpdateEvent_t *pPkt = (gapLinkUpdateEvent_t*) pMsg;

// Get the address from the connection handle
        linkDBInfo_t linkInfo;
        if (linkDB_GetInfo(pPkt->connectionHandle, &linkInfo) == SUCCESS)
        {
            if (pPkt->status == SUCCESS)
            {
            }
        }
// Check if there are any queued parameter updates
        mrConnHandleEntry_t *connHandleEntry = (mrConnHandleEntry_t*) List_get(
                &paramUpdateList);
        if (connHandleEntry != NULL)
        {
// Attempt to send queued update now
            multi_role_processParamUpdate(connHandleEntry->connHandle);
            ICall_free(connHandleEntry);
        }
        break;
    }

    default:
        break;
    }
}

static void multi_role_scanInit(void)
{
    uint8_t temp8;
    uint16_t temp16;

// Register callback to process Scanner events
    GapScan_registerCb(multi_role_scanCB, NULL);

// Set Scanner Event Mask
    GapScan_setEventMask(
            GAP_EVT_SCAN_ENABLED | GAP_EVT_SCAN_DISABLED | GAP_EVT_ADV_REPORT);

// Set Scan PHY parameters
    GapScan_setPhyParams(DEFAULT_SCAN_PHY, DEFAULT_SCAN_TYPE,
                         DEFAULT_SCAN_INTERVAL, DEFAULT_SCAN_WINDOW);

// Set Advertising report fields to keep
    temp16 = ADV_RPT_FIELDS;
    GapScan_setParam(SCAN_PARAM_RPT_FIELDS, &temp16);
// Set Scanning Primary PHY
    temp8 = DEFAULT_SCAN_PHY;
    GapScan_setParam(SCAN_PARAM_PRIM_PHYS, &temp8);
// Set LL Duplicate Filter
    temp8 = SCANNER_DUPLICATE_FILTER;
    GapScan_setParam(SCAN_PARAM_FLT_DUP, &temp8);

// Set PDU type filter -
// Only 'Connectable' and 'Complete' packets are desired.
// It doesn't matter if received packets are
// whether Scannable or Non-Scannable, whether Directed or Undirected,
// whether Scan_Rsp's or Advertisements, and whether Legacy or Extended.
    temp16 = SCAN_FLT_PDU_CONNECTABLE_ONLY | SCAN_FLT_PDU_COMPLETE_ONLY;
    GapScan_setParam(SCAN_PARAM_FLT_PDU_TYPE, &temp16);

// Set initiating PHY parameters
    GapInit_setPhyParam(DEFAULT_INIT_PHY, INIT_PHYPARAM_CONN_INT_MIN,
                        INIT_PHYPARAM_MIN_CONN_INT);
    GapInit_setPhyParam(DEFAULT_INIT_PHY, INIT_PHYPARAM_CONN_INT_MAX,
                        INIT_PHYPARAM_MAX_CONN_INT);
    scanInitDone = true;
}

static void multi_role_advertInit(void)
{
    BLE_LOG_INT_INT(0, BLE_LOG_MODULE_APP, "APP : ---- call GapAdv_create set=%d,%d\n", 1, 0);
// Create Advertisement sets and assign handle
    GapAdv_create(&multi_role_advCB, &advParams1, &advHandle);

// replace the address in advData (created by SysConfig)
    GapAdv_loadByHandle(advHandle, GAP_ADV_DATA_TYPE_ADV, sizeof(advData1),
                        advData1);

// Load scan response data for legacy, but not extended
    GapAdv_loadByHandle(advHandle, GAP_ADV_DATA_TYPE_SCAN_RSP,
                        sizeof(scanResData1), scanResData1);

// Set event mask for set #1
    GapAdv_setEventMask(
            advHandle,
            GAP_ADV_EVT_MASK_START_AFTER_ENABLE
                    | GAP_ADV_EVT_MASK_END_AFTER_DISABLE
                    | GAP_ADV_EVT_MASK_SET_TERMINATED);

    if (addrMode > ADDRMODE_RANDOM)
    {
        multi_role_updateRPA();
// Create one-shot clock for RPA check event.
        Util_constructClock(&clkRpaRead, multi_role_clockHandler,
        READ_RPA_PERIOD,
                            0, true, (UArg) &argRpaRead);
    }
    advertInitDone = true;
}

static void multi_role_advCB(uint32_t event, void *pBuf, uintptr_t arg)
{
    mrGapAdvEventData_t *pData = ICall_malloc(sizeof(mrGapAdvEventData_t));

    if (pData)
    {
        pData->event = event;
        pData->pBuf = pBuf;

        if (multi_role_enqueueMsg(MR_EVT_ADV, pData) != SUCCESS)
        {
            ICall_free(pData);
        }
    }
}

static uint8_t multi_role_processGATTMsg(gattMsgEvent_t *pMsg)
{
// Get connection index from handle
    uint8_t connIndex = multi_role_getConnIndex(pMsg->connHandle);
    MULTIROLE_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

    if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT)
    {
// ATT request-response or indication-confirmation flow control is
// violated. All subsequent ATT requests or indications will be dropped.
// The app is informed in case it wants to drop the connection.

// Display the opcode of the message that caused the violation.
//    Display_printf(dispHandle, MR_ROW_CUR_CONN, 0, "FC Violated: %d", pMsg->msg.flowCtrlEvt.opcode);
    }
    else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
    {
// MTU size updated
//    Display_printf(dispHandle, MR_ROW_CUR_CONN, 0, "MTU Size: %d", pMsg->msg.mtuEvt.MTU);
    }

// Messages from GATT server
    if (linkDB_Up(pMsg->connHandle))
    {
        if ((pMsg->method == ATT_READ_RSP)
                || ((pMsg->method == ATT_ERROR_RSP)
                        && (pMsg->msg.errorRsp.reqOpcode == ATT_READ_REQ)))
        {
            if (pMsg->method == ATT_ERROR_RSP)
            {
//        Display_printf(dispHandle, MR_ROW_CUR_CONN, 0, "Read Error %d", pMsg->msg.errorRsp.errCode);
            }
            else
            {
                // After a successful read, display the read value
//        Display_printf(dispHandle, MR_ROW_CUR_CONN, 0, "Read rsp: %d", pMsg->msg.readRsp.pValue[0]);
            }

        }
        else if ((pMsg->method == ATT_WRITE_RSP)
                || ((pMsg->method == ATT_ERROR_RSP)
                        && (pMsg->msg.errorRsp.reqOpcode == ATT_WRITE_REQ)))
        {

            if (pMsg->method == ATT_ERROR_RSP)
            {
//        Display_printf(dispHandle, MR_ROW_CUR_CONN, 0, "Write Error %d", pMsg->msg.errorRsp.errCode);
            }
            else
            {
                // After a succesful write, display the value that was written and
                // increment value
//        Display_printf(dispHandle, MR_ROW_CUR_CONN, 0, "Write sent: %d", charVal);
            }
        }
        else if (connList[connIndex].discState != BLE_DISC_STATE_IDLE)
        {
            multi_role_processGATTDiscEvent(pMsg);
        }
    } // Else - in case a GATT message came after a connection has dropped, ignore it.

// Free message payload. Needed only for ATT Protocol messages
    GATT_bm_free(&pMsg->msg, pMsg->method);

// It's safe to free the incoming message
    return (TRUE);
}

static void multi_role_processParamUpdate(uint16_t connHandle)
{
    gapUpdateLinkParamReq_t req;
    uint8_t connIndex;

    req.connectionHandle = connHandle;
#ifdef DEFAULT_SEND_PARAM_UPDATE_REQ
    req.connLatency = DEFAULT_DESIRED_SLAVE_LATENCY;
    req.connTimeout = DEFAULT_DESIRED_CONN_TIMEOUT;
    req.intervalMin = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    req.intervalMax = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
#endif

    connIndex = multi_role_getConnIndex(connHandle);
    MULTIROLE_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

// Deconstruct the clock object
    Clock_destruct(connList[connIndex].pUpdateClock);
// Free clock struct, only in case it is not NULL
    if (connList[connIndex].pUpdateClock != NULL)
    {
        ICall_free(connList[connIndex].pUpdateClock);
        connList[connIndex].pUpdateClock = NULL;
    }
// Free ParamUpdateEventData, only in case it is not NULL
    if (connList[connIndex].pParamUpdateEventData != NULL)
    {
        ICall_free(connList[connIndex].pParamUpdateEventData);
    }

// Send parameter update
    bStatus_t status = GAP_UpdateLinkParamReq(&req);

// If there is an ongoing update, queue this for when the udpate completes
    if (status == bleAlreadyInRequestedMode)
    {
        mrConnHandleEntry_t *connHandleEntry = ICall_malloc(
                sizeof(mrConnHandleEntry_t));
        if (connHandleEntry)
        {
            connHandleEntry->connHandle = connHandle;

            List_put(&paramUpdateList, (List_Elem*) connHandleEntry);
        }
    }
}

static void multi_role_processAppMsg(mrEvt_t *pMsg)
{
    bool safeToDealloc = TRUE;
    uint16_t timeRequest;

    if (pMsg->event <= APP_EVT_EVENT_MAX)
    {
        BLE_LOG_INT_STR(0, BLE_LOG_MODULE_APP, "APP : App msg status=%d, event=%s\n", 0, appEventStrings[pMsg->event]);
    }
    else
    {
        BLE_LOG_INT_INT(0, BLE_LOG_MODULE_APP, "APP : App msg status=%d, event=0x%x\n", 0, pMsg->event);
    }

    switch (pMsg->event)
    {
    case MR_EVT_CHAR_CHANGE:
    {
        multi_role_processCharValueChangeEvt(*(uint8_t*) (pMsg->pData));
        break;
    }

    case MR_EVT_ADV_REPORT:
    {
        GapScan_Evt_AdvRpt_t *pAdvRpt = (GapScan_Evt_AdvRpt_t*) (pMsg->pData);

// do UUID filtering
        if (multi_role_findSvcUuid(JUXTAPROFILE_SERV_UUID, pAdvRpt->pData,
                                   pAdvRpt->dataLen))
        {
            // pluck out appearance data
            juxtaGetAdvData(pAdvRpt->pData, pAdvRpt->dataLen,
                            (void*) &timeRequest,
                            GAP_ADTYPE_APPEARANCE);
            multi_role_addScanInfo(pAdvRpt->addr, pAdvRpt->addrType,
                                   pAdvRpt->rssi, timeRequest);
        }

// Free scan payload data
        if (pAdvRpt->pData != NULL)
        {
            ICall_free(pAdvRpt->pData);
        }
        break;
    }

    case MR_EVT_SCAN_ENABLED:
    {
// Discovering
        break;
    }

    case MR_EVT_SCAN_DISABLED:
    {
        juxtaGetAdvData(advData1, sizeof(advData1), (void*) &timeRequest,
        GAP_ADTYPE_APPEARANCE);
        // do not update their time if my time is not accurate
        if (isBase && timeRequest == IS_NOT_REQUESTING_TIME)
        {
            juxtaTryTimeUpdate(); // try right away
        }
        logScan();
        numScanRes = 0;
        break;
    }

//    case MR_EVT_SVC_DISC:
//    {
//        multi_role_startSvcDiscovery();
//        break;
//    }

    case MR_EVT_ADV:
    {
        multi_role_processAdvEvent((mrGapAdvEventData_t*) (pMsg->pData));
        break;
    }

    case MR_EVT_PAIRING_STATE:
    {
        multi_role_processPairState((mrPairStateData_t*) (pMsg->pData));
        break;
    }

    case MR_EVT_PASSCODE_NEEDED:
    {
        multi_role_processPasscode((mrPasscodeData_t*) (pMsg->pData));
        break;
    }

    case MR_EVT_SEND_PARAM_UPDATE:
    {
// Extract connection handle from data
        uint16_t locConnHandle =
                *(uint16_t*) (((mrClockEventData_t*) pMsg->pData)->data);
        multi_role_processParamUpdate(locConnHandle);
        safeToDealloc = FALSE;
        break;
    }

    case MR_EVT_READ_RPA:
    {
        multi_role_updateRPA();
        break;
    }

    case JUXTA_EVT_1HZ:
    {
        if (dumpResetFlag == 0) // busy
            break;
        setVoltage();
        setTemp();
        localTime = Seconds_get();
        simpleProfile_SetParameter(JUXTAPROFILE_LOCALTIME,
        JUXTAPROFILE_LOCALTIME_LEN,
                                   &localTime);
        break;
    }

    case JUXTA_EVT_SUBHZ:
    {
        if (dumpResetFlag == 0 || isConnected)
        {
            break;
        }
        setVoltage();
        localTime = Seconds_get();
        bool forceSave = false; // !! might be too often
        if (vbatt < VDROPOUT && juxtaMode != JUXTA_MODE_SHELF)
        {
            juxtaMode = JUXTA_MODE_SHELF;
            getJuxtaOptions(); // updates juxtaOptions characteristic
            forceSave = true;
            logMetaData(JUXTA_DATATYPE_MODE, juxtaMode, localTime);
            multi_role_enqueueMsg(JUXTA_EVT_DISCONNECTED, NULL); // simulate to engage juxtaMode
            break;
        }

        // shelf mode does not log any data or do expensive operations
        if (juxtaMode != JUXTA_MODE_SHELF || forceSave)
        {
            // assume that once base is set once it can go until battery dies without update
            if (localTime - lastTimeUpdate > REQUEST_TIME_EVERY && !isBase)
            {
                juxtaRequestTime(true); // logger device
            }

            setTemp();
            logMetaData(JUXTA_DATATYPE_TEMP, temperature_degC, localTime);
            logMetaData(JUXTA_DATATYPE_VBATT, vbatt, localTime);

            if (has_NAND)
            {
                uint32_t restoreLogAddr = logAddr;
                uint32_t restoreMetaAddr = metaAddr;
                ReturnType ret;
                uint8_t none = 0xFF;
                while (1)
                {
                    ret = NAND_Write(&logAddr, logBuffer, &none, 1);
                    if (ret == Flash_Success)
                        break;
                }
                logAddr = restoreLogAddr; // put addr back to where it was
                while (1)
                {
                    ret = NAND_Write(&metaAddr, metaBuffer, &none, 1);
                    if (ret == Flash_Success)
                        break;
                }
                metaAddr = restoreMetaAddr; // put addr back to where it was
            }
        }

        // see dumpData to fill memory
        break;
    }

    case JUXTA_EVT_INTERVAL_MODE: // entered by clock
    {
        // if time update is running, stop it
        if (isBase)
        {
            multi_role_enqueueMsg(JUXTA_EVT_CENTRAL_TIMEOUT, NULL); // cancel connection
            Util_stopClock(&clkJuxtaTimeUpdateTimeout);
            nScanRes = 0;
        }

        // add jitter, mitigates 2 nearby devices perfectly unaligned
        uint32_t randNum = rand() % 501;
        Util_restartClock(&clkJuxtaIntervalMode, juxtaScanInterval + randNum);
        doScan(JUXTA_SCAN_ONCE);
        break;
    }

    case JUXTA_EVT_LED_TIMEOUT:
    {
        shutdownLEDs();
        break;
    }

    case JUXTA_EVT_CONNECTED:
    {
        GPIO_write(LED1, 1);
        doScan(JUXTA_SCAN_DISABLE);
        doAdvertise(JUXTA_ADV_DISABLE);
        Util_stopClock(&clkJuxtaTimeUpdateTimeout); // connected within timeout
        Util_stopClock(&clkJuxtaIntervalMode);
        Util_stopClock(&clkJuxtaXLIntTimeout); // gets cleared at disconnect
        Util_stopClock(&clkJuxtaMGIntTimeout); // gets cleared at disconnect
        isConnected = true;
        dumpCount = 0; // should be reset, but make sure
        dumpResetFlag = 1;
        GPIO_disableInt(INT_XL_1);
        GPIO_disableInt(INT_XL_2);
        logMetaData(JUXTA_DATATYPE_CONN, 0, localTime);
        if (connList[0].role == GAP_PROFILE_CENTRAL)
        {
            mrConnHandle = connList[0].connHandle;
            multi_role_startSvcDiscovery();
            Util_startClock(&clkJuxtaCentralTimeout);
        }
        else
        {
            localTime = Seconds_get();
            simpleProfile_SetParameter(JUXTAPROFILE_LOCALTIME,
            JUXTAPROFILE_LOCALTIME_LEN,
                                       &localTime);
            Util_startClock(&clkJuxta1Hz);
        }
        break;
    }

    case JUXTA_EVT_DISCONNECTED:
    {
        GPIO_write(LED1, 0);
        isConnected = false;
        Util_stopClock(&clkJuxta1Hz);
        Util_stopClock(&clkJuxtaCentralTimeout);
        clearIntXL1();
        GPIO_enableInt(INT_XL_1); // movement on, req for shake to wake
        GapAdv_setParam(advHandle, GAP_ADV_PARAM_PRIMARY_INTERVAL_MIN,
                        &juxtaAdvInterval);
        GapAdv_setParam(advHandle, GAP_ADV_PARAM_PRIMARY_INTERVAL_MIN,
                        &juxtaAdvInterval);
        doAdvertise(JUXTA_ADV_FOREVER);
//        clearIntXL2();
//        GPIO_enableInt(INT_XL_2); // orientation
        multi_role_enqueueMsg(JUXTA_EVT_TIME_UPDATE, NULL);
        if (juxtaMode == JUXTA_MODE_INTERVAL)
        {
            Util_restartClock(&clkJuxtaIntervalMode, juxtaScanInterval);
        }
        break;
    }

    case JUXTA_EVT_INT_XL1:
    {
        lastXLIntTime = Seconds_get();
        // protect intInterval
        if (!Util_isActive(&clkJuxtaXLIntTimeout))
        {
            Util_restartClock(&clkJuxtaXLIntTimeout, JUXTA_INT_TIMEOUT); // controls logging
        }
        break;
    }

    case JUXTA_EVT_INT_XL1_TIMEOUT:
    {
        logMetaData(JUXTA_DATATYPE_XL, 0, lastXLIntTime);
        clearIntXL1();
        break;
    }

    case JUXTA_EVT_INT_XL2:
    {
        break;
    }

    case JUXTA_EVT_INT_MG:
    {
        timeoutLED(LED1);
        lastMGIntTime = Seconds_get();
        Util_restartClock(&clkJuxtaMGIntTimeout, JUXTA_INT_TIMEOUT); // controls logging
        if (!isConnected)
        {
            doAdvertise(JUXTA_ADV_ONCE); // clearIntMG() here
        }
        break;
    }

    case JUXTA_EVT_INT_MG_TIMEOUT:
    {
        break;
    }

    case JUXTA_EVT_CENTRAL_TIMEOUT: // once connected
    {
        // this event is also called when gatt write is successful (not always timeout)
        Util_stopClock(&clkJuxtaCentralTimeout); // clock may be running
        multi_role_doDisconnect(); // GAP_TerminateLinkReq(mrConnHandle,...)
        break;
    }

    case JUXTA_EVT_WRITE_GATT:
    {
        multi_role_doGattWrite();
        break;
    }

    case JUXTA_EVT_TIME_UPDATE: // when trying to connect
    {
        GapInit_cancelConnect();
        juxtaTryTimeUpdate();
        break;
    }

    case MR_EVT_INSUFFICIENT_MEM:
    {
// try stopping
        doScan(JUXTA_SCAN_DISABLE);
        doAdvertise(JUXTA_ADV_DISABLE);
        break;
    }

    default:
// Do nothing.
        break;
    }

    if ((safeToDealloc == TRUE) && (pMsg->pData != NULL))
    {
        ICall_free(pMsg->pData);
    }
}

static void multi_role_processAdvEvent(mrGapAdvEventData_t *pEventData)
{
    switch (pEventData->event)
    {
// *** ADVERTISE EVENTS ***
// Sent on the first advertisement after a @ref GapAdv_enable
    case GAP_EVT_ADV_START_AFTER_ENABLE:
        BLE_LOG_INT_TIME(0, BLE_LOG_MODULE_APP, "APP : ---- GAP_EVT_ADV_START_AFTER_ENABLE", 0);
        break;

        // Sent after advertising stops due to a @ref GapAdv_disable
    case GAP_EVT_ADV_END_AFTER_DISABLE:
        break;

        // Sent at the beginning of each advertisement (for legacy advertising) or at
        // the beginning of each each advertisement set (for extended advertising)
    case GAP_EVT_ADV_START:
        break;

        // Sent after each advertisement (for legacy advertising) or at the end of
        // each each advertisement set (for extended advertising)
    case GAP_EVT_ADV_END:
        break;

        // Sent when an advertisement set is terminated due to a connection/duration ended
    case GAP_EVT_ADV_SET_TERMINATED:
        break;

    case GAP_EVT_INSUFFICIENT_MEMORY:
        break;

    default:
        break;
    }

// All events have associated memory to free except the insufficient memory
// event
    if (pEventData->event != GAP_EVT_INSUFFICIENT_MEMORY)
    {
        ICall_free(pEventData->pBuf);
    }
}

#if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)

static bool multi_role_findSvcUuid(uint16_t uuid, uint8_t *pData,
                                   uint16_t dataLen)
{
    uint8_t adLen;
    uint8_t adType;
    uint8_t *pEnd;

    if (dataLen > 0)
    {
        pEnd = pData + dataLen - 1;

// While end of data not reached
        while (pData < pEnd)
        {
// Get length of next AD item
            adLen = *pData++;
            if (adLen > 0)
            {
                adType = *pData;

                // If AD type is for 16-bit service UUID
                if ((adType == GAP_ADTYPE_16BIT_MORE)
                        || (adType == GAP_ADTYPE_16BIT_COMPLETE))
                {
                    pData++;
                    adLen--;

                    // For each UUID in list
                    while (adLen >= 2 && pData < pEnd)
                    {
                        // Check for match
                        if ((pData[0] == LO_UINT16(uuid))
                                && (pData[1] == HI_UINT16(uuid)))
                        {
                            // Match found
                            return TRUE;
                        }

                        // Go to next
                        pData += 2;
                        adLen -= 2;
                    }

                    // Handle possible erroneous extra byte in UUID list
                    if (adLen == 1)
                    {
                        pData++;
                    }
                }
                else
                {
                    // Go to next item
                    pData += adLen;
                }
            }
        }
    }

// Match not found
    return FALSE;
}

static void multi_role_addScanInfo(uint8_t *pAddr, uint8_t addrType,
                                   int8_t rssi, uint16_t requestTime)
{
    uint8_t i;

    if (numScanRes < DEFAULT_MAX_SCAN_RES)
    {
// Check if device is already in scan results
        for (i = 0; i < numScanRes; i++)
        {
            if (memcmp(pAddr, scanList[i].addr, B_ADDR_LEN) == 0)
            {
                scanList[i].requestTime = requestTime; // update
                return; // values are equal, do not increment
            }
        }

        memcpy(scanList[numScanRes].addr, pAddr, B_ADDR_LEN);
        scanList[numScanRes].addrType = addrType;
        scanList[numScanRes].rssi = rssi;
        scanList[numScanRes].requestTime = requestTime;
        numScanRes++;
    }
}
#endif // DEFAULT_DEV_DISC_BY_SVC_UUID

void multi_role_scanCB(uint32_t evt, void *pMsg, uintptr_t arg)
{
    uint8_t event;

    if (evt & GAP_EVT_ADV_REPORT)
    {
        event = MR_EVT_ADV_REPORT;
    }
    else if (evt & GAP_EVT_SCAN_ENABLED)
    {
        event = MR_EVT_SCAN_ENABLED;
    }
    else if (evt & GAP_EVT_SCAN_DISABLED)
    {
        event = MR_EVT_SCAN_DISABLED;
    }
    else if (evt & GAP_EVT_INSUFFICIENT_MEMORY)
    {
        event = MR_EVT_INSUFFICIENT_MEM;
    }
    else
    {
        return;
    }

    if (multi_role_enqueueMsg(event, pMsg) != SUCCESS)
    {
        ICall_free(pMsg);
    }

}

static void multi_role_charValueChangeCB(uint8_t paramID)
{
    uint8_t *pData;

// Allocate space for the event data.
    if ((pData = ICall_malloc(sizeof(uint8_t))))
    {
        *pData = paramID;

// Queue the event.
        if (multi_role_enqueueMsg(MR_EVT_CHAR_CHANGE, pData) != SUCCESS)
        {
            ICall_free(pData);
        }
    }
}

static status_t multi_role_enqueueMsg(uint8_t event, void *pData)
{
    uint8_t success;
    mrEvt_t *pMsg = ICall_malloc(sizeof(mrEvt_t));

// Create dynamic pointer to message.
    if (pMsg)
    {
        pMsg->event = event;
        pMsg->pData = pData;

// Enqueue the message.
        success = Util_enqueueMsg(appMsgQueue, syncEvent, (uint8_t*) pMsg);
        return (success) ? SUCCESS : FAILURE;
    }

    return (bleMemAllocError);
}

static void multi_role_processCharValueChangeEvt(uint8_t paramId)
{
// some chars do not call simpleProfile_GetParameter, so set retProfile invalid here
    uint8_t len = 0;
    bStatus_t retProfile = INVALIDPARAMETER;
    ReturnType ret;

    switch (paramId)
    {
    case JUXTAPROFILE_LOGCOUNT:
        len = JUXTAPROFILE_LOGCOUNT_LEN;
        break;
    case JUXTAPROFILE_METACOUNT:
        len = JUXTAPROFILE_METACOUNT_LEN;
        break;
    case JUXTAPROFILE_LOCALTIME:
        len = JUXTAPROFILE_LOCALTIME_LEN;
        break;
    case JUXTAPROFILE_ADVMODE:
        len = JUXTAPROFILE_DATA_LEN;
        break;
    case JUXTAPROFILE_COMMAND:
        len = JUXTAPROFILE_COMMAND_LEN;
        break;
    case JUXTAPROFILE_SUBJECT:
        len = JUXTAPROFILE_SUBJECT_LEN;
        break;
    case JUXTAPROFILE_VBATT: // no writes
    case JUXTAPROFILE_TEMP: // no writes
    case JUXTAPROFILE_DATA: // no writes
    default:
        break;
    }

    uint8_t *pValue = ICall_malloc(len); // dynamic allocation

    switch (paramId)
    {
// only characteristics with GATT_PROP_WRITE, all others are written elsewhere
    case JUXTAPROFILE_LOGCOUNT: // LOG COUNT
        retProfile = simpleProfile_GetParameter(JUXTAPROFILE_LOGCOUNT, pValue);
// treat any write as erasing memory
        logCount = 0;
        logRecoveryCount = 0;
        logAddr = JUXTA_BASE_LOGS;
        logRecoveryAddr = JUXTA_BASE_LOGS;
        logMetaData(JUXTA_DATATYPE_RESET_LOGS, 0, localTime);
        saveConfigs();
        break;
    case JUXTAPROFILE_METACOUNT: // META COUNT
        retProfile = simpleProfile_GetParameter(JUXTAPROFILE_METACOUNT, pValue);
// treat any write as erasing memory
        metaCount = 0;
        metaRecoveryCount = 0;
        metaAddr = JUXTA_BASE_META;
        metaRecoveryAddr = JUXTA_BASE_META;
        logMetaData(JUXTA_DATATYPE_RESET_META, 0, localTime);
        logMetaData(JUXTA_DATATYPE_MODE, juxtaMode, localTime);
        logMetaData(JUXTA_DATATYPE_IS_BASE, isBase, localTime);
        saveConfigs();
        break;
    case JUXTAPROFILE_LOCALTIME: // LOCAL TIME
        retProfile = simpleProfile_GetParameter(JUXTAPROFILE_LOCALTIME, pValue);
        memcpy(&localTime, pValue, sizeof(uint32_t));
        localTime++; // accounts for transmission delay
        Seconds_set(localTime);
        lastTimeUpdate = localTime;
        logMetaData(JUXTA_DATATYPE_TIME_SYNC, 0, localTime);
        juxtaRequestTime(false); // SUBHZ will turn it back on
        break;
    case JUXTAPROFILE_ADVMODE: // ADVERTISE MODE
        retProfile = simpleProfile_GetParameter(JUXTAPROFILE_ADVMODE, pValue);
        setJuxtaOptions(pValue[0]);
        saveConfigs();
        break;
    case JUXTAPROFILE_COMMAND:
        retProfile = simpleProfile_GetParameter(JUXTAPROFILE_COMMAND, pValue);
        if (pValue[0] == LOGS_DUMP_KEY)
        {
            if (dumpResetFlag)
            {
                dumpResetFlag = 0;
                dumpAddr = JUXTA_BASE_LOGS; // reset
            }
            ret = dumpData(&logAddr, logBuffer,
            JUXTA_BASE_LOGS);
            setLogRecovery(logAddr, logCount, ret); // if page was written, save
        }
        else if (pValue[0] == META_DUMP_KEY)
        {
            if (dumpResetFlag)
            {
                dumpResetFlag = 0;
                dumpAddr = JUXTA_BASE_META; // reset
            }
            ret = dumpData(&metaAddr, metaBuffer,
            JUXTA_BASE_META);
            setMetaRecovery(metaAddr, metaCount, ret); // if page was written, save
        }
        else if (pValue[0] == DUMP_RESET_KEY)
        {
            // dump from beginning
            dumpCount = 0;
            dumpResetFlag = 1;
        }
        break;
    case JUXTAPROFILE_SUBJECT: // LOCAL TIME
        retProfile = simpleProfile_GetParameter(JUXTAPROFILE_SUBJECT, pValue);
        saveConfigs();
        break;
    case JUXTAPROFILE_VBATT: // no writes
    case JUXTAPROFILE_TEMP: // no writes
    case JUXTAPROFILE_DATA: // no writes
    default:
        break;
    }
    if (pValue) // len=0 should make pValue null; was: retProfile
    {
        ICall_free(pValue);
    }
}

static void multi_role_updateRPA(void)
{
    uint8_t *pRpaNew;

// Read the current RPA.
    pRpaNew = GAP_GetDevAddress(FALSE);

    if (memcmp(pRpaNew, rpa, B_ADDR_LEN))
    {
        memcpy(rpa, pRpaNew, B_ADDR_LEN);
    }
}

static void multi_role_clockHandler(UArg arg)
{
    mrClockEventData_t *pData = (mrClockEventData_t*) arg;

    if (pData->event == MR_EVT_READ_RPA)
    {
// Start the next period
        Util_startClock(&clkRpaRead);

// Send message to read the current RPA
        multi_role_enqueueMsg(MR_EVT_READ_RPA, NULL);
    }
    else if (pData->event == MR_EVT_SEND_PARAM_UPDATE)
    {
// Send message to app
        multi_role_enqueueMsg(MR_EVT_SEND_PARAM_UPDATE, pData);
    }
    else if (pData->event == JUXTA_EVT_1HZ)
    {
        multi_role_enqueueMsg(JUXTA_EVT_1HZ, NULL);
    }
    else if (pData->event == JUXTA_EVT_SUBHZ)
    {
        multi_role_enqueueMsg(JUXTA_EVT_SUBHZ, NULL);
    }
    else if (pData->event == JUXTA_EVT_INTERVAL_MODE)
    {
        multi_role_enqueueMsg(JUXTA_EVT_INTERVAL_MODE, NULL);
    }
    else if (pData->event == JUXTA_EVT_LED_TIMEOUT)
    {
        multi_role_enqueueMsg(JUXTA_EVT_LED_TIMEOUT, NULL);
    }
    else if (pData->event == JUXTA_EVT_STARTUP)
    {
        if (scanInitDone && advertInitDone)
        {
            Util_stopClock(&clkJuxtaStartup);
            multi_role_enqueueMsg(JUXTA_EVT_DISCONNECTED, NULL); // simulate to engage juxtaMode
        }
    }
    else if (pData->event == JUXTA_EVT_INT_XL1_TIMEOUT)
    {
        multi_role_enqueueMsg(JUXTA_EVT_INT_XL1_TIMEOUT, NULL);
    }
    else if (pData->event == JUXTA_EVT_INT_MG_TIMEOUT)
    {
        multi_role_enqueueMsg(JUXTA_EVT_INT_MG_TIMEOUT, NULL);
    }
    else if (pData->event == JUXTA_EVT_CENTRAL_TIMEOUT)
    {
        multi_role_enqueueMsg(JUXTA_EVT_CENTRAL_TIMEOUT, NULL);

    }
    else if (pData->event == JUXTA_EVT_TIME_UPDATE)
    {
        multi_role_enqueueMsg(JUXTA_EVT_TIME_UPDATE, NULL);
    }
}

static void multi_role_processGATTDiscEvent(gattMsgEvent_t *pMsg)
{
    uint8_t connIndex = multi_role_getConnIndex(pMsg->connHandle);
    MULTIROLE_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

    if (connList[connIndex].discState == BLE_DISC_STATE_MTU)
    {
// MTU size response received, discover simple service
        if (pMsg->method == ATT_EXCHANGE_MTU_RSP)
        {
            uint8_t uuid[ATT_BT_UUID_SIZE] =
                    { LO_UINT16(JUXTAPROFILE_SERV_UUID), HI_UINT16(
                            JUXTAPROFILE_SERV_UUID) };

            connList[connIndex].discState = BLE_DISC_STATE_SVC;

// Discovery simple service
            VOID GATT_DiscPrimaryServiceByUUID(pMsg->connHandle, uuid,
                                               ATT_BT_UUID_SIZE, selfEntity);
        }
    }
    else if (connList[connIndex].discState == BLE_DISC_STATE_SVC)
    {
// Service found, store handles
        if (pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP
                && pMsg->msg.findByTypeValueRsp.numInfo > 0)
        {
            svcStartHdl = ATT_ATTR_HANDLE(
                    pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
            svcEndHdl = ATT_GRP_END_HANDLE(
                    pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
        }

// If procedure complete
        if (((pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP)
                && (pMsg->hdr.status == bleProcedureComplete))
                || (pMsg->method == ATT_ERROR_RSP))
        {
            if (svcStartHdl != 0)
            {
                attReadByTypeReq_t req;

                // Discover LOCALTIME characteristic
                connList[connIndex].discState = BLE_DISC_STATE_CHAR;

                req.startHandle = svcStartHdl;
                req.endHandle = svcEndHdl;
                req.type.len = ATT_BT_UUID_SIZE;
                req.type.uuid[0] = LO_UINT16(JUXTAPROFILE_LOCALTIME_UUID);
                req.type.uuid[1] = HI_UINT16(JUXTAPROFILE_LOCALTIME_UUID);

                VOID GATT_DiscCharsByUUID(pMsg->connHandle, &req, selfEntity);
            }
        }
    }
    else if (connList[connIndex].discState == BLE_DISC_STATE_CHAR)
    {
// Characteristic found, store handle
        if ((pMsg->method == ATT_READ_BY_TYPE_RSP)
                && (pMsg->msg.readByTypeRsp.numPairs > 0))
        {
            uint8_t connIndex = multi_role_getConnIndex(mrConnHandle);

// connIndex cannot be equal to or greater than MAX_NUM_BLE_CONNS
            MULTIROLE_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

// Store the handle of the LOCALTIME characteristic
            connList[connIndex].charHandle = BUILD_UINT16(
                    pMsg->msg.readByTypeRsp.pDataList[3],
                    pMsg->msg.readByTypeRsp.pDataList[4]);

//      Display_printf(dispHandle, MR_ROW_CUR_CONN, 0, "Simple Svc Found");
// Now we can use GATT Read/Write
        }

        connList[connIndex].discState = BLE_DISC_STATE_IDLE;
        multi_role_enqueueMsg(JUXTA_EVT_WRITE_GATT, NULL);
    }
}

static uint16_t multi_role_getConnIndex(uint16_t connHandle)
{
    uint8_t i;
// Loop through connection
    for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
    {
// If matching connection handle found
        if (connList[i].connHandle == connHandle)
        {
            return i;
        }
    }

// Not found if we got here
    return (MAX_NUM_BLE_CONNS);
}

#ifndef Display_DISABLE_ALL
static char* multi_role_getConnAddrStr(uint16_t connHandle)
{
  uint8_t i;

  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if (connList[i].connHandle == connHandle)
    {
      return Util_convertBdAddr2Str(connList[i].addr);
    }
  }

  return NULL;
}
#endif

static uint8_t multi_role_clearConnListEntry(uint16_t connHandle)
{
    uint8_t i;
// Set to invalid connection index initially
    uint8_t connIndex = MAX_NUM_BLE_CONNS;

    if (connHandle != LINKDB_CONNHANDLE_ALL)
    {
        connIndex = multi_role_getConnIndex(connHandle);
// Get connection index from handle
        if (connIndex >= MAX_NUM_BLE_CONNS)
        {
            return bleInvalidRange;
        }
    }

// Clear specific handle or all handles
    for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
    {
        if ((connIndex == i) || (connHandle == LINKDB_CONNHANDLE_ALL))
        {
            connList[i].connHandle = LINKDB_CONNHANDLE_INVALID;
            connList[i].charHandle = 0;
            connList[i].discState = 0;
        }
    }

    return SUCCESS;
}

static void multi_role_pairStateCB(uint16_t connHandle, uint8_t state,
                                   uint8_t status)
{
    mrPairStateData_t *pData = ICall_malloc(sizeof(mrPairStateData_t));

// Allocate space for the event data.
    if (pData)
    {
        pData->state = state;
        pData->connHandle = connHandle;
        pData->status = status;

// Queue the event.
        if (multi_role_enqueueMsg(MR_EVT_PAIRING_STATE, pData) != SUCCESS)
        {
            ICall_free(pData);
        }
    }
}

static void multi_role_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                  uint8_t uiInputs, uint8_t uiOutputs,
                                  uint32_t numComparison)
{
    mrPasscodeData_t *pData = ICall_malloc(sizeof(mrPasscodeData_t));

// Allocate space for the passcode event.
    if (pData)
    {
        pData->connHandle = connHandle;
        memcpy(pData->deviceAddr, deviceAddr, B_ADDR_LEN);
        pData->uiInputs = uiInputs;
        pData->uiOutputs = uiOutputs;
        pData->numComparison = numComparison;

// Enqueue the event.
        if (multi_role_enqueueMsg(MR_EVT_PASSCODE_NEEDED, pData) != SUCCESS)
        {
            ICall_free(pData);
        }
    }
}

static void multi_role_processPairState(mrPairStateData_t *pPairData)
{
    uint8_t state = pPairData->state;
    uint8_t status = pPairData->status;

    switch (state)
    {
    case GAPBOND_PAIRING_STATE_STARTED:
//      Display_printf(dispHandle, MR_ROW_SECURITY, 0, "Pairing started");
        break;

    case GAPBOND_PAIRING_STATE_COMPLETE:
        if (status == SUCCESS)
        {
            linkDBInfo_t linkInfo;

//        Display_printf(dispHandle, MR_ROW_SECURITY, 0, "Pairing success");

            if (linkDB_GetInfo(pPairData->connHandle, &linkInfo) == SUCCESS)
            {
                // If the peer was using private address, update with ID address
                if ((linkInfo.addrType == ADDRTYPE_PUBLIC_ID
                        || linkInfo.addrType == ADDRTYPE_RANDOM_ID)
                        && !Util_isBufSet(linkInfo.addrPriv, 0, B_ADDR_LEN))

                {
                    // Update the address of the peer to the ID address
//            Display_printf(dispHandle, MR_ROW_NON_CONN, 0, "Addr updated: %s",
//                           Util_convertBdAddr2Str(linkInfo.addr));

                    // Update the connection list with the ID address
                    uint8_t i = multi_role_getConnIndex(pPairData->connHandle);

                    MULTIROLE_ASSERT(i < MAX_NUM_BLE_CONNS);
                    memcpy(connList[i].addr, linkInfo.addr, B_ADDR_LEN);
                }
            }
        }
        else
        {
//        Display_printf(dispHandle, MR_ROW_SECURITY, 0, "Pairing fail: %d", status);
        }
        break;

    case GAPBOND_PAIRING_STATE_ENCRYPTED:
        if (status == SUCCESS)
        {
//        Display_printf(dispHandle, MR_ROW_SECURITY, 0, "Encryption success");
        }
        else
        {
//        Display_printf(dispHandle, MR_ROW_SECURITY, 0, "Encryption failed: %d", status);
        }
        break;

    case GAPBOND_PAIRING_STATE_BOND_SAVED:
        if (status == SUCCESS)
        {
//        Display_printf(dispHandle, MR_ROW_SECURITY, 0, "Bond save success");
        }
        else
        {
//        Display_printf(dispHandle, MR_ROW_SECURITY, 0, "Bond save failed: %d", status);
        }

        break;

    default:
        break;
    }
}

static void multi_role_processPasscode(mrPasscodeData_t *pData)
{
// Display passcode to user
    if (pData->uiOutputs != 0)
    {
//    Display_printf(dispHandle, MR_ROW_SECURITY, 0, "Passcode: %d",
//                   B_APP_DEFAULT_PASSCODE);
    }

// Send passcode response
    GAPBondMgr_PasscodeRsp(pData->connHandle, SUCCESS, B_APP_DEFAULT_PASSCODE);
}

static void multi_role_startSvcDiscovery(void)
{
    uint8_t connIndex = multi_role_getConnIndex(mrConnHandle);

// connIndex cannot be equal to or greater than MAX_NUM_BLE_CONNS
    MULTIROLE_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

    attExchangeMTUReq_t req;

// Initialize cached handles
    svcStartHdl = svcEndHdl = 0;

    connList[connIndex].discState = BLE_DISC_STATE_MTU;

// Discover GATT Server's Rx MTU size
    req.clientRxMTU = mrMaxPduSize - L2CAP_HDR_SIZE;

// ATT MTU size should be set to the minimum of the Client Rx MTU
// and Server Rx MTU values
    VOID GATT_ExchangeMTU(mrConnHandle, &req, selfEntity);
}

static uint8_t multi_role_addConnInfo(uint16_t connHandle, uint8_t *pAddr,
                                      uint8_t role)
{
    uint8_t i;

    for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
    {
        if (connList[i].connHandle == LINKDB_CONNHANDLE_INVALID)
        {
// Found available entry to put a new connection info in
            connList[i].connHandle = connHandle;
            memcpy(connList[i].addr, pAddr, B_ADDR_LEN);
            connList[i].role = role;
            numConn++;

#ifdef DEFAULT_SEND_PARAM_UPDATE_REQ
            // If a peripheral, start the clock to send a connection parameter update
            if (role == GAP_PROFILE_PERIPHERAL)
            {
                // Allocate data to send through clock handler
                connList[i].pParamUpdateEventData = ICall_malloc(
                        sizeof(mrClockEventData_t) + sizeof(uint16_t));
                if (connList[i].pParamUpdateEventData)
                {
                    // Set clock data
                    connList[i].pParamUpdateEventData->event =
                    MR_EVT_SEND_PARAM_UPDATE;
                    *((uint16_t*) connList[i].pParamUpdateEventData->data) =
                            connHandle;

                    // Create a clock object and start
                    connList[i].pUpdateClock = (Clock_Struct*) ICall_malloc(
                            sizeof(Clock_Struct));

                    if (connList[i].pUpdateClock)
                    {
                        Util_constructClock(
                                connList[i].pUpdateClock,
                                multi_role_clockHandler,
                                SEND_PARAM_UPDATE_DELAY,
                                0, true,
                                (UArg) connList[i].pParamUpdateEventData);
                    }
                    else
                    {
                        // Clean up
                        ICall_free(connList[i].pParamUpdateEventData);
                    }
                }
                else
                {
                    // Memory allocation failed
                    MULTIROLE_ASSERT(false);
                }
            }
#endif

            break;
        }
    }

    return i;
}

void multi_role_clearPendingParamUpdate(uint16_t connHandle)
{
    List_Elem *curr;

    for (curr = List_head(&paramUpdateList); curr != NULL;
            curr = List_next(curr))
    {
        if (((mrConnHandleEntry_t*) curr)->connHandle == connHandle)
        {
            List_remove(&paramUpdateList, curr);
        }
    }
}

static uint8_t multi_role_removeConnInfo(uint16_t connHandle)
{
    uint8_t connIndex = multi_role_getConnIndex(connHandle);

    if (connIndex < MAX_NUM_BLE_CONNS)
    {
        Clock_Struct *pUpdateClock = connList[connIndex].pUpdateClock;

        if (pUpdateClock != NULL)
        {
// Stop and destruct the RTOS clock if it's still alive
            if (Util_isActive(pUpdateClock))
            {
                Util_stopClock(pUpdateClock);
            }

// Destruct the clock object
            Clock_destruct(pUpdateClock);
// Free clock struct
            ICall_free(pUpdateClock);
// Free ParamUpdateEventData
            ICall_free(connList[connIndex].pParamUpdateEventData);
        }
// Clear pending update requests from paramUpdateList
        multi_role_clearPendingParamUpdate(connHandle);
// Clear Connection List Entry
        multi_role_clearConnListEntry(connHandle);
        numConn--;
    }

    return connIndex;
}

bool multi_role_doConnect(uint8_t index)
{
// Temporarily disable advertising
    doAdvertise(JUXTA_ADV_DISABLE);
    GapInit_connect(scanList[index].addrType & MASK_ADDRTYPE_ID,
                    scanList[index].addr, mrInitPhy, 0); // cancel with clkJuxtaTimeUpdateTimeout
// Re-enable advertising
    doAdvertise(JUXTA_ADV_FOREVER);
//  Display_printf(dispHandle, MR_ROW_NON_CONN, 0, "Connecting...");
    return (true);
}

bool multi_role_doGattRead(void)
{
    attReadReq_t req;
    uint8_t connIndex = multi_role_getConnIndex(mrConnHandle);

// connIndex cannot be equal to or greater than MAX_NUM_BLE_CONNS
    MULTIROLE_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

    req.handle = connList[connIndex].charHandle;
    GATT_ReadCharValue(mrConnHandle, &req, selfEntity);

    return (true);
}

bool multi_role_doGattWrite(void)
{
    status_t status;
    uint32_t localTime = Seconds_get();
    attWriteReq_t req;

    req.pValue = GATT_bm_alloc(mrConnHandle, ATT_WRITE_REQ, sizeof(localTime),
    NULL);

    if (req.pValue != NULL)
    {
        uint8_t connIndex = multi_role_getConnIndex(mrConnHandle);

        // connIndex cannot be equal to or greater than MAX_NUM_BLE_CONNS
        MULTIROLE_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

        req.handle = connList[connIndex].charHandle;
        req.len = sizeof(localTime);
        memcpy(req.pValue, &localTime, sizeof(localTime));
        req.sig = 0;
        req.cmd = 0;

        status = GATT_WriteCharValue(mrConnHandle, &req, selfEntity);
        if (status != SUCCESS)
        {
            GATT_bm_free((gattMsg_t*) &req, ATT_WRITE_REQ);
            multi_role_enqueueMsg(JUXTA_EVT_WRITE_GATT, NULL); // try again
        }
        else
        {
            multi_role_enqueueMsg(JUXTA_EVT_CENTRAL_TIMEOUT, NULL);
        }
    }

    return (true);
}

bool multi_role_doDisconnect(void)
{
// Disconnect
    GAP_TerminateLinkReq(mrConnHandle, HCI_DISCONNECT_REMOTE_USER_TERM);
    return (true);
}
