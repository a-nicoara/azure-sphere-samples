/* Copyright (c) Microsoft Corporation. All rights reserved.
   Licensed under the MIT License. */

// This sample C application for Azure Sphere demonstrates Azure IoT SDK C APIs
// The application uses the Azure IoT SDK C APIs to
// 1. Use IoT Hub/Device Twin to control an LED.

// You will need to provide four pieces of information to use this application, all of which are set
// in the app_manifest.json.
// 1. The Scope Id for your IoT Central application (set in 'CmdArgs')
// 2. The Tenant Id obtained from 'azsphere tenant show-selected' (set in 'DeviceAuthentication')
// 3. The Azure DPS Global endpoint address 'global.azure-devices-provisioning.net'
//    (set in 'AllowedConnections')
// 4. The IoT Hub Endpoint address for your IoT Central application (set in 'AllowedConnections')

#include <errno.h>
#include <math.h>
#include <signal.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <time.h>

// applibs_versions.h defines the API struct versions to use for applibs APIs.
#include "applibs_versions.h"
#include <applibs/log.h>
#include <applibs/networking.h>
#include <applibs/gpio.h>
#include <applibs/i2c.h>
#include <applibs/eventloop.h>

// By default, this sample's CMake build targets hardware that follows the MT3620
// Reference Development Board (RDB) specification, such as the MT3620 Dev Kit from
// Seeed Studios.
//
// To target different hardware, you'll need to update the CMake build. The necessary
// steps to do this vary depending on if you are building in Visual Studio, in Visual
// Studio Code or via the command line.
//
// See https://github.com/Azure/azure-sphere-samples/tree/master/Hardware for more details.
//
// This #include imports the sample_hardware abstraction from that hardware definition.
#include <hw/sample_hardware.h>

#include "eventloop_timer_utilities.h"

// Azure IoT SDK
#include <iothub_client_core_common.h>
#include <iothub_device_client_ll.h>
#include <iothub_client_options.h>
#include <iothubtransportmqtt.h>
#include <iothub.h>
#include <azure_sphere_provisioning.h>

#include "parson.h" // used to parse Device Twin messages.

/// Exit codes for this application. These are used for the
/// application exit code.  They they must all be between zero and 255,
/// where zero is reserved for successful termination.
typedef enum {
    ExitCode_Success = 0,
    ExitCode_TermHandler_SigTerm = 1,
    ExitCode_Main_EventLoopFail = 2,
    ExitCode_AzureTimer_Consume = 4,
    ExitCode_Init_EventLoop = 5,
    ExitCode_Init_TwinStatusLed = 8,
    ExitCode_Init_AzureTimer = 10,
    //====
    ExitCode_PowerUpFailed = 14,
    ExitCode_ReadWhoAmI_IDRead = 15,
    ExitCode_ReadWhoAmI_InvalidID = 16,
    ExitCode_Init_OpenMaster = 18,
    ExitCode_Init_SetBusSpeed = 19,
    ExitCode_Init_SetTimeout = 20,
    ExitCode_Init_SetDefaultTarget = 2,
} ExitCode;

/// I2C Status codes
typedef enum {
    I2C_SUCCESS = 0,
    I2C_TRANSFER_LENGTH_MISMATCH = 1,
} I2CStatus;
/// TSL2561 Sensor: command register flags 
typedef enum { 
    SELECT_CMD_REG = 0x80,
    CLEAR_INTERRUPT = 0x40,
    WORD_PROTOCOL = 0x20,
    BLOCK_PROTOCOL = 0x10
} CommandRegBits;
/// TSL2561 Sensor: Registers
typedef enum {
    CONTROL_REG = 0x00,
    TIMING_REG = 0x01,
    INTERRUPT_THRESLOWLOW_REG = 0x02,
    INTERRUPT_THRESLOWHIGH_REG = 0x03,
    INTERRUPT_THRESHIGHLOW_REG = 0x04,
    INTERRUPT_THRESHIGHHIGH_REG = 0x05,
    INTERRUPT_CONTROL_REG = 0x06,
    ID_REG = 0x0A,
    DATA0LOW_REG = 0x0C,
    DATA0HIGH_REG = 0x0D,
    DATA1LOW_REG = 0x0E,
    DATA1HIGH_REG = 0x0F    
} TSL2561Registers;


static void SendMessageCallback(IOTHUB_CLIENT_CONFIRMATION_RESULT result, void *context);
static void TwinCallback(DEVICE_TWIN_UPDATE_STATE updateState, const unsigned char *payload,
                         size_t payloadSize, void *userContextCallback);
static void TwinReportBoolState(const char *propertyName, bool propertyValue);
static void ReportStatusCallback(int result, void *context);
static const char *GetReasonString(IOTHUB_CLIENT_CONNECTION_STATUS_REASON reason);
static const char *getAzureSphereProvisioningResultString(AZURE_SPHERE_PROV_RETURN_VALUE provisioningResult);
static void SendTelemetry(const unsigned char *key, const unsigned char *value);
static void SetupAzureClient(void);
static void AzureTimerEventHandler(EventLoopTimer *timer);
static void SendLightReading(void);
static ExitCode InitPeripheralsAndHandlers(void);
static void CloseFdAndPrintError(int fd, const char *fdName);
static void ClosePeripheralsAndHandlers(void);
static void TerminationHandler(int signalNumber);

static ExitCode ReadWhoAmI(void);
static ExitCode PowerUpSensor(void);
static float ToLux(uint16_t ch0, uint16_t ch1);
static I2CStatus WriteByte(uint8_t deviceReg, uint8_t dataByte);
static I2CStatus ReadByte(uint8_t deviceReg, uint8_t* dataByte);
static I2CStatus ReadWord(uint8_t deviceLowReg, uint16_t* dataByte);
static bool CheckTransferSize(const char *desc, size_t expectedBytes, ssize_t actualBytes);


#define SCOPEID_LENGTH 20
static char scopeId[SCOPEID_LENGTH]; // ScopeId for the Azure IoT Central application, set in
                                     // app_manifest.json, CmdArgs
static IOTHUB_DEVICE_CLIENT_LL_HANDLE iothubClientHandle = NULL;
static const int keepalivePeriodSeconds = 20;
static bool iothubAuthenticated = false;
static volatile sig_atomic_t exitCode = ExitCode_Success;

// I2C TSL2561 Sensor
static int i2cFd = -1;   // File descriptor - initialized to invalid value
// page 7: https://cdn-learn.adafruit.com/downloads/pdf/tsl2561.pdf
static const uint8_t lsl2561Address = 0x39;

// LED
static int deviceTwinStatusLedGpioFd = -1;
static bool statusLedOn = false;

// Timer / polling
static EventLoop *eventLoop = NULL;
static EventLoopTimer *azureTimer = NULL;

// Azure IoT poll periods
static const int AzureIoTDefaultPollPeriodSeconds = 5;
static const int AzureIoTMinReconnectPeriodSeconds = 60;
static const int AzureIoTMaxReconnectPeriodSeconds = 10 * 60;
static int azureIoTPollPeriodSeconds = -1;

// Main entry point for this sample.
int main(int argc, char *argv[]) {
    Log_Debug("IoT Hub/Central TSL2561 Application starting.\n");
    bool isNetworkingReady = false;
    if ((Networking_IsNetworkingReady(&isNetworkingReady) == -1) || !isNetworkingReady) {
        Log_Debug("WARNING: Network is not ready. Device cannot connect until network is ready.\n");
    }
    if (argc == 2) {
        Log_Debug("Setting Azure Scope ID %s\n", argv[1]);
        strncpy(scopeId, argv[1], SCOPEID_LENGTH);
    } else {
        Log_Debug("ScopeId needs to be set in the app_manifest CmdArgs\n");
        return -1;
    }
    exitCode = InitPeripheralsAndHandlers();
    // Main loop
    while (exitCode == ExitCode_Success) {
        EventLoop_Run_Result result = EventLoop_Run(eventLoop, -1, true);
        // Continue if interrupted by signal, e.g. due to breakpoint being set.
        if (result == EventLoop_Run_Failed && errno != EINTR) {
            exitCode = ExitCode_Main_EventLoopFail;
        }
    }
    ClosePeripheralsAndHandlers();
    Log_Debug("Application exiting.\n");
    return exitCode;
}

// Azure timer event:  Check connection status and send telemetry
static void AzureTimerEventHandler(EventLoopTimer *timer) {
    if (ConsumeEventLoopTimerEvent(timer) != 0) {
        exitCode = ExitCode_AzureTimer_Consume;
        return;
    }
    bool isNetworkReady = false;
    if (Networking_IsNetworkingReady(&isNetworkReady) != -1) {
        if (isNetworkReady && !iothubAuthenticated) {
            SetupAzureClient();
        }
    } else {
        Log_Debug("Failed to get Network state\n");
    }
    if (iothubAuthenticated) {
        SendLightReading();
        IoTHubDeviceClient_LL_DoWork(iothubClientHandle);
    }
}

// Read light sensor and sends current reading to IoT Hub.
void SendLightReading(void) {
    uint16_t data0, data1;
    
    if (ReadWord(DATA0LOW_REG, &data0) != I2C_SUCCESS) {
        Log_Debug("INFO: %d ERROR reading ADC channel0 0x02x\n", DATA0LOW_REG);
        return;
    }
    
    if (ReadWord(DATA1LOW_REG, &data1) != I2C_SUCCESS) {
        Log_Debug("INFO: %d ERROR reading ADC channel1 0x02x\n", DATA1LOW_REG);
    }
    float lux = ToLux(data0, data1);

    Log_Debug("INFO: light readingg: %6.2f lux\n", lux);
    
    char tempBuffer[20];
    int len = snprintf(tempBuffer, 20, "%6.2f", lux);
    if (len > 0)
        SendTelemetry("lux", tempBuffer);
}

// Set up SIGTERM termination handler, initialize peripherals, and set up event handlers.
// <returns>ExitCode_Success if all resources were allocated successfully; otherwise another
// ExitCode value which indicates the specific failure.</returns>
static ExitCode InitPeripheralsAndHandlers(void) {
    struct sigaction action;
    memset(&action, 0, sizeof(struct sigaction));
    action.sa_handler = TerminationHandler;
    sigaction(SIGTERM, &action, NULL);
    // Create main event loop
    eventLoop = EventLoop_Create();
    if (eventLoop == NULL) {
        Log_Debug("Could not create event loop.\n");
        return ExitCode_Init_EventLoop;
    }
    // Bring up I2C and TSL2561 sensor
    i2cFd = I2CMaster_Open(SAMPLE_TSL2561_I2C);
    if (i2cFd < 0) {
        Log_Debug("ERROR: I2CMaster_Open: errno=%d (%s)\n", errno, strerror(errno));
        return ExitCode_Init_OpenMaster;
    }
    int result = I2CMaster_SetBusSpeed(i2cFd, I2C_BUS_SPEED_STANDARD);
    if (result != 0) {
        Log_Debug("ERROR: I2CMaster_SetBusSpeed: errno=%d (%s)\n", errno, strerror(errno));
        return ExitCode_Init_SetBusSpeed;
    }
    result = I2CMaster_SetTimeout(i2cFd, 100);
    if (result != 0) {
        Log_Debug("ERROR: I2CMaster_SetTimeout: errno=%d (%s)\n", errno, strerror(errno));
        return ExitCode_Init_SetTimeout;
    }
    // This default address is used for POSIX read and write calls.  The AppLibs APIs take a target
    // address argument for each read or write.
    result = I2CMaster_SetDefaultTargetAddress(i2cFd, lsl2561Address);
    if (result != 0) {
        Log_Debug("ERROR: I2CMaster_SetDefaultTargetAddress: errno=%d (%s)\n", errno,
                  strerror(errno));
        return ExitCode_Init_SetDefaultTarget;
    }
    ExitCode localExitCode = PowerUpSensor();
    if (localExitCode != ExitCode_Success) {
        return localExitCode;
    }
    localExitCode = ReadWhoAmI();
    if (localExitCode != ExitCode_Success) {
        return localExitCode;
    }
    // LED 4 Blue is used to show Device Twin settings state
    Log_Debug("Opening SAMPLE_LED as output\n");
    deviceTwinStatusLedGpioFd =
        GPIO_OpenAsOutput(SAMPLE_LED, GPIO_OutputMode_PushPull, GPIO_Value_High);
    if (deviceTwinStatusLedGpioFd < 0) {
        Log_Debug("ERROR: Could not open LED: %s (%d).\n", strerror(errno), errno);
        return ExitCode_Init_TwinStatusLed;
    }
    // create azure timer that checks connectivity and if connected sends current temperature
    azureIoTPollPeriodSeconds = AzureIoTDefaultPollPeriodSeconds;
    struct timespec azureTelemetryPeriod = {.tv_sec = azureIoTPollPeriodSeconds, .tv_nsec = 0};
    azureTimer =
        CreateEventLoopPeriodicTimer(eventLoop, &AzureTimerEventHandler, &azureTelemetryPeriod);
    if (azureTimer == NULL) {
        return ExitCode_Init_AzureTimer;
    }
    return ExitCode_Success;
}

// Sets the IoT Hub authentication state for the app
// The SAS Token expires which will set the authentication state
static void HubConnectionStatusCallback(IOTHUB_CLIENT_CONNECTION_STATUS result,
                                        IOTHUB_CLIENT_CONNECTION_STATUS_REASON reason,
                                        void *userContextCallback) {
    iothubAuthenticated = (result == IOTHUB_CLIENT_CONNECTION_AUTHENTICATED);
    Log_Debug("IoT Hub Authenticated: %s\n", GetReasonString(reason));
}

// Sets up the Azure IoT Hub connection (creates the iothubClientHandle)
// When the SAS Token for a device expires the connection needs to be recreated
// which is why this is not simply a one time call.
static void SetupAzureClient(void) {
    if (iothubClientHandle != NULL) {
        IoTHubDeviceClient_LL_Destroy(iothubClientHandle);
    }
    AZURE_SPHERE_PROV_RETURN_VALUE provResult =
        IoTHubDeviceClient_LL_CreateWithAzureSphereDeviceAuthProvisioning(scopeId, 10000, &iothubClientHandle);
    Log_Debug("IoTHubDeviceClient_LL_CreateWithAzureSphereDeviceAuthProvisioning returned '%s'.\n",
              getAzureSphereProvisioningResultString(provResult));
    if (provResult.result != AZURE_SPHERE_PROV_RESULT_OK) {
        // If we fail to connect, reduce the polling frequency, starting at
        // AzureIoTMinReconnectPeriodSeconds and with a backoff up to
        // AzureIoTMaxReconnectPeriodSeconds
        if (azureIoTPollPeriodSeconds == AzureIoTDefaultPollPeriodSeconds) {
            azureIoTPollPeriodSeconds = AzureIoTMinReconnectPeriodSeconds;
        } else {
            azureIoTPollPeriodSeconds *= 2;
            if (azureIoTPollPeriodSeconds > AzureIoTMaxReconnectPeriodSeconds) {
                azureIoTPollPeriodSeconds = AzureIoTMaxReconnectPeriodSeconds;
            }
        }
        struct timespec azureTelemetryPeriod = {azureIoTPollPeriodSeconds, 0};
        SetEventLoopTimerPeriod(azureTimer, &azureTelemetryPeriod);

        Log_Debug("ERROR: failure to create IoTHub Handle - will retry in %i seconds.\n", azureIoTPollPeriodSeconds);
        return;
    }
    // Successfully connected, so make sure the polling frequency is back to the default
    azureIoTPollPeriodSeconds = AzureIoTDefaultPollPeriodSeconds;
    struct timespec azureTelemetryPeriod = {.tv_sec = azureIoTPollPeriodSeconds, .tv_nsec = 0};
    SetEventLoopTimerPeriod(azureTimer, &azureTelemetryPeriod);
    iothubAuthenticated = true;
    if (IoTHubDeviceClient_LL_SetOption(iothubClientHandle, OPTION_KEEP_ALIVE,
                                        &keepalivePeriodSeconds) != IOTHUB_CLIENT_OK) {
        Log_Debug("ERROR: failure setting option \"%s\"\n", OPTION_KEEP_ALIVE);
        return;
    }
    IoTHubDeviceClient_LL_SetDeviceTwinCallback(iothubClientHandle, TwinCallback, NULL);
    IoTHubDeviceClient_LL_SetConnectionStatusCallback(iothubClientHandle, HubConnectionStatusCallback, NULL);
}

// Callback invoked when a Device Twin update is received from IoT Hub.
// Updates local state for 'showEvents' (bool).
// <param name="payload">contains the Device Twin JSON document (desired and reported)</param>
// <param name="payloadSize">size of the Device Twin JSON document</param>
static void TwinCallback(DEVICE_TWIN_UPDATE_STATE updateState, const unsigned char *payload,
                         size_t payloadSize, void *userContextCallback) {
    size_t nullTerminatedJsonSize = payloadSize + 1;
    char *nullTerminatedJsonString = (char *)malloc(nullTerminatedJsonSize);
    if (nullTerminatedJsonString == NULL) {
        Log_Debug("ERROR: Could not allocate buffer for twin update payload.\n");
        abort();
    }
    // Copy the provided buffer to a null terminated buffer.
    memcpy(nullTerminatedJsonString, payload, payloadSize);
    // Add the null terminator at the end.
    nullTerminatedJsonString[nullTerminatedJsonSize - 1] = 0;
    JSON_Value *rootProperties = NULL;
    rootProperties = json_parse_string(nullTerminatedJsonString);
    if (rootProperties == NULL) {
        Log_Debug("WARNING: Cannot parse the string as JSON content.\n");
        goto cleanup;
    }
    JSON_Object *rootObject = json_value_get_object(rootProperties);
    JSON_Object *desiredProperties = json_object_dotget_object(rootObject, "desired");
    if (desiredProperties == NULL) {
        desiredProperties = rootObject;
    }
    // Handle the Device Twin Desired Properties here.
    JSON_Object *LEDState = json_object_dotget_object(desiredProperties, "StatusLED");
    if (LEDState != NULL) {
        statusLedOn = (bool)json_object_get_boolean(LEDState, "value");
        GPIO_SetValue(deviceTwinStatusLedGpioFd,
                      (statusLedOn == true ? GPIO_Value_Low : GPIO_Value_High));
        TwinReportBoolState("StatusLED", statusLedOn);
    }
cleanup:
    // Release the allocated memory.
    json_value_free(rootProperties);
    free(nullTerminatedJsonString);
}

// Converts the IoT Hub connection status reason to a string.
static const char *GetReasonString(IOTHUB_CLIENT_CONNECTION_STATUS_REASON reason) {
    static char *reasonString = "unknown reason";
    switch (reason) {
    case IOTHUB_CLIENT_CONNECTION_EXPIRED_SAS_TOKEN:
        reasonString = "IOTHUB_CLIENT_CONNECTION_EXPIRED_SAS_TOKEN";
        break;
    case IOTHUB_CLIENT_CONNECTION_DEVICE_DISABLED:
        reasonString = "IOTHUB_CLIENT_CONNECTION_DEVICE_DISABLED";
        break;
    case IOTHUB_CLIENT_CONNECTION_BAD_CREDENTIAL:
        reasonString = "IOTHUB_CLIENT_CONNECTION_BAD_CREDENTIAL";
        break;
    case IOTHUB_CLIENT_CONNECTION_RETRY_EXPIRED:
        reasonString = "IOTHUB_CLIENT_CONNECTION_RETRY_EXPIRED";
        break;
    case IOTHUB_CLIENT_CONNECTION_NO_NETWORK:
        reasonString = "IOTHUB_CLIENT_CONNECTION_NO_NETWORK";
        break;
    case IOTHUB_CLIENT_CONNECTION_COMMUNICATION_ERROR:
        reasonString = "IOTHUB_CLIENT_CONNECTION_COMMUNICATION_ERROR";
        break;
    case IOTHUB_CLIENT_CONNECTION_OK:
        reasonString = "IOTHUB_CLIENT_CONNECTION_OK";
        break;
    }
    return reasonString;
}

// Converts AZURE_SPHERE_PROV_RETURN_VALUE to a string.
static const char *getAzureSphereProvisioningResultString(
    AZURE_SPHERE_PROV_RETURN_VALUE provisioningResult) {
    switch (provisioningResult.result) {
    case AZURE_SPHERE_PROV_RESULT_OK:
        return "AZURE_SPHERE_PROV_RESULT_OK";
    case AZURE_SPHERE_PROV_RESULT_INVALID_PARAM:
        return "AZURE_SPHERE_PROV_RESULT_INVALID_PARAM";
    case AZURE_SPHERE_PROV_RESULT_NETWORK_NOT_READY:
        return "AZURE_SPHERE_PROV_RESULT_NETWORK_NOT_READY";
    case AZURE_SPHERE_PROV_RESULT_DEVICEAUTH_NOT_READY:
        return "AZURE_SPHERE_PROV_RESULT_DEVICEAUTH_NOT_READY";
    case AZURE_SPHERE_PROV_RESULT_PROV_DEVICE_ERROR:
        return "AZURE_SPHERE_PROV_RESULT_PROV_DEVICE_ERROR";
    case AZURE_SPHERE_PROV_RESULT_GENERIC_ERROR:
        return "AZURE_SPHERE_PROV_RESULT_GENERIC_ERROR";
    default:
        return "UNKNOWN_RETURN_VALUE";
    }
}

// Sends telemetry to IoT Hub
// <param name="key">The telemetry item to update</param>
// <param name="value">new telemetry value</param>
static void SendTelemetry(const unsigned char *key, const unsigned char *value) {
    static char eventBuffer[100] = {0};
    static const char *EventMsgTemplate = "{ \"%s\": \"%s\" }";
    int len = snprintf(eventBuffer, sizeof(eventBuffer), EventMsgTemplate, key, value);
    if (len < 0)
        return;
    Log_Debug("Sending IoT Hub Message: %s\n", eventBuffer);
    bool isNetworkingReady = false;
    if ((Networking_IsNetworkingReady(&isNetworkingReady) == -1) || !isNetworkingReady) {
        Log_Debug("WARNING: Cannot send IoTHubMessage because network is not up.\n");
        return;
    }
    IOTHUB_MESSAGE_HANDLE messageHandle = IoTHubMessage_CreateFromString(eventBuffer);
    if (messageHandle == 0) {
        Log_Debug("WARNING: unable to create a new IoTHubMessage\n");
        return;
    }
    if (IoTHubDeviceClient_LL_SendEventAsync(iothubClientHandle, messageHandle, SendMessageCallback,
                                             /*&callback_param*/ 0) != IOTHUB_CLIENT_OK) {
        Log_Debug("WARNING: failed to hand over the message to IoTHubClient\n");
    } else {
        Log_Debug("INFO: IoTHubClient accepted the message for delivery\n");
    }
    IoTHubMessage_Destroy(messageHandle);
}

// Callback confirming message delivered to IoT Hub.
// <param name="result">Message delivery status</param>
// <param name="context">User specified context</param>
static void SendMessageCallback(IOTHUB_CLIENT_CONFIRMATION_RESULT result, void *context) {
    Log_Debug("INFO: Message received by IoT Hub. Result is: %d\n", result);
}

// Creates and enqueues a report containing the name and value pair of a Device Twin reported
// property. The report is not sent immediately, but it is sent on the next invocation of
// IoTHubDeviceClient_LL_DoWork().
// <param name="propertyName">the IoT Hub Device Twin property name</param>
// <param name="propertyValue">the IoT Hub Device Twin property value</param>
static void TwinReportBoolState(const char *propertyName, bool propertyValue) {
    if (iothubClientHandle == NULL) {
        Log_Debug("ERROR: client not initialized\n");
    } else {
        static char reportedPropertiesString[30] = {0};
        int len = snprintf(reportedPropertiesString, 30, "{\"%s\":%s}", propertyName,
                           (propertyValue == true ? "true" : "false"));
        if (len < 0)
            return;
        if (IoTHubDeviceClient_LL_SendReportedState(
                iothubClientHandle, (unsigned char *)reportedPropertiesString,
                strlen(reportedPropertiesString), ReportStatusCallback, 0) != IOTHUB_CLIENT_OK) {
            Log_Debug("ERROR: failed to set reported state for '%s'.\n", propertyName);
        } else {
            Log_Debug("INFO: Reported state for '%s' to value '%s'.\n", propertyName,
                      (propertyValue == true ? "true" : "false"));
        }
    }
}

// Callback invoked when the Device Twin reported properties are accepted by IoT Hub.
static void ReportStatusCallback(int result, void *context) {
    Log_Debug("INFO: Device Twin reported properties update result: HTTP status code %d\n", result);
}


// Signal handler for termination requests. This handler must be async-signal-safe.
static void TerminationHandler(int signalNumber) {
    // Don't use Log_Debug here, as it is not guaranteed to be async-signal-safe.
    exitCode = ExitCode_TermHandler_SigTerm;
}


// Closes a file descriptor and prints an error on failure.
// <param name="fd">File descriptor to close</param>
// <param name="fdName">File descriptor name to use in error message</param>
static void CloseFdAndPrintError(int fd, const char *fdName) {
    if (fd >= 0) {
        int result = close(fd);
        if (result != 0) {
            Log_Debug("ERROR: Could not close fd %s: %s (%d).\n", fdName, strerror(errno), errno);
        }
    }
}

// Close peripherals and handlers.
static void ClosePeripheralsAndHandlers(void) {
    DisposeEventLoopTimer(azureTimer);
    EventLoop_Close(eventLoop);
    Log_Debug("Closing file descriptors\n");
    if (deviceTwinStatusLedGpioFd >= 0) { // Leave the LEDs off
        GPIO_SetValue(deviceTwinStatusLedGpioFd, GPIO_Value_High);
    }
    CloseFdAndPrintError(deviceTwinStatusLedGpioFd, "StatusLed");
}

static ExitCode PowerUpSensor() {
    if (WriteByte(CONTROL_REG, 0x03) != I2C_SUCCESS) {
        Log_Debug("ERROR: Writing ID_REG=0x%02x failed)\n", ID_REG);
        return ExitCode_PowerUpFailed;
    }
    return ExitCode_Success;
}

// Reads out ID register from device and checks it to perform 'presence' test
static ExitCode ReadWhoAmI(void)
{
    static const uint8_t expectedWhoAmI = 0x50;
    uint8_t actualWhoAmI;

    // check ID
    if (ReadByte(ID_REG, &actualWhoAmI) != I2C_SUCCESS) {
        Log_Debug("ERROR: Reading ID_REG=0x%02x failed)\n", ID_REG);
        return ExitCode_ReadWhoAmI_IDRead;
        
    }
    Log_Debug("INFO: WHO_AM_I=0x%02x\n", actualWhoAmI);
    if ((actualWhoAmI & 0xF0) != expectedWhoAmI) {
        Log_Debug("ERROR: Unexpected WHO_AM_I value: 0x%0x02\n", expectedWhoAmI);
        return ExitCode_ReadWhoAmI_InvalidID;
    }

    return ExitCode_Success;
}

// conversion to Lux as specified in the TSL2561 Datasheet p.24
// https://ams.com/documents/20143/36005/TSL2561_DS000110_3-00.pdf/18a41097-2035-4333-c70e-bfa544c0a98b
static float ToLux(uint16_t ch0, uint16_t ch1) {
    float ch1_div_ch0 = ((float) ch1) / ((float) ch0);
    if (ch1_div_ch0 <= 0.5f) {
        return 0.0304f * ch0 - 0.062f * ch0 * powf(ch1_div_ch0, 1.4f);
    } else if (ch1_div_ch0 <= 0.61f) {
        return 0.0224f * ch0 - 0.031f * ch1;
    } else if (ch1_div_ch0 <= 0.80f) {
        return 0.00128f * ch0 - 0.0153f * ch1; 
    } else if (ch1_div_ch0 <= 1.3f) {
        return 0.00146f * ch0 - 0.00112f * ch1;
    } else {
        return 0.0f;
    }
}

//
// I2C Helper functions
//
static I2CStatus WriteByte(uint8_t deviceReg, uint8_t dataByte) {
    static const uint8_t commandPrefix = SELECT_CMD_REG | CLEAR_INTERRUPT | WORD_PROTOCOL;
    uint8_t command[2] = { commandPrefix | (0x0F & deviceReg), dataByte };
    ssize_t transferredBytes =
            I2CMaster_Write(i2cFd, lsl2561Address, command, sizeof(command));
    if (!CheckTransferSize("I2CMaster_Write (WriteByte)",
                           sizeof(command), transferredBytes)) {
        return I2C_TRANSFER_LENGTH_MISMATCH;
    }
    return I2C_SUCCESS;
}

static I2CStatus ReadByte(uint8_t deviceReg, uint8_t* dataByte) {
    static const uint8_t commandPrefix = SELECT_CMD_REG | CLEAR_INTERRUPT | WORD_PROTOCOL;
    uint8_t command = commandPrefix | (0x0F & deviceReg);
    ssize_t transferredBytes = I2CMaster_WriteThenRead(
        i2cFd, lsl2561Address, &command, sizeof(command), dataByte, sizeof(uint8_t));
    if (!CheckTransferSize("I2CMaster_WriteThenRead",
                           sizeof(command) + sizeof(uint8_t), transferredBytes)) {
        exitCode = I2C_TRANSFER_LENGTH_MISMATCH;
    }
    return I2C_SUCCESS;
}

static I2CStatus ReadWord(uint8_t deviceLowReg, uint16_t* dataByte) {
    static const uint8_t commandPrefix = SELECT_CMD_REG | CLEAR_INTERRUPT | WORD_PROTOCOL;
    uint8_t command = commandPrefix | (0x0F & deviceLowReg);
    ssize_t transferredBytes = I2CMaster_WriteThenRead(
        i2cFd, lsl2561Address, &command, sizeof(command), (uint8_t*)dataByte, sizeof(uint16_t));
    if (!CheckTransferSize("I2CMaster_WriteThenRead",
                           sizeof(command) + sizeof(uint16_t), transferredBytes)) {
        exitCode = I2C_TRANSFER_LENGTH_MISMATCH;
    }
    return I2C_SUCCESS;
}

// Checks the number of transferred bytes for I2C functions and prints an error
// message if the functions failed or if the number of bytes is different than
// expected number of bytes to be transferred.
// <returns>true on success, or false on failure</returns>
static bool CheckTransferSize(const char *desc, size_t expectedBytes, ssize_t actualBytes)
{
    if (actualBytes < 0) {
        Log_Debug("ERROR: %s: errno=%d (%s)\n", desc, errno, strerror(errno));
        return false;
    }
    if (actualBytes != (ssize_t)expectedBytes) {
        Log_Debug("ERROR: %s: transferred %zd bytes; expected %zd\n", desc, actualBytes,
                  expectedBytes);
        return false;
    }

    return true;
}
