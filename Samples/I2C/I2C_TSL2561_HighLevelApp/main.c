/* Copyright (c) Microsoft Corporation. All rights reserved.
   Licensed under the MIT License. */

// This sample C application for Azure Sphere uses the Azure Sphere I2C APIs to display
// data from a light sensor connected via I2C.
//
// It uses the APIs for the following Azure Sphere application libraries:
// - log (messages shown in Visual Studio's Device Output window during debugging)
// - i2c (communicates with TSL2561 light sensor)
// - eventloop (system invokes handlers for timer events)

#include <errno.h>
#include <math.h>
#include <signal.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

// applibs_versions.h defines the API struct versions to use for applibs APIs.
#include "applibs_versions.h"
#include <applibs/log.h>
#include <applibs/i2c.h>
#include <applibs/eventloop.h>

// By default, this sample's CMake build targets hardware that follows the MT3620
// Reference Dev Board (RDB) specification, such as the MT3620 Dev Kit from Seeed Studios.
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

// Exit codes for this application. These are used for the application exit code. 
// They must all be between zero and 255, where zero is reserved for successful termination.
typedef enum {
    ExitCode_Success = 0,

    ExitCode_TermHandler_SigTerm = 1,

    ExitCode_SensorTimer_Consume = 2,
    ExitCode_SensorTimer_ReadStatus = 3,

    ExitCode_PowerUpFailed = 4,

    ExitCode_ReadWhoAmI_IDRead = 5,
    ExitCode_ReadWhoAmI_InvalidID = 6,

    ExitCode_Init_EventLoop = 15,
    ExitCode_Init_SensorTimer = 16,
    ExitCode_Init_OpenMaster = 17,
    ExitCode_Init_SetBusSpeed = 18,
    ExitCode_Init_SetTimeout = 19,
    ExitCode_Init_SetDefaultTarget = 20,

    ExitCode_Main_EventLoopFail = 21
} ExitCode;

typedef enum {
    I2C_SUCCESS = 0,
    I2C_TRANSFER_LENGTH_MISMATCH = 1,
} I2CStatus;

typedef enum { 
    SELECT_CMD_REG = 0x80,
    CLEAR_INTERRUPT = 0x40,
    WORD_PROTOCOL = 0x20,
    BLOCK_PROTOCOL = 0x10
} CommandRegBits;

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

// Support functions.
static void TerminationHandler(int signalNumber);
static void SensorTimerEventHandler(EventLoopTimer *timer);
static ExitCode ReadWhoAmI(void);
static ExitCode PowerUpSensor(void);
static float ToLux(uint16_t ch0, uint16_t ch1);

static bool CheckTransferSize(const char *desc, size_t expectedBytes, ssize_t actualBytes);
static ExitCode InitPeripheralsAndHandlers(void);
static void CloseFdAndPrintError(int fd, const char *fdName);
static void ClosePeripheralsAndHandlers(void);


// File descriptors - initialized to invalid value
static int i2cFd = -1;

static EventLoop *eventLoop = NULL;
static EventLoopTimer *sensorTimer = NULL;

// Termination state
static volatile sig_atomic_t exitCode = ExitCode_Success;

// page 7: https://cdn-learn.adafruit.com/downloads/pdf/tsl2561.pdf
static const uint8_t lsl2561Address = 0x39;



static I2CStatus WriteByte(uint8_t deviceReg, uint8_t dataByte) 
{
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


static I2CStatus ReadByte(uint8_t deviceReg, uint8_t* dataByte) 
{
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

static I2CStatus ReadWord(uint8_t deviceLowReg, uint16_t* dataByte) 
{
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


// conversion to Lux as specified in the TSL2561 Datasheet p.24
// https://ams.com/documents/20143/36005/TSL2561_DS000110_3-00.pdf/18a41097-2035-4333-c70e-bfa544c0a98b
static float ToLux(uint16_t ch0, uint16_t ch1)
{
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

// Signal handler for termination requests. This handler must be async-signal-safe.
static void TerminationHandler(int signalNumber)
{
    // Don't use Log_Debug here, as it is not guaranteed to be async-signal-safe.
    exitCode = ExitCode_TermHandler_SigTerm;
}

// Print latest data from the light sensor.
static void SensorTimerEventHandler(EventLoopTimer *timer)
{
    static int iter = 1;
    uint16_t data0, data1;
    ++iter;

    if (ConsumeEventLoopTimerEvent(timer) != 0) {
        exitCode = ExitCode_SensorTimer_Consume;
        return;
    }

    if (ReadWord(DATA0LOW_REG, &data0) != I2C_SUCCESS) {
        Log_Debug("INFO: %d ERROR reading ADC channel0 0x02x\n", DATA0LOW_REG);
        return;
    }
    
    if (ReadWord(DATA1LOW_REG, &data1) != I2C_SUCCESS) {
        Log_Debug("INFO: %d ERROR reading ADC channel1 0x02x\n", DATA1LOW_REG);
    }
    float lux = ToLux(data0, data1);

    Log_Debug("INFO: %d: light reading: DATA0 = %u, DATA1 = %u, %f lux\n", iter, data0, data1, lux);
    
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

// Set up SIGTERM termination handler, initialize peripherals, and set up event handlers.
// <returns>ExitCode_Success if all resources were allocated successful; 
// Otherwise another ExitCode value which indicates the specific failure.</returns>
static ExitCode InitPeripheralsAndHandlers(void)
{
    struct sigaction action;
    memset(&action, 0, sizeof(struct sigaction));
    action.sa_handler = TerminationHandler;
    sigaction(SIGTERM, &action, NULL);

    eventLoop = EventLoop_Create();
    if (eventLoop == NULL) {
        Log_Debug("Could not create event loop.\n");
        return ExitCode_Init_EventLoop;
    }

    // Print light data every second.
    static const struct timespec accelReadPeriod = {.tv_sec = 1, .tv_nsec = 0};
    sensorTimer = CreateEventLoopPeriodicTimer(eventLoop, &SensorTimerEventHandler, &accelReadPeriod);
    if (sensorTimer == NULL) {
        return ExitCode_Init_SensorTimer;
    }

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

    return ExitCode_Success;
}

// Closes a file descriptor and prints an error on failure.
// <param name="fd">File descriptor to close</param>
// <param name="fdName">File descriptor name to use in error message</param>
static void CloseFdAndPrintError(int fd, const char *fdName)
{
    if (fd >= 0) {
        int result = close(fd);
        if (result != 0) {
            Log_Debug("ERROR: Could not close fd %s: %s (%d).\n", fdName, strerror(errno), errno);
        }
    }
}


static void ClosePeripheralsAndHandlers(void)
{
    DisposeEventLoopTimer(sensorTimer);
    EventLoop_Close(eventLoop);

    Log_Debug("Closing file descriptors.\n");
    CloseFdAndPrintError(i2cFd, "i2c");
}


int main(int argc, char *argv[])
{
    Log_Debug("I2C light sensor application starting.\n");
    exitCode = InitPeripheralsAndHandlers();

    // Use event loop to wait for events and trigger handlers, until an error or SIGTERM happens
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
