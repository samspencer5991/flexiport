/*
 * flexiport.h
 *
 *  Created on: 26 Apr 2020
 *      Author: samspencer
 */

#ifndef FLEXIPORT_H_
#define FLEXIPORT_H_

#ifdef FRAMEWORK_STM32CUBE
    #ifdef STM32G4xx
        #include "stm32g4xx_hal.h"
    #elif defined(STM32H5xx)
        #include "stm32h5xx_hal.h"
    #endif
#endif
#include "buttons.h"
#include "midi.h"

#define NUM_DEVICE_LINK_SLAVES    32

#define NUM_FLEXI_ADC_BUF_SAMPLES 2

#ifndef FLEXI_ADC_HYSTERESIS_BOUNDRY
    #define FLEXI_ADC_HYSTERESIS_BOUNDRY 2
#endif

#define DEVICE_LINK_ALIVE_TIMEOUT    1000

#define FLEXI_MIDI_RX_BUF_SIZE       1024

// Device Link device list
#define DL_BRIDGE4                   0x0000
#define DL_BRIDGE6                   0x0001
#define DL_SLIDE4                    0x0002
#define DL_FLEXIHUB                  0x0003
#define DL_BRIDGE1                   0x0004
#define DL_TRI_SWITCH                0x0005

// Device Link generic command list
#define DEVICE_LINK_ALIVE_CHECK_BYTE 0x50
#define DEVICE_LINK_BANK_UP_CMD      0x51
#define DEVICE_LINK_BANK_DOWN_CMD    0x52
#define DEVICE_LINK_GOTO_BANK_CMD    0x53
#define DEVICE_LINK_BANK_NAME_CMD    0x54
#define DEVICE_LINK_SWITCH_GROUP_CMD 0x55

#define FLEXI_DEBOUNCE_LOW_TIME      20
#define FLEXI_DEBOUNCE_MED_TIME      50
#define FLEXI_DEBOUNCE_HIGH_TIME     100

#define FLEXIPORT_NONE               0
#define FLEXIPORT_TIP                1
#define FLEXIPORT_RING               2
#define FLEXIPORT_TIP_RING           3

typedef enum
{
    DeviceLinkAlive           = 0x50,
    DeviceLinkBankUp          = 0x51,
    DeviceLinkBankDown        = 0x52,
    DeviceLinkGoToBank        = 0x53,
    DeviceLinkBankName        = 0x54,
    DeviceLinkSwitchGroup     = 0x55,
    DeviceLinkBankNameRequest = 0x56
} DeviceLinkCommand;

typedef enum
{
    FlexiHalError,
    FlexiParamError,
    FlexiMemError,
    FlexiNewConnection,
    FlexiRemovedConnection,
    FlexiModeError,
    FlexiDeviceLinkTimeout,
    FlexiDeviceLinkConfigError,
    FlexiMidiBufferError,
    FlexiOk
} FlexiErrorState;

// Port wide configurations of a flexiport
typedef enum
{
    FlexiUnassigned,         // No function assigned
    FlexiMidiOutTypeA,       // MIDI output Type A
    FlexiMidiOutTypeB,       // MIDI output Type B
    FlexiMidiOutTip,         // MIDI output Tip Active
    FlexiMidiOutRing,        // MIDI output Ring Active
    FlexiDeviceLink,         // Connected to another unit to link. Master/slave is set in the device settings
    FlexiDualExpressionIn,   // 2x expression pedal
    FlexiSingleExpressionIn, // Single expression pedal
    FlexiSwitchIn,           // Dual switch input
    FlexiSwitchOut,          // Dual switch output
    FlexiVoltageOut,         // Dual voltage output
    FlexiTapTempoOut,
    FlexiPulseOut,
    FlexiFavSwitchOut, // Strymon single Fav switch emulation
    FlexiMidiInTypeA,
    FlexiMidiInTypeB
} FlexiportMode;

typedef enum
{
    FlexiConnected,
    FlexiRemoved, // No connector has been detected
} FlexiportStatus;

// All possible configurations for each channel of a flexiport
typedef enum
{
    FlexiGround,
    FlexiVcc,
    FlexiDisconnected
} ChannelMode;

typedef enum
{
    DeviceLinkNotActive,
    DeviceLinkConnected,
    DeviceLinkNotConnected
} DeviceLinkState;

typedef enum
{
    FlexiCloseSwitch,
    FlexiOpenSwitch
} FlexiSwitchState;

typedef enum
{
    FlexiUartTipTx,
    FlexiUartTipRx
} FlexiUartTrsConfig;

typedef enum
{
    // Tip
    FlexiSwitchOutTipClosed = 0x00,
    FlexiSwitchOutTipOpen   = 0x01,
    FlexiSwitchOutTipToggle = 0x02,
    // Ring
    FlexiSwitchOutRingClosed = 0x03,
    FlexiSwitchOutRingOpen   = 0x04,
    FlexiSwitchOutRingToggle = 0x05,
    // Tip + Ring
    FlexiSwitchOutTipRingClosed = 0x06,
    FlexiSwitchOutTipRingOpen   = 0x07,
    FlexiSwitchOutTipRingToggle = 0x08
} FlexiSwitchOutMessage;

typedef enum
{
    // Tip
    FlexiTipHigh   = 0x00,
    FlexiTipLow    = 0x01,
    FlexiTipToggle = 0x02,
    // Ring
    FlexiRingHigh   = 0x03,
    FlexiRingLow    = 0x04,
    FlexiRingToggle = 0x05,
    // Tip + Ring
    FlexiTipRingHigh   = 0x06,
    FlexiTipRingLow    = 0x07,
    FlexiTipRingToggle = 0x08
} FlexiVoltageOutMessage;

typedef enum
{
    DeviceLinkMasterRole,
    DeviceLinkSlaveRole
} DeviceLinkRole;

typedef struct
{
    uint16_t deviceType;
    uint8_t  firmwareVersion;
    uint8_t  deviceLinkVersion;
    uint8_t  networkId;
    uint8_t  numDeviceLinkPorts;
} DeviceLinkSlave;

typedef enum
{
    DeviceLinkUpstream,  // Port connects to a downstream slave device
    DeviceLinkDownstream // Port connects to an upstream slave device or master
} DeviceLinkPortDirection;

typedef struct
{
    /* non-volatile */
    FlexiportMode mode;      // Overall operating mode of the port
    int8_t        midiClock; // Assigned MIDI clock for tempo related functions. -1 = unassigned
    uint16_t      calibrationMinA;
    uint16_t      calibrationMaxA;
    uint16_t      calibrationMinB;
    uint16_t      calibrationMaxB;
    uint8_t       flexiHoldTime;
    uint8_t       flexiDebounceTime;
} FlexiportConfig;

// Flexiport configuration struct
typedef struct
{
    FlexiportConfig *config;	// Stored in the global settings
    FlexiportStatus status; 	// Operating state of the flexiport
	uint8_t                 flexiportLite;

    // Ports and pins
	// Pins used by the UART and switch GPIO
    GPIO_TypeDef *portA;
    GPIO_TypeDef *portB;
    uint16_t      pinA; 
    uint16_t      pinB;

	// Pins used by ADC
    GPIO_TypeDef *adcPortTip;
    GPIO_TypeDef *adcPortRing;
    uint16_t      adcPinTip;  
    uint16_t      adcPinRing;

    // ADC
    ADC_HandleTypeDef *hadc;
    DMA_HandleTypeDef *hadcDma;
	DMA_Channel_TypeDef *adcDmaChannel;

    uint32_t adcChannelA;
    uint32_t adcChannelB;

	uint32_t adcRawReadings[2];
    uint16_t adcReadingBuf[2][NUM_FLEXI_ADC_BUF_SAMPLES];
    uint16_t filteredReadings[2];
    uint16_t adcLastChangedReadings[2];
    uint8_t  adcReadIndex;
	uint8_t expReadingChanged[2];

	// UART
    UART_HandleTypeDef *huart;
    DMA_HandleTypeDef   *huartDma;
    DMA_Channel_TypeDef *uartDmaChannel;
    //IRQn_Type            uartDmaIrq;

    TIM_HandleTypeDef *auxHoldTim;
    volatile uint8_t   timerTriggered; // holds the bit index of which switch triggered the hold

	// External analog switches for full Flexiports
    //  Tip (A) to sleeve switch
    GPIO_TypeDef *sleeveAPort;
    uint16_t      sleeveAPin;
    // Ring (B) to sleeve switch
    GPIO_TypeDef *sleeveBPort;
    uint16_t      sleeveBPin;
    // Tip (A) to 3V switch
    GPIO_TypeDef *vAPort;
    uint16_t      vAPin;
    // Ring (B) to 3V switch
    GPIO_TypeDef *vBPort;
    uint16_t      vBPin;
    // Sleeve to ground switch
    GPIO_TypeDef *gndSleevePort;
    uint16_t      gndSleevePin;
    // Sleeve to ground switch
    GPIO_TypeDef *sensePort;
    uint16_t      sensePin;

	// MIDI
    MidiInterface *midiHandle; // Pointer to the MIDI handle for linking with MIDI library

    // External footswitch input variables
    // FlexiExtIn extSwitchIn;
    Button   extSwitchInTip;         // Physical Tip button
    Button   extSwitchInRing;        // Physical Ring button
    Button   extSwitchInTipRing;     // Emulated button, triggered in software from Tip and Ring events
    //uint32_t extSwitchInLastTime;    // Time since EXT event triggering for state debouncing. 0 = no event
    //uint8_t  extSwitchInLastTrigger; // Last EXT event. 0 = none, 1 = Tip, 2 = Ring

	// Polling external switch in
	GPIO_PinState lastPinStateTip;      // Last read state of Tip pin
    GPIO_PinState lastPinStateRing;     // Last read state of Ring pin
    uint32_t lastPollTime;              // Last time the pins were polled

    // Device Link
    DeviceLinkState         deviceLinkPortState;
    uint8_t                 aliveCheckSent; // Flag to indicate that an alive check has been sent
    DeviceLinkPortDirection deviceLinkDirection;
    
} Flexiport;

typedef struct
{
    DeviceLinkState state;
    uint8_t         aliveCheckReceived;
    DeviceLinkRole  role;
    uint16_t        deviceType;
    uint8_t         numActiveDeviceLinkPorts;
    // Control settings
    uint8_t rxBankName;
    uint8_t rxBankNav;
    uint8_t rxMidiStream;
    uint8_t rxSwitchGroups;
} DeviceLink;

void            flexi_initPort(Flexiport *flexiPort);
FlexiErrorState flexi_setModeMidiOut(Flexiport *flexiport, uint8_t midiType);
FlexiErrorState flexi_setModeMidiIn(Flexiport *flexiport, uint8_t midiType);
FlexiErrorState flexi_setModeUnassigned(Flexiport *flexiPort);
FlexiErrorState flexi_setModeExpIn(Flexiport *flexiPort, FlexiportMode expMode);
FlexiErrorState flexi_setModeDualSwitchOut(Flexiport *flexiPort);
FlexiErrorState flexi_setModeDualVoltageOut(Flexiport *flexiport);
FlexiErrorState flexi_setModeDualTapTempoOut(Flexiport *flexiPort);
FlexiErrorState flexi_setModeDualSwitchIn(Flexiport *flexiPort);
FlexiErrorState flexi_setModePulseOut(Flexiport *flexiport);
FlexiErrorState flexi_setModeDeviceLinkMaster(Flexiport *flexiPort);
FlexiErrorState flexi_setModeDeviceLinkSlave(Flexiport *flexiport);
FlexiErrorState flexi_setModeFavSwitchOut(Flexiport *flexiport);

void            flexi_initDeviceLink(DeviceLink *deviceLink);
FlexiErrorState flexi_deviceLinkAliveCheck(Flexiport *flexiport, DeviceLink *deviceLink);
FlexiErrorState flexi_deviceLinkSendBankUp(Flexiport *flexiport, DeviceLink *deviceLink, char *bankName, uint8_t bankNameLen);
FlexiErrorState flexi_deviceLinkSendBankDown(Flexiport *flexiport, DeviceLink *deviceLink, char *bankName, uint8_t bankNameLen);
FlexiErrorState flexi_deviceLinkSendGoToBank(Flexiport *flexiport, DeviceLink *deviceLink, uint8_t bank, char *bankName, uint8_t bankNameLen);
FlexiErrorState flexi_deviceLinkSendBankName(Flexiport *flexiport, DeviceLink *deviceLink, char *bankName, uint8_t bankNameLen);
FlexiErrorState flexi_deviceLinkSendBankNameRequest(Flexiport *flexiport, DeviceLink *deviceLink);
FlexiErrorState flexi_deviceLinkSendSwitchGroupEvent(Flexiport *flexiport, DeviceLink *deviceLink, uint8_t group, uint8_t switchEvent);

FlexiErrorState flexi_checkPorts(Flexiport *flexiPort);

// Switch control functions
FlexiErrorState flexi_setTipVSwitch(Flexiport *flexiPort, FlexiSwitchState state);
FlexiErrorState flexi_setRingVSwitch(Flexiport *flexiPort, FlexiSwitchState state);
FlexiErrorState flexi_setTipSleeveSwitch(Flexiport *flexiPort, FlexiSwitchState state);
FlexiErrorState flexi_setRingSleeveSwitch(Flexiport *flexiPort, FlexiSwitchState state);
FlexiErrorState flexi_setSleeveGroundSwitch(Flexiport *flexiport, FlexiSwitchState state);

FlexiErrorState flexi_gpioOutputInit(Flexiport *flexiport);

void            flexi_filterAdcReadings(Flexiport *flexiport);
FlexiErrorState flexi_setHoldTimer(Flexiport *flexiport, TIM_HandleTypeDef *timHandle, uint16_t time);
void flexi_pollSwitchInputs(Flexiport* flexiport);
#endif /* FLEXIPORT_H_ */
