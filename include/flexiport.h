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
#include "midi.h"
#include "buttons.h"

#define NUM_DEVICE_LINK_SLAVES	32

#define NUM_FLEXI_ADC_BUF_SAMPLES	3

#ifndef FLEXI_ADC_HYSTERESIS_BOUNDRY
#define FLEXI_ADC_HYSTERESIS_BOUNDRY 2
#endif

#define DEVICE_LINK_ALIVE_TIMEOUT			1000


#define FLEXI_MIDI_RX_BUF_SIZE 1024

// Device Link device list
#define DL_BRIDGE4 			0x0000
#define DL_BRIDGE6			0x0001
#define DL_SLIDE4				0x0002
#define DL_FLEXIHUB			0x0003
#define DL_BRIDGE1			0x0004
#define DL_TRI_SWITCH		0x0005

// Device Link generic command list
#define DEVICE_LINK_ALIVE_CHECK_BYTE			0x80
#define DEVICE_LINK_BANK_UP_CMD					0x81
#define DEVICE_LINK_BANK_DOWN_CMD				0x82
#define DEVICE_LINK_GOTO_BANK_CMD				0x83
#define DEVICE_LINK_BANK_NAME_CMD				0x84
#define DEVICE_LINK_SWITCH_GROUP_CMD			0x85

#define FLEXI_DEBOUNCE_LOW_TIME 		20
#define FLEXI_DEBOUNCE_MED_TIME		50
#define FLEXI_DEBOUNCE_HIGH_TIME 	100

#define FLEXIPORT_NONE		0
#define FLEXIPORT_TIP		1
#define FLEXIPORT_RING		2
#define FLEXIPORT_TIP_RING	3

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
	FlexiUnassigned,				// No function assigned
	FlexiMidiOutTypeA,			// MIDI output Type A
	FlexiMidiOutTypeB,			// MIDI output Type B
	FlexiMidiOutTip,				// MIDI output Tip Active
	FlexiMidiOutRing,				// MIDI output Ring Active
	FlexiDeviceLink,				// Connected to another unit to link. Master/slave is set in the device settings
	FlexiDualExpressionIn,		// 2x expression pedal
	FlexiSingleExpressionIn,	// Single expression pedal
	FlexiSwitchIn,					// Dual switch input
	FlexiSwitchOut,				// Dual switch output
	FlexiVoltageOut,				// Dual voltage output
	FlexiTapTempoOut,
	FlexiPulseOut,
	FlexiFavSwitchOut				// Strymon single Fav switch emulation
} FlexiportMode;

typedef enum
{
	FlexiConnected,
	FlexiRemoved,						// No connector has been detected
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
	FlexiSwitchOutOpen,
	FlexiSwitchOutTipClosed,
	FlexiSwitchOutRingClosed,
	FlexiSwitchOutTipRingClosed
} FlexiSwitchOutMessage;

typedef enum
{
	FlexiAllLow,
	FlexiTipHigh,
	FlexiRingHigh,
	FlexiTipRingHigh
} FlexiVoltageOutMessage;

typedef struct
{
	volatile ButtonState stateTip;
	volatile ButtonState lastStateTip;
	volatile ButtonState stateRing;
	volatile ButtonState lastStateRing;
	volatile ButtonState stateTipRing;
	volatile ButtonState lastStateTipRing;

	volatile uint32_t lastTimeTip;
	volatile uint32_t lastTimeRing;
	ButtonLogic logicMode;
	void(*handlerTip)(ButtonState state);
	void(*handlerRing)(ButtonState state);
	void(*handlerTipRing)(ButtonState state);
} FlexiExtIn;

typedef enum
{
	DeviceLinkMasterRole,
	DeviceLinkSlaveRole
} DeviceLinkRole;

typedef struct
{
	uint16_t deviceType;
	uint8_t firmwareVersion;
	uint8_t deviceLinkVersion;
	uint8_t networkId;
	uint8_t numDeviceLinkPorts;
} DeviceLinkSlave;

typedef enum
{
	DeviceLinkUpstream,					// Port connects to a downstream slave device
	DeviceLinkDownstream				// Port connects to an upstream slave device or master
} DeviceLinkPortDirection;

typedef struct
{
	/* non-volatile */
	FlexiportMode mode;					// Overall operating mode of the port
	int8_t midiClock;				// Assigned MIDI clock for tempo related functions. -1 = unassigned
	uint16_t calibrationMinA;
	uint16_t calibrationMaxA;
	uint16_t calibrationMinB;
	uint16_t calibrationMaxB;
	uint8_t flexiHoldTime;
	uint8_t flexiDebounceTime;
} FlexiportConfig;

// Flexiport configuration struct
typedef struct
{
	FlexiportConfig* config;
	/* Volatile */
	FlexiportStatus status;			// Operating state of the flexiport
	// Hardware handles and pointers
	GPIO_TypeDef* portA;
	GPIO_TypeDef* portB;
	uint16_t pinA;									// Pin used by the UART
	uint16_t pinB;									// Pin used by the UART

	GPIO_TypeDef* adcPortTip;
	GPIO_TypeDef* adcPortRing;
	uint16_t adcPinTip;							// Pin used by ADC
	uint16_t adcPinRing;						// Pin used by ADC

	// GPIO EXTI handles
	IRQn_Type extiPin1Irq;
	IRQn_Type extiPin2Irq;

	// ADC handles
	ADC_HandleTypeDef* hadc;
	DMA_HandleTypeDef* hadcDma;
	uint8_t adcDmaIrq;
	uint32_t adcChannelA;
	uint32_t adcChannelB;
	IRQn_Type adcIrq;
	uint32_t adcRawReadings[2];
	uint16_t adcReadingBuf[2][NUM_FLEXI_ADC_BUF_SAMPLES];
	uint16_t filteredReadings[2];
	uint8_t adcReadIndex;

	TIM_HandleTypeDef *auxHoldTim;
	volatile uint8_t timerTriggered;	// holds the bit index of which switch triggered the hold

	// UART handle
	UART_HandleTypeDef *huart;
	IRQn_Type uartIrq;
	uint32_t uartPeriphClk;
	DMA_HandleTypeDef* huartDma;
	uint8_t uartDmaIrq;

	FlexiUartTrsConfig trsWiringConfig;	// TX/RX order on the tip and ring
	// Tip (A) to sleeve switch
	GPIO_TypeDef* sleeveAPort;					// hardware port
	uint16_t sleeveAPin;								// hardware pin
	// Ring (B) to sleeve switch
	GPIO_TypeDef* sleeveBPort;					// hardware port
	uint16_t sleeveBPin;								// hardware pin
	// Tip (A) to 3V switch
	GPIO_TypeDef* vAPort;								// hardware port
	uint16_t vAPin;											// hardware pin
	// Ring (B) to 3V switch
	GPIO_TypeDef* vBPort;								// hardware port
	uint16_t vBPin;											// hardware pin
	// Sleeve to ground switch
	GPIO_TypeDef* gndSleevePort;				// hardware port
	uint16_t gndSleevePin;							// hardware pin
	// Sleeve to ground switch
	GPIO_TypeDef* sensePort;						// hardware port
	uint16_t sensePin;									// hardware pin

	MidiInterface* midiHandle;					// Pointer to the MIDI handle for linking with MIDI library
	uint8_t rawRxMidiBuf[FLEXI_MIDI_RX_BUF_SIZE];

	// External footswitch input variables
	//FlexiExtIn extSwitchIn;
	Button extSwitchInTip;		// Physical Tip button
	Button extSwitchInRing;		// Physical Ring button
	Button extSwitchInTipRing;	// Emulated button, triggered in software from Tip and Ring events
	uint32_t extSwitchInLastTime;	// Time since EXT event triggering for state debouncing. 0 = no event
	uint8_t extSwitchInLastTrigger; // Last EXT event. 0 = none, 1 = Tip, 2 = Ring

	// A separate pin/port may be used for EXTI events to avoid NVIC conflicts
	// A/B may be changed depending on UART configuration
	// So T(tip) and R(ring) are used to avoid confusion
	GPIO_TypeDef* extiPortTip;
	GPIO_TypeDef* extiPortRing;
	uint16_t extiPinTip;
	uint16_t extiPinRing;

	uint8_t expReadingChanged[2];

	// Device link variables
	DeviceLinkState deviceLinkPortState;
	uint8_t aliveCheckSent;					// Flag to indicate that an alive check has been sent
	DeviceLinkPortDirection deviceLinkDirection;
} Flexiport;

typedef struct
{
	DeviceLinkState state;
	uint8_t aliveCheckReceived;
	DeviceLinkRole role;
	uint16_t deviceType;
	uint8_t numActiveDeviceLinkPorts;
	// Control settings
	uint8_t rxBankName;
	uint8_t rxBankNav;
	uint8_t rxMidiStream;
	uint8_t rxSwitchGroups;
} DeviceLink;

void flexi_initPort(Flexiport* flexiPort);
FlexiErrorState flexi_midiBegin(Flexiport* flexiport);
FlexiErrorState flexi_setModeMidiOut(Flexiport* flexiport, uint8_t midiType);
FlexiErrorState flexi_setModeUnassigned(Flexiport* flexiPort);
FlexiErrorState flexi_setModeExpIn(Flexiport* flexiPort, FlexiportMode expMode);
FlexiErrorState flexi_setModeDualSwitchOut(Flexiport* flexiPort);
FlexiErrorState flexi_setModeDualVoltageOut(Flexiport* flexiport);
FlexiErrorState flexi_setModeDualTapTempoOut(Flexiport* flexiPort);
FlexiErrorState flexi_setModeDualSwitchIn(Flexiport* flexiPort);
FlexiErrorState flexi_setModePulseOut(Flexiport* flexiport);
FlexiErrorState flexi_setModeDeviceLinkMaster(Flexiport* flexiPort);
FlexiErrorState flexi_setModeDeviceLinkSlave(Flexiport* flexiport);
FlexiErrorState flexi_setModeFavSwitchOut(Flexiport* flexiport);

void flexi_initDeviceLink(DeviceLink* deviceLink);
FlexiErrorState flexi_masterAssignNetwork(Flexiport* flexiport, DeviceLink* deviceLink);
FlexiErrorState flexi_uartRxHandler(Flexiport* flexiport, DeviceLink* deviceLink);
FlexiErrorState flexi_deviceLinkStartListening(Flexiport* flexiport);
FlexiErrorState flexi_deviceLinkAliveCheck(Flexiport* flexiport, DeviceLink* deviceLink);
FlexiErrorState flexi_deviceLinkSendBankUp(Flexiport* flexiport, DeviceLink* deviceLink, char* bankName, uint8_t bankNameLen);
FlexiErrorState flexi_deviceLinkSendBankDown(Flexiport* flexiport, DeviceLink* deviceLink, char* bankName, uint8_t bankNameLen);
FlexiErrorState flexi_deviceLinkSendGoToBank(Flexiport* flexiport, DeviceLink* deviceLink, uint8_t bank, char* bankName, uint8_t bankNameLen);
FlexiErrorState flexi_deviceLinkSendSwitchGroupEvent(Flexiport* flexiport, DeviceLink* deviceLink,
																											uint8_t group, uint8_t switchEvent);
FlexiErrorState flexi_rxUartHandler(Flexiport* flexiport, uint16_t size);
FlexiErrorState flexi_deviceLinkSendAliveResponse(Flexiport* flexiport, DeviceLink* deviceLink);
void flexi_deviceLinkIncrementAliveCheck(Flexiport* flexiport, DeviceLink* deviceLink);

FlexiErrorState flexi_checkPorts(Flexiport* flexiPort);
FlexiErrorState flexi_pollAdcConversion(Flexiport* flexiPort);
void flexi_checkAdcConversion(Flexiport* flexiPort);

// Switch control functions
FlexiErrorState flexi_setTipVSwitch(Flexiport* flexiPort, FlexiSwitchState state);
FlexiErrorState flexi_setRingVSwitch(Flexiport* flexiPort, FlexiSwitchState state);
FlexiErrorState flexi_setTipSleeveSwitch(Flexiport* flexiPort, FlexiSwitchState state);
FlexiErrorState flexi_setRingSleeveSwitch(Flexiport* flexiPort, FlexiSwitchState state);
FlexiErrorState flexi_setSleeveGroundSwitch(Flexiport* flexiport, FlexiSwitchState state);

void flexi_extiGpioCallback(GPIO_TypeDef* port, uint16_t pin, Flexiport* flexiport);
void flexi_triggerPoll(Flexiport* flexiport);
void flexi_filterAdcReadings(Flexiport* flexiport);
FlexiErrorState flexi_setHoldTimer(Flexiport* flexiport, TIM_HandleTypeDef *timHandle, uint16_t time);
void flexi_holdTimerElapsed(Flexiport* flexiport);

void flexi_uartErrorHandler(Flexiport* flexiport);
#endif /* FLEXIPORT_H_ */
