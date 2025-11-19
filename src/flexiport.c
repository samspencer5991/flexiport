/*
 * flexiport.c
 *
 *  Created on: 26 Apr 2020
 *      Author: samspencer
 */

#include "flexiport.h"
#include "midi.h"
#include "stdlib.h"
#include <string.h>

#define CLEAR                       0

#define SWAP_UART_PINS              1
#define NO_SWAP_UART_PINS           0

#define TYPEA_DATA_WIRING           0
#define TYPEB_DATA_WIRING           1
#define TIP_DATA_WIRING             2
#define RING_DATA_WIRING            3

#define ADC_TIMEOUT                 5

#define UART_STANDARD_SPEED         0
#define UART_HIGH_SPEED             1

#define DEVICE_LINK_REQUEST_TIMEOUT 1500

#define NEW_FLEXI_DEBOUNCE_TIME     500

#define MIDI_RX_BUF_SIZE            256

#define MIDI_CLOCK_UNASSIGNED       -1

#define FALSE                       0
#define TRUE                        1

// Peripheral Init and De-Init private functions
// FlexiErrorState flexi_gpioOutputInit(Flexiport* flexiport);
FlexiErrorState flexi_gpioInputExtiInit(Flexiport *flexiport);
FlexiErrorState flexi_uartInit(Flexiport *flexiport, uint8_t swapPins, uint8_t speed);
FlexiErrorState flexi_adcDualInit(Flexiport *flexiport);
FlexiErrorState flexi_gpioHighZInit(Flexiport *flexiport);

void flexi_configureGpioClock(GPIO_TypeDef *port);
void flexi_configureUartAltFunction(GPIO_InitTypeDef *GPIO_InitStruct, Flexiport *flexiport);

// Channel switch mode functions
void flexi_setPortSwitchesIdle(Flexiport *flexiport);
void flexi_setPortSwitchesDeviceLink(Flexiport *flexiport);
void flexi_setPortSwitchesMidiOut(Flexiport *flexiport, uint8_t dataWiring);
void flexi_setPortSwitchesMidiIn(Flexiport *flexiport, uint8_t dataWiring);
void flexi_setPortSwitchesExpressionIn(Flexiport *flexiport, FlexiportMode expMode);
void flexi_setPortSwitchesSwitchIn(Flexiport *flexiport);
void flexi_setPortSwitchesPulseOut(Flexiport *flexiport);
void flexi_setPortSwitchesSwitchOut(Flexiport *flexiport);
void flexi_setPortSwitchesFavSwitchOut(Flexiport *flexiport);

uint16_t holdTime;

//--------------- Config & Utility ---------------//
void flexi_initPort(Flexiport *flexiport)
{
    flexiport->config->mode = FlexiUnassigned;
    // Set the switches to idle mode for safety
    flexi_setPortSwitchesIdle(flexiport);
    // If a connector is already inserted
    if(HAL_GPIO_ReadPin(flexiport->sensePort, flexiport->sensePin) == GPIO_PIN_SET)
    {
        flexiport->status = FlexiConnected;
    }
    else
    {
        flexiport->status = FlexiRemoved;
    }

    // Configure default values
    for(uint8_t i = 0; i < 2; i++)
    {
        for(uint8_t j = 0; j < NUM_FLEXI_ADC_BUF_SAMPLES; j++)
        {
            flexiport->adcReadingBuf[i][j] = 0;
        }
        flexiport->filteredReadings[i] = 0;
    }
    flexiport->adcReadIndex = 0;
    flexiport->config->midiClock = MIDI_CLOCK_UNASSIGNED;
    flexiport->expReadingChanged[0] = FALSE;
    flexiport->expReadingChanged[1] = FALSE;

    buttons_Init(&flexiport->extSwitchInTip);
    buttons_Init(&flexiport->extSwitchInRing);
    buttons_Init(&flexiport->extSwitchInTipRing);

    flexiport->aliveCheckSent = FALSE;
    flexiport->timerTriggered = CLEAR;
    flexiport->auxHoldTim = NULL;

    flexiport->deviceLinkDirection = DeviceLinkUpstream;
}

FlexiErrorState flexi_checkPorts(Flexiport *flexiport)
{
    static uint8_t  insertDetected = 0;
    static uint32_t detectTime = 0;
    // Read the flexiport's insertion pins
    // Check if a connector has been inserted
    if(flexiport->status == FlexiRemoved &&
       HAL_GPIO_ReadPin(flexiport->sensePort, flexiport->sensePin) == GPIO_PIN_SET)
    {
        // Use some basic debouncing to ensure steady state before prompting for config/deconfig
        // For the first detection
        if(!insertDetected)
        {
            detectTime = HAL_GetTick();
            insertDetected = 1;
        }
        else if((HAL_GetTick() - detectTime) > NEW_FLEXI_DEBOUNCE_TIME)
        {
            insertDetected = 0;
            detectTime = HAL_GetTick();
            // Re-check the pin state to make sure it's still plugged in
            if(HAL_GPIO_ReadPin(flexiport->sensePort, flexiport->sensePin) == GPIO_PIN_SET)
            {
                if(flexiport->config->mode == FlexiUnassigned)
                {
                    flexiport->status = FlexiConnected;
                    return FlexiNewConnection;
                }
                else if(flexiport->config->mode == FlexiMidiOutTypeA || flexiport->config->mode == FlexiMidiOutTypeB)
                {
                    flexi_setPortSwitchesMidiOut(flexiport, TIP_DATA_WIRING);
                }
                else if(flexiport->config->mode == FlexiDualExpressionIn)
                {
                    flexi_setPortSwitchesExpressionIn(flexiport, FlexiDualExpressionIn);
                }
                else if(flexiport->config->mode == FlexiSingleExpressionIn)
                {
                    flexi_setPortSwitchesExpressionIn(flexiport, FlexiSingleExpressionIn);
                }
                else if(flexiport->config->mode == FlexiSwitchIn)
                {
                    flexi_setPortSwitchesSwitchIn(flexiport);
                }
                else if(flexiport->config->mode == FlexiSwitchOut || flexiport->config->mode == FlexiTapTempoOut)
                {
                    flexi_setPortSwitchesSwitchIn(flexiport);
                }
                flexiport->status = FlexiConnected;
            }
        }
    }
    // Check for newly removed connectors from the ports
    else if(HAL_GPIO_ReadPin(flexiport->sensePort, flexiport->sensePin) == GPIO_PIN_RESET && flexiport->status == FlexiConnected)
    {
        flexi_setPortSwitchesIdle(flexiport);
        flexiport->status = FlexiRemoved;
        return FlexiRemovedConnection;
    }
    return FlexiOk;
}

void flexi_filterAdcReadings(Flexiport *flexiport)
{
    for(int i = 0; i < 2; i++)
    {
        if(i == 0 || flexiport->config->mode == FlexiDualExpressionIn)
        {
            flexiport->adcReadingBuf[i][flexiport->adcReadIndex] = flexiport->adcRawReadings[i];
            // uint16_t lastReading = flexiport->filteredReadings[i];
            uint8_t  lastReading8Bit = flexiport->adcLastChangedReadings[i] / 32;
            uint32_t total = 0;
            for(int j = 0; j < NUM_FLEXI_ADC_BUF_SAMPLES; j++)
            {
                total += flexiport->adcReadingBuf[i][j];
            }
            flexiport->filteredReadings[i] = total / NUM_FLEXI_ADC_BUF_SAMPLES;
            uint8_t newReading8Bit = flexiport->filteredReadings[i] / 32;
            // Check if the new reading is different to the previous one and mark it for application use
            // Use the optional hysteresis definition to help reduce jitter
            int readingDelta = flexiport->filteredReadings[i] - flexiport->adcLastChangedReadings[i];
            if(((readingDelta > FLEXI_ADC_HYSTERESIS_BOUNDRY) || (readingDelta < (-FLEXI_ADC_HYSTERESIS_BOUNDRY))) && newReading8Bit != lastReading8Bit)
            {
                flexiport->expReadingChanged[i] = TRUE;
                flexiport->adcLastChangedReadings[i] = flexiport->filteredReadings[i];
            }
            else
            {
                flexiport->expReadingChanged[i] = FALSE;
            }
        }
    }

    if(flexiport->adcReadIndex == (NUM_FLEXI_ADC_BUF_SAMPLES - 1))
    {
        flexiport->adcReadIndex = 0;
    }
    else
    {
        flexiport->adcReadIndex++;
    }
}

FlexiErrorState flexi_setHoldTimer(Flexiport *flexiport, TIM_HandleTypeDef *timHandle, uint16_t time)
{
    // Check parameters
    if(timHandle == NULL)
    {
        return FlexiParamError;
    }
    flexiport->auxHoldTim = timHandle;

    /* Calculate prescaler and period values based on CPU frequency
     * The prescaler is set so that the timer resolution is equal to a millisecond
     * This allows for easy setting of the Period directly in milliseconds
     */
    uint32_t cpuFreq = HAL_RCC_GetSysClockFreq();
    flexiport->auxHoldTim->Init.Prescaler = cpuFreq / 10000; // This assumes the clock is in the MHz range
    flexiport->auxHoldTim->Init.Period = time * 10;

    // Update timer instance with new timing values and clear the interrupt flag to prevent initial mis-fire (bug found previously)
    if(HAL_TIM_Base_Init(flexiport->auxHoldTim) != HAL_OK)
    {
        return FlexiHalError;
    }
    __HAL_TIM_CLEAR_FLAG(flexiport->auxHoldTim, TIM_IT_UPDATE);
    return FlexiOk;
}

void flexi_pollSwitchInputs(Flexiport *flexiport)
{
    if(flexiport->config->mode != FlexiSwitchIn)
        return;

    uint32_t currentTime = HAL_GetTick();
    if((currentTime - flexiport->lastPollTime) < 5)
        return;
    // Read current states
    GPIO_PinState currentTipState = HAL_GPIO_ReadPin(flexiport->portA, flexiport->pinA);
    GPIO_PinState currentRingState = HAL_GPIO_ReadPin(flexiport->portB, flexiport->pinB);

    // New tip+ring event
    if(flexiport->lastPinStateTip != currentTipState && flexiport->lastPinStateRing != currentRingState)
    {
        // Press events
        // New tip+ring press
        if(currentTipState == GPIO_PIN_RESET && currentRingState == GPIO_PIN_RESET)
        {
            buttons_ExtiGpioCallback(&flexiport->extSwitchInTipRing, ButtonEmulatePress);
        }
        // New Tip+Ring release
        else if((flexiport->extSwitchInTipRing.lastState == Pressed || flexiport->extSwitchInTipRing.lastState == Held ||
				flexiport->extSwitchInTipRing.lastState == DoublePressed) &&
                (currentTipState == GPIO_PIN_SET && currentRingState == GPIO_PIN_SET))
        {
            buttons_ExtiGpioCallback(&flexiport->extSwitchInTipRing, ButtonEmulateRelease);
        }
    }
    // New tip event
    else if(flexiport->lastPinStateTip != currentTipState)
    {
		// New tip press
		if(currentTipState == GPIO_PIN_RESET)
		{
			buttons_ExtiGpioCallback(&flexiport->extSwitchInTip, ButtonEmulateNone);
		}
		// New tip release
		else if(currentTipState == GPIO_PIN_SET)
		{
			buttons_ExtiGpioCallback(&flexiport->extSwitchInTip, ButtonEmulateNone);
		}
    }
    // New ring event
    else if(flexiport->lastPinStateRing != currentRingState)
    {
		// New ring press
		if(currentTipState == GPIO_PIN_RESET)
		{
			buttons_ExtiGpioCallback(&flexiport->extSwitchInRing, ButtonEmulateNone);
		}
		// New ring release
		else if(currentTipState == GPIO_PIN_SET)
		{
			buttons_ExtiGpioCallback(&flexiport->extSwitchInRing, ButtonEmulateNone);
		}
    }
    flexiport->lastPinStateTip = currentTipState;
	flexiport->lastPinStateRing = currentRingState;

	flexiport->lastPollTime = currentTime;
}

//--------------- Device Link ---------------//
void flexi_initDeviceLink(DeviceLink *deviceLink)
{
    deviceLink->state = DeviceLinkNotActive;
    deviceLink->aliveCheckReceived = FALSE;
    // deviceLink->rxBankName = FALSE;
    // deviceLink->rxBankNav = FALSE;
    // deviceLink->rxMidiStream = FALSE;
    // deviceLink->rxSwitchGroups = FALSE;
}

// Send an alive check message to the attached slave device
FlexiErrorState flexi_deviceLinkAliveCheck(Flexiport *flexiport, DeviceLink *deviceLink)
{
    if(deviceLink->role != DeviceLinkMasterRole || flexiport->config->mode != FlexiDeviceLink)
    {
        return FlexiParamError;
    }
    midi_Send(flexiport->midiHandle, ActiveSensing, 0, 0, 0);
    flexiport->aliveCheckSent = TRUE;
    return FlexiOk;
}

// Sends a bank up device link message. If bankName is NULL, message is passed as a slave (no bank name)
FlexiErrorState flexi_deviceLinkSendBankUp(Flexiport *flexiport, DeviceLink *deviceLink, char *bankName, uint8_t bankNameLen)
{
    if(deviceLink->role == DeviceLinkMasterRole && flexiport->config->mode == FlexiDeviceLink)
    {
        uint8_t bankUpMessage[2 + bankNameLen];
        bankUpMessage[0] = DEVICE_LINK_BANK_UP_CMD;
        bankUpMessage[1] = DEVICE_LINK_BANK_NAME_CMD;
        for(uint8_t i = 0; i < bankNameLen; i++)
        {
            bankUpMessage[i + 2] = bankName[i];
        }
        midi_SendSysEx(flexiport->midiHandle, bankUpMessage, 2 + bankNameLen, sysExId);
        return FlexiOk;
    }
    else if(deviceLink->role == DeviceLinkSlaveRole && flexiport->config->mode == FlexiDeviceLink)
    {
        uint8_t bankUpByte = DEVICE_LINK_BANK_UP_CMD;

        midi_SendSysEx(flexiport->midiHandle, &bankUpByte, 1, sysExId);
        return FlexiOk;
    }
    return FlexiParamError;
}

// Sends a bank down device link message. If bankName is NULL, message is passed as a slave (no bank name)
FlexiErrorState flexi_deviceLinkSendBankDown(Flexiport *flexiport, DeviceLink *deviceLink, char *bankName, uint8_t bankNameLen)
{
    if(deviceLink->role == DeviceLinkMasterRole)
    {
        uint8_t bankDownMessage[2 + bankNameLen];
        bankDownMessage[0] = DEVICE_LINK_BANK_DOWN_CMD;
        bankDownMessage[1] = DEVICE_LINK_BANK_NAME_CMD;
        for(uint8_t i = 0; i < bankNameLen; i++)
        {
            bankDownMessage[i + 2] = bankName[i];
        }
        midi_SendSysEx(flexiport->midiHandle, bankDownMessage, 2 + bankNameLen, sysExId);
        return FlexiOk;
    }
    else if(deviceLink->role == DeviceLinkSlaveRole)
    {
        uint8_t bankDownByte = DEVICE_LINK_BANK_DOWN_CMD;

        midi_SendSysEx(flexiport->midiHandle, &bankDownByte, 1, sysExId);
        return FlexiOk;
    }
    return FlexiParamError;
}

// Sends a go to bank device link message. If bankName is NULL, message is passed as a slave (no bank name)
FlexiErrorState flexi_deviceLinkSendGoToBank(Flexiport *flexiport, DeviceLink *deviceLink, uint8_t bank, char *bankName, uint8_t bankNameLen)
{
    if(bank > 127)
    {
        return FlexiParamError;
    }
    if(deviceLink->role == DeviceLinkMasterRole)
    {
        uint8_t goToBankMessage[3 + bankNameLen];
        goToBankMessage[0] = DEVICE_LINK_GOTO_BANK_CMD;
        goToBankMessage[1] = bank;
        goToBankMessage[2] = DEVICE_LINK_BANK_NAME_CMD;
        for(uint8_t i = 0; i < bankNameLen; i++)
        {
            goToBankMessage[i + 3] = bankName[i];
        }
        midi_SendSysEx(flexiport->midiHandle, goToBankMessage, 3 + bankNameLen, sysExId);
        return FlexiOk;
    }
    else if(deviceLink->role == DeviceLinkSlaveRole)
    {
        uint8_t goToBankMessage[2];
        goToBankMessage[0] = DEVICE_LINK_GOTO_BANK_CMD;
        goToBankMessage[1] = bank;

        midi_SendSysEx(flexiport->midiHandle, goToBankMessage, 2, sysExId);
        return FlexiOk;
    }
    return FlexiParamError;
}

// Sends the current bank name (master only)
FlexiErrorState flexi_deviceLinkSendBankName(Flexiport *flexiport, DeviceLink *deviceLink, char *bankName, uint8_t bankNameLen)
{
    if(deviceLink->role == DeviceLinkMasterRole)
    {
        uint8_t bankNameMessage[1 + bankNameLen];
        bankNameMessage[0] = DeviceLinkBankName;
        for(uint8_t i = 0; i < bankNameLen; i++)
        {
            bankNameMessage[i + 1] = bankName[i];
        }
        midi_SendSysEx(flexiport->midiHandle, bankNameMessage, 1 + bankNameLen, sysExId);
        return FlexiOk;
    }
    return FlexiParamError;
}

FlexiErrorState flexi_deviceLinkSendBankNameRequest(Flexiport *flexiport, DeviceLink *deviceLink)
{
    if(deviceLink->role == DeviceLinkSlaveRole)
    {
        uint8_t bankNameRequestMessage = DeviceLinkBankNameRequest;
        midi_SendSysEx(flexiport->midiHandle, &bankNameRequestMessage, 1, sysExId);
        return FlexiOk;
    }
    return FlexiParamError;
}

FlexiErrorState flexi_deviceLinkSendSwitchGroupEvent(Flexiport *flexiport, DeviceLink *deviceLink,
                                                     uint8_t group, uint8_t switchEvent)
{
    uint8_t packet[3];
    packet[0] = DEVICE_LINK_SWITCH_GROUP_CMD;
    packet[1] = group;
    packet[2] = switchEvent;
    midi_SendSysEx(flexiport->midiHandle, packet, 3, sysExId);
    return FlexiOk;
}

//--------------- Mode Configuration ---------------//
FlexiErrorState flexi_setModeUnassigned(Flexiport *flexiport)
{
    // If a special hardware mode has been assigned to the port, perform de-init
    flexiport->config->mode = FlexiUnassigned;
    flexi_setPortSwitchesIdle(flexiport);
    return FlexiOk;
}

/* MIDI OUT
 *
 * TIP 		- 3V switch closed (ON)
 * 				- Sleeve switch open (OFF)
 * RING		- 3V and sleeve switches open (OFF)
 * SLEEVE	- GND switch closed (ON)
 * PINA (TIP)	- Input with pullup (to protect pin and conserve power)
 * PINB (RING)- UART TX
 *
 * TODO: add support for non-standard MIDI pin configurations
 */
FlexiErrorState flexi_setModeMidiOut(Flexiport *flexiport, uint8_t midiType)
{
    // Type A
    if(midiType == MIDI_TYPE_A)
    {
        flexiport->config->mode = FlexiMidiOutTypeA;
        // Initialise the UART peripheral
        flexi_uartInit(flexiport, SWAP_UART_PINS, UART_STANDARD_SPEED);
        // Once all peripherals and pins are configured, route the pins to the connector
        flexi_setPortSwitchesMidiOut(flexiport, TYPEA_DATA_WIRING);
    }

    // Type B
    else if(midiType == MIDI_TYPE_B)
    {
        flexiport->config->mode = FlexiMidiOutTypeB;
        // Initialise the UART peripheral
        flexi_uartInit(flexiport, NO_SWAP_UART_PINS, UART_STANDARD_SPEED);
        // Once all peripherals and pins are configured, route the pins to the connector
        flexi_setPortSwitchesMidiOut(flexiport, TYPEB_DATA_WIRING);
    }

    // Tip active
    else if(midiType == MIDI_TIP)
    {
        flexiport->config->mode = FlexiMidiOutTip;
        // Initialise the UART peripheral
        flexi_uartInit(flexiport, SWAP_UART_PINS, UART_STANDARD_SPEED);
        // Once all peripherals and pins are configured, route the pins to the connector
        flexi_setPortSwitchesMidiOut(flexiport, TIP_DATA_WIRING);
    }

    // Ring active
    else if(midiType == MIDI_RING)
    {
        flexiport->config->mode = FlexiMidiOutRing;
        // Initialise the UART peripheral
        flexi_uartInit(flexiport, NO_SWAP_UART_PINS, UART_STANDARD_SPEED);
        // Once all peripherals and pins are configured, route the pins to the connector
        flexi_setPortSwitchesMidiOut(flexiport, RING_DATA_WIRING);
    }
    flexiport->midiHandle->direction = MidiOutOnly;
    return FlexiOk;
}

FlexiErrorState flexi_setModeMidiIn(Flexiport *flexiport, uint8_t midiType)
{
    // Type A
    if(midiType == MIDI_TYPE_A)
    {
        flexiport->config->mode = FlexiMidiInTypeA;
        flexi_uartInit(flexiport, NO_SWAP_UART_PINS, UART_STANDARD_SPEED);
        flexi_setPortSwitchesMidiIn(flexiport, TYPEA_DATA_WIRING);
    }

    // Type B
    else if(midiType == MIDI_TYPE_B)
    {
    }
	return FlexiOk;
}

/* DUAL SWITCH IN
 *
 * TIP 		- 3V and sleeve switches open (OFF)
 * RING		- 3V and sleeve switches open (OFF)
 * SLEEVE	- GND switch closed (ON)
 * PINA (TIP)	- EXTI
 * PINB (RING)- EXTI
 */
FlexiErrorState flexi_setModeDualSwitchIn(Flexiport *flexiport)
{
    flexiport->config->mode = FlexiSwitchIn;
    // Initialise the EXTI configuration
    flexi_gpioInputExtiInit(flexiport);
    // Once all peripherals and pins are configured, route the pins to the connector
    flexi_setPortSwitchesSwitchIn(flexiport);
    return FlexiOk;
}

/* PULSE MODE OUT
 *
 * TIP 		- Sleeve switch open (OFF)
 * 				- 3V switch toggled (ON/OFF)
 * RING		- Sleeve switch open (OFF)
 * 				- 3V switch open (OFF)
 * SLEEVE	- GND switch closed (ON)
 * PINA (TIP)	- Input with pullup (to protect pin and conserve power)
 * PINB (RING)- Input with pullup (to protect pin and conserve power)
 */
FlexiErrorState flexi_setModePulseOut(Flexiport *flexiport)
{
    flexiport->config->mode = FlexiPulseOut;
    // Initialise the GPIO pins
    flexi_gpioHighZInit(flexiport);
    // Once all peripherals and pins are configured, route the pins to the connector
    flexi_setPortSwitchesPulseOut(flexiport);
    return FlexiOk;
}

/* DUAL SWITCH OUT
 *
 * TIP 		- Sleeve switch toggled (ON/OFF)
 * 				- 3V switch open (OFF)
 * RING		- Sleeve switch toggled (ON/OFF)
 * 				- 3V switch open (OFF)
 * SLEEVE	- GND switch open (OFF)
 * PINA (TIP)	- Input with pullup (to protect pin and conserve power)
 * PINB (RING)- Input with pullup (to protect pin and conserve power)
 */
FlexiErrorState flexi_setModeDualSwitchOut(Flexiport *flexiport)
{
    flexiport->config->mode = FlexiSwitchOut;
    // Initialise the GPIO pins
    flexi_gpioHighZInit(flexiport);
    // Once all peripherals and pins are configured, route the pins to the connector
    flexi_setPortSwitchesSwitchOut(flexiport);
    return FlexiOk;
}

FlexiErrorState flexi_setModeDualVoltageOut(Flexiport *flexiport)
{
    flexiport->config->mode = FlexiVoltageOut;
    // Initialise the GPIO pins
    flexi_gpioHighZInit(flexiport);
    // Once all peripherals and pins are configured, route the pins to the connector
    flexi_setPortSwitchesSwitchOut(flexiport);
    return FlexiOk;
}

/* DUAL SWITCH OUT
 *
 * TIP 		- Sleeve switch toggled (ON/OFF)
 * 				- 3V switch open (OFF)
 * RING		- Sleeve switch toggled (ON/OFF)
 * 				- 3V switch open (OFF)
 * SLEEVE	- GND switch closed (OFF)
 * PINA (TIP)	- Input with pullup (to protect pin and conserve power)
 * PINB (RING)- Input with pullup (to protect pin and conserve power)
 */
FlexiErrorState flexi_setModeDualTapTempoOut(Flexiport *flexiport)
{
    flexiport->config->mode = FlexiTapTempoOut;
    // Initialise the GPIO pins
    flexi_gpioHighZInit(flexiport);
    // Once all peripherals and pins are configured, route the pins to the connector
    flexi_setPortSwitchesSwitchOut(flexiport);
    return FlexiOk;
}

/* EXP IN
 *
 * TIP 		- 3V and sleeve switches open (OFF)
 * RING		- 3V switch closed (ON)
 * 				- Sleeve switch open (OFF)
 * SLEEVE	- GND switch closed (ON)
 * PINA (TIP)	- Analog in
 * PINB (RING)- Input with pullup (to protect pin and conserve power)
 */
FlexiErrorState flexi_setModeExpIn(Flexiport *flexiport, FlexiportMode expMode)
{

    if(expMode != FlexiDualExpressionIn && expMode != FlexiSingleExpressionIn)
    {
        return FlexiModeError;
    }

    // Set both A and B pins to high z state.
    // This accounts for the flexiport having a separate pin for the ADC
    // flexi_gpioHighZInit(flexiport);
    // Initialise the ADC peripheral for both A and B channels of the flexiport
    flexi_adcDualInit(flexiport);

#if defined(STM32G491xx) || defined(STM32G473xx)
    if(HAL_ADCEx_Calibration_Start(flexiport->hadc, ADC_SINGLE_ENDED) != HAL_OK)
    {
        return FlexiHalError;
    }
#endif

#if defined(STM32G0B1xx)
    if(HAL_ADCEx_Calibration_Start(flexiport->hadc) != HAL_OK)
    {
        return FlexiHalError;
    }
#endif

    // Once all peripherals and pins are configured, route the pins to the connector
    flexi_setPortSwitchesExpressionIn(flexiport, expMode);
    if(expMode == FlexiSingleExpressionIn)
    {
        flexiport->config->mode = FlexiSingleExpressionIn;
    }
    else if(expMode == FlexiDualExpressionIn)
    {
        flexiport->config->mode = FlexiDualExpressionIn;
    }
    return FlexiOk;
}

/* DEVICE LINK
 *
 * TIP 		- 3V and sleeve switches open (OFF)
 * RING		- 3V and sleeve switches open (OFF)
 * 				- Sleeve switch closed (ON)
 * SLEEVE	- GND switch closed (ON)
 * PINA (TIP)	-
 * PINB (RING)-
 */
FlexiErrorState flexi_setModeDeviceLinkMaster(Flexiport *flexiport)
{
    flexiport->config->mode = FlexiDeviceLink;
    flexiport->midiHandle->direction = MidiFull;
    // Initialise the UART peripheral
    flexi_uartInit(flexiport, SWAP_UART_PINS, UART_HIGH_SPEED);

    // Once all peripherals and pins are configured, route the pins to the connector
    flexi_setPortSwitchesDeviceLink(flexiport);

    return FlexiOk;
}

/* DEVICE LINK
 *
 * TIP 		- 3V and sleeve switches open (OFF)
 * RING		- 3V and sleeve switches open (OFF)
 * 				- Sleeve switch closed (ON)
 * SLEEVE	- GND switch closed (ON)
 * PINA (TIP)	- A
 * PINB (RING)-
 */
FlexiErrorState flexi_setModeDeviceLinkSlave(Flexiport *flexiport)
{
    flexiport->config->mode = FlexiDeviceLink;
    flexiport->midiHandle->direction = MidiFull;
    // Initialise the UART peripheral
    flexi_uartInit(flexiport, NO_SWAP_UART_PINS, UART_HIGH_SPEED);

    // Once all peripherals and pins are configured, route the pins to the connector
    flexi_setPortSwitchesDeviceLink(flexiport);
    return FlexiOk;
}

/* PULSE MODE OUT
 *
 * TIP 		- Sleeve switch open (OFF)
 * 				- GND switch toggled (ON/OFF)
 * RING		- Sleeve switch open (OFF)
 * 				- 3V switch open (OFF)
 * SLEEVE	- GND switch closed (ON)
 * PINA (TIP)	- Input with pullup (to protect pin and conserve power)
 * PINB (RING)- Input with pullup (to protect pin and conserve power)
 */
FlexiErrorState flexi_setModeFavSwitchOut(Flexiport *flexiport)
{
    flexiport->config->mode = FlexiFavSwitchOut;
    // Initialise the GPIO pins
    flexi_gpioHighZInit(flexiport);
    // Once all peripherals and pins are configured, route the pins to the connector
    flexi_setPortSwitchesFavSwitchOut(flexiport);
    return FlexiOk;
}

//--------------- Switch Control ---------------//
FlexiErrorState flexi_setTipVSwitch(Flexiport *flexiport, FlexiSwitchState state)
{
    // Ensure that the action is available for the flexiport type
    if(flexiport->config->mode == FlexiPulseOut || flexiport->config->mode == FlexiVoltageOut)
    {
        // Close the switch
        if(state == FlexiCloseSwitch)
        {
            HAL_GPIO_WritePin(flexiport->vAPort, flexiport->vAPin, GPIO_PIN_SET);
            return FlexiOk;
        }

        // Open the switch
        else
        {
            HAL_GPIO_WritePin(flexiport->vAPort, flexiport->vAPin, GPIO_PIN_RESET);
            return FlexiOk;
        }
    }
    return FlexiParamError;
}

FlexiErrorState flexi_setRingVSwitch(Flexiport *flexiport, FlexiSwitchState state)
{
    // Ensure that the action is available for the flexiport type
    if(flexiport->config->mode == FlexiPulseOut || flexiport->config->mode == FlexiVoltageOut)
    {
        // Close the switch
        if(state == FlexiCloseSwitch)
        {
            HAL_GPIO_WritePin(flexiport->vBPort, flexiport->vBPin, GPIO_PIN_SET);
            return FlexiOk;
        }

        // Open the switch
        else
        {
            HAL_GPIO_WritePin(flexiport->vBPort, flexiport->vBPin, GPIO_PIN_RESET);
            return FlexiOk;
        }
    }
    return FlexiParamError;
}

FlexiErrorState flexi_setTipSleeveSwitch(Flexiport *flexiport, FlexiSwitchState state)
{
    // Ensure that the action is available for the flexiport type
    if(flexiport->config->mode == FlexiSwitchOut || flexiport->config->mode == FlexiTapTempoOut)
    {
        // Close the switch
        if(state == FlexiCloseSwitch)
        {
            HAL_GPIO_WritePin(flexiport->sleeveAPort, flexiport->sleeveAPin, GPIO_PIN_SET);
            return FlexiOk;
        }

        // Open the switch
        else
        {
            HAL_GPIO_WritePin(flexiport->sleeveAPort, flexiport->sleeveAPin, GPIO_PIN_RESET);
            return FlexiOk;
        }
    }
    return FlexiParamError;
}

FlexiErrorState flexi_setRingSleeveSwitch(Flexiport *flexiport, FlexiSwitchState state)
{
    // Ensure that the action is available for the flexiport type
    if(flexiport->config->mode == FlexiSwitchOut || flexiport->config->mode == FlexiTapTempoOut)
    {
        // Close the switch
        if(state == FlexiCloseSwitch)
        {
            HAL_GPIO_WritePin(flexiport->sleeveBPort, flexiport->sleeveBPin, GPIO_PIN_SET);
            return FlexiOk;
        }

        // Open the switch
        else
        {
            HAL_GPIO_WritePin(flexiport->sleeveBPort, flexiport->sleeveBPin, GPIO_PIN_RESET);
            return FlexiOk;
        }
    }
    return FlexiParamError;
}

FlexiErrorState flexi_setSleeveGroundSwitch(Flexiport *flexiport, FlexiSwitchState state)
{
    // Ensure that the action is available for the flexiport type
    if(flexiport->config->mode == FlexiPulseOut)
    {
        // Close the switch
        if(state == FlexiCloseSwitch)
        {
            HAL_GPIO_WritePin(flexiport->gndSleevePort, flexiport->gndSleevePin, GPIO_PIN_SET);
            return FlexiOk;
        }

        // Open the switch
        else
        {
            HAL_GPIO_WritePin(flexiport->gndSleevePort, flexiport->gndSleevePin, GPIO_PIN_RESET);
            return FlexiOk;
        }
    }
    return FlexiParamError;
}

//--------------- Signal Switching ---------------//
void flexi_setPortSwitchesIdle(Flexiport *flexiport)
{
    if(!flexiport->flexiportLite)
    {
        // Disconnect all the signal switches
        HAL_GPIO_WritePin(flexiport->sleeveAPort, flexiport->sleeveAPin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(flexiport->sleeveBPort, flexiport->sleeveBPin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(flexiport->vAPort, flexiport->vAPin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(flexiport->vBPort, flexiport->vBPin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(flexiport->gndSleevePort, flexiport->gndSleevePin, GPIO_PIN_SET);
    }
}

void flexi_setPortSwitchesMidiOut(Flexiport *flexiport, uint8_t dataWiring)
{
    if(!flexiport->flexiportLite)
    {
        HAL_GPIO_WritePin(flexiport->sleeveAPort, flexiport->sleeveAPin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(flexiport->sleeveBPort, flexiport->sleeveBPin, GPIO_PIN_RESET);
        if(dataWiring == TYPEA_DATA_WIRING)
        {
            HAL_GPIO_WritePin(flexiport->vAPort, flexiport->vAPin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(flexiport->vBPort, flexiport->vBPin, GPIO_PIN_SET);
        }
        else if(dataWiring == TYPEB_DATA_WIRING)
        {
            HAL_GPIO_WritePin(flexiport->vAPort, flexiport->vAPin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(flexiport->vBPort, flexiport->vBPin, GPIO_PIN_RESET);
        }
        else if(dataWiring == TIP_DATA_WIRING || dataWiring == RING_DATA_WIRING)
        {
            HAL_GPIO_WritePin(flexiport->vAPort, flexiport->vAPin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(flexiport->vBPort, flexiport->vBPin, GPIO_PIN_RESET);
        }
        HAL_GPIO_WritePin(flexiport->gndSleevePort, flexiport->gndSleevePin, GPIO_PIN_SET);
    }
    else
    {
        // Flexiport lite does not have external analog switches
        // All pin configuration is done in the HAL_UART_MspInit function (hardware_def.c)
    }
}

void flexi_setPortSwitchesMidiIn(Flexiport *flexiport, uint8_t dataWiring)
{
    if(!flexiport->flexiportLite)
    {
    }
    else
    {
        // Flexiport lite does not have external analog switches
        // All pin configuration is done in the HAL_UART_MspInit function (hardware_def.c)
    }
}

void flexi_setPortSwitchesDeviceLink(Flexiport *flexiport)
{
    if(!flexiport->flexiportLite)
    {
        HAL_GPIO_WritePin(flexiport->sleeveAPort, flexiport->sleeveAPin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(flexiport->sleeveBPort, flexiport->sleeveBPin, GPIO_PIN_RESET);

        HAL_GPIO_WritePin(flexiport->vAPort, flexiport->vAPin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(flexiport->vBPort, flexiport->vBPin, GPIO_PIN_RESET);

        HAL_GPIO_WritePin(flexiport->gndSleevePort, flexiport->gndSleevePin, GPIO_PIN_SET);
    }
}

void flexi_setPortSwitchesSwitchIn(Flexiport *flexiport)
{
    if(!flexiport->flexiportLite)
    {
        // Disconnect all the signal switches
        HAL_GPIO_WritePin(flexiport->sleeveAPort, flexiport->sleeveAPin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(flexiport->sleeveBPort, flexiport->sleeveBPin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(flexiport->vAPort, flexiport->vAPin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(flexiport->vBPort, flexiport->vBPin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(flexiport->gndSleevePort, flexiport->gndSleevePin, GPIO_PIN_SET);
    }
}

void flexi_setPortSwitchesSwitchOut(Flexiport *flexiport)
{
    if(!flexiport->flexiportLite)
    {
        // Disconnect the signal sleeve switches first to prevent shorts
        HAL_GPIO_WritePin(flexiport->sleeveAPort, flexiport->sleeveAPin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(flexiport->sleeveBPort, flexiport->sleeveBPin, GPIO_PIN_RESET);
        // Disconnect channel A and B from the microcontroller pins
        HAL_GPIO_WritePin(flexiport->vAPort, flexiport->vAPin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(flexiport->vBPort, flexiport->vBPin, GPIO_PIN_RESET);
        // Disconnect ground switch from the sleeve
        HAL_GPIO_WritePin(flexiport->gndSleevePort, flexiport->gndSleevePin, GPIO_PIN_SET);
    }
}

void flexi_setPortSwitchesFavSwitchOut(Flexiport *flexiport)
{
    if(!flexiport->flexiportLite)
    {
        // Disconnect the signal sleeve switches first to prevent shorts
        HAL_GPIO_WritePin(flexiport->sleeveAPort, flexiport->sleeveAPin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(flexiport->sleeveBPort, flexiport->sleeveBPin, GPIO_PIN_RESET);
        // Disconnect channel A and B from the microcontroller pins
        HAL_GPIO_WritePin(flexiport->vAPort, flexiport->vAPin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(flexiport->vBPort, flexiport->vBPin, GPIO_PIN_RESET);
        // Disconnect ground switch from the sleeve
        HAL_GPIO_WritePin(flexiport->gndSleevePort, flexiport->gndSleevePin, GPIO_PIN_SET);
    }
}

void flexi_setPortSwitchesPulseOut(Flexiport *flexiport)
{
    // Disconnect the signal sleeve switches first to prevent shorts
    if(!flexiport->flexiportLite)
    {
        HAL_GPIO_WritePin(flexiport->sleeveAPort, flexiport->sleeveAPin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(flexiport->sleeveBPort, flexiport->sleeveBPin, GPIO_PIN_RESET);
        // Disconnect channel A and B from the microcontroller pins
        HAL_GPIO_WritePin(flexiport->vAPort, flexiport->vAPin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(flexiport->vBPort, flexiport->vBPin, GPIO_PIN_RESET);
        // Connect ground switch to the sleeve
        HAL_GPIO_WritePin(flexiport->gndSleevePort, flexiport->gndSleevePin, GPIO_PIN_SET);
    }
}

void flexi_setPortSwitchesExpressionIn(Flexiport *flexiport, FlexiportMode expMode)
{
    // For a single passive expression pedal, the ring needs to be connected to a constant voltage
    // However for a dual expression pedal, the pedal outputs it's own voltage.
    if(!flexiport->flexiportLite)
    {
        HAL_GPIO_WritePin(flexiport->sleeveAPort, flexiport->sleeveAPin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(flexiport->sleeveBPort, flexiport->sleeveBPin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(flexiport->vAPort, flexiport->vAPin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(flexiport->gndSleevePort, flexiport->gndSleevePin, GPIO_PIN_SET);
        if(expMode == FlexiSingleExpressionIn)
        {
            HAL_GPIO_WritePin(flexiport->vBPort, flexiport->vBPin, GPIO_PIN_SET);
        }
        else if(expMode == FlexiDualExpressionIn)
        {
            HAL_GPIO_WritePin(flexiport->vBPort, flexiport->vBPin, GPIO_PIN_RESET);
        }
    }
    else
    {
        if(expMode == FlexiSingleExpressionIn)
        {
            HAL_GPIO_WritePin(flexiport->portB, flexiport->pinB, GPIO_PIN_SET);
        }
    }
}

//--------------- Peripheral Initialisation ---------------//
FlexiErrorState flexi_gpioOutputInit(Flexiport *flexiport)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    HAL_GPIO_WritePin(flexiport->portA, flexiport->pinA, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(flexiport->portB, flexiport->pinB, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = flexiport->pinA;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(flexiport->portA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = flexiport->pinB;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(flexiport->portB, &GPIO_InitStruct);
    return FlexiOk;
}

FlexiErrorState flexi_gpioInputExtiInit(Flexiport *flexiport)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = flexiport->pinA;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(flexiport->portA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = flexiport->pinB;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(flexiport->portB, &GPIO_InitStruct);
    return FlexiOk;
}

FlexiErrorState flexi_gpioHighZInit(Flexiport *flexiport)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = flexiport->pinA;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(flexiport->portA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = flexiport->pinB;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(flexiport->portB, &GPIO_InitStruct);

    return FlexiOk;
}

FlexiErrorState flexi_adcDualInit(Flexiport *flexiport)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = flexiport->adcPinTip;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(flexiport->adcPortTip, &GPIO_InitStruct);

    if(flexiport->config->mode == FlexiSingleExpressionIn && flexiport->flexiportLite)
    {
        // For a single expression pedal on a flexiport lite, the ring pin is not used for ADC
        // Instead it is set to a high state to provide a constant voltage reference
        HAL_GPIO_WritePin(flexiport->portB, flexiport->pinB, GPIO_PIN_SET);
        GPIO_InitStruct.Pin = flexiport->adcPinRing;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(flexiport->adcPortRing, &GPIO_InitStruct);
        return FlexiOk;
    }

    return FlexiOk;
}

FlexiErrorState flexi_uartInit(Flexiport *flexiport, uint8_t swapPins, uint8_t speed)
{
    __disable_irq();
    if(speed == UART_HIGH_SPEED)
    {
        flexiport->huart->Init.BaudRate = 115200;
    }
    else if(speed == UART_STANDARD_SPEED)
    {
        flexiport->huart->Init.BaudRate = 31250;
    }
    flexiport->huart->Init.WordLength = UART_WORDLENGTH_8B;
    flexiport->huart->Init.StopBits = UART_STOPBITS_1;
    flexiport->huart->Init.Parity = UART_PARITY_NONE;
    flexiport->huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    flexiport->huart->Init.OverSampling = UART_OVERSAMPLING_16;
    flexiport->huart->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    flexiport->huart->Init.ClockPrescaler = UART_PRESCALER_DIV1;
    if(flexiport->config->mode == FlexiDeviceLink)
    {
        flexiport->huart->Init.Mode = UART_MODE_TX_RX;
    }
    else if(flexiport->config->mode == FlexiMidiOutTypeA || flexiport->config->mode == FlexiMidiOutTypeB ||
            flexiport->config->mode == FlexiMidiOutTip || flexiport->config->mode == FlexiMidiOutRing)
    {
        flexiport->huart->Init.Mode = UART_MODE_TX;
    }
    else if(flexiport->config->mode == FlexiMidiInTypeA || flexiport->config->mode == FlexiMidiInTypeB)
    {
        flexiport->huart->Init.Mode = UART_MODE_RX;
    }
    if(swapPins)
    {
        flexiport->huart->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_SWAP_INIT;
        flexiport->huart->AdvancedInit.Swap = UART_ADVFEATURE_SWAP_ENABLE;
    }
    else
    {
        flexiport->huart->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    }

    if(HAL_UART_Init(flexiport->huart) != HAL_OK)
    {
        return FlexiHalError;
    }
    if(HAL_UARTEx_SetTxFifoThreshold(flexiport->huart, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
    {
        return FlexiHalError;
    }
    if(HAL_UARTEx_SetRxFifoThreshold(flexiport->huart, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
    {
        return FlexiHalError;
    }
    if(HAL_UARTEx_DisableFifoMode(flexiport->huart) != HAL_OK)
    {
        return FlexiHalError;
    }

    __enable_irq();

    return FlexiOk;
}

FlexiErrorState flexi_gpioInputPullupInit(GPIO_TypeDef *port, uint16_t pin)
{
    flexi_configureGpioClock(port);
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(port, &GPIO_InitStruct);
    return FlexiOk;
}