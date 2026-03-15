// ADAR1000_Manager.cpp
#include "main.h"
#include "stm32f7xx_hal.h"
#include "ADAR1000_Manager.h"
#include <cmath>
#include <cstring>

extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart3;

// Chip Select GPIO definitions
static const struct {
    GPIO_TypeDef* port;
    uint16_t pin;
} CHIP_SELECTS[4] = {
    {ADAR_1_CS_3V3_GPIO_Port, ADAR_1_CS_3V3_Pin}, // ADAR1000 #1
    {ADAR_2_CS_3V3_GPIO_Port, ADAR_2_CS_3V3_Pin}, // ADAR1000 #2
    {ADAR_3_CS_3V3_GPIO_Port, ADAR_3_CS_3V3_Pin}, // ADAR1000 #3
    {ADAR_4_CS_3V3_GPIO_Port, ADAR_4_CS_3V3_Pin}  // ADAR1000 #4
};

// Vector Modulator lookup tables
const uint8_t ADAR1000Manager::VM_I[128] = {
    // ... (same as in your original file)
};

const uint8_t ADAR1000Manager::VM_Q[128] = {
    // ... (same as in your original file)
};

const uint8_t ADAR1000Manager::VM_GAIN[128] = {
    // ... (same as in your original file)
};

ADAR1000Manager::ADAR1000Manager() {
    for (int i = 0; i < 4; ++i) {
        devices_.push_back(std::make_unique<ADAR1000Device>(i));
    }
}

ADAR1000Manager::~ADAR1000Manager() {
    // Automatic cleanup by unique_ptr
}

// System Management
bool ADAR1000Manager::powerUpSystem() {
    const uint8_t msg[] = "Starting System Power-Up Sequence...\r\n";
    HAL_UART_Transmit(&huart3, msg, sizeof(msg) - 1, 1000);

    // Power-up sequence steps...
    HAL_GPIO_WritePin(EN_P_3V3_VDD_SW_GPIO_Port, EN_P_3V3_VDD_SW_Pin, GPIO_PIN_SET);
    HAL_Delay(2);

    HAL_GPIO_WritePin(EN_P_3V3_SW_GPIO_Port, EN_P_3V3_SW_Pin, GPIO_PIN_SET);
    HAL_Delay(2);

    // Initialize devices
    if (!initializeAllDevices()) {
        const uint8_t err[] = "ERROR: ADAR1000 initialization failed!\r\n";
        HAL_UART_Transmit(&huart3, err, sizeof(err) - 1, 1000);
        return false;
    }

    // Start in RX mode
    switchToRXMode();

    const uint8_t success[] = "System Power-Up Sequence Completed Successfully.\r\n";
    HAL_UART_Transmit(&huart3, success, sizeof(success) - 1, 1000);
    return true;
}

bool ADAR1000Manager::powerDownSystem() {
    switchToRXMode();
    HAL_Delay(10);

    disablePASupplies();
    disableLNASupplies();
    HAL_GPIO_WritePin(EN_P_3V3_SW_GPIO_Port, EN_P_3V3_SW_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EN_P_3V3_VDD_SW_GPIO_Port, EN_P_3V3_VDD_SW_Pin, GPIO_PIN_RESET);

    return true;
}

// Mode Switching
void ADAR1000Manager::switchToTXMode() {
    setLNABias(false);
    delayUs(10);
    enablePASupplies();
    delayUs(100);
    setPABias(true);
    delayUs(50);
    setADTR1107Control(true);

    for (uint8_t dev = 0; dev < devices_.size(); ++dev) {
        adarWrite(dev, REG_RX_ENABLES, 0x00, BROADCAST_OFF);
        adarWrite(dev, REG_TX_ENABLES, 0x0F, BROADCAST_OFF);
        adarSetTxBias(dev, BROADCAST_OFF);
        devices_[dev]->current_mode = BeamDirection::TX;
    }
    current_mode_ = BeamDirection::TX;
}

void ADAR1000Manager::switchToRXMode() {
    setPABias(false);
    delayUs(50);
    disablePASupplies();
    delayUs(10);
    setADTR1107Control(false);
    enableLNASupplies();
    delayUs(50);
    setLNABias(true);
    delayUs(50);

    for (uint8_t dev = 0; dev < devices_.size(); ++dev) {
        adarWrite(dev, REG_TX_ENABLES, 0x00, BROADCAST_OFF);
        adarWrite(dev, REG_RX_ENABLES, 0x0F, BROADCAST_OFF);
        devices_[dev]->current_mode = BeamDirection::RX;
    }
    current_mode_ = BeamDirection::RX;
}

void ADAR1000Manager::fastTXMode() {
    setADTR1107Control(true);
    for (uint8_t dev = 0; dev < devices_.size(); ++dev) {
        adarWrite(dev, REG_RX_ENABLES, 0x00, BROADCAST_OFF);
        adarWrite(dev, REG_TX_ENABLES, 0x0F, BROADCAST_OFF);
        devices_[dev]->current_mode = BeamDirection::TX;
    }
    current_mode_ = BeamDirection::TX;
}

void ADAR1000Manager::fastRXMode() {
    setADTR1107Control(false);
    for (uint8_t dev = 0; dev < devices_.size(); ++dev) {
        adarWrite(dev, REG_TX_ENABLES, 0x00, BROADCAST_OFF);
        adarWrite(dev, REG_RX_ENABLES, 0x0F, BROADCAST_OFF);
        devices_[dev]->current_mode = BeamDirection::RX;
    }
    current_mode_ = BeamDirection::RX;
}

void ADAR1000Manager::pulseTXMode() {
    setADTR1107Control(true);
    last_switch_time_us_ = HAL_GetTick() * 1000;
}

void ADAR1000Manager::pulseRXMode() {
    setADTR1107Control(false);
    last_switch_time_us_ = HAL_GetTick() * 1000;
}

// Beam Steering
bool ADAR1000Manager::setBeamAngle(float angle_degrees, BeamDirection direction) {
    uint8_t phase_settings[4];
    calculatePhaseSettings(angle_degrees, phase_settings);

    if (direction == BeamDirection::TX) {
        setAllDevicesTXMode();
    } else {
        setAllDevicesRXMode();
    }

    for (uint8_t dev = 0; dev < devices_.size(); ++dev) {
        for (uint8_t ch = 0; ch < 4; ++ch) {
            if (direction == BeamDirection::TX) {
                adarSetTxPhase(dev, ch + 1, phase_settings[ch], BROADCAST_OFF);
                adarSetTxVgaGain(dev, ch + 1, kDefaultTxVgaGain, BROADCAST_OFF);
            } else {
                adarSetRxPhase(dev, ch + 1, phase_settings[ch], BROADCAST_OFF);
                adarSetRxVgaGain(dev, ch + 1, kDefaultRxVgaGain, BROADCAST_OFF);
            }
        }
    }
    return true;
}

bool ADAR1000Manager::setCustomBeamPattern(const uint8_t phase_settings[4], const uint8_t gain_settings[4], BeamDirection direction) {
    for (uint8_t dev = 0; dev < devices_.size(); ++dev) {
        for (uint8_t ch = 0; ch < 4; ++ch) {
            if (direction == BeamDirection::TX) {
                adarSetTxPhase(dev, ch + 1, phase_settings[ch], BROADCAST_OFF);
                adarSetTxVgaGain(dev, ch + 1, gain_settings[ch], BROADCAST_OFF);
            } else {
                adarSetRxPhase(dev, ch + 1, phase_settings[ch], BROADCAST_OFF);
                adarSetRxVgaGain(dev, ch + 1, gain_settings[ch], BROADCAST_OFF);
            }
        }
    }
    return true;
}

// Beam Sweeping
void ADAR1000Manager::startBeamSweeping() {
    beam_sweeping_active_ = true;
    current_beam_index_ = 0;
    last_beam_update_time_ = HAL_GetTick();
}

void ADAR1000Manager::stopBeamSweeping() {
    beam_sweeping_active_ = false;
}

void ADAR1000Manager::updateBeamPosition() {
    if (!beam_sweeping_active_) return;

    uint32_t current_time = HAL_GetTick();
    const std::vector<BeamConfig>& sequence =
        (current_mode_ == BeamDirection::TX) ? tx_beam_sequence_ : rx_beam_sequence_;

    if (sequence.empty()) return;

    if (current_time - last_beam_update_time_ >= beam_dwell_time_ms_) {
        const BeamConfig& beam = sequence[current_beam_index_];
        setCustomBeamPattern(beam.phase_settings, beam.gain_settings, current_mode_);

        current_beam_index_ = (current_beam_index_ + 1) % sequence.size();
        last_beam_update_time_ = current_time;
    }
}

void ADAR1000Manager::setBeamSequence(const std::vector<BeamConfig>& sequence, BeamDirection direction) {
    if (direction == BeamDirection::TX) {
        tx_beam_sequence_ = sequence;
    } else {
        rx_beam_sequence_ = sequence;
    }
}

void ADAR1000Manager::clearBeamSequence(BeamDirection direction) {
    if (direction == BeamDirection::TX) {
        tx_beam_sequence_.clear();
    } else {
        rx_beam_sequence_.clear();
    }
}

// Monitoring and Diagnostics
float ADAR1000Manager::readTemperature(uint8_t deviceIndex) {
    if (deviceIndex >= devices_.size() || !devices_[deviceIndex]->initialized) {
        return -273.15f;
    }

    uint8_t temp_raw = adarAdcRead(deviceIndex, BROADCAST_OFF);
    return (temp_raw * 0.5f) - 50.0f;
}

bool ADAR1000Manager::verifyDeviceCommunication(uint8_t deviceIndex) {
    if (deviceIndex >= devices_.size()) return false;

    uint8_t test_value = 0xA5;
    adarWrite(deviceIndex, REG_SCRATCHPAD, test_value, BROADCAST_OFF);
    HAL_Delay(1);
    uint8_t readback = adarRead(deviceIndex, REG_SCRATCHPAD);
    return (readback == test_value);
}

uint8_t ADAR1000Manager::readRegister(uint8_t deviceIndex, uint32_t address) {
    return adarRead(deviceIndex, address);
}

void ADAR1000Manager::writeRegister(uint8_t deviceIndex, uint32_t address, uint8_t value) {
    adarWrite(deviceIndex, address, value, BROADCAST_OFF);
}

// Configuration
void ADAR1000Manager::setSwitchSettlingTime(uint32_t us) {
    switch_settling_time_us_ = us;
}

void ADAR1000Manager::setFastSwitchMode(bool enable) {
    fast_switch_mode_ = enable;
    if (enable) {
        switch_settling_time_us_ = 10;
        enablePASupplies();
        enableLNASupplies();
        setPABias(true);
        setLNABias(true);
    } else {
        switch_settling_time_us_ = 50;
    }
}

void ADAR1000Manager::setBeamDwellTime(uint32_t ms) {
    beam_dwell_time_ms_ = ms;
}

// Private helper methods (implementation continues...)
// ... include all the private method implementations from your original file
// ============================================================================
// PRIVATE HELPER METHODS - Add these to the end of ADAR1000_Manager.cpp
// ============================================================================

bool ADAR1000Manager::initializeAllDevices() {


    // Initialize each ADAR1000
    for (uint8_t i = 0; i < devices_.size(); ++i) {
        if (!initializeSingleDevice(i)) {
            return false;
        }
    }

    setAllDevicesTXMode();
    return true;
}

bool ADAR1000Manager::initializeSingleDevice(uint8_t deviceIndex) {
    if (deviceIndex >= devices_.size()) return false;

    adarSoftReset(deviceIndex);
    HAL_Delay(10);

    adarWriteConfigA(deviceIndex, INTERFACE_CONFIG_A_SDO_ACTIVE, BROADCAST_OFF);
    adarSetRamBypass(deviceIndex, BROADCAST_OFF);

    // Initialize ADC
    adarWrite(deviceIndex, REG_ADC_CONTROL, ADAR1000_ADC_2MHZ_CLK | ADAR1000_ADC_EN, BROADCAST_OFF);

    devices_[deviceIndex]->initialized = true;
    return true;
}

bool ADAR1000Manager::initializeADTR1107Sequence() {

	//Powering up ADTR1107 TX mode
    const uint8_t msg[] = "Starting ADTR1107 Power Sequence...\r\n";
    HAL_UART_Transmit(&huart3, msg, sizeof(msg) - 1, 1000);

    // Step 1: Connect all GND pins to ground (assumed in hardware)

    // Step 2: Set VDD_SW to 3.3V
    HAL_GPIO_WritePin(EN_P_3V3_VDD_SW_GPIO_Port, EN_P_3V3_VDD_SW_Pin, GPIO_PIN_SET);
    HAL_Delay(1);

    // Step 3: Set VSS_SW to -3.3V
    HAL_GPIO_WritePin(EN_P_3V3_SW_GPIO_Port, EN_P_3V3_SW_Pin, GPIO_PIN_SET);
    HAL_Delay(1);

    // Step 4: Set CTRL_SW to RX mode initially via GPIO
    setADTR1107Control(false); // RX mode
    HAL_Delay(1);

    // Step 5: Set VGG_LNA to 0
    uint8_t lna_bias_voltage = kLnaBiasOff;
    for (uint8_t dev = 0; dev < devices_.size(); ++dev) {
        adarWrite(dev, REG_LNA_BIAS_ON, lna_bias_voltage, BROADCAST_OFF);
        adarWrite(dev, REG_LNA_BIAS_OFF, kLnaBiasOff, BROADCAST_OFF);
    }

    // Step 6: Set VDD_LNA to 0V for TX mode
    HAL_GPIO_WritePin(EN_P_3V3_ADTR_GPIO_Port, EN_P_3V3_ADTR_Pin, GPIO_PIN_RESET);
    HAL_Delay(2);

    // Step 7: Set VGG_PA to safe negative voltage (PA off for TX mode)
    /*A 0x00 value in the
    on or off bias registers, correspond to a 0 V output. A 0xFF in the
    on or off bias registers correspond to a −4.8 V output.*/
    uint8_t safe_pa_bias = kPaBiasTxSafe; // Safe negative voltage (-1.75V) to keep PA off
    for (uint8_t dev = 0; dev < devices_.size(); ++dev) {
        adarWrite(dev, REG_PA_CH1_BIAS_ON, safe_pa_bias, BROADCAST_OFF);
        adarWrite(dev, REG_PA_CH2_BIAS_ON, safe_pa_bias, BROADCAST_OFF);
        adarWrite(dev, REG_PA_CH3_BIAS_ON, safe_pa_bias, BROADCAST_OFF);
        adarWrite(dev, REG_PA_CH4_BIAS_ON, safe_pa_bias, BROADCAST_OFF);
    }
    HAL_Delay(10);

    // Step 8: Set VDD_PA to 0V (PA powered up for TX mode)
    enablePASupplies();
    HAL_Delay(50);

    // Step 9: Adjust VGG_PA voltage between −1.75 V and −0.25 V to achieve the desired IDQ_PA=220mA
    //Set VGG_PA to safe negative voltage (PA off for TX mode)
    /*A 0x00 value in the
    on or off bias registers, correspond to a 0 V output. A 0xFF in the
    on or off bias registers correspond to a −4.8 V output.*/
    uint8_t Idq_pa_bias = kPaBiasIdqCalibration; // Safe negative voltage (-0.2447V) to keep PA off
    for (uint8_t dev = 0; dev < devices_.size(); ++dev) {
        adarWrite(dev, REG_PA_CH1_BIAS_ON, Idq_pa_bias, BROADCAST_OFF);
        adarWrite(dev, REG_PA_CH2_BIAS_ON, Idq_pa_bias, BROADCAST_OFF);
        adarWrite(dev, REG_PA_CH3_BIAS_ON, Idq_pa_bias, BROADCAST_OFF);
        adarWrite(dev, REG_PA_CH4_BIAS_ON, Idq_pa_bias, BROADCAST_OFF);
    }
    HAL_Delay(10);


    const uint8_t success[] = "ADTR1107 power sequence completed.\r\n";
    HAL_UART_Transmit(&huart3, success, sizeof(success) - 1, 1000);

    return true;
}

bool ADAR1000Manager::setAllDevicesTXMode() {
    // Set ADTR1107 to TX mode first
    setADTR1107Mode(BeamDirection::TX);

    // Then configure ADAR1000 for TX
    for (uint8_t dev = 0; dev < devices_.size(); ++dev) {
        // Disable RX first
        adarWrite(dev, REG_RX_ENABLES, 0x00, BROADCAST_OFF);

        // Enable TX channels and set bias
        adarWrite(dev, REG_TX_ENABLES, 0x0F, BROADCAST_OFF); // Enable all 4 channels
        adarSetTxBias(dev, BROADCAST_OFF);

        devices_[dev]->current_mode = BeamDirection::TX;
    }
    current_mode_ = BeamDirection::TX;
    return true;
}

bool ADAR1000Manager::setAllDevicesRXMode() {
    // Set ADTR1107 to RX mode first
    setADTR1107Mode(BeamDirection::RX);

    // Then configure ADAR1000 for RX
    for (uint8_t dev = 0; dev < devices_.size(); ++dev) {
        // Disable TX first
        adarWrite(dev, REG_TX_ENABLES, 0x00, BROADCAST_OFF);

        // Enable RX channels
        adarWrite(dev, REG_RX_ENABLES, 0x0F, BROADCAST_OFF); // Enable all 4 channels

        devices_[dev]->current_mode = BeamDirection::RX;
    }
    current_mode_ = BeamDirection::RX;
    return true;
}

void ADAR1000Manager::setADTR1107Mode(BeamDirection direction) {
    if (direction == BeamDirection::TX) {
        setADTR1107Control(true); // TX mode

        // Step 1: Disable LNA power first
        disableLNASupplies();
        HAL_Delay(5);

        // Step 2: Set LNA bias to safe off value
        for (uint8_t dev = 0; dev < devices_.size(); ++dev) {
            adarWrite(dev, REG_LNA_BIAS_ON, kLnaBiasOff, BROADCAST_OFF); // Turn off LNA bias
        }
        HAL_Delay(5);

        // Step 3: Enable PA power
        enablePASupplies();
        HAL_Delay(10);

        // Step 4: Set PA bias to operational value
        uint8_t operational_pa_bias = kPaBiasOperational; // Maximum bias for full power
        for (uint8_t dev = 0; dev < devices_.size(); ++dev) {
            adarWrite(dev, REG_PA_CH1_BIAS_ON, operational_pa_bias, BROADCAST_OFF);
            adarWrite(dev, REG_PA_CH2_BIAS_ON, operational_pa_bias, BROADCAST_OFF);
            adarWrite(dev, REG_PA_CH3_BIAS_ON, operational_pa_bias, BROADCAST_OFF);
            adarWrite(dev, REG_PA_CH4_BIAS_ON, operational_pa_bias, BROADCAST_OFF);
        }
        HAL_Delay(5);

        // Step 5: Set TR switch to TX mode
        for (uint8_t dev = 0; dev < devices_.size(); ++dev) {
            adarSetBit(dev, REG_SW_CONTROL, 2, BROADCAST_OFF); // TR_SOURCE = 1 (TX)
            adarSetBit(dev, REG_MISC_ENABLES, 5, BROADCAST_OFF); // BIAS_EN
        }

    } else {
        // RECEIVE MODE: Enable LNA, Disable PA
        setADTR1107Control(false); // RX mode

        // Step 1: Disable PA power first
        disablePASupplies();
        HAL_Delay(5);

        // Step 2: Set PA bias to safe negative voltage
        uint8_t safe_pa_bias = kPaBiasRxSafe;
        for (uint8_t dev = 0; dev < devices_.size(); ++dev) {
            adarWrite(dev, REG_PA_CH1_BIAS_ON, safe_pa_bias, BROADCAST_OFF);
            adarWrite(dev, REG_PA_CH2_BIAS_ON, safe_pa_bias, BROADCAST_OFF);
            adarWrite(dev, REG_PA_CH3_BIAS_ON, safe_pa_bias, BROADCAST_OFF);
            adarWrite(dev, REG_PA_CH4_BIAS_ON, safe_pa_bias, BROADCAST_OFF);
        }
        HAL_Delay(5);

        // Step 3: Enable LNA power
        enableLNASupplies();
        HAL_Delay(10);

        // Step 4: Set LNA bias to operational value
        uint8_t operational_lna_bias = kLnaBiasOperational;
        for (uint8_t dev = 0; dev < devices_.size(); ++dev) {
            adarWrite(dev, REG_LNA_BIAS_ON, operational_lna_bias, BROADCAST_OFF);
        }
        HAL_Delay(5);

        // Step 5: Set TR switch to RX mode
        for (uint8_t dev = 0; dev < devices_.size(); ++dev) {
            adarResetBit(dev, REG_SW_CONTROL, 2, BROADCAST_OFF); // TR_SOURCE = 0 (RX)
            adarSetBit(dev, REG_MISC_ENABLES, 4, BROADCAST_OFF); // LNA_BIAS_OUT_EN
        }
    }
}

void ADAR1000Manager::setADTR1107Control(bool tx_mode) {
    for (uint8_t dev = 0; dev < devices_.size(); ++dev) {
        setTRSwitchPosition(dev, tx_mode);
    }
    delayUs(switch_settling_time_us_);
}

void ADAR1000Manager::setTRSwitchPosition(uint8_t deviceIndex, bool tx_mode) {
    if (tx_mode) {
        // TX mode: Set TR_SOURCE = 1
        adarSetBit(deviceIndex, REG_SW_CONTROL, 2, BROADCAST_OFF);
    } else {
        // RX mode: Set TR_SOURCE = 0
        adarResetBit(deviceIndex, REG_SW_CONTROL, 2, BROADCAST_OFF);
    }
}

// Add the new public method
bool ADAR1000Manager::setCustomBeamPattern16(const uint8_t phase_pattern[16], BeamDirection direction) {
    for (uint8_t dev = 0; dev < 4; ++dev) {
        for (uint8_t ch = 0; ch < 4; ++ch) {
            uint8_t phase = phase_pattern[dev * 4 + ch];
            if (direction == BeamDirection::TX) {
                adarSetTxPhase(dev, ch + 1, phase, BROADCAST_OFF);
            } else {
                adarSetRxPhase(dev, ch + 1, phase, BROADCAST_OFF);
            }
        }
    }
    return true;
}

void ADAR1000Manager::enablePASupplies() {
    HAL_GPIO_WritePin(EN_P_5V0_PA1_GPIO_Port, EN_P_5V0_PA1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(EN_P_5V0_PA2_GPIO_Port, EN_P_5V0_PA2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(EN_P_5V0_PA3_GPIO_Port, EN_P_5V0_PA3_Pin, GPIO_PIN_SET);
}

void ADAR1000Manager::disablePASupplies() {
    HAL_GPIO_WritePin(EN_P_5V0_PA1_GPIO_Port, EN_P_5V0_PA1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EN_P_5V0_PA2_GPIO_Port, EN_P_5V0_PA2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EN_P_5V0_PA3_GPIO_Port, EN_P_5V0_PA3_Pin, GPIO_PIN_RESET);
}

void ADAR1000Manager::enableLNASupplies() {
    HAL_GPIO_WritePin(EN_P_3V3_ADTR_GPIO_Port, EN_P_3V3_ADTR_Pin, GPIO_PIN_SET);
}

void ADAR1000Manager::disableLNASupplies() {
    HAL_GPIO_WritePin(EN_P_3V3_ADTR_GPIO_Port, EN_P_3V3_ADTR_Pin, GPIO_PIN_RESET);
}

void ADAR1000Manager::setPABias(bool enable) {
    uint8_t pa_bias = enable ? kPaBiasOperational : kPaBiasRxSafe; // Operational vs safe bias

    for (uint8_t dev = 0; dev < devices_.size(); ++dev) {
        adarWrite(dev, REG_PA_CH1_BIAS_ON, pa_bias, BROADCAST_OFF);
        adarWrite(dev, REG_PA_CH2_BIAS_ON, pa_bias, BROADCAST_OFF);
        adarWrite(dev, REG_PA_CH3_BIAS_ON, pa_bias, BROADCAST_OFF);
        adarWrite(dev, REG_PA_CH4_BIAS_ON, pa_bias, BROADCAST_OFF);
    }
}

void ADAR1000Manager::setLNABias(bool enable) {
    uint8_t lna_bias = enable ? kLnaBiasOperational : kLnaBiasOff; // Operational vs off

    for (uint8_t dev = 0; dev < devices_.size(); ++dev) {
        adarWrite(dev, REG_LNA_BIAS_ON, lna_bias, BROADCAST_OFF);
    }
}

void ADAR1000Manager::delayUs(uint32_t microseconds) {
    // Simple implementation - for F7 @ 216MHz, each loop ~7 cycles ≈ 0.032us
    volatile uint32_t cycles = microseconds * 10; // Adjust this multiplier for your clock
    while (cycles--) {
        __NOP();
    }
}

void ADAR1000Manager::calculatePhaseSettings(float angle_degrees, uint8_t phase_settings[4]) {
    const float freq = 10.5e9;
    const float c = 3e8;
    const float wavelength = c / freq;
    const float element_spacing = wavelength / 2;

    float angle_rad = angle_degrees * M_PI / 180.0;
    float phase_shift = (2 * M_PI * element_spacing * sin(angle_rad)) / wavelength;

    for (int i = 0; i < 4; ++i) {
        float element_phase = i * phase_shift;
        while (element_phase < 0) element_phase += 2 * M_PI;
        while (element_phase >= 2 * M_PI) element_phase -= 2 * M_PI;
        phase_settings[i] = static_cast<uint8_t>((element_phase / (2 * M_PI)) * 128);
    }
}

bool ADAR1000Manager::performSystemCalibration() {
    for (uint8_t i = 0; i < devices_.size(); ++i) {
        if (!verifyDeviceCommunication(i)) {
            return false;
        }
    }
    return true;
}

// ============================================================================
// LOW-LEVEL SPI COMMUNICATION METHODS
// ============================================================================

uint32_t ADAR1000Manager::spiTransfer(uint8_t* txData, uint8_t* rxData, uint32_t size) {
    HAL_StatusTypeDef status;

    if (rxData) {
        status = HAL_SPI_TransmitReceive(&hspi1, txData, rxData, size, 1000);
    } else {
        status = HAL_SPI_Transmit(&hspi1, txData, size, 1000);
    }

    return (status == HAL_OK) ? size : 0;
}

void ADAR1000Manager::setChipSelect(uint8_t deviceIndex, bool state) {
    if (deviceIndex >= devices_.size()) return;
    HAL_GPIO_WritePin(CHIP_SELECTS[deviceIndex].port,
                      CHIP_SELECTS[deviceIndex].pin,
                      state ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void ADAR1000Manager::adarWrite(uint8_t deviceIndex, uint32_t mem_addr, uint8_t data, uint8_t broadcast) {
    uint8_t instruction[3];

    if (broadcast) {
        instruction[0] = 0x08;
    } else {
        instruction[0] = ((devices_[deviceIndex]->dev_addr & 0x03) << 5);
    }

    instruction[0] |= (0x1F00 & mem_addr) >> 8;
    instruction[1] = (0xFF & mem_addr);
    instruction[2] = data;

    setChipSelect(deviceIndex, true);
    spiTransfer(instruction, nullptr, sizeof(instruction));
    setChipSelect(deviceIndex, false);
}

uint8_t ADAR1000Manager::adarRead(uint8_t deviceIndex, uint32_t mem_addr) {
    uint8_t instruction[3] = {0};
    uint8_t rx_buffer[3] = {0};

    // Set SDO active
    adarWrite(deviceIndex, REG_INTERFACE_CONFIG_A, INTERFACE_CONFIG_A_SDO_ACTIVE, 0);

    instruction[0] = 0x80 | ((devices_[deviceIndex]->dev_addr & 0x03) << 5);
    instruction[0] |= ((0xff00 & mem_addr) >> 8);
    instruction[1] = (0xff & mem_addr);
    instruction[2] = 0x00;

    setChipSelect(deviceIndex, true);
    spiTransfer(instruction, rx_buffer, sizeof(instruction));
    setChipSelect(deviceIndex, false);

    // Set SDO Inactive
    adarWrite(deviceIndex, REG_INTERFACE_CONFIG_A, 0, 0);

    return rx_buffer[2];
}

void ADAR1000Manager::adarSetBit(uint8_t deviceIndex, uint32_t mem_addr, uint8_t bit, uint8_t broadcast) {
    uint8_t temp = adarRead(deviceIndex, mem_addr);
    uint8_t data = temp | (1 << bit);
    adarWrite(deviceIndex, mem_addr, data, broadcast);
}

void ADAR1000Manager::adarResetBit(uint8_t deviceIndex, uint32_t mem_addr, uint8_t bit, uint8_t broadcast) {
    uint8_t temp = adarRead(deviceIndex, mem_addr);
    uint8_t data = temp & ~(1 << bit);
    adarWrite(deviceIndex, mem_addr, data, broadcast);
}

void ADAR1000Manager::adarSoftReset(uint8_t deviceIndex) {
    uint8_t instruction[3];
    instruction[0] = ((devices_[deviceIndex]->dev_addr & 0x03) << 5);
    instruction[1] = 0x00;
    instruction[2] = 0x81;

    setChipSelect(deviceIndex, true);
    spiTransfer(instruction, nullptr, sizeof(instruction));
    setChipSelect(deviceIndex, false);
}

void ADAR1000Manager::adarWriteConfigA(uint8_t deviceIndex, uint8_t flags, uint8_t broadcast) {
    adarWrite(deviceIndex, REG_INTERFACE_CONFIG_A, flags, broadcast);
}

void ADAR1000Manager::adarSetRamBypass(uint8_t deviceIndex, uint8_t broadcast) {
    uint8_t data = (MEM_CTRL_BIAS_RAM_BYPASS | MEM_CTRL_BEAM_RAM_BYPASS);
    adarWrite(deviceIndex, REG_MEM_CTL, data, broadcast);
}

void ADAR1000Manager::adarSetRxPhase(uint8_t deviceIndex, uint8_t channel, uint8_t phase, uint8_t broadcast) {
    uint8_t i_val = VM_I[phase % 128];
    uint8_t q_val = VM_Q[phase % 128];

    uint32_t mem_addr_i = REG_CH1_RX_PHS_I + (channel & 0x03) * 2;
    uint32_t mem_addr_q = REG_CH1_RX_PHS_Q + (channel & 0x03) * 2;

    adarWrite(deviceIndex, mem_addr_i, i_val, broadcast);
    adarWrite(deviceIndex, mem_addr_q, q_val, broadcast);
    adarWrite(deviceIndex, REG_LOAD_WORKING, 0x1, broadcast);
}

void ADAR1000Manager::adarSetTxPhase(uint8_t deviceIndex, uint8_t channel, uint8_t phase, uint8_t broadcast) {
    uint8_t i_val = VM_I[phase % 128];
    uint8_t q_val = VM_Q[phase % 128];

    uint32_t mem_addr_i = REG_CH1_TX_PHS_I + (channel & 0x03) * 2;
    uint32_t mem_addr_q = REG_CH1_TX_PHS_Q + (channel & 0x03) * 2;

    adarWrite(deviceIndex, mem_addr_i, i_val, broadcast);
    adarWrite(deviceIndex, mem_addr_q, q_val, broadcast);
    adarWrite(deviceIndex, REG_LOAD_WORKING, 0x1, broadcast);
}

void ADAR1000Manager::adarSetRxVgaGain(uint8_t deviceIndex, uint8_t channel, uint8_t gain, uint8_t broadcast) {
    uint32_t mem_addr = REG_CH1_RX_GAIN + (channel & 0x03);
    adarWrite(deviceIndex, mem_addr, gain, broadcast);
    adarWrite(deviceIndex, REG_LOAD_WORKING, 0x1, broadcast);
}

void ADAR1000Manager::adarSetTxVgaGain(uint8_t deviceIndex, uint8_t channel, uint8_t gain, uint8_t broadcast) {
    uint32_t mem_addr = REG_CH1_TX_GAIN + (channel & 0x03);
    adarWrite(deviceIndex, mem_addr, gain, broadcast);
    adarWrite(deviceIndex, REG_LOAD_WORKING, LD_WRK_REGS_LDTX_OVERRIDE, broadcast);
}

void ADAR1000Manager::adarSetTxBias(uint8_t deviceIndex, uint8_t broadcast) {
    adarWrite(deviceIndex, REG_BIAS_CURRENT_TX, kTxBiasCurrent, broadcast);
    adarWrite(deviceIndex, REG_BIAS_CURRENT_TX_DRV, kTxDriverBiasCurrent, broadcast);
    adarWrite(deviceIndex, REG_LOAD_WORKING, 0x2, broadcast);
}

uint8_t ADAR1000Manager::adarAdcRead(uint8_t deviceIndex, uint8_t broadcast) {
    adarWrite(deviceIndex, REG_ADC_CONTROL, ADAR1000_ADC_ST_CONV, broadcast);

    // Wait for conversion
    while (!(adarRead(deviceIndex, REG_ADC_CONTROL) & 0x01)) {
        // Busy wait
    }

    return adarRead(deviceIndex, REG_ADC_OUT);
}
