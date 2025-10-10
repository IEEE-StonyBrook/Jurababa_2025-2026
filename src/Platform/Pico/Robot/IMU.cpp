#include "../../../Include/Platform/Pico/Robot/IMU.h"
#include "../../../Include/Common/LogSystem.h"

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <cmath>
#include <cstdio>

// ==== I2C IMU settings ====
#define SDA_PIN   4
#define SCL_PIN   5
#define I2C_PORT  i2c0
#define I2C_BAUD  400000

// BNO085 default I2C address (0x4A or 0x4B depending on SA0 pin)
#define IMU_ADDR  0x4A

// BNO085 channels & feature IDs
#define CH_CONTROL   0
#define CH_REPORTS   2
#define FEATURE_ROTATION_VECTOR 0x05

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Buffers
static uint8_t header[4];
static uint8_t payload[128];
static uint8_t seqnum_control = 0;

IMU* IMU::imuInstance = nullptr;

// -------------------------------------------------
IMU::IMU() {
    imuInstance = this;
    yawReady = false;
    robotYawDegrees = 0.0f;
    yawOffset = 0.0f;

    // Init I²C
    i2c_init(I2C_PORT, I2C_BAUD);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    LOG_DEBUG("IMU (I2C, no INT/RST) initialized");
}

// -------------------------------------------------
bool IMU::sendPacket(uint8_t channel, const uint8_t* data, uint16_t len) {
    uint8_t packet[132];
    uint16_t total = len + 4;
    packet[0] = total & 0xFF;
    packet[1] = (total >> 8) & 0x7F;
    packet[2] = channel;
    packet[3] = seqnum_control++;
    for (uint16_t i = 0; i < len; i++) packet[4 + i] = data[i];
    return (i2c_write_blocking(I2C_PORT, IMU_ADDR, packet, total, false) >= 0);
}

bool IMU::readPacket(uint8_t &channel, uint16_t &len) {
    // Try to read 4-byte header
    if (i2c_read_blocking(I2C_PORT, IMU_ADDR, header, 4, false) < 0) return false;

    len = header[0] | ((header[1] & 0x7F) << 8);
    channel = header[2];
    if (len <= 4) return false;

    len -= 4;
    return (i2c_read_blocking(I2C_PORT, IMU_ADDR, payload, len, false) >= 0);
}

bool IMU::enableRotationVector() {
    uint8_t cmd[21] = {0};
    cmd[0] = 0xFD;                // Set feature command
    cmd[1] = FEATURE_ROTATION_VECTOR;
    uint32_t interval = 20000;    // 20 ms = 50 Hz
    cmd[3] = interval & 0xFF;
    cmd[4] = (interval >> 8) & 0xFF;
    cmd[5] = (interval >> 16) & 0xFF;
    cmd[6] = (interval >> 24) & 0xFF;
    return sendPacket(CH_CONTROL, cmd, sizeof(cmd));
}

void IMU::updateYaw() {
    uint8_t channel;
    uint16_t len;
    if (!readPacket(channel, len)) return;

    if (channel == CH_REPORTS && payload[0] == FEATURE_ROTATION_VECTOR) {
        if (len < 14) return;

        int16_t qi_i = payload[4]  | (payload[5]  << 8);
        int16_t qj_i = payload[6]  | (payload[7]  << 8);
        int16_t qk_i = payload[8]  | (payload[9]  << 8);
        int16_t qr_i = payload[10] | (payload[11] << 8);

        const float scale = 1.0f / (1 << 14);
        float qi = qi_i * scale;
        float qj = qj_i * scale;
        float qk = qk_i * scale;
        float qr = qr_i * scale;

        float yaw = atan2f(2.0f * (qr*qk + qi*qj),
                           1.0f - 2.0f * (qj*qj + qk*qk)) * 180.0f / M_PI;

        // Normalize into [-180, 180]
        if (yaw > 180) yaw -= 360;
        if (yaw < -180) yaw += 360;

        robotYawDegrees = yaw;
        yawReady = true;
    }
}

// -------------------------------------------------
float IMU::getIMUYawDegreesNeg180ToPos180() {
    updateYaw(); // Poll latest data
    if (!yawReady) return 0.0f;
    return robotYawDegrees - yawOffset;
}

void IMU::resetIMUYawToZero() {
    updateYaw();
    if (!yawReady) return;
    yawOffset = robotYawDegrees;
}