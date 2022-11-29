/*
 * Copyright 2022 Peter Buelow (goballstate at gmail)
 *
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.
3. Neither the name of the copyright holders nor the
names of its contributors may be used to endorse or promote products
derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#include "tcs34725.h"

TCS34725::TCS34725(uint8_t address, uint8_t integration, tcs34725Gain_t gain) : m_address(address), m_integration(integration), m_gain(gain), m_open(false), m_enabled(false)
{
    m_device = "/dev/i2c-";
    m_device += std::to_string(address);
    if ((m_fd = open(m_device.c_str(), O_RDWR)) < 0) {
        m_open = false;
        m_lastErrno = errno;
        m_lastError = strerror(errno);
        return;
    }

    if (ioctl(m_fd, I2C_SLAVE, address) < 0) {
        m_lastErrno = errno;
        m_lastError = strerror(errno);
        close(m_fd);
        m_open = false;
        return;
    }

    m_id = read8(TCS34725_ID);
    if ((m_id != 0x4d) && (m_id != 0x44) && (m_id != 0x10)) {
        return;
    }

    /* Set default integration time and gain */
    setIntegrationTime(integration);
    setGain(gain);

    /* Note: by default, the device is in power down mode on bootup */
    enable();
    m_open = true;
}

TCS34725::~TCS34725()
{
    close(m_fd);
    m_open = false;
    m_enabled = false;
}

bool TCS34725::write8(uint8_t reg, uint8_t value)
{
    if (m_open && m_enabled) {
        uint8_t buffer[2] = {(uint8_t)(TCS34725_COMMAND_BIT | reg), value};
        if (write(m_fd, buffer, 2) < 0) {
            m_lastErrno = errno;
            m_lastError = strerror(errno);
            return false;
        }
    }
    else {
        return false;
    }
    return true;
}

uint8_t TCS34725::read8(uint8_t reg)
{
    if (m_open && m_enabled) {
        uint8_t buffer[1] = {(uint8_t)(TCS34725_COMMAND_BIT | reg)};
        if (write(m_fd, buffer, 1) == 0) {
            if (read(m_fd, buffer, 1) == 0) {
                return buffer[0];
            }
            else {
                m_lastErrno = errno;
                m_lastError = strerror(errno);
                return 0;
            }
        }
        else {
            m_lastErrno = errno;
            m_lastError = strerror(errno);
            return 0;
        }
    }
    return 0;
}

uint16_t TCS34725::read16(uint8_t reg)
{
    if (m_open && m_enabled) {
        uint8_t buffer[2] = {(uint8_t)(TCS34725_COMMAND_BIT | reg), 0};
        if (write(m_fd, buffer, 1) == 0) {
            if (read(m_fd, buffer, 2) == 0) {
                return (uint16_t(buffer[1]) << 8) | (uint16_t(buffer[0]) & 0xFF);
            }
            else {
                m_lastErrno = errno;
                m_lastError = strerror(errno);
                return 0;
            }
        }
        else {
            m_lastErrno = errno;
            m_lastError = strerror(errno);
            return 0;
        }
    }
    return 0;
}

void TCS34725::delay(unsigned int delay)
{
    struct timespec sleeper;

    sleeper.tv_sec  = (time_t)(delay / 1000) ;
    sleeper.tv_nsec = (long)(delay % 1000) * 1000000 ;

    nanosleep (&sleeper, NULL) ;
}

void TCS34725::enable()
{
    if (write8(TCS34725_ENABLE, TCS34725_ENABLE_PON)) {
        delay(3);
        if (write8(TCS34725_ENABLE, TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN)) {
            /* Set a delay for the integration time.
            This is only necessary in the case where enabling and then
            immediately trying to read values back. This is because setting
            AEN triggers an automatic integration, so if a read RGBC is
            performed too quickly, the data is not yet valid and all 0's are
            returned */
            /* 12/5 = 2.4, add 1 to account for integer truncation */
            delay((256 - m_integration) * 12 / 5 + 1);
            m_enabled = true;
        }
    }
}

void TCS34725::disable()
{
    uint8_t reg = 0;
    reg = read8(TCS34725_ENABLE);
    if (write8(TCS34725_ENABLE, reg & ~(TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN)))
        m_enabled = false;
}

void TCS34725::setIntegrationTime(uint8_t it)
{
    if (m_enabled) {
        if (write8(TCS34725_ATIME, it))
            m_integration = it;
    }
}

void TCS34725::setGain(tcs34725Gain_t gain)
{
    if (m_enabled) {
        if (write8(TCS34725_CONTROL, gain))
            m_gain = gain;
    }
}

void TCS34725::getRawData(uint16_t& r, uint16_t& g, uint16_t& b, uint16_t& c)
{
    if (m_enabled) {
        c = read16(TCS34725_CDATAL);
        r = read16(TCS34725_RDATAL);
        g = read16(TCS34725_GDATAL);
        b = read16(TCS34725_BDATAL);

        /* Set a delay for the integration time */
        /* 12/5 = 2.4, add 1 to account for integer truncation */
        delay((256 - m_integration) * 12 / 5 + 1);
    }
}

void TCS34725::getRawDataOneShot(uint16_t& r, uint16_t& g, uint16_t& b, uint16_t& c)
{
    enable();
    getRawData(r, g, b, c);
    disable();
}

void TCS34725::getRGB(float& r, float& g, float& b)
{
    uint16_t red, green, blue, clear;
    getRawData(red, green, blue, clear);
    uint32_t sum = clear;

    // Avoid divide by zero errors ... if clear = 0 return black
    if (clear == 0) {
        r = g = b = 0;
        return;
    }

    r = (float)red / sum * 255.0;
    g = (float)green / sum * 255.0;
    b = (float)blue / sum * 255.0;
}

uint16_t TCS34725::calculateColorTemperature(uint16_t r, uint16_t g, uint16_t b)
{
    float X, Y, Z; /* RGB to XYZ correlation      */
    float xc, yc;  /* Chromaticity co-ordinates   */
    float n;       /* McCamy's formula            */
    float cct;

    if (r == 0 && g == 0 && b == 0) {
        return 0;
    }

    /* 1. Map RGB values to their XYZ counterparts.    */
    /* Based on 6500K fluorescent, 3000K fluorescent   */
    /* and 60W incandescent values for a wide range.   */
    /* Note: Y = Illuminance or lux                    */
    X = (-0.14282F * r) + (1.54924F * g) + (-0.95641F * b);
    Y = (-0.32466F * r) + (1.57837F * g) + (-0.73191F * b);
    Z = (-0.68202F * r) + (0.77073F * g) + (0.56332F * b);

    /* 2. Calculate the chromaticity co-ordinates      */
    xc = (X) / (X + Y + Z);
    yc = (Y) / (X + Y + Z);

    /* 3. Use McCamy's formula to determine the CCT    */
    n = (xc - 0.3320F) / (0.1858F - yc);

    /* Calculate the final CCT */
    cct = (449.0F * powf(n, 3)) + (3525.0F * powf(n, 2)) + (6823.3F * n) + 5520.33F;

    /* Return the results in degrees Kelvin */
    return (uint16_t)cct;
}

uint16_t TCS34725::calculateColorTemperature_dn40(uint16_t r, uint16_t g, uint16_t b, uint16_t c)
{
    uint16_t r2, b2; /* RGB values minus IR component */
    uint16_t sat;    /* Digital saturation level */
    uint16_t ir;     /* Inferred IR content */

    if (c == 0) {
        return 0;
    }

    /* Analog/Digital saturation:
     *
     * (a) As light becomes brighter, the clear channel will tend to
     *     saturate first since R+G+B is approximately equal to C.
     * (b) The TCS34725 accumulates 1024 counts per 2.4ms of integration
     *     time, up to a maximum values of 65535. This means analog
     *     saturation can occur up to an integration time of 153.6ms
     *     (64*2.4ms=153.6ms).
     * (c) If the integration time is > 153.6ms, digital saturation will
     *     occur before analog saturation. Digital saturation occurs when
     *     the count reaches 65535.
    */
    if ((256 - m_integration) > 63) {
        /* Track digital saturation */
        sat = 65535;
    } else {
        /* Track analog saturation */
        sat = 1024 * (256 - m_integration);
    }

    /* Ripple rejection:
     *
     * (a) An integration time of 50ms or multiples of 50ms are required to
     *     reject both 50Hz and 60Hz ripple.
     * (b) If an integration time faster than 50ms is required, you may need
     *     to average a number of samples over a 50ms period to reject ripple
     *     from fluorescent and incandescent light sources.
     *
     * Ripple saturation notes:
     *
     * (a) If there is ripple in the received signal, the value read from C
     *     will be less than the max, but still have some effects of being
     *     saturated. This means that you can be below the 'sat' value, but
     *     still be saturating. At integration times >150ms this can be
     *     ignored, but <= 150ms you should calculate the 75% saturation
     *       level to avoid this problem.
     */
    if ((256 - m_integration) <= 63) {
        /* Adjust sat to 75% to avoid analog saturation if atime < 153.6ms */
        sat -= sat / 4;
    }

    /* Check for saturation and mark the sample as invalid if true */
    if (c >= sat) {
        return 0;
    }

    /* AMS RGB sensors have no IR channel, so the IR content must be */
    /* calculated indirectly. */
    ir = (r + g + b > c) ? (r + g + b - c) / 2 : 0;

    /* Remove the IR component from the raw RGB values */
    r2 = r - ir;
    b2 = b - ir;

    if (r2 == 0) {
        return 0;
    }

    /* A simple method of measuring color temp is to use the ratio of blue */
    /* to red light, taking IR cancellation into account. */
    uint16_t cct = (3810 * (uint32_t)b2) /  (uint32_t)r2 + 1391; /** Color temp offset. */

    return cct;
}

uint16_t TCS34725::calculateLux(uint16_t r, uint16_t g, uint16_t b)
{
    float illuminance;

    /* This only uses RGB ... how can we integrate clear or calculate lux */
    /* based exclusively on clear since this might be more reliable?      */
    illuminance = (-0.32466F * r) + (1.57837F * g) + (-0.73191F * b);

    return (uint16_t)illuminance;
}

void TCS34725::setInterrupt(bool flag)
{
    if (m_open && m_enabled) {
        uint8_t r = read8(TCS34725_ENABLE);
        if (flag) {
            r |= TCS34725_ENABLE_AIEN;
        } else {
            r &= ~TCS34725_ENABLE_AIEN;
        }
        write8(TCS34725_ENABLE, r);
    }
}

void TCS34725::clearInterrupt()
{
    if (m_open && m_enabled) {
        uint8_t buffer[1] = {TCS34725_COMMAND_BIT | 0x66};
        write(m_fd, buffer, 1);
    }
}

void TCS34725::setIntLimits(uint16_t low, uint16_t high)
{
    write8(0x04, low & 0xFF);
    write8(0x05, low >> 8);
    write8(0x06, high & 0xFF);
    write8(0x07, high >> 8);
}
