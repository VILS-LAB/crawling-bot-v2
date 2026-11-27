#include "Display.h"
#include "../../include/config.h"

Display::Display() : oled(nullptr), cursorX(0), cursorY(0), ready(false)
{
    // The library speeds the bus up for a transfer and restores `clkAfter`.
    // Its default of 100 kHz would throttle the 200 Hz IMU reads that share the
    // bus, so both values are the shared bus rate.
    oled = new Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET,
                                I2C_CLOCK_HZ, I2C_CLOCK_HZ);
}

Display::~Display()
{
    delete oled;
}

void Display::begin()
{
    if (oled == nullptr)
    {
        Serial.println(F("SSD1306 allocation failed"));
        return;
    }

    // The IMU driver shares this bus; i2c::begin() starts the peripheral once
    // and owns the clock rate for both devices.
    if (!i2c::begin())
    {
        Serial.println(F("ERROR,I2C_BUS_UNAVAILABLE,display_disabled"));
        return;
    }

    i2c::Guard bus(I2C_LOCK_TIMEOUT_MS * 4);
    if (!bus.locked())
    {
        Serial.println(F("ERROR,SSD1306_BUS_TIMEOUT"));
        return;
    }

    // A missing or miswired panel must not hang the boot, so a failure here is
    // reported and the robot keeps running without a display.
    if (!oled->begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS, true, false))
    {
        Serial.println(F("ERROR,SSD1306_INIT_FAILED"));
        return;
    }

    oled->clearDisplay();
    oled->setTextSize(1);
    oled->setTextColor(SSD1306_WHITE);
    oled->setCursor(0, 0);
    oled->display();
    ready = true;
}

void Display::clear()
{
    if (!ready)
        return;

    i2c::Guard bus(I2C_LOCK_TIMEOUT_MS);
    if (!bus.locked())
        return;

    oled->clearDisplay();
    cursorX = 0;
    cursorY = 0;
    oled->setCursor(0, 0);
}

void Display::print(const char *text, uint8_t x, uint8_t y)
{
    if (text == nullptr || !ready)
        return;

    i2c::Guard bus(I2C_LOCK_TIMEOUT_MS);
    if (!bus.locked())
        return;

    if (x != 0 || y != 0)
    {
        oled->setCursor(x, y);
        cursorX = x;
        cursorY = y;
    }
    else
    {
        oled->setCursor(cursorX, cursorY);
    }
    oled->print(text);
    // Update cursor position after printing
    cursorX = oled->getCursorX();
    cursorY = oled->getCursorY();
}

void Display::print(const String &text, uint8_t x, uint8_t y)
{
    print(text.c_str(), x, y);
}

void Display::print(int value, uint8_t x, uint8_t y)
{
    if (!ready)
        return;

    i2c::Guard bus(I2C_LOCK_TIMEOUT_MS);
    if (!bus.locked())
        return;

    if (x != 0 || y != 0)
    {
        oled->setCursor(x, y);
        cursorX = x;
        cursorY = y;
    }
    else
    {
        oled->setCursor(cursorX, cursorY);
    }
    oled->print(value);
    // Update cursor position after printing
    cursorX = oled->getCursorX();
    cursorY = oled->getCursorY();
}

void Display::printFloat(float value, uint8_t decimals)
{
    if (!ready)
        return;

    i2c::Guard bus(I2C_LOCK_TIMEOUT_MS);
    if (!bus.locked())
        return;

    oled->print(value, decimals);
    cursorX = oled->getCursorX();
    cursorY = oled->getCursorY();
}

void Display::println(const char *text, uint8_t x, uint8_t y)
{
    if (text == nullptr || !ready)
        return;

    i2c::Guard bus(I2C_LOCK_TIMEOUT_MS);
    if (!bus.locked())
        return;

    if (x != 0 || y != 0)
    {
        oled->setCursor(x, y);
    }
    else
    {
        oled->setCursor(cursorX, cursorY);
    }
    oled->println(text);
    cursorY += 8; // Move cursor down by one line
    oled->display();
}

void Display::setCursor(uint8_t x, uint8_t y)
{
    if (!ready)
        return;

    i2c::Guard bus(I2C_LOCK_TIMEOUT_MS);
    if (!bus.locked())
        return;

    cursorX = x;
    cursorY = y;
    oled->setCursor(x, y);
}

void Display::setTextSize(uint8_t size)
{
    if (!ready)
        return;

    i2c::Guard bus(I2C_LOCK_TIMEOUT_MS);
    if (!bus.locked())
        return;

    oled->setTextSize(size);
}

void Display::display()
{
    refresh();
}

void Display::refresh()
{
    if (!ready)
        return;

    if (!ready)
        return;

    i2c::Guard bus(I2C_LOCK_TIMEOUT_MS);
    if (!bus.locked())
        return;

    oled->display();
}

void Display::drawProgressBar(uint8_t percentage)
{
    if (!ready)
        return;

    i2c::Guard bus(I2C_LOCK_TIMEOUT_MS);
    if (!bus.locked())
        return;

    cursorX = 0;
    cursorY = 0;
    oled->clearDisplay();
    oled->print("OTA Progress:");

    // Draw progress bar
    uint8_t barWidth = 100;
    uint8_t barHeight = 10;
    uint8_t barX = 14;
    uint8_t barY = 20;

    oled->drawRect(barX, barY, barWidth, barHeight, SSD1306_WHITE);
    uint8_t fillWidth = (barWidth - 2) * percentage / 100;
    oled->fillRect(barX + 1, barY + 1, fillWidth, barHeight - 2, SSD1306_WHITE);

    // Display percentage
    oled->setCursor(50, 35);
    oled->print(percentage);
    oled->print("%");
    oled->display();
}
