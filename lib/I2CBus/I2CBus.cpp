#include "I2CBus.h"
#include "../../include/config.h"

namespace i2c
{
namespace
{
SemaphoreHandle_t busMutex = nullptr;
bool busStarted = false;
} // namespace

bool begin()
{
    if (busMutex == nullptr)
    {
        busMutex = xSemaphoreCreateRecursiveMutex();
        if (busMutex == nullptr)
        {
            Serial.println("ERROR,I2C_MUTEX_CREATE_FAILED");
            return false;
        }
    }

    if (!busStarted)
    {
        // Start the peripheral here, once, so neither driver races the other
        // into Wire.begin(). Display and AHRS both pass periphBegin = false.
        if (!Wire.begin())
        {
            Serial.println("ERROR,I2C_BEGIN_FAILED");
            return false;
        }
        Wire.setClock(I2C_CLOCK_HZ);
        busStarted = true;
    }

    return true;
}

bool lock(uint32_t timeoutMs)
{
    if (busMutex == nullptr)
        return false;
    return xSemaphoreTakeRecursive(busMutex, pdMS_TO_TICKS(timeoutMs)) == pdTRUE;
}

void unlock()
{
    if (busMutex != nullptr)
        xSemaphoreGiveRecursive(busMutex);
}

Guard::Guard(uint32_t timeoutMs) : acquired(lock(timeoutMs))
{
}

Guard::~Guard()
{
    if (acquired)
        unlock();
}

} // namespace i2c
