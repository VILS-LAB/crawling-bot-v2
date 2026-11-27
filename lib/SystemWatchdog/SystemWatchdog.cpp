#include "SystemWatchdog.h"
#include "../../include/config.h"

#include <esp_err.h>
#include <esp_task_wdt.h>
#include <freertos/task.h>

namespace watchdog
{
namespace
{
bool initialized = false;
} // namespace

bool begin(uint32_t timeoutSeconds, bool panicOnTimeout)
{
#if !WATCHDOG_ENABLE
    // Explicitly off (bench debugging): never configure or subscribe anything.
    initialized = false;
    return false;
#else
    // esp_task_wdt_init() is documented as "safe to call again": it updates the
    // timeout and panic settings of an already running watchdog, and in this
    // project nothing else configures it.
    const esp_err_t err = esp_task_wdt_init(timeoutSeconds, panicOnTimeout);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
        Serial.printf("ERROR,WDT_INIT,%d\n", static_cast<int>(err));
        return false;
    }

    // IDF subscribes idle task 0 by default, and idle 0 only feeds the
    // watchdog when core 0 actually goes idle. This firmware watches its own
    // tasks explicitly, so leaving the idle subscription in place would turn a
    // busy WiFi/OTA stretch into a spurious reset now that the timeout is 3 s.
    const TaskHandle_t idle0 = xTaskGetIdleTaskHandleForCPU(0);
    if (idle0 != nullptr && esp_task_wdt_status(idle0) == ESP_OK)
        esp_task_wdt_delete(idle0);

    initialized = true;
    return addCurrentTask();
#endif
}

void feed()
{
    if (!initialized)
        return;
    esp_task_wdt_reset();
}

bool addCurrentTask()
{
    if (!initialized)
        return false;

    const esp_err_t err = esp_task_wdt_add(nullptr);
    // ESP_ERR_INVALID_ARG means "already subscribed", which is fine.
    return err == ESP_OK || err == ESP_ERR_INVALID_ARG;
}

void removeCurrentTask()
{
    if (!initialized)
        return;
    esp_task_wdt_delete(nullptr);
}

bool isCurrentTaskWatched()
{
    if (!initialized)
        return false;
    return esp_task_wdt_status(nullptr) == ESP_OK;
}

Suspend::Suspend() : wasWatched(isCurrentTaskWatched())
{
    if (wasWatched)
        removeCurrentTask();
}

Suspend::~Suspend()
{
    if (wasWatched)
        addCurrentTask();
}

} // namespace watchdog
