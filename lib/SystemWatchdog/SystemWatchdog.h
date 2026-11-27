#ifndef SYSTEM_WATCHDOG_H
#define SYSTEM_WATCHDOG_H

// Thin wrapper around esp_task_wdt.
//
// The Arduino core starts the loop task with the watchdog disconnected, so a
// firmware hang leaves the servos powered and the robot unresponsive. Here the
// loop task and the IMU sampling task subscribe explicitly, which turns a hang
// into a panic + reset.
//
// Blocking calibration is the one legitimate exception: it runs for minutes on
// the loop task while waiting for a human, so that task is unsubscribed for the
// duration instead of being fed from inside the calibration loops.

#include <Arduino.h>

namespace watchdog
{

// Configures the timeout and subscribes the calling task (the Arduino loop).
bool begin(uint32_t timeoutSeconds = 3, bool panicOnTimeout = true);

// Feeds the watchdog on behalf of the current task.
void feed();

// Subscribe / unsubscribe the current task, e.g. the IMU task entry point.
bool addCurrentTask();
void removeCurrentTask();

// True when the current task is currently watched.
bool isCurrentTaskWatched();

// RAII suspension for a blocking section on the current task.
class Suspend
{
public:
    Suspend();
    ~Suspend();

    Suspend(const Suspend &) = delete;
    Suspend &operator=(const Suspend &) = delete;

private:
    bool wasWatched;
};

} // namespace watchdog

#endif // SYSTEM_WATCHDOG_H
