#ifndef COMMAND_SERVER_H
#define COMMAND_SERVER_H

// ASCII line protocol, shared by USB serial and TCP.
//
//   PING                        -> OK PONG <uptime_ms>
//   ID                          -> OK id=<n> ssid=<ssid> ip=<ip>
//   SETID <1..8>                -> OK id=<n>
//   GET                         -> OK SNAP,<key=value fields>
//   STREAM <0..50>              -> OK stream=<hz>
//   ACT <down> <up>             -> OK ACT d=<clamped_d> u=<clamped_u>
//   STOP                        -> OK STOP
//   PARK                        -> OK PARK
//   CAL [all|gyro|accel|mag]    -> OK CAL sensors=<list>
//
// Every command replies with exactly one OK or ERR line, so a client always
// knows whether the robot accepted it. While STREAM is on, the robot also emits
// `IMU,...` CSV frames at the requested rate; those are events, not replies, so
// the existing visualizers keep working.
//
// See docs/PROTOCOL.md for the grammar, the snapshot fields and examples.

#include <Arduino.h>
#include <WiFi.h>

#include "../../include/config.h"
#include "AHRS.h"
#include "Network.h"
#include "ServoControl.h"

class CommandServer
{
public:
    CommandServer(AHRS *ahrs, ServoControl *servos, Network *network);
    ~CommandServer();

    // Opens the TCP listener on PROTOCOL_TCP_PORT. Call after the AP is up.
    void begin();

    // Reads serial and TCP input, dispatches complete lines and emits STREAM
    // frames. Non-blocking apart from an explicit CAL command.
    void update();

private:
    struct Client
    {
        WiFiClient socket;
        char buffer[PROTOCOL_LINE_MAX] = {};
        size_t length = 0;
        uint8_t streamHz = 0;
        uint32_t lastStreamMs = 0;
    };

    AHRS *ahrs;
    ServoControl *servos;
    Network *network;

    // max_clients is the listen backlog; the firmware serves at most
    // PROTOCOL_MAX_CLIENTS established connections at once.
    WiFiServer server{PROTOCOL_TCP_PORT, PROTOCOL_MAX_CLIENTS};
    Client clients[PROTOCOL_MAX_CLIENTS];

    // Streaming is per transport: the serial port keeps the legacy default
    // rate so the existing visualizers work unchanged, while every TCP client
    // starts quiet and opts in with STREAM <hz>.
    uint8_t serialStreamHz;
    uint32_t serialLastStreamMs;
    uint32_t serialHeaderMs;

    // The column header is re-sent periodically (and on STREAM) so a client that
    // attaches later can map columns by name.
    static constexpr uint32_t kHeaderRepeatMs = 5000;

    char serialBuffer[PROTOCOL_LINE_MAX] = {};
    size_t serialLength;

    // --- transports --------------------------------------------------------
    void pollSerial();
    void pollTcp();
    void acceptClients();
    void closeClient(Client &client);

    // --- dispatch ----------------------------------------------------------
    // `client` is nullptr for the USB serial port, so a handler always knows
    // which transport to answer on.
    void handleLine(char *line, Client *client);

    void reply(Client *client, const char *line);

    void cmdPing(Client *client);
    void cmdId(Client *client);
    void cmdSetId(Client *client, char *argument);
    void cmdGet(Client *client);
    void cmdStream(Client *client, char *argument);
    void cmdAct(Client *client, char *arguments);
    void cmdStop(Client *client);
    void cmdPark(Client *client);
    void cmdCal(Client *client, char *argument);

    // --- helpers -----------------------------------------------------------
    void emitStream();
    void sendHeader(Client *client);
    void maybeRepeatSerialHeader();

    // CAL progress is routed back to whichever client asked for it.
    static void calProgressSink(const char *line, void *context);
    Client *calClient;
};

#endif // COMMAND_SERVER_H
