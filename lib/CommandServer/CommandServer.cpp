#include "CommandServer.h"
#include "CommandParser.h"
#include "ImuTelemetry.h"

#include <errno.h>
#include <lwip/sockets.h>

namespace
{
// The longest reply is the GET snapshot. Measured worst case (all fields at
// their widest) is ~360 bytes, so 448 leaves headroom for new fields.
constexpr size_t kReplyMax = 448;

const char *const kUnknownCommand =
    "ERR unknown_command,use_PING_ID_SETID_GET_STREAM_ACT_STOP_PARK_CAL";

// WiFiClient::write() retries for up to ~10 s when the peer stops reading,
// which would starve the loop and trip the task watchdog. This writes with
// MSG_DONTWAIT instead and reports whether the whole line went out.
//
// A partial frame would corrupt the client's line framing, so the caller
// disconnects such a client rather than sending a truncated line.
enum class WriteResult
{
    Complete,
    WouldBlock,
    Partial,
    Failed,
};

WriteResult writeNonBlocking(WiFiClient &socket, const char *data, size_t length)
{
    const int descriptor = socket.fd();
    if (descriptor < 0)
        return WriteResult::Failed;

    const int sent = send(descriptor, data, length, MSG_DONTWAIT);
    if (sent == static_cast<int>(length))
        return WriteResult::Complete;
    if (sent > 0)
        return WriteResult::Partial;
    if (sent < 0 && (errno == EWOULDBLOCK || errno == EAGAIN || errno == ENOBUFS))
        return WriteResult::WouldBlock;
    return WriteResult::Failed;
}

} // namespace

CommandServer::CommandServer(AHRS *ahrs, ServoControl *servos, Network *network)
    : ahrs(ahrs), servos(servos), network(network),
      serialStreamHz(static_cast<uint8_t>(1000 / TELEMETRY_INTERVAL_MS)),
      serialLastStreamMs(0), serialHeaderMs(0), serialLength(0), calClient(nullptr)
{
}

CommandServer::~CommandServer()
{
    for (int i = 0; i < PROTOCOL_MAX_CLIENTS; ++i)
    {
        if (clients[i].socket)
            clients[i].socket.stop();
    }
    server.end();
}

void CommandServer::begin()
{
    server.begin();
    server.setNoDelay(true);

    // Announce the contract once so a fresh serial monitor knows both
    // transports are live and what the telemetry columns are.
    Serial.println(ImuTelemetryOutput::headerLine());
    Serial.print("PROTOCOL,READY,serial=");
    Serial.print(SERIAL_BAUD_RATE);
    Serial.print(",tcp_port=");
    Serial.print(PROTOCOL_TCP_PORT);
    Serial.print(",stream_hz=");
    Serial.println(serialStreamHz);
}

// ---------------------------------------------------------------------------
// Transports
// ---------------------------------------------------------------------------
void CommandServer::reply(Client *client, const char *line)
{
    if (client == nullptr)
    {
        Serial.println(line);
        return;
    }

    if (!client->socket)
        return;

    char buffer[kReplyMax + 2];
    const size_t length = strlen(line);
    if (length > sizeof(buffer) - 2)
        return;

    memcpy(buffer, line, length);
    buffer[length] = '\n';

    // Same reasoning as the stream path: never let a stalled peer block the
    // loop task long enough for the watchdog to fire. A peer that cannot take a
    // single short reply is not reading at all, so drop it rather than stay out
    // of sync (a missing reply would leave the client waiting forever).
    if (writeNonBlocking(client->socket, buffer, length + 1) != WriteResult::Complete)
    {
        Serial.println("PROTOCOL,CLIENT,dropped,reason=reply_failed");
        closeClient(*client);
    }
}

void CommandServer::pollSerial()
{
    while (Serial.available())
    {
        const char character = static_cast<char>(Serial.read());

        if (character == '\r' || character == '\n')
        {
            if (serialLength == 0)
                continue;
            serialBuffer[serialLength] = '\0';
            handleLine(serialBuffer, nullptr);
            serialLength = 0;
            continue;
        }

        if (serialLength + 1 < sizeof(serialBuffer))
        {
            serialBuffer[serialLength++] = character;
            continue;
        }

        // Overlong line: drop it and report, so the client is not left waiting
        // for a reply that will never arrive.
        serialLength = 0;
        reply(nullptr, "ERR line_too_long");
    }
}

void CommandServer::acceptClients()
{
    while (server.hasClient())
    {
        WiFiClient incoming = server.accept();
        if (!incoming)
            break;

        int slot = -1;
        for (int i = 0; i < PROTOCOL_MAX_CLIENTS; ++i)
        {
            if (!clients[i].socket || !clients[i].socket.connected())
            {
                slot = i;
                break;
            }
        }

        if (slot < 0)
        {
            incoming.print("ERR busy\n");
            incoming.stop();
            continue;
        }

        if (clients[slot].socket)
            clients[slot].socket.stop();
        clients[slot].socket = incoming;
        clients[slot].length = 0;
        clients[slot].streamHz = 0;
        clients[slot].lastStreamMs = 0;

        Serial.print("PROTOCOL,CLIENT,connected,ip=");
        Serial.println(incoming.remoteIP().toString().c_str());
    }
}

void CommandServer::closeClient(Client &client)
{
    if (client.socket)
        client.socket.stop();
    client.socket = WiFiClient();
    client.length = 0;
    client.streamHz = 0;
}

void CommandServer::pollTcp()
{
    acceptClients();

    for (int i = 0; i < PROTOCOL_MAX_CLIENTS; ++i)
    {
        Client &client = clients[i];
        if (!client.socket)
            continue;
        if (!client.socket.connected())
        {
            closeClient(client);
            continue;
        }

        while (client.socket.available() > 0)
        {
            const int raw = client.socket.read();
            if (raw < 0)
                break;
            const char character = static_cast<char>(raw);

            if (character == '\r' || character == '\n')
            {
                if (client.length == 0)
                    continue;
                client.buffer[client.length] = '\0';
                handleLine(client.buffer, &client);
                client.length = 0;
                continue;
            }

            if (client.length + 1 < sizeof(client.buffer))
            {
                client.buffer[client.length++] = character;
                continue;
            }

            client.length = 0;
            reply(&client, "ERR line_too_long");
        }
    }
}

// ---------------------------------------------------------------------------
// Dispatch
// ---------------------------------------------------------------------------
void CommandServer::handleLine(char *line, Client *client)
{
    const command::Request request = command::parse(line);

    switch (request.verb)
    {
    case command::Verb::Ping:
        cmdPing(client);
        break;
    case command::Verb::Id:
        cmdId(client);
        break;
    case command::Verb::SetId:
        cmdSetId(client, request.argument);
        break;
    case command::Verb::Get:
        cmdGet(client);
        break;
    case command::Verb::Stream:
        cmdStream(client, request.argument);
        break;
    case command::Verb::Act:
        cmdAct(client, request.argument);
        break;
    case command::Verb::Stop:
        cmdStop(client);
        break;
    case command::Verb::Park:
        cmdPark(client);
        break;
    case command::Verb::Cal:
        cmdCal(client, request.argument);
        break;
    case command::Verb::Invalid:
        // A blank line is not an error; a real but unknown verb is.
        if (request.hadVerb)
            reply(client, kUnknownCommand);
        break;
    }
}

void CommandServer::cmdPing(Client *client)
{
    char line[48];
    snprintf(line, sizeof(line), "OK PONG %lu", static_cast<unsigned long>(millis()));
    reply(client, line);
}

void CommandServer::cmdId(Client *client)
{
    char line[kReplyMax];
    snprintf(line, sizeof(line), "OK id=%u ssid=%s ip=%s",
             static_cast<unsigned>(network->getRobotNumber()), network->getSSID(),
             network->getIPAddress());
    reply(client, line);
}

void CommandServer::cmdSetId(Client *client, char *argument)
{
    long value = 0;
    if (!command::parseWholeNumber(argument, value) || value < MIN_ROBOT_NUM ||
        value > MAX_ROBOT_NUM)
    {
        reply(client, "ERR SETID_range,1_to_8");
        return;
    }

    if (!network->setRobotNumber(static_cast<uint8_t>(value)))
    {
        reply(client, "ERR SETID_save_failed");
        return;
    }

    char line[32];
    snprintf(line, sizeof(line), "OK id=%ld", value);
    reply(client, line);
}

void CommandServer::cmdGet(Client *client)
{
    // Copy the newest published snapshot so the reply is internally consistent
    // (the IMU task may publish a new one mid-format).
    ahrs->update();

    const ImuTelemetry &imu = ahrs->getTelemetry();
    char line[kReplyMax];
    const int written = snprintf(
        line, sizeof(line),
        "OK SNAP,id=%u,t=%lu,roll=%.2f,pitch=%.2f,yaw=%.2f,"
        "ax=%.3f,ay=%.3f,az=%.3f,gx=%.2f,gy=%.2f,gz=%.2f,"
        "mx=%.2f,my=%.2f,mz=%.2f,temp_c=%.2f,"
        "still=%d,trust=%.3f,rate_hz=%.1f,cal=%d,"
        "d=%d,dt=%d,u=%d,ut=%d,act_moving=%d",
        static_cast<unsigned>(network->getRobotNumber()),
        static_cast<unsigned long>(millis()), imu.roll, imu.pitch, imu.yaw,
        imu.accel[0], imu.accel[1], imu.accel[2], imu.gyro[0], imu.gyro[1],
        imu.gyro[2], imu.mag[0], imu.mag[1], imu.mag[2], imu.temperature,
        imu.still ? 1 : 0, imu.trust, imu.imuRateHz,
        ahrs->isCalibrated() ? 1 : 0, servos->getCurrentDownAngle(),
        servos->getDownTarget(), servos->getCurrentUpAngle(),
        servos->getUpTarget(), servos->isMoving() ? 1 : 0);

    if (written <= 0 || written >= static_cast<int>(sizeof(line)))
    {
        reply(client, "ERR SNAP_unavailable");
        return;
    }
    reply(client, line);
}

void CommandServer::cmdStream(Client *client, char *argument)
{
    long requested = 0;
    if (!command::parseWholeNumber(argument, requested) || requested < 0)
    {
        reply(client, "ERR STREAM_range,0_to_50");
        return;
    }

    const long hz = command::clampStreamHz(requested, PROTOCOL_MAX_STREAM_HZ);

    if (client == nullptr)
    {
        serialStreamHz = static_cast<uint8_t>(hz);
        serialLastStreamMs = millis();
    }
    else
    {
        client->streamHz = static_cast<uint8_t>(hz);
        client->lastStreamMs = millis();
    }

    char line[32];
    snprintf(line, sizeof(line), "OK stream=%ld", hz);
    reply(client, line);

    // Turn streaming on with the header in front of the data, so a client can
    // map columns without waiting for the periodic repeat.
    if (hz > 0)
        sendHeader(client);
}

void CommandServer::sendHeader(Client *client)
{
    if (client != nullptr)
        reply(client, ImuTelemetryOutput::headerLine());
}

void CommandServer::cmdAct(Client *client, char *arguments)
{
#if !ENABLE_SERVOS
    reply(client, "ERR ACT_servos_disabled");
#else
    char *cursor = arguments;
    char *downText = command::nextToken(cursor);
    char *upText = command::nextToken(cursor);
    long down = 0;
    long up = 0;

    if (!command::parseWholeNumber(downText, down) ||
        !command::parseWholeNumber(upText, up))
    {
        reply(client, "ERR ACT_usage,ACT_<down>_<up>");
        return;
    }

    // moveBothTo() clamps to the soft limits; report the clamped target so the
    // client learns what the robot actually accepted.
    servos->moveBothTo(static_cast<int>(down), static_cast<int>(up));

    char line[48];
    snprintf(line, sizeof(line), "OK ACT d=%d u=%d", servos->getDownTarget(),
             servos->getUpTarget());
    reply(client, line);
#endif
}

void CommandServer::cmdStop(Client *client)
{
#if !ENABLE_SERVOS
    reply(client, "ERR STOP_servos_disabled");
#else
    servos->stop();
    reply(client, "OK STOP");
#endif
}

void CommandServer::cmdPark(Client *client)
{
#if !ENABLE_SERVOS
    reply(client, "ERR PARK_servos_disabled");
#else
    servos->moveBothTo(180, 180);
    reply(client, "OK PARK");
#endif
}

void CommandServer::cmdCal(Client *client, char *argument)
{
    bool gyro = false;
    bool accel = false;
    bool mag = false;
    const char *label = "all";

    if (argument == nullptr || *argument == '\0' || strcasecmp(argument, "all") == 0)
    {
        gyro = accel = mag = true;
        label = "all";
    }
    else if (strcasecmp(argument, "gyro") == 0)
    {
        gyro = true;
        label = "gyro";
    }
    else if (strcasecmp(argument, "accel") == 0)
    {
        accel = true;
        label = "accel";
    }
    else if (strcasecmp(argument, "mag") == 0)
    {
        mag = true;
        label = "mag";
    }
    else
    {
        reply(client, "ERR CAL_usage,CAL_[all|gyro|accel|mag]");
        return;
    }

    if (!ahrs->isInitialized())
    {
        reply(client, "ERR CAL_imu_unavailable");
        return;
    }

    char line[48];
    snprintf(line, sizeof(line), "OK CAL sensors=%s", label);
    reply(client, line);

    // Blocking by design: the guided procedure pauses the IMU task, drives the
    // OLED and waits for a human. runGuidedCalibration() suspends the loop
    // task's watchdog subscription for the duration.
    calClient = client;
    ahrs->setProgressSink(&CommandServer::calProgressSink, this);
    const bool ok = ahrs->runGuidedCalibration(gyro, accel, mag);
    ahrs->setProgressSink(nullptr, nullptr);
    calClient = nullptr;

    char result[48];
    snprintf(result, sizeof(result), "CALRESULT,ok=%d,sensors=%s", ok ? 1 : 0, label);
    reply(client, result);
}

// ---------------------------------------------------------------------------
// Streaming
// ---------------------------------------------------------------------------
void CommandServer::maybeRepeatSerialHeader()
{
    if (serialStreamHz == 0)
        return;

    const uint32_t now = millis();
    if (serialHeaderMs != 0 && now - serialHeaderMs < kHeaderRepeatMs)
        return;

    serialHeaderMs = now;
    Serial.println(ImuTelemetryOutput::headerLine());
}

void CommandServer::emitStream()
{
    char line[512];
    size_t bytes = 0; // 0 means "not formatted yet for this tick"

    auto formatOnce = [&]() -> bool {
        const int length = ImuTelemetryOutput::formatLine(*ahrs, line, sizeof(line) - 1);
        if (length <= 0)
            return false;
        line[length] = '\n';
        bytes = static_cast<size_t>(length) + 1;
        return true;
    };

    maybeRepeatSerialHeader();

    const uint32_t now = millis();

    // Serial keeps running at the configured rate so the existing visualizers
    // are unaffected by the new TCP transport.
    if (serialStreamHz > 0)
    {
        const uint32_t intervalMs = 1000UL / serialStreamHz;
        if (now - serialLastStreamMs >= intervalMs)
        {
            // Only emit when the whole line fits, so a host that stops reading
            // cannot stall the loop (and with it the watchdog feed and the
            // servo ramp). Skipping a frame loses one sample, not liveness.
            if (formatOnce())
            {
                const int room = Serial.availableForWrite();
                if (room < 0 || static_cast<size_t>(room) >= bytes)
                {
                    serialLastStreamMs = now;
                    Serial.write(reinterpret_cast<const uint8_t *>(line), bytes);
                }
            }
        }
    }

    for (int i = 0; i < PROTOCOL_MAX_CLIENTS; ++i)
    {
        Client &client = clients[i];
        if (!client.socket || !client.socket.connected() || client.streamHz == 0)
            continue;

        const uint32_t intervalMs = 1000UL / client.streamHz;
        if (now - client.lastStreamMs < intervalMs)
            continue;
        client.lastStreamMs = now;

        if (bytes == 0 && !formatOnce())
            continue;

        const WriteResult result = writeNonBlocking(client.socket, line, bytes);
        if (result == WriteResult::Partial || result == WriteResult::Failed)
        {
            // The peer is not keeping up or is gone. Dropping the connection
            // keeps the byte stream well framed and never blocks the loop.
            Serial.println("PROTOCOL,CLIENT,dropped,reason=stream_stalled");
            closeClient(client);
        }
    }
}

void CommandServer::calProgressSink(const char *line, void *context)
{
    CommandServer *self = static_cast<CommandServer *>(context);
    if (self == nullptr)
        return;

    // Serial already carries the detailed CAL,... log emitted by AHRS, so only
    // the requesting TCP client needs the compact stage messages here.
    self->reply(self->calClient, line);
}

void CommandServer::update()
{
    pollSerial();
    pollTcp();
    emitStream();
}
