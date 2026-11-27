#include "CommandParser.h"

#include <stdlib.h>
#include <string.h>
#include <strings.h>

namespace command
{
namespace
{
constexpr int kMaxHz = 50;

Verb verbFromToken(const char *token)
{
    if (token == nullptr)
        return Verb::Invalid;

    if (strcasecmp(token, "PING") == 0)
        return Verb::Ping;
    if (strcasecmp(token, "ID") == 0)
        return Verb::Id;
    if (strcasecmp(token, "SETID") == 0)
        return Verb::SetId;
    if (strcasecmp(token, "GET") == 0)
        return Verb::Get;
    if (strcasecmp(token, "STREAM") == 0)
        return Verb::Stream;
    if (strcasecmp(token, "ACT") == 0)
        return Verb::Act;
    if (strcasecmp(token, "STOP") == 0)
        return Verb::Stop;
    if (strcasecmp(token, "PARK") == 0)
        return Verb::Park;
    if (strcasecmp(token, "CAL") == 0)
        return Verb::Cal;

    return Verb::Invalid;
}
} // namespace

char *nextToken(char *&cursor)
{
    if (cursor == nullptr)
        return nullptr;

    while (*cursor == ' ' || *cursor == '\t')
        ++cursor;
    if (*cursor == '\0')
        return nullptr;

    char *token = cursor;
    while (*cursor != '\0' && *cursor != ' ' && *cursor != '\t')
        ++cursor;
    if (*cursor != '\0')
        *cursor++ = '\0';

    return token;
}

Request parse(char *line)
{
    Request request;
    if (line == nullptr)
        return request;

    char *cursor = line;
    char *verb = nextToken(cursor);
    request.hadVerb = verb != nullptr;
    request.verb = verbFromToken(verb);

    // The argument is everything after the verb, not just one token: ACT needs
    // two numbers and CAL may grow more stages. Inner spacing is preserved so a
    // command-specific parser can split it again; trailing whitespace is
    // trimmed so equality checks are simple.
    if (cursor != nullptr)
    {
        while (*cursor == ' ' || *cursor == '\t')
            ++cursor;

        char *end = cursor + strlen(cursor);
        while (end > cursor && (end[-1] == ' ' || end[-1] == '\t'))
            --end;
        *end = '\0';

        if (*cursor != '\0')
            request.argument = cursor;
    }

    return request;
}

bool parseWholeNumber(const char *text, long &value)
{
    if (text == nullptr || *text == '\0')
        return false;

    char *end = nullptr;
    const long parsed = strtol(text, &end, 10);
    if (end == text)
        return false;
    while (*end == ' ' || *end == '\t')
        ++end;
    if (*end != '\0')
        return false;

    value = parsed;
    return true;
}

int clampStreamHz(long requested, int maxHz)
{
    if (maxHz <= 0)
        maxHz = kMaxHz;
    if (requested < 0)
        return -1; // caller reports the range error
    if (requested > maxHz)
        return maxHz;
    return static_cast<int>(requested);
}

} // namespace command
