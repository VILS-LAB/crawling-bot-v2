#ifndef COMMAND_PARSER_H
#define COMMAND_PARSER_H

// Pure parsing helpers for the robot command protocol.
//
// Deliberately free of Arduino, WiFi and FreeRTOS so the grammar can be
// host-tested (see test/host/test_command_parser.cpp). The transport in
// CommandServer only splits lines and calls into here.

#include <stddef.h>

namespace command
{

// Commands, in the order they are documented in docs/PROTOCOL.md.
enum class Verb
{
    Invalid,
    Ping,
    Id,
    SetId,
    Get,
    Stream,
    Act,
    Stop,
    Park,
    Cal,
};

// A parsed command line. `argument` points into the caller's buffer (it is
// NUL-terminated in place) and is nullptr when the command had no arguments.
struct Request
{
    Verb verb = Verb::Invalid;
    char *argument = nullptr;
    // False for a line that contains no token at all (empty or whitespace only).
    // Such a line is not an error; a real but unknown verb is.
    bool hadVerb = false;
};

// Splits `line` in place into a verb and the remaining argument text.
// `line` must be mutable and NUL-terminated. Leading/trailing whitespace is
// ignored and the match is case-insensitive.
Request parse(char *line);

// Returns the next space/tab separated token and advances `cursor` past it.
char *nextToken(char *&cursor);

// Parses one token as a whole integer. Returns false when the token is missing,
// is not a number, or has trailing junk, so "ACT abc 10" can be rejected instead
// of silently becoming a move to 0.
bool parseWholeNumber(const char *text, long &value);

// Clamps a stream rate to the supported range [0, maxHz].
int clampStreamHz(long requested, int maxHz);

} // namespace command

#endif // COMMAND_PARSER_H
