// Host-side tests for the robot command protocol grammar.
//
// The parser is deliberately free of Arduino/WiFi/FreeRTOS, so the exact code
// the firmware runs can be exercised here without hardware.

#include <stdio.h>
#include <string.h>

#include "../../lib/CommandServer/CommandParser.h"

namespace
{

int gFailures = 0;
int gChecks = 0;

void check(bool condition, const char *what)
{
    ++gChecks;
    if (!condition)
    {
        ++gFailures;
        printf("  FAIL: %s\n", what);
    }
}

void section(const char *name)
{
    printf("\n[%s]\n", name);
}

// Parses a mutable copy of `text` and reports the verb.
command::Verb verbOf(const char *text)
{
    char buffer[96];
    snprintf(buffer, sizeof(buffer), "%s", text);
    return command::parse(buffer).verb;
}

void testVerbs()
{
    section("verb recognition");

    check(verbOf("PING") == command::Verb::Ping, "PING");
    check(verbOf("ID") == command::Verb::Id, "ID");
    check(verbOf("SETID 3") == command::Verb::SetId, "SETID");
    check(verbOf("GET") == command::Verb::Get, "GET");
    check(verbOf("STREAM 20") == command::Verb::Stream, "STREAM");
    check(verbOf("ACT 10 20") == command::Verb::Act, "ACT");
    check(verbOf("STOP") == command::Verb::Stop, "STOP");
    check(verbOf("PARK") == command::Verb::Park, "PARK");
    check(verbOf("CAL") == command::Verb::Cal, "CAL");
    check(verbOf("CAL mag") == command::Verb::Cal, "CAL with argument");
}

void testCaseInsensitivity()
{
    section("case insensitivity");

    check(verbOf("ping") == command::Verb::Ping, "lowercase ping");
    check(verbOf("Ping") == command::Verb::Ping, "mixed case Ping");
    check(verbOf("\tCAL\tGYRO") == command::Verb::Cal, "leading tab");
}

void testWhitespaceHandling()
{
    section("whitespace");

    char buffer[96];

    // Extra spaces between verb and argument must not produce empty tokens.
    snprintf(buffer, sizeof(buffer), "ACT    10    20");
    command::Request request = command::parse(buffer);
    check(request.verb == command::Verb::Act, "ACT with wide spacing");

    char *cursor = request.argument;
    check(strcmp(command::nextToken(cursor), "10") == 0, "first ACT token is 10");
    check(strcmp(command::nextToken(cursor), "20") == 0, "second ACT token is 20");
    check(command::nextToken(cursor) == nullptr, "no third ACT token");

    // Trailing whitespace only.
    snprintf(buffer, sizeof(buffer), "PING   ");
    request = command::parse(buffer);
    check(request.verb == command::Verb::Ping, "PING with trailing spaces");
    check(request.argument == nullptr, "PING has no argument");

    // Empty and whitespace-only lines are not commands, and must not be
    // reported to the client as an unknown-command error.
    snprintf(buffer, sizeof(buffer), "");
    check(command::parse(buffer).verb == command::Verb::Invalid, "empty line");
    check(!command::parse(buffer).hadVerb, "empty line had no verb");
    snprintf(buffer, sizeof(buffer), "   \t ");
    check(command::parse(buffer).verb == command::Verb::Invalid, "whitespace-only line");
    check(!command::parse(buffer).hadVerb, "whitespace-only line had no verb");
}

void testUnknownVerbs()
{
    section("unknown verbs");

    check(verbOf("HELLO") == command::Verb::Invalid, "unknown verb");

    char buffer2[96];
    snprintf(buffer2, sizeof(buffer2), "HELLO");
    check(command::parse(buffer2).hadVerb, "unknown verb still had a verb token");
    check(verbOf("PINGX") == command::Verb::Invalid, "prefix is not a match");
    check(verbOf("d 90") == command::Verb::Invalid, "legacy servo short form");
    check(verbOf("q") == command::Verb::Invalid, "legacy menu key");
}

void testArguments()
{
    section("arguments");

    char buffer[96];
    snprintf(buffer, sizeof(buffer), "SETID 4");
    command::Request request = command::parse(buffer);
    check(request.argument != nullptr && strcmp(request.argument, "4") == 0,
          "SETID argument");

    snprintf(buffer, sizeof(buffer), "SETID");
    request = command::parse(buffer);
    check(request.verb == command::Verb::SetId, "SETID without argument still parses");
    check(request.argument == nullptr, "SETID without argument has no argument text");

    snprintf(buffer, sizeof(buffer), "CAL  all ");
    request = command::parse(buffer);
    check(request.argument != nullptr && strcmp(request.argument, "all") == 0,
          "CAL argument trimmed");
}

void testNumbers()
{
    section("whole numbers");

    long value = 0;
    check(command::parseWholeNumber("90", value) && value == 90, "90");
    check(command::parseWholeNumber("-20", value) && value == -20, "-20");
    check(command::parseWholeNumber(" 45 ", value) && value == 45, "padded 45");

    check(!command::parseWholeNumber(nullptr, value), "null rejected");
    check(!command::parseWholeNumber("", value), "empty rejected");
    check(!command::parseWholeNumber("abc", value), "letters rejected");
    check(!command::parseWholeNumber("12abc", value), "trailing junk rejected");
    check(!command::parseWholeNumber("1.5", value), "fraction rejected");
}

void testStreamClamp()
{
    section("stream rate clamp");

    check(command::clampStreamHz(0, 50) == 0, "0 stays off");
    check(command::clampStreamHz(10, 50) == 10, "10 passes through");
    check(command::clampStreamHz(50, 50) == 50, "max passes through");
    check(command::clampStreamHz(999, 50) == 50, "above max clamps");
    check(command::clampStreamHz(-3, 50) == -1, "negative signals a range error");
}

} // namespace

int main()
{
    printf("command protocol parser tests\n");

    testVerbs();
    testCaseInsensitivity();
    testWhitespaceHandling();
    testUnknownVerbs();
    testArguments();
    testNumbers();
    testStreamClamp();

    printf("\n%d checks, %d failures\n", gChecks, gFailures);
    return gFailures == 0 ? 0 : 1;
}
