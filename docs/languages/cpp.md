# C / C++

No library needed: the protocol is line oriented. This snippet uses POSIX
sockets and compiles with `c++ -std=c++17 robot.cpp -o robot`.

```cpp
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <string>
#include <vector>

class Robot {
public:
    bool connect(const std::string &host, int port = 4242) {
        fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
        if (fd_ < 0) return false;
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(port);
        inet_pton(AF_INET, host.c_str(), &addr.sin_addr);
        return ::connect(fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) == 0;
    }

    // Sends one command, returns its OK/ERR reply. Event lines (IMU,/CAL,)
    // are handed to `on_event` instead of being returned.
    std::string command(const std::string &text,
                        void (*on_event)(const std::string &) = nullptr) {
        const std::string line = text + "\n";
        ::send(fd_, line.data(), line.size(), 0);

        std::string reply;
        while (true) {
            std::string current = read_line();
            if (current.empty()) continue;
            const bool is_reply = current.rfind("OK", 0) == 0 ||
                                  current.rfind("ERR", 0) == 0;
            if (is_reply) return current;
            if (on_event) on_event(current);
        }
    }

    ~Robot() { if (fd_ >= 0) ::close(fd_); }

private:
    std::string read_line() {
        while (buffer_.find('\n') == std::string::npos) {
            char chunk[1024];
            const ssize_t n = ::recv(fd_, chunk, sizeof(chunk), 0);
            if (n <= 0) return {};        // connection closed
            buffer_.append(chunk, static_cast<size_t>(n));
        }
        const size_t newline = buffer_.find('\n');
        std::string line = buffer_.substr(0, newline);
        buffer_.erase(0, newline + 1);
        if (!line.empty() && line.back() == '\r') line.pop_back();
        return line;
    }

    int fd_ = -1;
    std::string buffer_;
};
```

Use it:

```cpp
Robot bot;
if (!bot.connect("192.168.4.1")) return 1;

std::cout << bot.command("GET") << "\n";        // parse the key=value fields
std::cout << bot.command("ACT 120 60") << "\n";

// Live telemetry: STREAM first, then read frames.
bot.command("STREAM 50");
for (int i = 0; i < 100; ++i)
    std::cout << bot.command("GET") << "\n";    // or parse IMU, lines directly
```

## Notes

* One buffered `std::string` holds partial reads; never call `recv` per line.
* `ACT` replies with the **clamped** targets — read them, do not assume.
* Serial instead of TCP: open the device at 115200 with `termios`, or use
  PlatformIO's monitor during development.

## Build integration

If you already build C++ with CMake, drop the class into your project; there are
no firmware headers to include, which is the point of the protocol.
