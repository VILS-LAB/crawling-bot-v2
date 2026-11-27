# JavaScript / Node.js

Uses only the standard library (`net`).

```javascript
const net = require("net");

class Robot {
  constructor(socket) {
    this.socket = socket;
    this.buffer = "";
    this.pending = [];      // resolver for the command in flight
  }

  static connect(host = "192.168.4.1", port = 4242) {
    return new Promise((resolve, reject) => {
      const socket = net.connect({ host, port }, () => resolve(new Robot(socket)));
      socket.on("error", reject);
    });
  }

  _attach() {
    this.socket.setEncoding("ascii");
    this.socket.on("data", (chunk) => {
      this.buffer += chunk;
      let newline;
      while ((newline = this.buffer.indexOf("\n")) >= 0) {
        const line = this.buffer.slice(0, newline).replace(/\r$/, "");
        this.buffer = this.buffer.slice(newline + 1);
        this._onLine(line);
      }
    });
  }

  _onLine(line) {
    if (!line) return;
    // A reply resolves the waiting command; anything else is an event.
    if ((line.startsWith("OK") || line.startsWith("ERR")) && this.pending.length) {
      const { resolve, reject } = this.pending.shift();
      if (line.startsWith("ERR")) {
        const [code, detail = ""] = line.slice(4).split(",");
        reject(new Error(detail ? `${code}: ${detail}` : code));
      } else {
        resolve(line);
      }
      return;
    }
    if (this.onEvent) this.onEvent(line);
  }

  command(text, timeoutMs = 2000) {
    return new Promise((resolve, reject) => {
      const timer = setTimeout(() => reject(new Error(`timeout: ${text}`)), timeoutMs);
      this.pending.push({
        resolve: (v) => { clearTimeout(timer); resolve(v); },
        reject: (e) => { clearTimeout(timer); reject(e); },
      });
      this.socket.write(text + "\n");
    });
  }

  close() { this.socket.end(); }
}
```

Use it:

```javascript
const bot = await Robot.connect("192.168.4.1");
bot._attach();

bot.onEvent = (line) => {
  if (line.startsWith("IMU,")) { /* telemetry frame */ }
};

console.log(await bot.command("GET"));
console.log(await bot.command("ACT 120 60"));

await bot.command("STREAM 50");
setTimeout(() => bot.command("STREAM 0").then(() => bot.close()), 5000);
```

## Notes

* Node streams can deliver several lines per `data` event; always keep a buffer.
* The robot drops a client that stops reading while streaming. Handle `close`
  and reconnect.
* In a browser, use WebSocket → a small proxy, or `fetch` against a bridge; a
  page cannot open raw TCP.
