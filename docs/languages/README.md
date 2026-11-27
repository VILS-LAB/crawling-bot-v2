# Client guides by language

The robot speaks plain ASCII over a socket, so every language is equally
first-class. Each guide shows the same four things: open a connection, send one
command, read its reply while skipping event lines, and drive the legs.

Shared rules (true in every language):

1. **One reply per command.** Lines starting with `IMU,` or `CAL,` are events.
   Skip them while waiting for `OK`/`ERR`.
2. **One buffered reader per connection.** Recreating a reader per call can
   discard already-received bytes.
3. **Never block the robot.** Enable `STREAM <hz>` and drain it, or poll `GET`.

| Language | Guide | Reference implementation |
|---|---|---|
| Python | [python.md](python.md) | [`examples/python`](../../examples/python/README.md) — tested |
| C / C++ | [cpp.md](cpp.md) | snippets only |
| JavaScript / Node | [javascript.md](javascript.md) | snippets only |
| MATLAB | [matlab.md](matlab.md) | snippets only |
| Any other | use the Python guide as the reference | — |

The Python client is the reference: when this document and the firmware
disagree, the firmware wins and both are fixed together.

## Minimal exchange

```text
>> ID
<< OK id=1 ssid=ESP32-AP-1 ip=192.168.4.1
>> GET
<< OK SNAP,id=1,t=51402,roll=1.24,pitch=-0.31,yaw=97.05,...,cal=1,d=180,dt=180,u=180,ut=180,act_moving=0
>> ACT 120 60
<< OK ACT d=120 u=60
```

`>>` is client to robot, `<<` is robot to client. Field meanings are in
[../PROTOCOL.md](../PROTOCOL.md).
