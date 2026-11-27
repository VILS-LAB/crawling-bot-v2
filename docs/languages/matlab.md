# MATLAB

MATLAB can talk TCP directly with `tcpclient`. Useful for quick experiments and
for plotting without leaving the course's usual environment.

```matlab
function robot = connectRobot(address, port)
    if nargin < 1, address = "192.168.4.1"; end
    if nargin < 2, port = 4242; end
    robot = tcpclient(address, port, "Timeout", 5);
    configureTerminator(robot, "LF");
end

function reply = command(robot, text)
    % Send one command; skip telemetry/calibration events while waiting.
    writeline(robot, text);
    while true
        line = readline(robot);
        if startsWith(line, "OK") || startsWith(line, "ERR")
            reply = line;
            return
        end
        % IMU,/CAL, event: keep reading
    end
end

function snap = snapshot(robot)
    % GET returns key=value pairs separated by commas.
    line = command(robot, "GET");
    body = extractAfter(line, "SNAP,");
    parts = split(body, ",");
    snap = struct();
    for k = 1:numel(parts)
        pair = split(parts{k}, "=");
        if numel(pair) == 2
            snap.(pair{1}) = str2double(pair{2});
        end
    end
end
```

Use it:

```matlab
robot = connectRobot("192.168.4.1");

disp(command(robot, "PING"));
snap = snapshot(robot);
fprintf("roll %.2f pitch %.2f yaw %.2f\n", snap.roll, snap.pitch, snap.yaw);

command(robot, "ACT 120 60");
command(robot, "PARK");

clear robot     % closes the connection
```

## Notes

* `configureTerminator(robot, "LF")` matches the robot's `\n`.
* Keep one `tcpclient` for the session; reconnect after a robot reboot.
* For live plots, enable `STREAM 50` and read `IMU,` lines with `readline` in a
  loop, then parse the CSV with `sscanf` or `split`.

## Serial instead of TCP

```matlab
robot = serialport("COM3", 115200, "Timeout", 5);
configureTerminator(robot, "LF");
command(robot, "STREAM 0");   % silence telemetry first
```
