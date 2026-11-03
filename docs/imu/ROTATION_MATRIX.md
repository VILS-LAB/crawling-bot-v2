# قرارداد دوران IMU

مسیر فعلی از قرارداد دوران ترتیبی مورد تأیید پروژه استفاده می‌کند:

```text
body acceleration -> rotation X -> rotation Y -> rotation Z -> world acceleration
```

برای زاویه‌های `x`, `y`, `z`:

```text
Ax1 = AccX
Ay1 = AccY*cos(x) - AccZ*sin(x)
Az1 = AccY*sin(x) + AccZ*cos(x)

Ax2 = Ax1*cos(y) + Az1*sin(y)
Az2 = -Ax1*sin(y) + Az1*cos(y)

RealAccX = Ax2*cos(z) - Ay1*sin(z)
RealAccY = Ax2*sin(z) + Ay1*cos(z)
RealAccZ = Az2
```

بعد از تبدیل به فریم جهانی، gravity reference از مؤلفه‌ی Z کم می‌شود. این
reference در سکون به‌آرامی با اندازه‌گیری واقعی تطبیق پیدا می‌کند تا offset
ثابت کوچک در Z باعث drift نشود.

کد اجرایی در [`ImuRotation.cpp`](../../lib/IMU/ImuRotation.cpp) و orchestration
در [`AHRS.cpp`](../../lib/IMU/AHRS.cpp) است. این سند قرارداد پروژه است و ادعا
نمی‌کند که این مسیر جایگزین EKF کامل است.
