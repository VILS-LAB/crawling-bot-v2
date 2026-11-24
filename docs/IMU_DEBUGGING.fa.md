# عیب‌یابی و ثبت داده‌ی IMU

## اجرای visualizer

Firmware روی Serial با سرعت `115200` خطوط CSV با پیشوند `IMU,` می‌فرستد:

```bash
python3 -m pip install pyserial matplotlib
python3 tools/imu_visualizer.py /dev/cu.usbmodem1101
```

برای نگه‌داشتن نمونه‌های بیشتر:

```bash
python3 tools/imu_visualizer.py /dev/cu.usbmodem1101 --samples 150
```

## تست سکون

ربات را روی سطح ثابت بگذارید و چند ثانیه صبر کنید. در وضعیت سکون:

- `lax,lay,laz` باید نزدیک صفر باشند؛
- `flax,flay,flaz` باید نرم‌تر و نزدیک صفر باشند؛
- velocity باید صفر بماند؛
- `still` باید یک باشد؛
- `moving` باید صفر باشد.

## تست تک‌محوره

ربات را فقط در یک محور حرکت دهید و این سه ستون را مقایسه کنید:

```text
lax,lay,laz
flax,flay,flaz
vx,vy,vz
```

مقدار کمی coupling به‌علت نصب و cross-axis طبیعی است، اما یک مؤلفه‌ی بزرگ و
پایدار در محور دیگر نشان‌دهنده‌ی مشکل mounting، attitude یا calibration است.

## کنترل سریال

استریم به‌صورت پیش‌فرض فعال است. برای منوی کوتاه `q` بفرستید:

```text
1 gyro
2 accelerometer
3 magnetometer
4 همه
5 status
6 پاک‌کردن calibration
0 بستن منو
```

در زمان calibration، `x` لغو و `?` راهنماست. جزئیات کامل مراحل در
[IMU_GUIDE.fa.md](./IMU_GUIDE.fa.md) آمده است.

## تفسیر ستون‌های مهم

```text
ax..az       accelerometer calibrated، فریم بدنه
lax..laz     linear acceleration، قبل از فیلتر
flax..flaz   linear acceleration، بعد از فیلتر و ورودی velocity
gx..gz       gyro calibrated
fgx..fgz     gyro filtered و ورودی attitude
roll/pitch/yaw attitude
vx..vz       velocity dead reckoning
px..pz       position dead reckoning
still        سکون تأییدشده پس از hold حداقل ۲۰۰ms
still_failure_mask
             بیت ۰: میانگین norm شتاب پنجره‌ی ۴۰ نمونه خارج از هیسترزیس سکون
             (ورود زیر 0.03g، خروج بالای 0.05g)
             بیت ۱: norm ژیروسکوپ حداقل 0.5 dps
             بیت ۲: انحراف معیار norm شتاب در پنجره‌ی ۴۰ نمونه
```

موقعیت IMU-only در مسیر طولانی قابل اعتماد نیست و باید با مرجع بیرونی ارزیابی
شود.
