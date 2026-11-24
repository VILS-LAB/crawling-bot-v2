# راهنمای عملی IMU/AHRS

این سند مرجع روزآمد کار با مسیر فعلی MPU9250 در Crawling Bot V2 است.
جزئیات معماری در [مستند معماری IMU](imu/ARCHITECTURE.md) و مراحل کالیبراسیون در
[راهنمای کالیبراسیون](imu/CALIBRATION.md) آمده است.

## سخت‌افزار و تنظیمات اصلی

| مورد | مقدار |
|---|---|
| IMU | GY-91 / MPU9250، آدرس I2C `0x68` |
| Magnetometer | AK8963 درون MPU9250 |
| شتاب‌سنج | `±4g` |
| ژیروسکوپ | `±500 dps` |
| نرخ نمونه | `200 Hz` |
| Magnetometer | `100 Hz` |
| پردازش | task اختصاصی روی core 1 |

فریم بدنه **FLU** است: محور X جلو، Y چپ و Z بالا. فریم جهانی X north،
Y west و Z up است. دوران شتاب و attitude در همین قرارداد پروژه انجام می‌شود.

## کالیبراسیون

مانیتور سریال را با baud `115200` باز کنید و کلید `q` را بفرستید:

```text
1 = gyro
2 = accelerometer
3 = magnetometer
4 = هر سه
5 = status
6 = پاک‌کردن رکورد
0 = بستن منوی IMU
```

### gyro

ربات را روی سطح ثابت نگه دارید. collection حدود ۱۲ ثانیه و قبل از آن settle
حدود ۱.۵ ثانیه اجرا می‌شود. انحراف معیار هر محور نباید از `0.20 dps` و اختلاف
میانه با میانگین از `0.6 dps` بیشتر باشد.

### accelerometer

روبای شش وضعیت هدایت می‌شود: X+/X−، Y+/Y− و Z+/Z−. هر وضعیت باید با gyro کمتر
از `3 dps` ثابت باشد و اندازه‌ی accel بین `0.85g` تا `1.15g` قرار داشته باشد.
برای هر وضعیت ۱۸۰ نمونه جمع می‌شود. خروجی شامل bias و ماتریس کامل `3×3` است.

### magnetometer

ربات را ۲۰ ثانیه آرام و در جهات گوناگون، ترجیحاً در شکل ۸، بچرخانید. شدت میدان
زمین معمولاً باید در بازه‌ی حدود `20–70 µT` باشد و RMS خوب معمولاً زیر `1 µT`
است. فیت نهایی می‌تواند sphere یا ellipsoid باشد.

رکورد calibration برای هر شماره‌ی ربات در NVS ذخیره می‌شود. کلید `5` وضعیت
آخرین calibration را چاپ می‌کند و کلید `6` رکورد فعلی را پاک می‌کند.

## مسیر پردازش

```text
raw accel/gyro/mag
 -> correction با calibration
 -> gyro: median(3) -> notch -> low-pass 40 Hz
 -> accel: low-pass 30 Hz
 -> mag: low-pass 5 Hz
 -> attitude estimator PX4
 -> تشخیص سکون و ZUPT
 -> rotation به world، حذف gravity و dead reckoning
 -> telemetry
```

Estimator شتاب را برای roll/pitch و magnetometer را برای yaw اصلاح می‌کند و bias
ژیروسکوپ را نیز تخمین می‌دهد. این مسیر quaternion ولی تبدیل شتاب برای contract
ثبت‌شده‌ی پروژه همچنان به‌صورت دوران ترتیبی X سپس Y سپس Z انجام می‌شود.

## تشخیص سکون

سه بررسی درون پنجره‌ی لغزان ۴۰ نمونه اجرا می‌شود:

| بیت | شرط |
|---|---|
| ۰ | میانگین |accel| نسبت به ۱g؛ ورود زیر `0.03g` و خروج بالای `0.05g` |
| ۱ | norm gyro کمتر از `0.5 dps` |
| ۲ | standard deviation بردار accel کمتر از `0.004g` |

پنجاه نمونه در ثانیه نیست: در نرخ ۲۰۰Hz پنجره حدود ۲۰۰ms داده می‌گیرد. پس از
۲۰۰ms پیوسته با ماسک صفر، ربات static اعلام می‌شود و velocity صفر می‌شود.

`still=1` تضمینی برای نبود حرکت مطلق نیست؛ ماسک و stdA را همراه هم بررسی کنید.

## telemetry و ZUPT

خط CSV هر `100ms` ارسال می‌شود و یک header نیز در boot و هر ۵ ثانیه ارسال می‌شود.
ستون‌های مهم عبارت‌اند از:

```text
ax..az       accel calibrated، m/s²، بدنه
gx..gz       gyro bias-corrected، dps
roll/pitch/yaw
vx..vz       velocity dead reckoning
px..pz       position dead reckoning
stdA         انحراف بردار accel در پنجره ۴۰ نمونه
still        وضعیت نهایی سکون
still_failure_mask
             بیت ۰: میانگین norm نسبت به هیسترزیس سکون
             بیت ۱: gyro
             بیت ۲: انحراف بردار accel
trust        وزن تصحیح accelerometer
accel_rms_g  residual کالیبراسیون accel
mag_rms_ut   residual کالیبراسیون mag
```

ZUPT فقط velocity را هنگام سکون صفر می‌کند. موقعیت با انتگرال دوبل شتاب باقی
می‌ماند و dead reckoning کوتاه‌مدت است، نه جای GPS یا fusion بیرونی.
