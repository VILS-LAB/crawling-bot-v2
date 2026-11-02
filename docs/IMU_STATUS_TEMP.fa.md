# وضعیت موقت IMU

این سند وضعیت فعلی IMU را تا قبل از اضافه‌کردن EKF ثبت می‌کند.

## ساختار ماژول‌ها

- `ImuCalibration.*`: داده‌ی calibration، ellipsoid fit، اصلاح ماتریسی و NVS.
- `ImuFilters.*`: median سه‌نمونه‌ای، low-pass Butterworth مرتبه‌ی دوم و Adaptive Kalman.
- `ImuRotation.*`: دوران body-to-world و complementary attitude.
- `AHRS.*`: مدیریت سنسور، telemetry، کنسول و orchestration.

## کارهایی که تاکنون انجام شده

- تنظیم MPU9250 روی بازه‌ی `±4g` و `±500 dps`، نرخ نمونه‌برداری 200Hz و DLPF حدود 41/45Hz.
- کالیبراسیون مستقل gyro با 2500 نمونه، محاسبه‌ی bias و sigma و ردکردن calibration پرنویز.
- کالیبراسیون accelerometer اکنون به روش شش‌وضعیتی راهنمایی‌شده انجام می‌شود: هر محور دو بار، با نگه‌داشتن ربات ثابت در دو سمت مخالف.
- برای هر وضعیت ۱۸۰ نمونه‌ی پایدار جمع می‌شود. از میانگین دو سمت هر محور bias و scale مستقل محاسبه می‌شود؛ این روش برای کالیبراسیون دستی ساده‌تر و قابل تکرارتر است، اما مانند ellipsoid fit کامل، misalignment بین محورها را تخمین نمی‌زند.
- کاربر لازم نیست مقدار local `g` را وارد کند؛ داده‌ی MPU9250 بر حسب `g` نرمال شده است و مقدار محلی گرانش برای این کاربرد تفاوت ناچیزی دارد.
- استخراج bias و ماتریس اصلاحی کامل `3x3` برای accelerometer:
  `a_corrected = M_acc * (a_raw - bias)`.
- ذخیره‌ی calibration در NVS بر اساس شماره‌ی ربات.
- کالیبراسیون مستقل magnetometer با مسیر کتابخانه‌ی فعلی.
- پنل Serial برای اجرای جداگانه‌ی `cal gyro`، `cal accel`، `cal mag` و `cal all`.
- امکان لغو calibration با `x`، گزارش زمان باقی‌مانده و پیشرفت، و بازخورد LED.
- complementary attitude:
  - انتگرال‌گیری gyro برای roll/pitch/yaw
  - تصحیح آهسته‌ی roll/pitch با accelerometer فقط وقتی norm نزدیک 1g است.
- تصحیح accelerometer برای attitude فقط وقتی انجام می‌شود که هم norm در بازه‌ی نزدیک 1g باشد و هم gyro تقریباً ساکن باشد؛ این مانع تفسیر شتاب خطی ربات به‌عنوان تغییر شیب می‌شود.
- median سه‌نمونه‌ای برای حذف spike، low-pass Butterworth مرتبه‌ی دوم و scalar Adaptive Kalman برای شتاب خطی، gyro و magnetometer.

## مسیر دقیق فیلترها

Calibration روی نمونه‌ی خام انجام می‌شود و فیلتر نیست:

```text
raw accelerometer
  -> ellipsoid correction
  -> gravity removal
  -> median(3)
  -> Butterworth 2nd order, 12 Hz
  -> scalar Adaptive Kalman
  -> filteredLinearAccel
```

```text
raw gyro
  -> bias subtraction
  -> median(3)
  -> Butterworth 2nd order, 25 Hz
  -> scalar Adaptive Kalman
  -> filteredGyro
  -> complementary attitude
```

```text
raw magnetometer
  -> library calibration correction
  -> median(3)
  -> Butterworth 2nd order, 5 Hz
  -> scalar Adaptive Kalman
  -> filteredMag
```

Adaptive Kalman فعلی یک فیلتر scalar مستقل برای هر محور است، نه EKF و نه PX4 EKF2. وقتی innovation بزرگ شود، measurement noise مؤثر افزایش می‌یابد و فیلتر کمتر به آن نمونه اعتماد می‌کند. این فیلتر هنوز covariance کامل بین محورها، مدل دینامیکی، bias state یا fusion چندحسگری ندارد.
- تشخیص سکون با نزدیک‌بودن norm شتاب calibrated به `1g` و کوچک‌بودن gyro.
- هنگام سکون، velocity صفر می‌شود؛ این از drift جلوگیری می‌کند اما estimator کامل نیست.
- خروجی CSV قابل کنترل با `stream on` و `stream off`.
- visualizer جداگانه‌ی X/Y/Z و نمایش مسیر تقریبی سه‌بعدی از `px,py,pz`.

## چه چیزهایی هنوز نداریم؟

| قابلیت | وضعیت فعلی |
|---|---|
| Adaptive weighting | فقط داخل scalar Adaptive Kalman و بر اساس innovation |
| Adaptive Kalman | داریم؛ مستقل برای هر محور، اما نه EKF کامل |
| EKF2 | نداریم |
| EKF کامل وضعیت/سرعت/بایاس | نداریم |
| ZUPT واقعی | نداریم؛ فقط velocity reset هنگام تشخیص سکون داریم |
| تشخیص تماس پا/لغزش | نداریم |
| مرجع بیرونی برای position | نداریم |
| اصلاح mounting rotation با مرجع بدنه | نداریم |

## مقایسه با PX4

PX4 معمولاً یک زنجیره‌ی کامل‌تر دارد:

1. calibration و بررسی کیفیت سنسور؛
2. فیلتر low-pass و notch متناسب با ارتعاش؛
3. تخمین attitude با الگوریتم‌هایی مانند EKF2/ECL؛
4. state vector شامل attitude، velocity، position و biasها؛
5. measurement fusion از GPS، optical flow، range، vision یا سایر مراجع؛
6. innovation gating و تنظیم adaptive اعتماد measurementها.

کد فعلی فقط بخش calibration، low-pass ساده، complementary attitude و dead-reckoning داخلی را پوشش می‌دهد. بنابراین مسیر سه‌بعدی فعلی برای حرکت کوتاه و مقایسه‌ی کیفی مناسب است، نه موقعیت مطلق یا مسیر طولانی.

## مقایسه با Betaflight

Betaflight برای کنترل پرواز تمرکز بیشتری روی حلقه‌ی سریع و کم‌تاخیر دارد:

- gyro با نرخ بالا؛
- gyro low-pass، dynamic notch و در صورت نیاز RPM filtering؛
- کالیبراسیون و مدیریت دما/سنسور؛
- attitude برای کنترل سریع، نه تخمین موقعیت مطلق با IMU تنها.

در پروژه‌ی ما dynamic notch و RPM filtering هنوز وجود ندارد و چون crawler مرجع RPM/ارتعاشی تعریف‌شده ندارد، نباید بدون اندازه‌گیری ارتعاش مستقیماً کپی شوند.

## وضعیت position

موقعیت فعلی از دو انتگرال‌گیری به‌دست می‌آید:

```text
acceleration -> velocity -> position
```

حتی bias بسیار کوچک accelerometer در این روش به drift بزرگ منجر می‌شود. تشخیص سکون و صفرکردن velocity کمک می‌کند، اما جایگزین EKF یا ZUPT مبتنی بر measurement واقعی نیست. نمودار position در visualizer باید با عنوان `dead reckoning` تفسیر شود.

## ترتیب پیشنهادی مراحل بعدی

1. ثبت داده‌ی خام و calibrated در حالت سکون و حرکت کنترل‌شده.
2. اندازه‌گیری ارتعاش و انتخاب low-pass/notch مناسب.
3. اضافه‌کردن confidence تطبیقی accelerometer بر اساس norm و gyro.
4. تبدیل static reset فعلی به ZUPT صریح با نوآوری و آستانه‌های قابل گزارش.
5. اضافه‌کردن estimator سبک برای attitude/bias.
6. فقط در صورت نیاز واقعی، پیاده‌سازی EKF کامل با state و measurementهای مشخص.
