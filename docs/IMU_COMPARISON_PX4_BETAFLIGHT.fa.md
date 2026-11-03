# مقایسه‌ی روش فعلی با PX4 و Betaflight

این مقایسه درباره‌ی مسیر واقعی فعلی پروژه است، نه قابلیت‌هایی که هنوز
پیاده‌سازی نشده‌اند.

## خلاصه

| موضوع | پروژه‌ی فعلی | PX4 | Betaflight |
|---|---|---|---|
| هدف اصلی | اندازه‌گیری و dead reckoning کوتاه‌مدت crawler | تخمین حالت و کنترل پرواز | کنترل سریع پرواز |
| کالیبراسیون gyro | bias و sigma با تشخیص سکون | guided calibration و کنترل کیفیت | کالیبراسیون سریع و runtime مناسب flight controller |
| accelerometer | شش‌وضعیتی، bias/scale مستقل | چندجهتی با بررسی کیفیت و alignment | بیشتر مناسب attitude و کنترل، نه ellipsoid کامل در مسیر عادی |
| اصلاح cross-axis | ندارد | در calibrationهای کامل‌تر دارد | معمولاً تمرکز اصلی روی latency و vibration است |
| gyro filter | Median(5)، Butterworth، Adaptive scalar | low-pass، notch و estimator | low-pass، dynamic notch و RPM filter |
| accel filter | Median(5)، Butterworth 8Hz، Adaptive scalar | فیلتر قبل از EKF و مدل estimator | فیلترهای کنترل و attitude |
| attitude | complementary، بدون quaternion برای دوران پروژه | EKF/ECL یا estimator کامل | attitude سریع برای کنترل |
| bias state | gyro bias ثابت از calibration | biasها در estimator تخمین زده می‌شوند | بسته به estimator و کاربرد کنترل |
| position | دو انتگرال‌گیری IMU | fusion با GPS/vision/flow/range و غیره | هدف اصلی position نیست |
| ZUPT | deadband و velocity reset ساده | innovation/state estimator | معمولاً برای position مستقل استفاده نمی‌شود |

## کالیبراسیون

### پروژه‌ی فعلی

کاربر با OLED و Serial در شش وضعیت هدایت می‌شود. برای هر محور دو سمت گرفته
می‌شود و از میانگین آن‌ها bias و scale ساخته می‌شود. این روش:

- ساده و قابل انجام بدون ابزار آزمایشگاهی است؛
- برای خطای offset و gain مناسب است؛
- cross-axis coupling و misalignment کامل را اندازه نمی‌گیرد.

gyro با نمونه‌های ساکن و sigma بررسی می‌شود. magnetometer مسیر داخلی کتابخانه
را دارد. رکورد هر ربات جداگانه در NVS ذخیره می‌شود و self-check در boot دارد.

### PX4

PX4 برای calibration کاربر را در وضعیت‌های مشخص راهنمایی می‌کند، داده‌ی کافی
جمع می‌کند و کیفیت پوشش/پایداری را بررسی می‌کند. در سامانه‌ی کامل، خروجی
calibration به estimator و بررسی‌های preflight متصل است. مزیت مهم PX4 این است
که calibration بخشی از یک زنجیره‌ی بزرگ‌تر شامل تخمین bias و innovation check
است، نه فقط تبدیل raw به corrected.

### Betaflight

Betaflight برای flight controller روی راه‌اندازی سریع، gyro قابل اعتماد و
رفتار مناسب در ارتعاش تمرکز دارد. calibration آن باید کم‌هزینه و مناسب حلقه‌ی
کنترل باشد. این با نیاز پروژه‌ی ما برای اندازه‌گیری سه‌محوره و dead reckoning
کوتاه‌مدت یکسان نیست.

## فیلتر و ارتعاش

پروژه‌ی فعلی برای هر محور این زنجیره را دارد:

```text
Median(5) -> Butterworth -> scalar adaptive Kalman
```

Median spikeهای کوتاه را حذف می‌کند. Butterworth لرزش‌های بالاتر از cutoff را
کم می‌کند. Adaptive Kalman با innovation بزرگ اعتماد به اندازه‌گیری را کاهش
می‌دهد. خروجی filtered accelerometer مستقیماً وارد velocity integration می‌شود.

PX4 معمولاً فیلتر را بخشی از estimator می‌بیند و covariance، innovation gating،
bias و مدل حرکت را همراه آن مدیریت می‌کند. بنابراین فیلتر فعلی از نظر ساختار
جای EKF2 را نمی‌گیرد.

Betaflight علاوه بر low-pass از dynamic notch و در کاربردهای مناسب RPM filtering
استفاده می‌کند. این فیلترها به فرکانس واقعی موتور و ارتعاش وابسته‌اند و بدون
اندازه‌گیری vibration نباید کورکورانه به پروژه‌ی crawler اضافه شوند.

## attitude

پروژه‌ی فعلی:

- gyro فیلترشده را برای propagation استفاده می‌کند؛
- accelerometer فقط در حالت نزدیک به سکون roll/pitch را آهسته اصلاح می‌کند؛
- yaw بدون مرجع بیرونی drift می‌کند؛
- تبدیل شتاب طبق دوران ترتیبی X سپس Y سپس Z انجام می‌شود.

PX4 از estimator چندحالته استفاده می‌کند که attitude، velocity، position و
biasها را در یک مدل مشترک نگه می‌دارد. این باعث می‌شود accelerometer صرفاً با
یک شرط ساده‌ی norm وارد attitude نشود و measurementها innovation و covariance
داشته باشند.

Betaflight عمدتاً attitude را برای کنترل سریع می‌خواهد، نه position مطلق.
بنابراین latency و پاسخ سریع معمولاً از دقت dead reckoning مهم‌تر است.

## position و ZUPT

در پروژه‌ی فعلی:

```text
filtered acceleration -> velocity -> position
```

و در سکون velocity صفر می‌شود. این یک ZUPT کامل نیست؛ چون contact model،
covariance، innovation و تخمین هم‌زمان bias ندارد. deadband از drift کوتاه‌مدت
کم می‌کند، ولی bias بسیار کوچک در مسیر طولانی همچنان position را خراب می‌کند.

PX4 در صورت وجود GPS، optical flow، vision، range یا مرجع دیگر این اندازه‌گیری‌ها
را با EKF fusion می‌کند. بدون مرجع بیرونی، PX4 هم از IMU تنها موقعیت مطلق
پایدار تولید نمی‌کند.

Betaflight معمولاً position estimator عمومی ندارد، چون مأموریت اصلی آن کنترل
attitude و rate است.

## چه چیزهایی از PX4 برای این پروژه مفید است؟

1. guided calibration با تشخیص کیفیت و پیام واضح؛
2. نگه‌داشتن gyro bias در estimator، نه فقط یک مقدار ثابت؛
3. innovation gating برای ردکردن شتاب نامعتبر هنگام حرکت؛
4. covariance یا confidence تطبیقی برای accelerometer؛
5. ZUPT واقعی فقط پس از ساختن stationary detector قابل اعتماد؛
6. مرجع بیرونی در صورت نیاز به position طولانی‌مدت.

## چه چیزهایی از Betaflight مفید است؟

1. فیلتر سریع و قابل تنظیم gyro؛
2. اندازه‌گیری vibration واقعی قبل از انتخاب notch؛
3. پرهیز از فیلترهای بیش‌ازحد کند در مسیر کنترل؛
4. جداکردن مسیر telemetry از مسیر سریع sensor update.

## جمع‌بندی صادقانه

پروژه‌ی فعلی برای calibration دستی، attitude پایدار، مشاهده‌ی acceleration و
حرکت کوتاه مناسب‌تر شده است، اما هنوز PX4 EKF2 یا estimator کامل Betaflight
نیست. مهم‌ترین تفاوت با PX4 نبودن state estimator با bias/covariance و fusion
مرجع بیرونی است. مهم‌ترین تفاوت با Betaflight نبودن dynamic notch/RPM filtering
و متفاوت‌بودن هدف crawler است.
