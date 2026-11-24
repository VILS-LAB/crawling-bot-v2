# مقایسه‌ی مسیر فعلی IMU با PX4 و Betaflight

این سند مسیر پیاده‌شده‌ی فعلی را مقایسه می‌کند؛ نه مسیرهای برنامه‌ریزی‌شده یا
نسخه‌های قدیمی archive شده.

| موضوع | پروژه‌ی فعلی | PX4 | Betaflight |
|---|---|---|---|
| هدف | attitude و dead reckoning کوتاه‌مدت crawler | navigation و کنترل پرواز با state estimator | attitude/rate control سریع |
| فریم | FLU / world N-W-U | FRD / NED | معمولاً NED/FRD یا قرارداد داخلی سازگار |
| gyro | median(3) -> notch -> LPF2p | low-pass، notch و dynamic notch | low-pass/dynamic notch/RPM filter |
| accel | LPF2p 30 Hz | فیلتر شده قبل از estimator/EKF | فیلترهای زاویه و لرزش |
| mag | LPF2p 5 Hz | fusion و magnetic declination | معمولاً نقش ثانویه یا کنترل هدینگ |
| attitude | پورت `attitude_estimator_q` PX4 | attitude/position/velocity estimator | quaternion/gyro control internals |
| calibration accel | شش وضعیت با ماتریس کامل `3×3` | guided multi-position و quality gate | معمولاً ساده‌تر؛ روی attitude فوکوس است |
| calibration mag | LM sphere سپس ellipsoid | fitting و magnetic model | محدود و وابسته به کاربرد |
| سکون | پنجره ۴۰ نمونه، ۳ بیت fail، هیسترزیس accel | معیارهای stationary و estimator state | مبتنی روی آرگومان‌های control/gyro |
| موقعیت | dead reckoning کوتاه‌مدت | GPS/flow/vision/range/LiDAR + EKF | هدف اصلی position نیست |

## مسیر PX4 که در پروژه آمده

پروژه موارد زیر را مستقیماً یا با port محلی دارد:

- فیلتر median، notch و low-pass از PX4؛
- `attitude_estimator_q` با bias state؛
- کالیبراسیون gyro، accel و mag با quality gate؛
- fit sphere/ellipsoid برای magnetometer؛
- مدل اصلاح کامل ماتریسی accelerometer.

تغییرات عمده نسبت به PX4 عبارت‌اند از قرارداد FLU/N-W-U، واحدهای پروژه،
ماتریس کامل accel، وزن estimator کوچکتر برای gyro bias و نبود EKF/GPS/uORB.

## تشخیص سکون فعلی

پنجره ۴۰ نمونه روی accel calibrated است. بیت ۰ میانگین norm شتاب را می‌سنجد:

- ورود به حالت still فقط وقتی خطا کمتر از `0.03g` باشد؛
- خروج از حالت still وقتی خطا بالاتر از `0.05g` باشد.

بیت ۱ gyro کمتر از `0.5 dps` را می‌سنجد و بیت ۲ انحراف بردار accel کمتر از
`0.004g` را می‌سنجد. ماسک صفر به‌همراه hold ۲۰۰ms و velocity reset را سکون اعلام
می‌کند.

## Betaflight

Betaflight به‌خصوص برای loopهای سریع و سرویس‌دهی به attitude/rate controller
طراحی شده است. استفاده از dynamic notch، RPM filter و فیلترهای کم‌latency برای
آن مناسب است اما خودکار کردن همه‌ی آن‌ها برای crawler بدون سنسور واقعی vibration
یا telemetry موتور پروژه‌ی مستقلی لازم دارد.

## جمع‌بندی

پروژه از PX4 برای ریاضیات estimator، filters و calibration استفاده می‌کند اما
یک EKF کامل، fusion GPS/vision یا مدل position چندحسگری ندارد. بنابراین برای
attitude و حرکت کوتاه مفید است؛ برای موقعیت طولانی‌مدت و مطلق باید مرجع بیرونی
و estimator موقعیت اضافه شود.
