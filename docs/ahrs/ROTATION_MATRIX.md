این سند قرارداد ماتریس دوران مسیر فعلی را مشخص می‌کند. مسیر اصلی دیگر از
`MPU9250::getLinearAcc*()` استفاده نمی‌کند؛ شتاب خام را با کواترنیون به فریم
جهانی می‌برد و سپس بردار جاذبه را در همان فریم کم می‌کند.

فرض می‌کنیم بردار شتاب اولیه برابر باشد با:

\\[ \mathbf{a}= \begin{bmatrix} AccX\\\ AccY\\\ AccZ \end{bmatrix} \\]

## ۱. دوران حول محور X

ماتریس دوران:

\\[ \boxed{ R_x(\theta_x)= \begin{bmatrix} 1&0&0\\\ 0&\cos\theta_x&-\sin\theta_x\\\ 0&\sin\theta_x&\cos\theta_x \end{bmatrix} } \\]

روابط شتاب پس از دوران:

\\[ \boxed{ \begin{aligned} AccX' &= AccX\\\ AccY' &= AccY\cos\theta_x-AccZ\sin\theta_x\\\ AccZ' &= AccY\sin\theta_x+AccZ\cos\theta_x \end{aligned} } \\]

## ۲. دوران حول محور Y

ماتریس دوران:

\\[ \boxed{ R_y(\theta_y)= \begin{bmatrix} \cos\theta_y&0&\sin\theta_y\\\ 0&1&0\\\ -\sin\theta_y&0&\cos\theta_y \end{bmatrix} } \\]

روابط شتاب پس از دوران:

\\[ \boxed{ \begin{aligned} AccX' &= AccX\cos\theta_y+AccZ\sin\theta_y\\\ AccY' &= AccY\\\ AccZ' &= -AccX\sin\theta_y+AccZ\cos\theta_y \end{aligned} } \\]

## ۳. دوران حول محور Z

ماتریس دوران:

\\[ \boxed{ R_z(\theta_z)= \begin{bmatrix} \cos\theta_z&-\sin\theta_z&0\\\ \sin\theta_z&\cos\theta_z&0\\\ 0&0&1 \end{bmatrix} } \\]

روابط شتاب پس از دوران:

\\[ \boxed{ \begin{aligned} AccX' &= AccX\cos\theta_z-AccY\sin\theta_z\\\ AccY' &= AccX\sin\theta_z+AccY\cos\theta_z\\\ AccZ' &= AccZ \end{aligned} } \\]

## ۴. ماتریس دوران کواترنیونی

برای کواترنیون نرمال‌شده‌ی `q = (w, x, y, z)`، ابتدا ماتریس استاندارد
`R(q)` ساخته می‌شود. چون کتابخانه داده را در قرارداد هواپیمایی داخلی با
تبدیل `(-bodyY, -bodyX, bodyZ)` به فیلتر می‌دهد، تبدیل نهایی معادل
`P * R(q) * P` است؛ در نتیجه:

```text
body' = [-bodyY, -bodyX, bodyZ]
world' = R(q) * body'
world  = [-world'Y, -world'X, world'Z]
```

سپس:

```text
world_acceleration = R * body_acceleration
linear_acceleration = world_acceleration - [0, 0, 9.80665]
```

این ترتیب مهم است: جاذبه باید بعد از تبدیل به فریم جهانی کم شود، نه این‌که
همیشه از مؤلفه‌ی Z بدنه کم شود.

## ۵. دوران هم‌زمان حول هر سه محور

اگر مانند کد خودت ابتدا حول X، سپس Y و در نهایت Z دوران انجام شود:

\\[ \boxed{ R=R_z(\theta_z)R_y(\theta_y)R_x(\theta_x) } \\]

در نتیجه روابط نهایی عبارت‌اند از:

\\[ \boxed{ \begin{aligned} RealAccX={}& \cos\theta_z \left[ AccX\cos\theta_y+ \sin\theta_y(AccY\sin\theta_x+AccZ\cos\theta_x) \right]\\\ &+\sin\theta_z(AccZ\sin\theta_x-AccY\cos\theta_x) \end{aligned} } \\]

\\[ \boxed{ \begin{aligned} RealAccY={}& \sin\theta_z \left[ AccX\cos\theta_y+ \sin\theta_y(AccY\sin\theta_x+AccZ\cos\theta_x) \right]\\\ &+\cos\theta_z(AccY\cos\theta_x-AccZ\sin\theta_x) \end{aligned} } \\]

\\[ \boxed{ RealAccZ= \cos\theta_y(AccY\sin\theta_x+AccZ\cos\theta_x) -AccX\sin\theta_y-9.81 } \\]

روابط Euler برای توضیح ترتیب دوران مفید هستند، اما در فریم‌ور از آن‌ها برای
محاسبه‌ی شتاب استفاده نمی‌شود؛ کواترنیون از فیلتر وضعیت کتابخانه مستقیماً به
ماتریس تبدیل می‌شود تا مشکل ترتیب دوران و تکینگی Euler وارد مسیر شتاب نشود.



```cpp
// Convert angles from degrees to radians
float AngleXr = radians(AngleX);
float AngleYr = radians(AngleY);
float AngleZr = radians(AngleZ);

// Rotation around X axis
float Ax1 = AccX;
float Ay1 = AccY * cos(AngleXr) - AccZ * sin(AngleXr);
float Az1 = AccY * sin(AngleXr) + AccZ * cos(AngleXr);

// Rotation around Y axis
float Ax2 = Ax1 * cos(AngleYr) + Az1 * sin(AngleYr);
float Ay2 = Ay1;
float Az2 = -Ax1 * sin(AngleYr) + Az1 * cos(AngleYr);

// Rotation around Z axis
RealAccX = Ax2 * cos(AngleZr) - Ay2 * sin(AngleZr);
RealAccY = Ax2 * sin(AngleZr) + Ay2 * cos(AngleZr);
RealAccZ = Az2;

// Remove gravity
RealAccZ = RealAccZ - 9.81f;
```
