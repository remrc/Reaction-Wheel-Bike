# Reaction-Wheel-Bike

Gimbal controller BGC 3.1 (use as Arduino), gimbal motor EMAX 2806-100KV, MPU6050, magnetic encoder AS5048A connected over SPI.
One of the servos I modified for continuous rotation.

I use the Simple FOC library to control the BLDC motor.

Both bikes works with the same firmware. Difference only in design.

For remote control I use Joy BT Commander.

Balancing controller can be tunning remotely over bluetooth.

Example:

Send p+ (or p+p+p+p+p+p+p+) for increase K1.

Send p- (or p-p-p-p-p-p-p-) for decrease K1.

The same for K2, K3, K4. Send "s", "i", "d".

<img src="/pictures/bike0.jpg" alt="Bike pic"/>
<img src="/pictures/bike1.jpg" alt="Bike pic"/>
 
More about this bikes:

https://youtu.be/UzjqdoTVhOU

https://youtu.be/6snsCNyjHRU

https://youtu.be/17oNO7piRTU
