### What vl mini does
1. so vl mini stores something in the first block and skips it.
2. keeps writing data from the imu to the sd card until it gets filled up.

# Data layout
/// m/s^2
acc: [f32; 3],
/// deg/s
gyro: [f32; 3],
