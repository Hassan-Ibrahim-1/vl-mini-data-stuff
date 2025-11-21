### What vl mini does
1. so vl mini stores something in the first block and skips it.
2. keeps writing data from the imu to the sd card until it gets filled up.

# Data layout
/// m/s^2
acc: [f32; 3],
/// deg/s
gyro: [f32; 3],

### Hmm

sd_card_read seems fine, I don't think I have to modify it much. I have to
figure out what needs to be modified in main config. Shouldn't be too much
since they're both stm32's but still.
