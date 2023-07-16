[![Build](https://github.com/aubino/Robot_Firmware/actions/workflows/pio_ci.yml/badge.svg)](https://github.com/aubino/Robot_Firmware/actions/workflows/pio_ci.yml)

# Robot_Firmware
This package contains the low level firmware of my robot. 
It is used to drive wheels and poerform a closed loop control over them for now. 

# Hardware configuration 
I use the ESP32 to write this low level firmware, 6V DC motors with quadrature encoders driven by the L289N  H bridge. For Data fusion purposes, i'm also using the __MPU6050__ IMU to extract orientation.  
The pinout layout is as follows : 
```yaml
- Left wheel : 
    - Power : 
        - Forward : D15
        - Backward: D2
        - PWM : D4
    - Encoders : 
        - PinA : D17
        - PinB : D16
- Right wheel : 
    - Power : 
        - Forward : D26
        - Backward : D27 
        - PWM : D14
    - Encoders : 
        - PinA : D13
        - PinB : D12
```

# Software Configuration 
