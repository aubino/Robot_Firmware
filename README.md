[![Build](https://github.com/aubino/Robot_Firmware/actions/workflows/pio_ci.yml/badge.svg)](https://github.com/aubino/Robot_Firmware/actions/workflows/pio_ci.yml)

# Robot_Firmware
This package contains the low level firmware of my mobile differential drive robot. 
This package is able to do the following functions : 
- Estimate wheel speed for each wheel
- Perform a closed looped control over the wheels
- Compute direct and inverse Kinematics
- Deduce odometry infos
- Recieve commands through ROS topics with WiFi
  

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
- For continuous integration and unitests look [here](https://piolabs.com/blog/insights/unit-testing-part-1.html) for PlatformIO instructions . 
- For ros trough wifi : https://registry.platformio.org/libraries/frankjoshua/Rosserial%20Arduino%20Library/examples/Odom/Odom.pde
