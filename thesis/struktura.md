

Motivation and context
Goal

Driver description

FOC/SVPWM introduction

Peripheral implementation
    - MCU config
        - CUBEMX 
        - timers
        - tim1 - pwm inject (periodic timer)
    - driver
        - spi communication
        - configuration
        - ADC calibration
    - encoder
        - spi communication
        - angle calculation
        - calibration
        - turns calculation
        - velocity calculation
        - velocity filtering
        - mechanical vs electrical angle


    - CAN communication
        - communication protocol
        - daisy chaining
    - CAN client showcase

    - PI/PID control loops

Software topology
User abstraction layer


Test/Benchmark
Results

Future improvements - discussion
Conclusion