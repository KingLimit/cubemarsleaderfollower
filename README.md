# CubeMars Motor Leader Follower Control

This project uses an STM32 to control multiple CubeMars motors over CAN in a leader–follower setup. One motor is moved by hand (leader), and another motor follows its motion in real time.

The goal is to get smooth, responsive motion where the follower tracks the leader without jitter, overshoot, or instability. It’s designed to feel natural rather than rigid.

## Hardware
- STM32 Nucleo-F446RE  
- Waveshare CAN Bus Shield  
- CubeMars AK-series motor
  - AK70-10 KV100 Tested
 
## Software
- STM32CubeMX used to create and edit initial .ioc file and generate project
- STM32CubeIDE used to edit main.c and flash to board
- Python script used to log data for data analysis and plots

## Notes
CAN communication is handled through the Waveshare shield, with PB8/PB9 used for RX/TX. Basic filtering is applied to accept all messages.

The leader motor is passive and can be moved freely by hand, while the follower motor tracks its position using a PD controller.

An offset is applied when assigning a follower so it does not jump to the leader’s position immediately. This allows smooth engagement at any position.

A small amount of smoothing is applied to the follower command to reduce twitching and aggressive corrections.

Gains (Kp, Kd) and smoothing can be adjusted depending on the setup and how stiff or compliant you want the follower to feel.
