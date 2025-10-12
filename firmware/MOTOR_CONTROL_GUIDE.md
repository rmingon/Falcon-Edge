# Motor & Servo Control Guide

## Hardware Configuration

Your fixed-wing drone now has full motor and servo control!

### Pin Assignments

| Component | GPIO | Signal Type |
|-----------|------|-------------|
| **Left Wing Servo** | GPIO 26 | PWM (50Hz) |
| **Right Wing Servo** | GPIO 25 | PWM (50Hz) |
| **ESC (Motor)** | GPIO 33 | PWM (50Hz) |

### PWM Signal Specs

- **Frequency**: 50 Hz (20ms period)
- **Servo Range**: 1000-2000μs (±45° from center)
- **Servo Center**: 1500μs (neutral position)
- **ESC Range**: 1000-2000μs (0-100% throttle)
- **ESC Idle**: 1100μs (motor spinning, low throttle)

## Control Mixing (Fixed-Wing)

Your firmware implements **fixed-wing control mixing** that converts PID outputs to servo/motor commands:

### Roll Control (Ailerons)
```
Left Servo Angle  = +Roll_Output * 0.3
Right Servo Angle = -Roll_Output * 0.3
```
- Roll right (+): Left servo up, right servo down
- Roll left (-): Left servo down, right servo up

### Pitch Control (Elevators)
```
Both Servos = +Pitch_Output * 0.3
```
- Pitch up (+): Both servos up
- Pitch down (-): Both servos down

### Yaw Control (Rudder via Differential)
```
Left Servo  += +Yaw_Output * 0.1
Right Servo += -Yaw_Output * 0.1
```
- Small differential aileron for yaw stability

### Altitude Control (Throttle)
```
Throttle = 50% + Altitude_Output * 0.5
```
- Base throttle: 50%
- Adjusts ±50% based on altitude PID

## Safety Features

### Arming Sequence
1. System checks calibration
2. **ESC arming**: 2-second low throttle signal
3. PIDs reset to zero
4. Flight controller armed
5. Control outputs active

### Disarming
- All servos return to center (0°)
- Throttle set to 0%
- Control loop stops updating outputs

### Fail-Safe
When disarmed:
- Servos: Centered
- Motor: Zero throttle
- No PID updates

## Testing

### Bench Testing (NO PROPELLER!)

**Test servos:**
```c
// In your code or via serial commands
motor_set_servo_left(&motor, 20.0f);   // 20° up
motor_set_servo_right(&motor, -10.0f); // 10° down
```

**Test ESC (REMOVE PROPELLER FIRST!):**
```c
motor_set_throttle(&motor, 10.0f);  // 10% throttle - should spin slowly
```

### Serial Monitor Output

When arming:
```
I (12345) MotorControl: Motor control initialized - Left:26 Right:25 ESC:33
I (23456) FlightController: Arming flight controller
I (23457) MotorControl: Arming ESC sequence...
I (25457) MotorControl: ESC armed
I (25458) FlightController: Flight controller armed - ready to fly
```

## Tuning

### Servo Travel Adjustment

In [motor_control.c](src/motor_control.c:17):
```c
// Increase for more servo throw
float pulse = SERVO_CENTER_PULSE_US + (angle / 45.0f) * 500.0f;
                                                    // ^^^^ change this
```

### Control Surface Gains

In [flight_controller.c](src/flight_controller.c:216-226):
```c
// Aileron gain (roll)
left_servo += fc->roll_output * 0.3f;   // Increase for more roll authority

// Elevator gain (pitch)
left_servo += fc->pitch_output * 0.3f;  // Increase for more pitch authority

// Rudder gain (yaw)
left_servo += fc->yaw_output * 0.1f;    // Increase for more yaw authority
```

### Throttle Control

```c
float throttle = 50.0f;  // Base throttle (cruise speed)
throttle += fc->altitude_output * 0.5f;  // Altitude response gain
```

## Servo Reversing

If a servo moves in the wrong direction, reverse it in the mixing:

### Reverse Left Servo
```c
left_servo -= fc->roll_output * 0.3f;  // Changed + to -
```

### Reverse Right Servo
```c
right_servo += fc->roll_output * 0.3f;  // Changed - to +
```

## ESC Calibration

Some ESCs need calibration. If your ESC doesn't work:

1. **Power off** drone
2. Set throttle to **max** in code temporarily:
   ```c
   motor_set_throttle(&motor, 100.0f);
   ```
3. **Power on** drone with battery
4. Wait for ESC beeps (confirms max throttle)
5. Set throttle to **min**:
   ```c
   motor_set_throttle(&motor, 0.0f);
   ```
6. Wait for ESC beeps (confirms min throttle)
7. **Remove** temporary code, upload normal firmware

## Common Issues

### Servos jittering
- Check power supply (servos draw current)
- Add capacitor (100-470μF) to servo power rail
- Ensure common ground between ESP32 and servo power

### ESC not responding
- Check ESC is receiving 5V/3.3V signal (not vice versa)
- Verify PWM signal with oscilloscope
- Try ESC calibration procedure above
- Some ESCs need 3.3V → 5V level shifter

### Servos moving wrong direction
- See "Servo Reversing" section above
- Or swap servo connections physically

### ESC beeping continuously
- Usually means low voltage or no signal
- Check battery voltage
- Verify PWM signal present on GPIO 33

## API Reference

### Set Individual Outputs
```c
motor_set_servo_left(&motor, angle);   // -45 to +45 degrees
motor_set_servo_right(&motor, angle);  // -45 to +45 degrees
motor_set_throttle(&motor, percent);   // 0 to 100%
```

### Set All Outputs
```c
motor_set_all(&motor, left_angle, right_angle, throttle);
```

### Safety Functions
```c
motor_arm_esc(&motor);    // Run ESC arming sequence
motor_disarm(&motor);     // Stop all outputs
```

## Wiring Diagram

```
ESP32                     Servos/ESC
┌─────────┐
│ GPIO 26 ├──────────┬───► Left Wing Servo (Signal)
│         │          │
│ GPIO 25 ├──────────┼───► Right Wing Servo (Signal)
│         │          │
│ GPIO 33 ├──────────┼───► ESC (Signal)
│         │          │
│    GND  ├──────────┴───► Common Ground
└─────────┘

Power:
- Servos: 5V from BEC/ESC
- ESC: Main battery (LiPo)
- ESP32: 5V from USB or BEC (via voltage regulator)

⚠️ IMPORTANT: Common ground required!
```

## Next Steps

1. **Bench test** servos without propeller
2. **Calibrate ESC** if needed
3. **Tune mixing gains** for your aircraft
4. **Test on ground** with prop, full arm sequence
5. **Flight test** in stable conditions

Happy flying! 🚀
