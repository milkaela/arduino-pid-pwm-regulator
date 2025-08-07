# arduino-pid-pwm-regulator
This project implements a closed-loop PID controller on an Arduino using analog voltage input. The system allows the user to input a target voltage (in volts), then adjusts a PWM output signal in real-time to achieve and maintain the target using a PID algorithm.
## Features

- Serial-based configuration of:
  - Target voltage (in volts)
  - PID coefficients: P, I, D
- ADC-based input reading (from pin A5)
- PWM output to control an actuator (pin 4)
- Live data monitoring via Serial Monitor / Plotter
- Built-in integral anti-windup and noise filtering
- Adjustable sampling interval

## PID Formula

\[
\text{output} = P \cdot \text{error} + I \cdot \int{\text{error} \cdot dt} + D \cdot \frac{d(\text{error})}{dt}
\]

Where:
- **error** = target ADC value − measured ADC value
- **P** controls reaction to present error
- **I** compensates steady-state offset
- **D** dampens rapid changes

## How to Use

1. Upload the sketch to your Arduino (Uno/Nano/etc.)
2. Open the Serial Monitor at 9600 baud
3. Enter the desired **target voltage** (e.g., 2.5)
4. Enter the PID coefficients when prompted (e.g., P = 0.4, I = 0.05, D = 0.01)
5. Observe ADC and PWM feedback in Serial Monitor

## Wiring

- A5 → Analog voltage input (e.g., center of a potentiometer)
- 4 → PWM output (connect to LED, MOSFET, or driver circuit)
- Ensure GND and 5V connections are stable
