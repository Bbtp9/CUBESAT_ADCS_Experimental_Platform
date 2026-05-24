"""
ESP32 MicroPython example skeleton — single-axis reaction wheel control

This is a minimal, hardware-abstracted example. Replace `read_imu()` and
`set_wheel_torque(tau)` with the platform-specific sensor/motor driver code.
"""
import time
from math import radians

# --- Parameters ---
J = 0.000634
Jw = 4.607e-05
Kp = 0.05
Kd = 0.02
Kd_detumble = 0.03
tau_max = 0.002
omega_th = radians(0.5)

theta_ref = radians(30)
dt = 0.05  # control loop period [s]

def read_imu():
    # TODO: return (theta, omega)
    # Implement using IMU + AHRS or simple gyro/angle fusion
    return 0.0, 0.0

def set_wheel_torque(tau):
    # TODO: convert torque command to motor PWM / current command
    # Implement hardware driver here
    pass

def control_step(theta, omega):
    if abs(omega) > omega_th:
        tau_cmd = -Kd_detumble * omega
    else:
        # wrap error to [-pi,pi] not shown here for brevity
        err = theta - theta_ref
        tau_cmd = -Kp * err - Kd * omega

    # saturate
    if tau_cmd > tau_max:
        tau_cmd = tau_max
    elif tau_cmd < -tau_max:
        tau_cmd = -tau_max
    return tau_cmd

def main():
    last = time.ticks_ms()
    while True:
        theta, omega = read_imu()
        tau = control_step(theta, omega)
        set_wheel_torque(tau)

        # watchdog / safety: check limits, stop if out-of-bounds
        # TODO: implement safety checks (current, wheel speed, comms)

        # wait until next cycle
        now = time.ticks_ms()
        elapsed = (now - last) / 1000.0
        to_wait = dt - elapsed
        if to_wait > 0:
            time.sleep(to_wait)
        last = time.ticks_ms()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('Stopped')
