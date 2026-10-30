"""
PID Controller implementation for drone movement control.
Implements proper PID with integral windup protection and output saturation.
"""

from utils.config import (
    KP_X, KI_X, KD_X,
    KP_Y, KI_Y, KD_Y,
    MAX_VELOCITY,
    DEADBAND,
    INTEGRAL_WINDUP_LIMIT,
    CONTROLLER_OUTPUT_SATURATION,
    DRY_RUN
)
from utils.logger import logger
import time


class PIDController:
    def __init__(self, kp, ki, kd, name="PID"):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.name = name

        self.reset()

    def reset(self):
        """Reset the PID controller"""
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = time.time()
        self.output = 0.0

    def update(self, error):
        """Update PID controller with error and return output"""
        current_time = time.time()
        dt = current_time - self.prev_time

        # Prevent division by zero and handle large dt
        if dt <= 0.0 or dt > 1.0:
            dt = 0.1

        # Proportional term
        proportional = self.kp * error

        # Integral term with windup protection
        self.integral += error * dt
        if abs(self.integral) > INTEGRAL_WINDUP_LIMIT:
            self.integral = INTEGRAL_WINDUP_LIMIT * (1 if self.integral > 0 else -1)
        integral = self.ki * self.integral

        # Derivative term
        derivative = self.kd * (error - self.prev_error) / dt if dt > 0 else 0.0

        # Calculate output
        output = proportional + integral + derivative

        # Apply deadband
        if abs(error) < DEADBAND:
            output = 0.0
            # Reset integral when in deadband to prevent windup
            self.integral = 0.0

        # Apply output saturation
        output = max(min(output, CONTROLLER_OUTPUT_SATURATION), -CONTROLLER_OUTPUT_SATURATION)

        # Store values for next iteration
        self.prev_error = error
        self.prev_time = current_time
        self.output = output

        if DRY_RUN:
            logger.debug(f"[{self.name}] Error: {error:.3f}, Output: {output:.3f}, P: {proportional:.3f}, I: {integral:.3f}, D: {derivative:.3f}")

        return output


# Global controller instances
pid_x = PIDController(KP_X, KI_X, KD_X, "PID_X")
pid_y = PIDController(KP_Y, KI_Y, KD_Y, "PID_Y")


def pixel_velocity(Error_X_m, Error_Y_m):
    """
    Calculate velocity commands from position errors using PID controllers.

    Args:
        Error_X_m: Error in X direction (meters)
        Error_Y_m: Error in Y direction (meters)

    Returns:
        tuple: (vx, vy) velocity commands in m/s
    """
    vx = pid_x.update(Error_X_m)
    vy = pid_y.update(Error_Y_m)

    # Apply final velocity limits
    vx = max(min(vx, MAX_VELOCITY), -MAX_VELOCITY)
    vy = max(min(vy, MAX_VELOCITY), -MAX_VELOCITY)

