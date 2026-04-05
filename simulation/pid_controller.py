"""PID Controller simulation for line follower robot."""
import numpy as np
from typing import List, Tuple, Dict, Optional
from dataclasses import dataclass, field


@dataclass
class PIDConfig:
    """PID controller configuration."""
    kp: float = 1.0
    ki: float = 0.0
    kd: float = 0.0
    setpoint: float = 0.0
    output_min: float = -255.0
    output_max: float = 255.0
    integral_limit: float = 100.0
    sample_time_ms: float = 10.0


@dataclass
class PIDState:
    """Internal state of PID controller."""
    error: float = 0.0
    previous_error: float = 0.0
    integral: float = 0.0
    derivative: float = 0.0
    output: float = 0.0
    history: List[Dict] = field(default_factory=list)


class PIDController:
    """Digital PID controller with anti-windup and derivative filtering."""

    def __init__(self, config: Optional[PIDConfig] = None):
        self.config = config or PIDConfig()
        self.state = PIDState()
        self._iteration = 0

    def compute(self, measurement: float) -> float:
        self._iteration += 1
        error = self.config.setpoint - measurement

        self.state.integral += error * (self.config.sample_time_ms / 1000.0)
        self.state.integral = np.clip(
            self.state.integral,
            -self.config.integral_limit,
            self.config.integral_limit
        )

        dt = self.config.sample_time_ms / 1000.0
        self.state.derivative = (error - self.state.previous_error) / dt if dt > 0 else 0.0

        output = (
            self.config.kp * error +
            self.config.ki * self.state.integral +
            self.config.kd * self.state.derivative
        )

        output = np.clip(output, self.config.output_min, self.config.output_max)

        self.state.error = error
        self.state.previous_error = error
        self.state.output = output

        self.state.history.append({
            "iteration": self._iteration,
            "error": error,
            "integral": self.state.integral,
            "derivative": self.state.derivative,
            "output": output,
            "measurement": measurement,
        })
        return output

    def reset(self) -> None:
        self.state = PIDState()
        self._iteration = 0

    def tune(self, kp: float, ki: float, kd: float) -> None:
        self.config.kp = kp
        self.config.ki = ki
        self.config.kd = kd
        self.reset()

    @property
    def performance_metrics(self) -> Dict:
        if not self.state.history:
            return {}
        errors = [h["error"] for h in self.state.history]
        return {
            "iterations": len(errors),
            "final_error": errors[-1],
            "max_error": max(abs(e) for e in errors),
            "avg_error": np.mean(np.abs(errors)),
            "settling_time": self._calculate_settling_time(errors),
            "overshoot": self._calculate_overshoot(errors),
        }

    def _calculate_settling_time(self, errors: List[float], threshold: float = 0.05) -> int:
        for i in range(len(errors) - 1, -1, -1):
            if abs(errors[i]) > threshold:
                return i + 1
        return 0

    def _calculate_overshoot(self, errors: List[float]) -> float:
        if not errors:
            return 0.0
        min_error = min(errors)
        return abs(min_error) if min_error < 0 else 0.0


class LineFollowerSimulator:
    """Simulates a line follower robot with PID control."""

    def __init__(self, pid: PIDController, track_width: float = 20.0):
        self.pid = pid
        self.track_width = track_width
        self.position = 0.0
        self.velocity = 0.0
        self.base_speed = 150.0
        self.history: List[Dict] = []

    def simulate_straight(self, steps: int = 100, noise_std: float = 0.5) -> List[Dict]:
        self.pid.reset()
        self.position = np.random.uniform(-2, 2)
        results = []

        for step in range(steps):
            sensor_reading = self.position + np.random.normal(0, noise_std)
            correction = self.pid.compute(sensor_reading)

            left_speed = self.base_speed + correction
            right_speed = self.base_speed - correction

            left_speed = np.clip(left_speed, 0, 255)
            right_speed = np.clip(right_speed, 0, 255)

            speed_diff = (right_speed - left_speed) / 255.0
            self.position += speed_diff * 2.0

            results.append({
                "step": step,
                "position": self.position,
                "sensor": sensor_reading,
                "correction": correction,
                "left_speed": left_speed,
                "right_speed": right_speed,
            })

        self.history = results
        return results

    def simulate_curve(self, steps: int = 100, curve_radius: float = 50.0,
                       noise_std: float = 0.5) -> List[Dict]:
        self.pid.reset()
        self.position = 0.0
        results = []

        for step in range(steps):
            curve_offset = (self.track_width / 2) * np.sin(step / curve_radius)
            sensor_reading = self.position - curve_offset + np.random.normal(0, noise_std)
            correction = self.pid.compute(sensor_reading)

            left_speed = np.clip(self.base_speed + correction, 0, 255)
            right_speed = np.clip(self.base_speed - correction, 0, 255)

            speed_diff = (right_speed - left_speed) / 255.0
            self.position += speed_diff * 2.0

            results.append({
                "step": step,
                "position": self.position,
                "target": curve_offset,
                "sensor": sensor_reading,
                "correction": correction,
            })

        self.history = results
        return results

    def evaluate_run(self) -> Dict:
        if not self.history:
            return {}
        positions = [h["position"] for h in self.history]
        return {
            "total_steps": len(self.history),
            "avg_deviation": float(np.mean(np.abs(positions))),
            "max_deviation": float(np.max(np.abs(positions))),
            "stayed_on_track": all(abs(p) < self.track_width / 2 for p in positions),
            "smoothness": float(np.std(np.diff(positions))),
        }


def ziegler_nichols_tuning(ku: float, tu: float, controller_type: str = "PID") -> PIDConfig:
    """Calculate PID gains using Ziegler-Nichols method."""
    configs = {
        "P":   PIDConfig(kp=0.5 * ku),
        "PI":  PIDConfig(kp=0.45 * ku, ki=1.2 * ku / tu),
        "PID": PIDConfig(kp=0.6 * ku, ki=2.0 * ku / tu, kd=ku * tu / 8.0),
    }
    if controller_type not in configs:
        raise ValueError(f"Unknown type: {controller_type}. Use P, PI, or PID")
    return configs[controller_type]
