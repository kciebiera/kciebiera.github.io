import control as ctrl
import random
import numpy as np

# Define system matrices
A = np.array([[1, 2], [3, 4]])
B = np.array([[5], [6]])


class System:
    def __init__(self):
        self.x = np.array([[1], [2]])  # Initial state

    def measure_state(self):
        return self.x

    def apply_control(self, u):
        dt = 0.01
        self.x = self.x + np.dot(A, self.x) * dt + np.dot(B, u) * dt

    def print_state(self):
        print(f"{self.x[0].item():.2f}, {self.x[1].item():.2f}")


system = System()

for _ in range(100):
    x = system.measure_state()
    # Compute control signal
    signal = random.uniform(-10, 10)

    system.apply_control(0)
    system.print_state()
