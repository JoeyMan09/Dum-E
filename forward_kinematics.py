import math
import numpy as np


class ForwardKinematics:

    def __init__(self):
        self.L1 = 120
        self.L2 = 120
        self.L3 = 130
        self.H = 125

        self.wrist_commands = [
            85, 100, 120, 140, 150, 170
        ]

        self.wrist_heights = [
            0, 15, 35, 65, 85, 105
        ]

        self.wrist_angles = [
            math.degrees(math.asin(h / 110)) - 59
            for h in self.wrist_heights
        ]

    def wrist_angle(self, s2):

        if not 85 <= s2 <= 170:
            raise ValueError(
                "S2 outside calibrated range"
            )

        return float(np.interp(
            s2,
            self.wrist_commands,
            self.wrist_angles
        ))

    def calculate(self, s2, s4, s5, s6):

        theta2 = math.radians(self.wrist_angle(s2))
        theta4 = math.radians(90 - s4)
        theta5 = math.radians(s5 - 26)
        theta6 = math.radians(s6 - 90)

        a1 = theta5
        a2 = theta5 + theta4
        a3 = theta5 + theta4 + theta2

        r = (
            self.L1 * math.cos(a1)
            + self.L2 * math.cos(a2)
            + self.L3 * math.cos(a3)
        )

        z = (
            self.H
            + self.L1 * math.sin(a1)
            + self.L2 * math.sin(a2)
            + self.L3 * math.sin(a3)
        )

        x = r * math.cos(theta6)
        y = r * math.sin(theta6)

        return x, y, z


if __name__ == "__main__":

    fk = ForwardKinematics()

    x, y, z = fk.calculate(
        s2=85,
        s4=90,
        s5=85,
        s6=90
    )

    print(f"X: {x:.2f} mm")
    print(f"Y: {y:.2f} mm")
    print(f"Z: {z:.2f} mm")