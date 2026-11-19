
import numpy as np

# Measured gripper-horizontal calibration points.
# Shoulder -> {Elbow: Wrist pitch}

CALIBRATION = {
    85: {
        90: 85,
        95: 92,
        100: 100,
    },
    87: {
        90: 80,
        95: 88,
        97: 92,
        99: 96,
        100: 98,
    },
    89: {
        90: 77,
        95: 85,
        100: 95,
    },
    90: {
        90: 75,
        95: 83,
        100: 93,
    },
}

SHOULDER_MIN = 85
SHOULDER_MAX = 90

ELBOW_MIN = 90
ELBOW_MAX = 100


def wrist_level(shoulder, elbow):
    """
    Calculate wrist pitch to keep Shmolik's gripper
    horizontal using measured calibration data.

    Shoulder: 85 to 90 degrees
    Elbow:    90 to 100 degrees

    Returns an integer wrist angle.
    """

    shoulder = float(shoulder)
    elbow = float(elbow)

    if not SHOULDER_MIN <= shoulder <= SHOULDER_MAX:
        raise ValueError(
            f"Shoulder must be between "
            f"{SHOULDER_MIN} and {SHOULDER_MAX}"
        )

    if not ELBOW_MIN <= elbow <= ELBOW_MAX:
        raise ValueError(
            f"Elbow must be between "
            f"{ELBOW_MIN} and {ELBOW_MAX}"
        )

    shoulder_angles = sorted(CALIBRATION)

    # First interpolate wrist pitch along the elbow
    # axis for each calibrated shoulder position.
    wrist_values = []

    for shoulder_angle in shoulder_angles:
        measurements = CALIBRATION[shoulder_angle]

        elbow_angles = sorted(measurements)
        wrist_angles = [
            measurements[e] for e in elbow_angles
        ]

        wrist = np.interp(
            elbow,
            elbow_angles,
            wrist_angles
        )

        wrist_values.append(wrist)

    # Then interpolate across shoulder positions.
    result = np.interp(
        shoulder,
        shoulder_angles,
        wrist_values
    )

    return int(np.floor(float(result) + 0.5))


if __name__ == "__main__":
    print("Shmolik calibration verification\n")

    errors = 0

    for shoulder, measurements in CALIBRATION.items():
        for elbow, expected in measurements.items():
            actual = wrist_level(shoulder, elbow)

            passed = actual == expected

            print(
                f"S={shoulder:3} "
                f"E={elbow:3} "
                f"W={actual:3} "
                f"{'PASS' if passed else 'FAIL'}"
            )

            if not passed:
                errors += 1

    print(f"\nErrors: {errors}")

    print("\nExample:")
    print("S=87, E=97 ->", wrist_level(87, 97))
