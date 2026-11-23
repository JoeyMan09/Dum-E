import time
import serial

from arm_calibration import wrist_level

# ==================== CONFIG ====================

PORT = "COM6"
BAUD_RATE = 115200

SHOULDER_SERVO = 5
ELBOW_SERVO = 4
WRIST_SERVO = 2

RESPONSE_TIMEOUT = 3.0


# ==================== SERIAL ====================

def read_until_prefix(arduino, prefix):
    deadline = time.monotonic() + RESPONSE_TIMEOUT

    while time.monotonic() < deadline:
        line = arduino.readline().decode(
            "utf-8", errors="replace"
        ).strip()

        if not line:
            continue

        print("Arduino:", line)

        if line.startswith(prefix):
            return line

    raise TimeoutError(
        f"No response starting with {prefix!r}"
    )


def send_command(arduino, command, expected_prefix):
    print("Python:", command)

    arduino.write((command + "\n").encode("utf-8"))
    arduino.flush()

    return read_until_prefix(
        arduino,
        expected_prefix
    )


def check_connection(arduino):
    response = send_command(
        arduino,
        "PING",
        "PONG"
    )

    if response != "PONG":
        raise RuntimeError("Unexpected PING response")

    print("Connection successful!")

    status = send_command(
        arduino,
        "STATUS",
        "STATUS:"
    )

    if "PWM=1" not in status:
        raise RuntimeError(
            "Servo controller is not ready: " + status
        )

    print("Servo controller reports ready.")


# ==================== SERVO CONTROL ====================

def send_servo(arduino, servo, angle):
    angle = int(angle)

    response = send_command(
        arduino,
        f"SERVO:{servo}:{angle}",
        f"SERVO_RESULT:{servo}:"
    )

    accepted_angle = int(response.rsplit(":", 1)[1])

    if accepted_angle != angle:
        raise RuntimeError(
            f"Servo {servo}: requested {angle}, " 
            f"Arduino returned {accepted_angle}"
        )



def move_arm(arduino, shoulder, elbow):
    wrist = wrist_level(shoulder, elbow)
    wrist_preposition = wrist - 2

    print("\n==============================")
    print(f"Shoulder: {shoulder} degrees")
    print(f"Elbow:    {elbow} degrees")
    print(f"Wrist:    {wrist} degrees")
    print("==============================")

    # Move the wrist to the lower approach angle.
    send_servo(
        arduino,
        WRIST_SERVO,
        wrist_preposition
    )

    # Move shoulder and elbow to their targets.
    send_servo(
        arduino,
        SHOULDER_SERVO,
        shoulder
    )

    send_servo(
        arduino,
        ELBOW_SERVO,
        elbow
    )

    # Allow the arm to settle.
    time.sleep(2)

    # Approach the calibrated wrist angle
    # from 2 degrees below.
    send_servo(
        arduino,
        WRIST_SERVO,
        wrist
    )

    time.sleep(0.5)

    print("Wrist leveling completed.")
    print(f"Final wrist angle: {wrist} degrees")


# ==================== MAIN ====================

def main():
    print("Starting Shmolik calibration test...")

    with serial.Serial(
        PORT,
        BAUD_RATE,
        timeout=0.2,
        write_timeout=2
    ) as arduino:

        print(f"Connected to {PORT}")
        time.sleep(2)

        check_connection(arduino)

        print("\nCalibration test mode")
        print("Enter shoulder and elbow angles.")
        print("Wrist pitch is calculated automatically.")
        print("Type Q to quit.")

        confirmation = input(
            "\nArm supported and ready? Type YES: "
        ).strip()

        if confirmation != "YES":
            print("Movement cancelled.")
            return

        while True:
            user_input = input(
                "\nShoulder Elbow (example: 87 97): "
            ).strip()

            if user_input.lower() == "q":
                break

            try:
                parts = user_input.split()

                if len(parts) != 2:
                    print("Enter exactly two angles.")
                    continue

                shoulder = int(parts[0])
                elbow = int(parts[1])

                move_arm(
                    arduino,
                    shoulder,
                    elbow
                )

            except ValueError as error:
                print("Invalid input:", error)

            except (TimeoutError, RuntimeError) as error:
                print("Communication error:", error)
                print("Stopping the test.")
                break

    print("\nTest finished.")


if __name__ == "__main__":
    main()
