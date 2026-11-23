import math
import time
import cv2
import serial
from ultralytics import YOLO

# ==================== CONFIGURATION ====================

MODEL_PATH = "yolo11n.pt"

SERIAL_PORT = "COM6"
BAUD_RATE = 115200
CAMERA_INDEX = 0
HORIZONTAL_FOV = 70

SENSOR_OFFSET_MM = 10
GRIPPER_OFFSET_MM = 20

# Arduino mapping: {0, 1, 2, 3, 4, 5}
# Logical servo 6 -> PCA9685 channel 5
# Logical servo 2 -> PCA9685 channel 1

# Horizontal / base
BASE_SERVO = 6
BASE_ANGLE = 90
BASE_MIN = 30
BASE_MAX = 140
SERVO_DIRECTION = -1

# Vertical
VERTICAL_SERVO = 2
VERTICAL_ANGLE = 70
VERTICAL_MIN = 50
VERTICAL_MAX = 180
VERTICAL_DIRECTION = -1

# Tracking
CENTER_TOLERANCE_X = 5
CENTER_TOLERANCE_Y = 5

# One degree per command
MAX_STEP = 1

UPDATE_INTERVAL_X = 0.12
UPDATE_INTERVAL_Y = 0.12

AUTO_TRACKING = False

# ==================== INITIALIZATION ====================

print("Loading YOLO model...")
model = YOLO(MODEL_PATH)
model.to("cuda:0")

print("Opening camera...")
cap = cv2.VideoCapture(CAMERA_INDEX)

if not cap.isOpened():
    raise RuntimeError("Cannot open camera")

print("Opening Arduino serial port...")
arduino = serial.Serial(
    SERIAL_PORT,
    BAUD_RATE,
    timeout=0,
    write_timeout=1
)

last_distance = None
serial_buffer = bytearray()

current_base_angle = BASE_ANGLE
current_vertical_angle = VERTICAL_ANGLE

last_update_x = 0
last_update_y = 0

# ==================== SERIAL ====================

def send_command(command):
    arduino.write((command.strip() + "\n").encode("utf-8"))


def send_servo(servo, angle):
    angle = int(angle)
    send_command(f"SERVO:{servo}:{angle}")
    print(f"Sent servo {servo}: {angle}")


def read_serial():
    global last_distance

    waiting = arduino.in_waiting

    if waiting > 0:
        serial_buffer.extend(arduino.read(waiting))

    while b"\n" in serial_buffer:
        line, _, remaining = serial_buffer.partition(b"\n")
        serial_buffer[:] = remaining

        data = line.decode(
            "utf-8", errors="replace"
        ).strip()

        if data.startswith("Distance:"):
            try:
                sensor_distance = int(data.split()[1])

                if sensor_distance < 0:
                    last_distance = None
                    continue

                h = math.hypot(
                    sensor_distance,
                    SENSOR_OFFSET_MM
                )

                if h >= GRIPPER_OFFSET_MM:
                    last_distance = math.sqrt(
                        h * h - GRIPPER_OFFSET_MM ** 2
                    )
                else:
                    last_distance = None

            except (ValueError, IndexError):
                pass

        elif data == "Invalid measurement":
            last_distance = None

        elif data:
            print("Arduino:", data)

    return last_distance


# ==================== OBJECT DETECTION ====================

def get_object_position(results, height, width):
    if len(results[0].boxes) == 0:
        return None

    box = results[0].boxes[0]
    x1, y1, x2, y2 = map(int, box.xyxy[0])

    object_x = (x1 + x2) / 2
    object_y = (y1 + y2) / 2

    error_x = object_x - width / 2
    error_y = object_y - height / 2

    centered_x = abs(error_x) <= CENTER_TOLERANCE_X
    centered_y = abs(error_y) <= CENTER_TOLERANCE_Y

    return error_x, error_y, centered_x, centered_y


def calculate_angle(error_x, width):
    if error_x is None:
        return None

    fov_rad = math.radians(HORIZONTAL_FOV)
    focal_length = width / (2 * math.tan(fov_rad / 2))

    return math.degrees(
        math.atan(error_x / focal_length)
    )


# ==================== TRACKING ====================

def update_horizontal(error_x):
    global current_base_angle, last_update_x

    if not AUTO_TRACKING or error_x is None:
        return

    if abs(error_x) <= CENTER_TOLERANCE_X:
        return

    now = time.monotonic()

    if now - last_update_x < UPDATE_INTERVAL_X:
        return

    direction = 1 if error_x > 0 else -1

    target = (
        current_base_angle
        + direction * SERVO_DIRECTION * MAX_STEP
    )

    target = int(max(BASE_MIN, min(BASE_MAX, target)))

    if target != current_base_angle:
        send_servo(BASE_SERVO, target)
        current_base_angle = target
        last_update_x = now


def update_vertical(error_y):
    global current_vertical_angle, last_update_y

    if not AUTO_TRACKING or error_y is None:
        return

    if abs(error_y) <= CENTER_TOLERANCE_Y:
        return

    now = time.monotonic()

    if now - last_update_y < UPDATE_INTERVAL_Y:
        return

    direction = 1 if error_y > 0 else -1

    target = (
        current_vertical_angle
        + direction * VERTICAL_DIRECTION * MAX_STEP
    )

    target = int(max(
        VERTICAL_MIN,
        min(VERTICAL_MAX, target)
    ))

    if target != current_vertical_angle:
        send_servo(VERTICAL_SERVO, target)
        current_vertical_angle = target
        last_update_y = now


# ==================== DISPLAY ====================

def draw_text(frame, text, y, color=(255, 255, 255)):
    cv2.putText(
        frame,
        text,
        (20, y),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        color,
        2
    )


def draw_center(frame, centered):
    height, width = frame.shape[:2]

    color = (
        (0, 255, 0)
        if centered
        else (0, 0, 255)
    )

    cv2.circle(
        frame,
        (width // 2, height // 2),
        10,
        color,
        -1
    )


# ==================== MAIN ====================

def detect_object():
    global AUTO_TRACKING
    global last_update_x, last_update_y

    try:
        print("Starting Shmolik...")
        time.sleep(2)

        send_command("PING")

        print("Controls:")
        print("P - Test connection")
        print("T - Toggle tracking")
        print("S - Stop arm")
        print("Q - Quit")

        while True:
            distance_mm = read_serial()
            ret, frame = cap.read()

            if not ret:
                print("Camera error")
                break

            frame = cv2.rotate(
                frame,
                cv2.ROTATE_180
            )

            height, width = frame.shape[:2]

            results = model(
                frame,
                classes=[39],
                max_det=1,
                conf=0.5,
                verbose=False
            )

            position = get_object_position(
                results, height, width
            )

            if position is None:
                error_x = None
                error_y = None
                centered_x = False
                centered_y = False
            else:
                (
                    error_x,
                    error_y,
                    centered_x,
                    centered_y
                ) = position

            # Independent control of both axes
            update_horizontal(error_x)
            update_vertical(error_y)

            annotated_frame = results[0].plot()

            draw_center(
                annotated_frame,
                centered_x and centered_y
            )

            if error_x is None:
                draw_text(
                    annotated_frame,
                    "Bottle: Not detected",
                    40
                )
            else:
                draw_text(
                    annotated_frame,
                    f"X: {error_x:+.0f} px | "
                    f"Y: {error_y:+.0f} px",
                    40
                )

            if distance_mm is None:
                draw_text(
                    annotated_frame,
                    "Distance: N/A",
                    80
                )
            else:
                draw_text(
                    annotated_frame,
                    f"Distance: {distance_mm:.1f} mm",
                    80
                )

            draw_text(
                annotated_frame,
                f"Base: {current_base_angle} deg | "
                f"Vertical: {current_vertical_angle} deg",
                120
            )

            draw_text(
                annotated_frame,
                f"X centered: {centered_x} | "
                f"Y centered: {centered_y}",
                160
            )

            draw_text(
                annotated_frame,
                "AUTO TRACKING: ON"
                if AUTO_TRACKING
                else "AUTO TRACKING: OFF",
                200,
                (0, 255, 0)
                if AUTO_TRACKING
                else (0, 255, 255)
            )

            cv2.imshow(
                "Dum-E - AI Robotic Arm",
                annotated_frame
            )

            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                break

            elif key == ord("p"):
                send_command("PING")

            elif key == ord("s"):
                AUTO_TRACKING = False
                send_command("STOP")
                print("Arm stopped - PWM disabled")

            elif key == ord("t"):
                AUTO_TRACKING = not AUTO_TRACKING

                if AUTO_TRACKING:
                    # These must match the physical
                    # servo positions before enabling.
                    send_servo(
                        BASE_SERVO,
                        current_base_angle
                    )
                    send_servo(
                        VERTICAL_SERVO,
                        current_vertical_angle
                    )

                    now = time.monotonic()
                    last_update_x = now
                    last_update_y = now

                    print("Tracking enabled")

                else:
                    print("Tracking disabled")

    finally:
        cap.release()
        arduino.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    detect_object()
