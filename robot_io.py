"""Webots 장치 API를 나머지 코드에서 감추는 작은 인터페이스."""

import math

import numpy as np

from config import DEVICE_NAME_CANDIDATES, LIDAR_REVERSED, MAX_SPEED, TIME_STEP


class RobotIO:
    def __init__(self, robot):
        self.robot = robot
        self.devices = self._list_devices()

        self.left_motor = self._find_device("left_motor", "Motor", "left")
        self.right_motor = self._find_device("right_motor", "Motor", "right")
        self.rear_left_motor = self._find_device("rear_left_motor", "Motor")
        self.rear_right_motor = self._find_device("rear_right_motor", "Motor")
        self.lidar = self._find_device("lidar", "Lidar")
        self.camera = self._find_device("camera", "Camera")

        if self.left_motor is None or self.right_motor is None:
            available = ", ".join(self.devices) or "없음"
            raise RuntimeError(
                "좌/우 바퀴 모터를 찾지 못했습니다. config.py의 "
                f"DEVICE_NAME_CANDIDATES를 수정하세요. 현재 장치: {available}"
            )

        # position=inf는 위치 제어가 아닌 연속 회전(velocity mode)을 뜻합니다.
        self.left_motors = [self.left_motor]
        self.right_motors = [self.right_motor]
        if self.rear_left_motor is not None:
            self.left_motors.append(self.rear_left_motor)
        if self.rear_right_motor is not None:
            self.right_motors.append(self.rear_right_motor)

        for motor in self.left_motors + self.right_motors:
            motor.setPosition(float("inf"))
            motor.setVelocity(0.0)

        if self.lidar is not None:
            self.lidar.enable(TIME_STEP)
        else:
            print("[RobotIO] LiDAR를 찾지 못했습니다.")

        if self.camera is not None:
            self.camera.enable(TIME_STEP)
        else:
            print("[RobotIO] Camera를 찾지 못했습니다. STEP 1~4에는 필수가 아닙니다.")

        print("[RobotIO] 감지한 장치:", ", ".join(self.devices))

    def _list_devices(self):
        """월드에 실제로 존재하는 장치 이름을 먼저 조사합니다."""
        result = {}
        for index in range(self.robot.getNumberOfDevices()):
            device = self.robot.getDeviceByIndex(index)
            result[device.getName()] = device
        return result

    def _find_device(self, role, class_name, name_hint=None):
        """설정 후보를 우선 사용하고, 실패하면 장치 타입/이름으로 탐색합니다."""
        for name in DEVICE_NAME_CANDIDATES[role]:
            if name in self.devices:
                return self.devices[name]

        matching_type = [
            (name, device)
            for name, device in self.devices.items()
            if device.__class__.__name__ == class_name
        ]
        if name_hint is not None:
            for name, device in matching_type:
                if name_hint in name.lower():
                    return device
        if len(matching_type) == 1:
            return matching_type[0][1]
        return None

    def set_wheel_speed(self, left, right):
        left = float(left)
        right = float(right)
        if not math.isfinite(left) or not math.isfinite(right):
            left = right = 0.0
        else:
            left = max(-MAX_SPEED, min(MAX_SPEED, left))
            right = max(-MAX_SPEED, min(MAX_SPEED, right))
        for motor in self.left_motors:
            motor_limit = min(MAX_SPEED, float(motor.getMaxVelocity()))
            motor.setVelocity(max(-motor_limit, min(motor_limit, left)))
        for motor in self.right_motors:
            motor_limit = min(MAX_SPEED, float(motor.getMaxVelocity()))
            motor.setVelocity(max(-motor_limit, min(motor_limit, right)))

    def stop(self):
        self.set_wheel_speed(0.0, 0.0)

    def move_forward(self, speed):
        self.set_wheel_speed(speed, speed)

    def turn_left(self, speed):
        self.set_wheel_speed(-speed, speed)

    def turn_right(self, speed):
        self.set_wheel_speed(speed, -speed)

    def get_lidar(self):
        if self.lidar is None:
            return None
        ranges = list(self.lidar.getRangeImage())
        if LIDAR_REVERSED:
            ranges.reverse()
        return ranges

    def get_lidar_directions(self, ranges=None):
        """전체 스캔에서 전/좌/우 부채꼴의 최솟값을 반환합니다."""
        if ranges is None:
            ranges = self.get_lidar()
        if not ranges:
            return None

        count = len(ranges)
        sector_width = max(1, count // 18)  # 전체 시야의 약 20도

        def sector_min(center_index):
            values = []
            for offset in range(-sector_width, sector_width + 1):
                value = ranges[(center_index + offset) % count]
                if math.isfinite(value):
                    values.append(value)
            return min(values) if values else float("inf")

        # practice.wbt의 360도 LiDAR 기준: 중앙=앞, 1/4=오른쪽, 3/4=왼쪽
        return {
            "front": sector_min(count // 2),
            "left": sector_min(3 * count // 4),
            "right": sector_min(count // 4),
        }

    def get_camera_image(self):
        if self.camera is None:
            return None
        return self.camera.getImage()

    def get_camera_bgr(self):
        """Return the current Webots Camera BGRA frame as a BGR ndarray."""
        if self.camera is None:
            return None
        raw_image = self.camera.getImage()
        if raw_image is None:
            return None
        width = self.camera.getWidth()
        height = self.camera.getHeight()
        expected_size = width * height * 4
        if len(raw_image) != expected_size:
            raise ValueError("Webots camera image size does not match its dimensions")
        bgra = np.frombuffer(raw_image, dtype=np.uint8).reshape((height, width, 4))
        return bgra[:, :, :3].copy()
