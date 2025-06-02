from math import pi
from threading import Lock

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.node import Node, ParameterDescriptor
from rclpy.qos import qos_profile_system_default

from std_msgs.msg import Int32
from mirte_msgs.msg import Encoder


class FakeEncoder(Node):
    def __init__(self, **kwargs):
        super().__init__("fake_encoder", **kwargs)

        device_param_descriptor = ParameterDescriptor()
        device_param_descriptor.type = 4
        device_param_descriptor.read_only = True
        device_param_descriptor.description = "The telemetrix device name"

        self.declare_parameter("device_name", "", descriptor=device_param_descriptor)

        rate_param_descriptor = ParameterDescriptor()
        rate_param_descriptor.read_only = True
        rate_param_descriptor.description = "The publication rate"

        self.declare_parameter("rate", 20.0, descriptor=rate_param_descriptor)

        self.declare_parameter("max_motor_speed", 6.0 * pi)
        self.declare_parameter("ticks_per_rotation", 20.0)

        device_name = (
            self.get_parameter("device_name").get_parameter_value().string_value
        )

        self.publisher_ = self.create_publisher(
            Encoder, "encoder/" + device_name, qos_profile_system_default
        )

        self._lock = Lock()
        self._position: float = 0.0
        self._speed = 0
        self._time = self.get_clock().now()

        self.subscriber_ = self.create_subscription(
            Int32,
            f"motor/{device_name}/speed",
            self.speed_callback,
            qos_profile_system_default,
        )
        rate = self.get_parameter("rate").get_parameter_value().double_value
        self.timer_ = self.create_timer(1 / rate, self.timer_callback)

    def update_position(self, time: Time):
        speed_to_rad = (
            self.get_parameter("max_motor_speed").get_parameter_value().double_value
        )

        duration: Duration = time - self._time
        dt = duration.nanoseconds * 1e-9

        self._position += speed_to_rad * dt * self._speed / 100.0
        self._time = time

    def speed_callback(self, msg: Int32) -> None:
        current_time = self.get_clock().now()
        with self._lock:
            self.update_position(current_time)
            self._speed = msg.data

    def timer_callback(self) -> None:
        current_time = self.get_clock().now()
        msg = Encoder()
        with self._lock:
            self.update_position(current_time)
            ticks_per_rotation = (
                self.get_parameter("ticks_per_rotation")
                .get_parameter_value()
                .double_value
            )
            msg.header.stamp = current_time.to_msg()
            msg.value = int(self._position / (2 * pi) * ticks_per_rotation)
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    fake_encoder = FakeEncoder()

    try:
        rclpy.spin(fake_encoder)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as err:
        raise err
    finally:
        fake_encoder.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    import sys

    main(sys.argv)
