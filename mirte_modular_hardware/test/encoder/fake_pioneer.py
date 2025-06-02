import rclpy
from rclpy.parameter import Parameter
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException

from fake_encoder_based_on_speed import FakeEncoder


def main(args=None):
    rclpy.init(args=args)

    try:
        left_encoder = FakeEncoder(
            namespace="io",
            parameter_overrides=[Parameter("device_name", value="left")],
            cli_args=["--ros-args", "-r", "__node:=left_fake_encoder"],
        )
        right_encoder = FakeEncoder(
            namespace="io",
            parameter_overrides=[Parameter("device_name", value="right")],
            cli_args=["--ros-args", "-r", "__node:=right_fake_encoder"],
        )

        executor = MultiThreadedExecutor()
        executor.add_node(left_encoder)
        executor.add_node(right_encoder)

        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as err:
        raise err
    finally:
        executor.remove_node(left_encoder)
        executor.remove_node(right_encoder)

        left_encoder.destroy_node()
        right_encoder.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    import sys

    main(sys.argv)
