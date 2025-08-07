# Copyright 2025, TU Delft
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Jasper van Brakel

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
