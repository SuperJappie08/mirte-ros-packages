```bash
echo "deb [trusted=yes] https://github.com/SuperJappie08/mirte-ros-packages/raw/ros_mirte_humble_jammy_amd64_develop/ ./" | sudo tee /etc/apt/sources.list.d/SuperJappie08_mirte-ros-packages.list
echo "yaml https://github.com/SuperJappie08/mirte-ros-packages/raw/ros_mirte_humble_jammy_amd64_develop/local.yaml humble" | sudo tee /etc/ros/rosdep/sources.list.d/1-SuperJappie08_mirte-ros-packages.list
```
