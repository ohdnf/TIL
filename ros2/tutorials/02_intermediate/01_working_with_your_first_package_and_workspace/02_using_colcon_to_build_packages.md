## 📕Using colcon to build packages

`colcon`을 사용하여 만든 패키지를 빌드하는 방법입니다. `colcon`은 ROS build tools인 `catkin_make`, `catkin_make_isolated`, `catkin_tools`, `ament_tools`를 돌리는 명령어입니다. 자세한 사항은 [공식 문서](https://design.ros2.org/articles/build_tool.html)를 참고하세요.

ROS 작업공간은 특정한 구조를 가지고 있습니다. `src`라는 디렉토리 안에는 ROS 패키지들이 들어있습니다. `colcon`은 `src ` 안의 패키지들을 빌드해 다음의 디렉토리들을 만듭니다.

- `build` 디렉토리는 빌드 중 생성된 임시 파일들을 저장합니다. 패키지마다 이름을 따 서브디렉토리가 생성됩니다.
- `install` 디렉토리는 패키지가 설치되는 공간입니다.
- `log` 디렉토리는 로그 정보를 담고 있습니다.

### Create a workspace

```shell
md \dev\ros2_example_ws\src
cd \dev\ros2_example_ws
```

### Add some sources

```shell
git clone https://github.com/ros2/examples src/examples
```

ROS 버전 확인

```shell
cd ~/ros2_example_ws/src/examples/
git checkout eloquent
cd ~/ros2_example_ws
```

폴더 구조

```
.
└── src
    └── examples
        ├── CONTRIBUTING.md
        ├── LICENSE
        ├── rclcpp
        ├── rclpy
        └── README.md

4 directories, 3 files
```

### Source an underlay

- underlay: ROS2 binary installation으로 생성된 환경
- overlay: 현재 사용할 패키지들이 있는 작업 환경

```shell
call C:\dev\ros2_eloquent\setup.bat
```

### Build the workspace

> #### 주의!
>
> Windows 환경에서 ROS2를 빌드하기 위해선 관리자 권한으로 실행된 Visual Studio Command Prompt (“x64 Native Tools Command Prompt for VS 2019”)가 필요합니다. ([참고](https://index.ros.org/doc/ros2/Installation/Rolling/Windows-Development-Setup/#windows-dev-build-ros2))

```shell
colcon build --symlink-install
```

### Run tests(에러 발생...)

```shell
colcon test

...
Summary: 9 packages finished [7.11s]
  7 packages failed: examples_rclpy_executors examples_rclpy_minimal_action_client examples_rclpy_minimal_action_server examples_rclpy_minimal_client examples_rclpy_minimal_publisher examples_rclpy_minimal_service examples_rclpy_minimal_subscriber
  7 packages had stderr output: examples_rclpy_executors examples_rclpy_minimal_action_client examples_rclpy_minimal_action_server examples_rclpy_minimal_client examples_rclpy_minimal_publisher examples_rclpy_minimal_service examples_rclpy_minimal_subscriber
```

### Source the environment

빌드에 성공하면 `install` 디렉토리가 생성되는데, 이 안에 있는 실행파일을 사용하기 위해 batch 작업을 한다

```shell
call install\setup.bat
```

### Try a demo

Terminal A

```shell
ros2 run examples_rclcpp_minimal_subscriber subscriber_member_function
```

Terminal B

```shell
ros2 run examples_rclcpp_minimal_publisher publisher_member_function
```

### Create your own package

`colcon`은 `package.xml` 명세를 사용하며, `ament_cmake`, `ament_python`, 순수 `cmake` 와 같은 다양한 빌드 타입을 지원합니다. `ament_python` 빌드의 [예시](https://github.com/ament/ament_index/tree/master/ament_index_python)를 보면 `setup.py`가 빌드를 위한 primary entry point라는 것을 알 수 있습니다. `demo_nodes_cpp`는 `ament_cmake`의 빌드 타입 [예시](https://github.com/ros2/demos/tree/master/demo_nodes_cpp)입니다.