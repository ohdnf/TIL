## 📕Launching/monitoring multiple nodes with Launch

여러 노드들을 묶어서 한 번에 실행시키고자 하는 launch 파일 작성법 및 실행법에 대한 튜토리얼입니다.

### ROS2 launch system

ROS2의 launch system은 사용자가 시스템 구성을 설정하고 동작하는데 도움을 주는 역할을 맡고 있습니다. 시스템 구성은 어떤 프로그램을 실행할지, 어디서 실행할지, 어떤 인자를 넘겨줄지, 그리고 시스템 전체에 걸쳐 컴포넌트를 다 시 사용할 수 있게 하는 ROS convention을 포함하고 있습니다. 또한 launch된 프로세스들의 상태를 모니터링하고 상태값의 변화를 관찰, 대응하는 역할을 담당합니다.

### Writing a ROS2 launch file

ROS2에서 launch 파일을 생성하는 방법 중 하나는 ROS2 CLI tool인 `ros2 launch`를 사용해 Python 파일로 만드는 것입니다. `ros2 pkg create <pkg-name> --dependencies [deps]` 명령어로 생성된 패키지에서 새로운 `launch` 디렉토리를 생성합니다.

#### Python Packages

파이썬 패키지는 다음과 같이 구성되어 있습니다:

```
src/
    my_package/
        launch/
        setup.py
        setup.cfg
        package.xml
```

`colcon`이 launch 파일을 찾도록 하기 위해, launch 파일의 파이썬 setup tool에 `setup(data_files=[...])`를 통해 알려줄 필요가 있습니다.

```python
# setup.py

import os
from glob import glob
from setuptools import setup

package_name = 'my_package'

setup(
    # Other parameters ...
    data_files=[
        # ... Other data files
        # Include all launch files. This is the most important line here!
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ]
)
```

#### C++ Packages

C++ 패키지라면, 다음 내용을 `CMakeLists.txt` 마지막(`ament_package()`보다는 이전) 줄에 추가하면 됩니다:

```txt
# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
```

#### Writing the launch file

launch 디렉토리 안에 `.launch.py` 확장명의 파일을 생성합니다.(예. `my_script.launch.py`)

꼭 `.launch.py`일 필요는 없고 `_launch.py`여도 상관 없습니다. 변경 후에는 `setup.py` 안에서  `glob()` 인자를 수정하기 바랍니다.

launch 파일 안에는 `ros2 launch`에서 사용할 `launch.LaunchDescription()`을 반환할  `generate_launch_description()`이 정의되어 있어야 합니다.

```python
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'node_prefix',
            default_value=[launch.substitutions.EnvironmentVariable('USER'), '_'],
            description='Prefix for node names'),
        launch_ros.actions.Node(
            package='demo_nodes_cpp', node_executable='talker', output='screen',
            node_name=[launch.substitutions.LaunchConfiguration('node_prefix'), 'talker']),
    ])
```

#### Usage

launch 파일이 독립형 scripts로 작성되었다면, ROS2 tools로 호출할 수 있습니다.

`colcon build`로 빌드한 다음, 환경 변수를 호출한 후에 다음과 같이 명령합니다.

```shell
ros2 launch my_package script.launch.py
```

