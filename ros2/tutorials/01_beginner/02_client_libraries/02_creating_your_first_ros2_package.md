## 📕Creating your first ROS2 package

원하는 프로젝트에 대한 *패키지(ROS2 코드를 담는 컨테이너)*를 생성하는 튜토리얼입니다.

### ROS2 Package

- `package.xml` 패키지에 대한 메타 정보를 담고 있는 파일
- `setup.py`는 패키지 설치에 대한 설명을 담고 있습니다
- `setup.cfg`는 패키지가 실행가능한 파일을 가지고 있을 때 `ros2 run`이 찾을 수 있게 해줍니다.
- `/<package_name>` 패키지와 똑같은 이름을 가진 디렉토리로, ROS2로 하여금 해당 package를 찾을 수 있게 해줍니다. `__init__.py`를 포함하고 있습니다.

### Packages in a workspace

하나의 workspace는 복수의 package를 가질 수 있습니다. 아래와 같이 workspace에 `src` 폴더를 만들고 그 안에 패키지를 넣는 것이 가장 좋은 방법입니다.

```
workspace_folder/
    src/
      package_1/
          CMakeLists.txt
          package.xml

      package_2/
          setup.py
          package.xml
          resource/package_2
      ...
      package_n/
          CMakeLists.txt
          package.xml
```

### Create a package

```shell
# 기본
ros2 pkg create --build-type ament_python <package_name>

# 옵션
ros2 pkg create --build-type ament_python --node-name my_node my_package
```

### Build a package

package들을 하나의 workspace에서 관리하는 것이 특별히 좋은 이유는 workspace root 경로에서 `colcon build` 명령어 한 번만 실행하면 모든 패키지에 대한 빌드가 끝나기 때문입니다.

```shell
# Return to the root of your workspace
cd C:\Users\multicampus\Workspace\catkin_ws\

colcon build --packages-select my_package

# Source the setup file
call install\local_setup.bat

# Use the package
ros2 run my_package my_node
Hi from my_package.
```

### Examine package contents

Customize `dev_ws\src\my_package\package.xml`

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_package</name>
  <version>0.0.0</version>
  <!-- <description>TODO: Package description</description> -->
  <description>Beginner client libraries tutorials practice package</description>
  <!-- <maintainer email="user@todo.todo">user</maintainer> -->
  <maintainer email="jupyohong7@gmail.com">Jupyo Hong</maintainer>
  <!-- <license>TODO: License declaration</license> -->
  <license>Apache License 2.0</license>

  <buildtool_depend>ament_python</buildtool_depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>

```

Set`dev_ws\src\my_package\setup.py` also

```python
from setuptools import setup

package_name = 'my_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    # maintainer='TODO',
    # maintainer_email='TODO',
    # description='TODO: Package description',
    # license='TODO: License declaration',
    maintainer='Jupyo Hong',
    maintainer_email='jupyohong7@gmail.com',
    description='Beginner client libraries tutorials practice package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = my_package.my_node:main'
        ],
    },
)

```

