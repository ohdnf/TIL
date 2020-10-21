# 🚸꼭 해봐야 할 ROS2 튜토리얼

## 📚목차

- [x] [Creating a workspace](ROS2_tutorial.md#creating-a-workspace)
- [x] [Creating your first ROS2 package](ROS2_tutorial.md#creating-your-first-ros2-package)
- [x] [Developing a ROS2 package](ROS2_tutorial.md#developing-a-ros2-package)
- [x] [Writing a simple publisher and subscriber (Python)](ROS2_tutorial.md#writing-a-simple-publisher-and-subscriber-python)
- [ ] [Synchronous vs. asynchronous service clients](ROS2_tutorial.md#synchronous-vs-asynchronous-service-clients)
- [x] [Creating custom ROS2 msg and srv files](ROS2_tutorial.md#creating-custom-ros2-msg-and-srv-files)
- [x] [Using colcon to build packages](ROS2_tutorial.md#using-colcon-to-build-packages)
- [ ] [Launching/monitoring multiple nodes with Launch](ROS2_tutorial.md#launchingmonitoring-multiple-nodes-with-launch)
- [ ] [Using tf2 with ROS2](ROS2_tutorial.md#using-tf2-with-ros2)
- [ ] [Use the robot state publisher to publish joint states and TF](ROS2_tutorial.md#use-the-robot-state-publisher-to-publish-joint-states-and-tf)
- [ ] [Turtlebot2 demo using ROS2](ROS2_tutorial.md#turtlebot2-demo-using-ros2)
- [ ] [MoveIt2 demo using ROS2](ROS2_tutorial.md#moveit2-demo-using-ros2)

## 📕Creating a workspace

패키지들과 기타 노드 파일들을 넣고 빌드하는 Workspace에 대한 내용입니다.

### Source the underlay(Terminal 1)

```powershell
call C:\dev\ros2_eloquent\local_setup.bat
```

### Create a workspace & Build

```powershell
md \dev_ws\src
cd \dev_ws\src

git clone https://github.com/ros/ros_tutorials.git -b eloquent-devel

dir ros_tutorials
roscpp_tutorials  rospy_tutorials  ros_tutorials  turtlesim

ls \dev_ws
build  install  log  src

# from the root of the workspace
colcon build --merge-install
```

> 알아두면 쓸모 있는 `colcon build`의 옵션 상식
>
> - `--packages-up-to`는 원하는 패키지를 빌드하고, 해당 패키지에 대한 의존성만 더해줘 빌드 시간을 단축해줍니다
> - `--symlink-install`은 파이썬 스크립트를 변경할 때마다 자동으로 rebuild해줍니다.
> - `--event-handlers console_direct+`는 빌드 과정을 콘솔에 표시해줍니다.

### Source the overlay(Terminal 2)

```powershell
# source the main ROS2 environment as the underlay
call C:\dev\ros2\setup.bat
# then source the overlay
call C:\dev_ws\install/setup.bat
# run the package from the overlay
ros2 run turtlesim turtlesim_node
```



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



## 📕Developing a ROS2 package

생성한 패키지의 `setup.py`에 대한 설명과, 실행가능한 노드를 엔트리에 넣는 사항에 대한 내용입니다.

### Creating a package

```shell
ros2 pkg create <pkg-name> --dependencies [deps] --build-type ament_python
```

ROS2는 Python의 `setuptools`를 이용해 Standard module distribution process를 따릅니다.  Python 패키지에는 C++ 패키지의 `CMakeLists.txt`와 동일한 역할을 하는 `setup.py`라는 파일이 있습니다. 

ROS2 패키지에는 `setup.cfg`라는 파일이 존재합니다.

```cfg
[develop]
script-dir=$base/lib/<package-name>
[install]
install-script=$base/lib/<package-name>
```

그리고 `setup.py`은 다음과 같습니다.

```python
import os
from glob import glob
from setuptools import setup

package_name = 'my_package'

setup(
    name=package_name,
    version='0.0.0',
    # Packages to export
    packages=[package_name],
    # Files we want to install, specifically launch files
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Include our package.xml file
        (os.path.join('share', package_name), ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob('*.launch.py'))
    ],
    # This is important as well
    install_requires=['setuptools'],
    zip_safe=True,
    author='ROS 2 Developer',
    author_email='ros2@ros.com',
    maintainer='ROS 2 Developer',
    maintainer_email='ros2@ros.com',
    keywords=['foo', 'bar'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: TODO',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='My awesome package.',
    license='TODO',
    # Like the CMakeLists add_executable macro, you can add your python
    # scripts here.
    entry_points={
        'console_scripts': [
            'my_script = my_package.my_script:main'
        ],
    },
)
```



## 📕Writing a simple publisher and subscriber (Python)

ROS2 topic 통신에 가장 기본적인 publisher-subscriber 노드들을 직접 만들어보고 통신시키는 내용입니다.

### Create a package

```shell
ros2 pkg create --build-type ament_python py_pubsub
```

### Write the publisher node

1. `publisher_member_function.py` [예시 소스 코드](https://raw.githubusercontent.com/ros2/examples/master/rclpy/topics/minimal_publisher/examples_rclpy_minimal_publisher/publisher_member_function.py) 분석

   먼저 `rclpy`와 `Node`를 import 합니다. `std_msgs`는 built-in string message type으로 노드가 topic으로 보내는 데이터를 구조화할 때 사용됩니다.

   ```python
   import rclpy
   from rclpy.node import Node
   
   from std_msgs.msg import String
   ```

   위 라인들은 `package.xml`에 추가할 빌드 종속성들입니다.

   `MinimalPublisher`는 `Node`를 상속받고 `super().__init__`을 호출해 `Node`의 생성자로 하여금 `minimal_publisher`라는 노드 이름을 설정합니다.

   ```python
   class MinimalPublisher(Node):
   
       def __init__(self):
           super().__init__('minimal_publisher')
           self.publisher_ = self.create_publisher(String, 'topic', 10)
           timer_period = 0.5  # seconds
           self.timer = self.create_timer(timer_period, self.timer_callback)
           self.i = 0
   ```

   `create_publisher`는 노드가  `String` 타입이고, `topic`이라는 이름의 토픽을 "queue size"를 10으로 설정해 메시지를 보내게 정의합니다. Queue size는 필수 QoS 설정으로 메시지의 양을 제한해 subscriber가 빠르게 메시지를 받지 못할 경우 데이터가 쌓이는 것을 방지합니다.

   `timer`는 콜백 함수로 생성되어 0.5초마다 실행됩니다.

   ```python
   def timer_callback(self):
       msg = String()
       msg.data = 'Hello World: %d' % self.i
       self.publisher_.publish(msg)
       self.get_logger().info('Publishing: "%s"' % msg.data)
       self.i += 1
   ```

   `timer_callback`은 counter 값(`self.i`)을 담은 메시지를 생성하고, `get_logger().info`를 통해 콘솔에 출력합니다.

   `rclpy` 라이브러리가 초기화되고, 노드가 생성되면 "spin"시켜 callback 함수들이 호출될 수 있게 합니다. 

   ```python
   def main(args=None):
       rclpy.init(args=args)
   
       minimal_publisher = MinimalPublisher()
   
       rclpy.spin(minimal_publisher)
   
       # Destroy the node explicitly
       # (optional - otherwise it will be done automatically
       # when the garbage collector destroys the node object)
       minimal_publisher.destroy_node()
       rclpy.shutdown()
   ```

2. Add dependencies

   `package.xml`을 열어 `<description>`과 `<maintainer>`, `<license>`를 수정해줍니다.

   ```xml
   <description>Examples of minimal publisher/subscriber using rclpy</description>
   <maintainer email="you@email.com">Your Name</maintainer>
   <license>Apache License 2.0</license>
   ```

   `ament_python` 빌드툴 종속성 코드 다음 줄에 import한 종속성들을 추가합니다.

   ```xml
   <exec_depend>rclpy</exec_depend>
   <exec_depend>std_msgs</exec_depend>
   ```

3. Add an entry point

   `setup.py` 파일을 열어 `package.xml`과 동일하게  `maintainer`, `maintainer_email`, `description`, `license` 필드를 수정합니다.

   ```python
   maintainer='YourName',
   maintainer_email='you@email.com',
   description='Examples of minimal publisher/subscriber using rclpy',
   license='Apache License 2.0',
   ```

   `entry_point`의 `console_scripts`에 노드를 등록합니다.

   ```python
   entry_points={
       'console_scripts': [
           'talker = py_pubsub.publisher_member_function:main',
       ],
   },
   ```

4. Check setup.cfg

   ```cfg
   [develop]
   script-dir=$base/lib/py_pubsub
   [install]
   install-scripts=$base/lib/py_pubsub
   ```

   자동적으로 생성되어있습니다. 이는 setuptools에게 실행파일들을 `lib`에 두어 `ros2 run`으로 하여금 찾을 수 있게 합니다.

### Write the subscriber node

1. `subscirber_member_function.py` [예시 소스 코드](https://raw.githubusercontent.com/ros2/examples/master/rclpy/topics/minimal_subscriber/examples_rclpy_minimal_subscriber/subscriber_member_function.py) 분석

   ```python
   self.subscription = self.create_subscription(
       String,
       'topic',
       self.listener_callback,
       10)
   ```

   subscriber 노드의 코드는 publisher와 거의 동일합니다. subscriber의 생성자와 콜백함수는 어떠한 timer도 포함하지 않습니다. 필요가 없기 때문이죠. subscriber의 콜백 함수는 메시지를 받는 순간 호출됩니다.

   콜백 함수는 단순히 받아온 정보 메시지(publisher에서 `msg.data = 'Hello World: %d' % self.i`로 넘겨준 것)를 콘솔에 출력합니다. 

   ```python
   def listener_callback(self, msg):
       self.get_logger().info('I heard: "%s"' % msg.data)
   ```

   `main`문도 publisher 노드를 subscriber 노드로 바꾼 것 외에는 거의 동일합니다.

   ```python
   minimal_subscriber = MinimalSubscriber()
   
   rclpy.spin(minimal_subscriber)
   ```

   publisher와 똑같은 종속성을 가지기 때문에 `package.xml`에도 추가할 것이 없습니다. `setup.cfg` 파일 또한 마찬가지입니다.

2. Add an entry point

   `setup.py`에 subscriber 노드를 추가합니다.

   ```python
   entry_points={
   'console_scripts': [
   'talker = py_pubsub.publisher_member_function:main',
   'listener = py_pubsub.subscriber_member_function:main',
   ],
   },
   ```

### Build and run

작업폴더 최상단 경로(예. `dev_ws`)에서 빌드합니다.

```shell
colcon build --packages-select py_pubsub
```

talker 노드(publisher)를 실행합니다.

```shell
ros2 run py_pubsub talker
```

터미널은 0.5초마다가 메시지를 출력합니다.

```shell
[INFO] [minimal_publisher]: Publishing: "Hello World: 0"
[INFO] [minimal_publisher]: Publishing: "Hello World: 1"
[INFO] [minimal_publisher]: Publishing: "Hello World: 2"
[INFO] [minimal_publisher]: Publishing: "Hello World: 3"
[INFO] [minimal_publisher]: Publishing: "Hello World: 4"
...
```

새로운 터미널을 열어 환경변수를 등록하고 listener 노드(subscriber)를 실행합니다.

```shell
call C:\dev\ros2\setup.bat & call C:\Users\user\dev_ws\install\local_setup.bat

ros2 run py_pubsub listener
```

실행하는 순간에 talker 노드에서 보낸 메시지를 받아 콘솔에 출력합니다.

```shell
[INFO] [minimal_subscriber]: I heard: "Hello World: 10"
[INFO] [minimal_subscriber]: I heard: "Hello World: 11"
[INFO] [minimal_subscriber]: I heard: "Hello World: 12"
[INFO] [minimal_subscriber]: I heard: "Hello World: 13"
[INFO] [minimal_subscriber]: I heard: "Hello World: 14"
```

`Ctrl+C`로 노드가 spinning하는 것을 중단시킵니다.



## 📖Synchronous vs. asynchronous service clients

[Synchronous vs. asynchronous service clients](https://index.ros.org/doc/ros2/Tutorials/Sync-Vs-Async/)

### Introduction

본 가이드는 파이썬 동기 서비스 클라이언트인 `call()` API와 연관된 위험성을 경고하기 위해 작성되었습니다. ROS의 서비스들을 동기적으로 호출할 경우 교착상태에 빠질 수 있기 때문에 `call()` API 사용은 추천하지 않습니다.



ROS를 능숙하게 다룰 줄 아는 사용자들 중 동기적 호출을 사용하고 싶어하는 분들을 위해 `call()` API를 올바르게 사용하는 예시들을 다뤄보겠습니다. 또한 대표적인 실수들과 이에 동반되는 교착상태를 피하는 예시들을 알아보겠습니다.



동기 호출을 가급적 지양하는 것을 권장하기 때문에, 본 가이드에선 대안책인 비동기 호출 API, `call_async()`의 특징과 사용법에 대해서도 알아보겠습니다.



C++ 서비스 호출 API는 오직 비동기만 가능하기 때문에, 본 가이드의 비교 및 예시들은 파이썬 서비스와 클라이언트에만 한정됩니다. 비동기와 관련된 정의들은 C++에도 적용되며 예외가 있을 수 있습니다.



### 1. Synchronous calls

동기적 클라이언트는 다른 서비스로 요청을 보내고 응답을 받을 때까지 호출 스레드를 차지하고 있기 때문에 해당 스레드는 호출동안 다른 무엇도 할 수 없습니다. 호출이 끝나는 시간은 변동적입니다. 응답은 클라이언트에게 곧바로 반환됩니다.

다음 예시는 동기적 클라이언트 노드를 올바르게 실행하고 있으며, [Simple Service and Client 튜토리얼](https://index.ros.org/doc/ros2/Tutorials/Writing-A-Simple-Py-Service-And-Client/#pysrvcli)의 비동기 노드와 유사합니다.

```python
import sys
from threading import Thread

from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalClientSync(Node):

    def __init__(self):
        super().__init__('minimal_client_sync')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self):
        self.req.a = int(sys.argv[1])
        self.req.b = int(sys.argv[2])
        return self.cli.call(self.req)
        # This only works because rclpy.spin() is called in a separate thread below.
        # Another configuration, like spinning later in main() or calling this method from a timer callback, would result in a deadlock.

def main():
    rclpy.init()

    minimal_client = MinimalClientSync()

    spin_thread = Thread(target=rclpy.spin, args=(minimal_client,))
    spin_thread.start()

    response = minimal_client.send_request()
    minimal_client.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' %
        (minimal_client.req.a, minimal_client.req.b, response.sum))

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

`main()` 안에 분리된 스레드에서 `rclpy.spin`을 호출한다는 것을 주목하세요. `send_request`와 `rclpy.spin` 모두 스레드를 막기 때문에 스레드를 분리하여야 합니다.



### 1.1 Sync deadlock



### 2. Asynchronous calls



### Summary





## 📕Creating custom ROS2 msg and srv files

패키지에서 ROS2의 기본 메시지 외에 사용자가 직접 만들어서 사용하고자 하는 메시지를 customize하는 방법에 대한 내용입니다.

### Create a new package

이 튜토리얼에서는 custom `.msg`, `.src` 파일을 만들어봅니다.

```shell
ros2 pkg create --build-type ament_cmake tutorial_interfaces
```

`tutorial_interfaces`는 새로운 CMake 패키지입니다. Python 패키지에 `.msg`나 `.srv` 파일을 생성해주지 않기 때문에 CMake 패키지에서 custom interface를 생성해준 다음에 Python 노드에서 사용합니다.

`.msg`나 `.srv` 파일을 다음과 같은 폴더 구조로 만들어 줍니다.

```
📁dev_ws
└─📁src
    ├─📁tutorial_interfaces
    │   ├─📁msg
    │   │   └─Num.msg
    │   └─📁srv
    │       └─Num.msg
    ├─📁py_pubsub
    └─📁py_srvcli
```

### Create custom definition

1. msg definition

   `tutorial_interfaces/msg` 디렉토리에 생성한 `Num.msg`에 다음 코드를 입력합니다.

   ```msg
   int64 num
   ```

2. srv definition

   `tutorial_interfaces/srv` 디렉토리에 생성한 `AddThreeInts.srv`에 다음 코드를 입력합니다.

   ```srv
   int64 a
   int64 b
   int64 c
   ---
   int64 sum
   ```

### `CMakeLists.txt`

interface들을 Python 코드로 변환하기 위해 `CMakeLists.txt`에 다음 코드를 추가합니다.

```txt
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Num.msg"
  "srv/AddThreeInts.srv"
 )
```

### `package.xml`

interface 변환은  `rosidl_default_generators`에 의존하기 때문에 `package.xml`에 종속성을 선언해야 합니다.

```xml
<build_depend>rosidl_default_generators</build_depend>

<exec_depend>rosidl_default_runtime</exec_depend>

<member_of_group>rosidl_interface_packages</member_of_group>
```

### Build the `tutorial_interfaces` package

최상단 경로에서 빌드합니다.

```shell
colcon build --packages-select tutorial_interfaces
```

### Confirm msg and srv creation

새로운 터미널(작업 폴더 최상단 경로)에서 환경 변수를 설정합니다.

```shell
call install/setup.bat
```

이제 `ros2 interface show` 명령어로 interface 생성을 확인할 수 있습니다.

```shell
ros2 interface show tutorial_interfaces/msg/Num

# 결과
int64 num

ros2 interface show tutorial_interfaces/srv/AddThreeInts

# 결과
int64 a
int64 b
int64 c
---
int64 sum
```

### Test the new interfaces

`py_srvcli`를 수정하여 테스트합니다.

1. Service

   ```python
   from tutorial_interfaces.srv import AddThreeInts     # CHANGE
   
   import rclpy
   from rclpy.node import Node
   
   
   class MinimalService(Node):
   
       def __init__(self):
           super().__init__('minimal_service')
           self.srv = self.create_service(AddThreeInts, 'add_three_ints', self.add_three_ints_callback)        # CHANGE
   
       def add_three_ints_callback(self, request, response):
           response.sum = request.a + request.b + request.c                                                  # CHANGE
           self.get_logger().info('Incoming request\na: %d b: %d c: %d' % (request.a, request.b, request.c)) # CHANGE
   
           return response
   
   def main(args=None):
       rclpy.init(args=args)
   
       minimal_service = MinimalService()
   
       rclpy.spin(minimal_service)
   
       rclpy.shutdown()
   
   if __name__ == '__main__':
       main()
   ```

2. Client

   ```python
   from tutorial_interfaces.srv import AddThreeInts       # CHANGE
   import sys
   import rclpy
   from rclpy.node import Node
   
   
   class MinimalClientAsync(Node):
   
       def __init__(self):
           super().__init__('minimal_client_async')
           self.cli = self.create_client(AddThreeInts, 'add_three_ints')       # CHANGE
           while not self.cli.wait_for_service(timeout_sec=1.0):
               self.get_logger().info('service not available, waiting again...')
           self.req = AddThreeInts.Request()                                   # CHANGE
   
       def send_request(self):
           self.req.a = int(sys.argv[1])
           self.req.b = int(sys.argv[2])
           self.req.c = int(sys.argv[3])                  # CHANGE
           self.future = self.cli.call_async(self.req)
   
   
   def main(args=None):
       rclpy.init(args=args)
   
       minimal_client = MinimalClientAsync()
       minimal_client.send_request()
   
       while rclpy.ok():
           rclpy.spin_once(minimal_client)
           if minimal_client.future.done():
               try:
                   response = minimal_client.future.result()
               except Exception as e:
                   minimal_client.get_logger().info(
                       'Service call failed %r' % (e,))
               else:
                   minimal_client.get_logger().info(
                       'Result of add_three_ints: for %d + %d + %d = %d' %                               # CHANGE
                       (minimal_client.req.a, minimal_client.req.b, minimal_client.req.c, response.sum)) # CHANGE
               break
   
       minimal_client.destroy_node()
       rclpy.shutdown()
   
   
   if __name__ == '__main__':
       main()
   ```

3. `package.xml`

   ```xml
   <exec_depend>tutorial_interfaces</exec_depend>
   ```

4. Build the package

   ```shell
   colcon build --packages-select py_srvcli
   ```

5. Run

   두 터미널에 환경 변수 설정 후 실행합니다.

   ```shell
   ros2 run py_srvcli service
   ```

   ```shell
   ros2 run py_srvcli client 2 3 1
   ```



## 📕Expanding on ROS2 interfaces

전용 인터페이스(메세지)뿐만 아니라, 하나의 패키지에서 인터페이스를 전부 생성시켜 전역으로 사용하는 방법에 대한 내용입니다.

### Create a package

`more_interfaces`라는 패키지를 만들고 그 안에 msg 파일을 담을 폴더 `msg`를 만듭니다.

```shell
cd ~/dev_ws/src
ros2 pkg create --build-type ament_cmake more_interfaces
mkdir more_interfaces/msg
```

### Create a msg file

`more_interfaces/msg` 안에 새로운 파일 `AddressBook.msg`를 만듭니다.

```msg
bool FEMALE=true
bool MALE=false

string first_name
string last_name
bool gender
uint8 age
string address
```

이 메시지는 다섯 개의 필드로 구성되어 있습니다:

- `first_name` string 타입
- `last_name` string 타입
- `gender` bool 타입, `MALE` 또는 `FEMALE`
- `age` uint8 타입
- `address` string 타입

필드 타입 등 자세한 interface customize 방식은 [About ROS 2 interfaces](https://index.ros.org/doc/ros2/Concepts/About-ROS-Interfaces/#interfaceconcept)를 참고합니다.

1. Build a msg file

   `more_interfaces`의 `package.xml`을 열고 다음 코드를 추가합니다.

   ```xml
   <buildtool_depend>rosidl_default_generators</buildtool_depend>
   
   <exec_depend>rosidl_default_runtime</exec_depend>
   
   <member_of_group>rosidl_interface_packages</member_of_group>
   ```

   다음으로 `CMakeLists.txt`를 열고 다음 코드를 추가합니다:

   msg/srv 파일에서 message code를 생성하는 패키지를 찾습니다.

   ```txt
   find_package(rosidl_default_generators REQUIRED)
   ```

   생성하고자 하는 msg 목록을 선언합니다:

   ```txt
   set(msg_files
     "msg/AddressBook.msg"
   )
   ```

   `.msg` 파일들을 수동으로 추가함으로써, CMake가 다른 `.msg` 파일이 추가될 때마다 reconfigure하도록 할 수 있습니다.

   메시지들을 생성합니다:

   ```txt
   rosidl_generate_interfaces(${PROJECT_NAME}
     ${msg_files}
   )
   ```

   또한 message runtime dependency를 export하는 것을 잊지 않습니다.

   ```txt
   ament_export_dependencies(rosidl_default_runtime)
   ```

   이제 정의한 msg를 생성할 준비가 되었습니다.

2. (Extra) Set multiple interfaces

   > `set`을 사용해 interface 목록을 정리할 수 있습니다.
   >
   > ```text
   > set(msg_files
   > "msg/Message1.msg"
   > "msg/Message2.msg"
   > # etc
   > )
   > 
   > set(srv_files
   > "srv/Service1.srv"
   > "srv/Service2.srv"
   > # etc
   > )
   > ```
   >
   > 이 리스트들을 한꺼번에 생성합니다.
   >
   > ```txt
   > rosidl_generate_interfaces(${PROJECT_NAME}
   > ${msg_files}
   > ${srv_files}
   > )
   > ```

### Use an interface from the same package

이제 생성한 msg를 사용하는 코드를 작성해봅니다.

1. `more_interfaces/src` 에 `publish_address_book.cpp` 파일을 만들고 다음 코드를 붙여넣습니다.

   ```cpp
   #include <chrono>
   #include <memory>
   
   #include "rclcpp/rclcpp.hpp"
   #include "more_interfaces/msg/address_book.hpp"		// 새로운 메시지를 header로 include
   
   using namespace std::chrono_literals;
   
   // AddressBook라는 publisher 노드를 생성
   class AddressBookPublisher : public rclcpp::Node
   {
   public:
     AddressBookPublisher()
     : Node("address_book_publisher")
     {
       address_book_publisher_ =
         this->create_publisher<more_interfaces::msg::AddressBook>("address_book", 10);
   
       // 콜백 함수
       auto publish_msg = [this]() -> void {
           auto message = more_interfaces::msg::AddressBook();	// AddressBook 메시지 인스턴스 생성
           // AddressBook의 필드 populate
           message.first_name = "John";
           message.last_name = "Doe";
           message.age = 30;
           message.gender = message.MALE;
           message.address = "unknown";
   
           // send message
           std::cout << "Publishing Contact\nFirst:" << message.first_name <<
             "  Last:" << message.last_name << std::endl;
   
           this->address_book_publisher_->publish(message);
         };
       timer_ = this->create_wall_timer(1s, publish_msg);	// 1초 간격으로 publish_msg 호출
     }
   
   private:
     rclcpp::Publisher<more_interfaces::msg::AddressBook>::SharedPtr address_book_publisher_;
     rclcpp::TimerBase::SharedPtr timer_;
   };
   
   int main(int argc, char * argv[])
   {
     rclcpp::init(argc, argv);
     rclcpp::spin(std::make_shared<AddressBookPublisher>());
     rclcpp::shutdown();
   
     return 0;
   }
   ```

2. Build the publisher

   `CMakeLists.txt`에 생성한 노드를 위한 target을 생성합니다.

   ```txt
   find_package(rclcpp REQUIRED)
   
   add_executable(publish_address_book
     src/publish_address_book.cpp
   )
   
   ament_target_dependencies(publish_address_book
     "rclcpp"
   )
   
   install(TARGETS publish_address_book
    DESTINATION lib/${PROJECT_NAME})
   ```

3. Link against the interface

   같은 패키지 않에서 생성된 메시지를 사용하려면 다음 CMake 코드를 사용해야 합니다.

   ```txt
   rosidl_target_interfaces(publish_address_book
     ${PROJECT_NAME} "rosidl_typesupport_cpp")
   ```

   이 코드는 `AddressBook.msg`에서 생성된 C++ 코드를 찾아 연결해줍니다.

   interface들이 (빌드가) 서로 다른 패키지에서 사용되는 경우 이러한 과정은 필요하지 않습니다. 이 CMake 코드는 같은 패키지 안에서 이미 쓰이고 있는 interface를 사용할 때 작성합니다.

4. Try it out

   build

   ```shell
   cd ~/dev_ws
   
   colcon build --packages-up-to more_interfaces
   ```

   source the workspace and run

   ```shell
   . install/local_setup.bash
   
   ros2 run more_interfaces publish_address_book
   ```

   open another terminal and call `topic echo` to confirm the message is being published on the `address_book`

   ```shell
   . install/local_setup.bash
   
   ros2 topic echo /address_book
   ```

5. (Extra) Use an existing interface definition

   > 이미 존재하는 interface definition을 새로운 interface definition에서 사용할 수 있습니다. 예를 들어, 메시지 `Contact.msg`가 `rosidl_tutorials_msgs`라는 패키지에 속해있다고 가정하고, 이것과 동일한 `AddressBook.msg` interface를 정의한다고 해봅시다.
   >
   > 이런 경우 `AddressBook.msg`(노드들이 있는 패키지에 있는 interface)를 `Contact`(다른 패키지에 있는 interface) 타입으로 정의합니다.
   >
   > ```txt
   > rosidl_tutorials_msgs/Contact[] address_book
   > 
   > array 타입으로 정의할 수도 있습니다!
   > ```
   >
   > 메시지를 생성하기 위해 `Contact.msg`의 패키지 `rosidl_tutorials_msg`에서 종속성 선언을 해줍니다.
   >
   > ```xml
   > <!-- rosidl_tutorials_msg/package.xml -->
   > 
   > <build_depend>rosidl_tutorials_msgs</build_depend>
   > 
   > <exec_depend>rosidl_tutorials_msgs</exec_depend>
   > ```
   >
   > ```txt
   > rosidl_tutorials_msg/CMakeLists.txt
   > find_package(rosidl_tutorials_msgs REQUIRED)
   > 
   > rosidl_generate_interfaces(${PROJECT_NAME}
   > ${msg_files}
   > DEPENDENCIES rosidl_tutorials_msgs
   > )
   > ```
   >
   > publisher 노드에 `Contact.msg`를 헤더로 include하면 `address_book`에 `contacts`를 추가할 수 있습니다.
   >
   > ```cpp
   > #include "rosidl_tutorials_msgs/msg/contact.hpp"
   > ```
   >
   > 콜백 함수 또한 아래와 같이 바꿀 수 있습니다.
   >
   > ```cpp
   > auto publish_msg = [this]() -> void {
   > auto msg = std::make_shared<more_interfaces::msg::AddressBook>();
   > {
   >   rosidl_tutorials_msgs::msg::Contact contact;
   >   contact.first_name = "John";
   >   contact.last_name = "Doe";
   >   contact.age = 30;
   >   contact.gender = contact.MALE;
   >   contact.address = "unknown";
   >   msg->address_book.push_back(contact);
   > }
   > {
   >   rosidl_tutorials_msgs::msg::Contact contact;
   >   contact.first_name = "Jane";
   >   contact.last_name = "Doe";
   >   contact.age = 20;
   >   contact.gender = contact.FEMALE;
   >   contact.address = "unknown";
   >   msg->address_book.push_back(contact);
   > }
   > 
   > std::cout << "Publishing address book:" << std::endl;
   > for (auto contact : msg->address_book) {
   >   std::cout << "First:" << contact.first_name << "  Last:" << contact.last_name <<
   >     std::endl;
   > }
   > 
   > address_book_publisher_->publish(*msg);
   > };
   > ```



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



## 📖Using tf2 with ROS2

ROS1에도 쓰였던 좌표변환 메세지인 tf가 ROS2의 tf2로 바뀌면서, 이전과 달라진 좌표변환 정보들을 송/수신하는 방법 등을 간략하게 다루는 내용입니다.

## 📕Use the robot state publisher to publish joint states and TF

각 로봇팔의 좌표변환 내용들을 로봇의 state와 같이 수신하여` rviz2`에 어떻게 시각화하는지에 대한 데모입니다.

## 📕Turtlebot2 demo using ROS2

ROS2를 사용한 2차원 이동로봇 시뮬레이터 터틀봇2 데모입니다.

## 📕MoveIt2 demo using ROS2

ROS2를 사용한 로봇팔 시뮬레이션 MoveIt2 데모입니다.