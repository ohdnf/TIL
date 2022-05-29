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