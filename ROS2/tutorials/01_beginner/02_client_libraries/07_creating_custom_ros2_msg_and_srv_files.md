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

