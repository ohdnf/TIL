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

