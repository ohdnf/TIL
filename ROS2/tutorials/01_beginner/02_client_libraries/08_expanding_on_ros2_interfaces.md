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
   > rosidl_tutorials_msgs::msg::Contact contact;
   > contact.first_name = "John";
   > contact.last_name = "Doe";
   > contact.age = 30;
   > contact.gender = contact.MALE;
   > contact.address = "unknown";
   > msg->address_book.push_back(contact);
   > }
   > {
   > rosidl_tutorials_msgs::msg::Contact contact;
   > contact.first_name = "Jane";
   > contact.last_name = "Doe";
   > contact.age = 20;
   > contact.gender = contact.FEMALE;
   > contact.address = "unknown";
   > msg->address_book.push_back(contact);
   > }
   > 
   > std::cout << "Publishing address book:" << std::endl;
   > for (auto contact : msg->address_book) {
   > std::cout << "First:" << contact.first_name << "  Last:" << contact.last_name <<
   >  std::endl;
   > }
   > 
   > address_book_publisher_->publish(*msg);
   > };
   > ```

