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
> - `--packages-up-to`는 원하는 패키지를 빌드하고, 해당 패키지에 대한 의존성만 더해줘 빌드 시간을 단축해줍니다.
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
