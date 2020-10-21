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

