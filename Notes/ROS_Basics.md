# ROS Learning Notes

> 前言：
> 本学习笔记基于赵虚左老师的教程资源在下方列表

* [赵老师B站视频连接](https://www.bilibili.com/video/BV1Ci4y1L7ZZ?from=search&seid=12959207388855423868)
* [讲义文稿](http://www.autolabor.com.cn/book/ROSTutorials/)
* [硬件购买](http://www.autolabor.com.cn/)
* [我个人在github上 上传的笔记](https://github.com/Alexbeast-CN/Notes2ROS)


## 第一节
## 我的第一个ROS程序
### 1. 创建一个ROS工作空间:
直接在终端使用以下代码,在自定义空间名称处我使用的是ROS_ws

```
mkdir -p 自定义空间名称/src
cd 自定义空间名称
catkin_make
```

之后我们点击左边的文件夹，进入到home，就可以看到我们刚刚创建的 ROS_ws文件夹。

@import "./pics/1.png"
在此文件夹下面我们又可以看到由`catkin_make`创建的3个文件夹。

@import "./pics/2.png"

### 2. 创建 ROS 包
进入src中，右键在终端中打开，然后输入以下指令：
```
catkin_create_pkg 自定义ROS包名 roscpp rospy std_msgs

```
上述命令，会在工作空间下生成一个功能包，该功能包依赖于 roscpp、rospy 与 std_msgs，其中roscpp是使用C++实现的库，而rospy则是使用python实现的库，std_msgs是标准消息库，创建ROS功能包时，一般都会依赖这三个库实现。

由于我们本次的程序就是使用ros输出一个hello world，所以我在自定义ROS包名出命名为：hello_world

这时我们会看到在文件夹中，出现了下面的两个文件：

@import "./pics/3.png"

### 3. 写一个 Hello World 程序
然后我们进入hello_world中，在hello_world的src中右键创建一个文本文件，重命名为：hello_world.cpp

打开hello_world.cpp，在其中输入以下代码后保存文件。
``` cpp
#include "ros/ros.h"

int main(int argc, char *argv[])
{
    //执行 ros 节点初始化
    ros::init(argc,argv,"hello");
    //创建 ros 节点句柄(非必须)
    ros::NodeHandle n;
    //控制台输出 hello world
    ROS_INFO("hello world!");

    return 0;
}
```

### 4. 编辑 Cmakelist.txt 文件
然后退回到上一级文件目录，打开CMakeLists.txt文件，找到第136和第149-151行。其内容为：
```
add_executable(任意名称
  src/步骤3的源文件名.cpp
)
target_link_libraries(任意名称
  ${catkin_LIBRARIES}
)
```
删除#号键，取消注释，并成以下内容，任意名称处我使用的是haha

```cpp
## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
 add_executable(haha src/hello_world.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(haha
   ${catkin_LIBRARIES}
 )
 ```


@import "./pics/4.png"

### 5. 编译
进入工作空间

``` 
cd 自定义的空间名称
catkin_make
```

生成 `build devel`

### 6. 执行

首先我们需要打开一个ROS Master：
在终端输入：

```
roscore
```

然后`ctrl`+`alt`+`t`打开另外一个终端，输入：

```
cd 工作空间
source ./devel/setup.bash
rosrun 包名 C++节点
```

为了省事，在终端输入：
```
echo "source ~/工作空间/devel/setup.bash" >> ~/.bashrc
```
下一次就不用再输入`source ./devel/setup.bash`

结果如下：
@import"./pics/5.png"















