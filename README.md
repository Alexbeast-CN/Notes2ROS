
# ROS学习笔记 

> 前言：
> 本学习笔记基于赵虚左老师的教程资源在下方列表

* [赵老师B站视频连接](https://www.bilibili.com/video/BV1Ci4y1L7ZZ?from=search&seid=12959207388855423868)
* [讲义文稿](http://www.autolabor.com.cn/book/ROSTutorials/)
* [硬件购买](http://www.autolabor.com.cn/)
* [我个人在github上 上传的笔记](https://github.com/Alexbeast-CN/Notes2ROS)

---
## 目录： 
- [ROS学习笔记](#ros学习笔记)
  - [目录：](#目录)
- [第一章 ROS概述](#第一章-ros概述)
  - [第一节 我的第一个ROS程序](#第一节-我的第一个ros程序)
    - [1. 创建一个ROS工作空间:](#1-创建一个ros工作空间)
    - [2. 创建 ROS 包](#2-创建-ros-包)
    - [3. 写一个 Hello World 程序](#3-写一个-hello-world-程序)
    - [4. 编辑 Cmakelist.txt 文件](#4-编辑-cmakelisttxt-文件)
    - [5. 编译](#5-编译)
    - [6. 执行](#6-执行)
  - [第二节 Code in VSCode](#第二节-code-in-vscode)
    - [2.1 下载 vscode](#21-下载-vscode)
  - [2.2 搭建 ROS 的开发环境](#22-搭建-ros-的开发环境)
  - [2.3 开始写cpp程序](#23-开始写cpp程序)
  - [第三节 launch文件的编写](#第三节-launch文件的编写)
    - [3.1 一个ROS工作空间里的内容](#31-一个ros工作空间里的内容)
    - [3.2 launch文件的格式](#32-launch文件的格式)
    - [3.3 启动launch文件](#33-启动launch文件)
---

# 第一章 ROS概述
## 第一节 我的第一个ROS程序
### 1. 创建一个ROS工作空间:
直接在终端使用以下代码,在自定义空间名称处我使用的是ROS_ws

```
mkdir -p 自定义空间名称/src
cd 自定义空间名称
catkin_make
```

之后我们点击左边的文件夹，进入到home，就可以看到我们刚刚创建的 ROS_ws文件夹。

![](https://github.com/Alexbeast-CN/Notes2ROS/blob/main/Notes/pics/1.png)
在此文件夹下面我们又可以看到由`catkin_make`创建的3个文件夹。

![](https://github.com/Alexbeast-CN/Notes2ROS/blob/main/Notes/pics/2.png)

### 2. 创建 ROS 包
进入src中，右键在终端中打开，然后输入以下指令：
```
catkin_create_pkg 自定义ROS包名 roscpp rospy std_msgs

```
上述命令，会在工作空间下生成一个功能包，该功能包依赖于 roscpp、rospy 与 std_msgs，其中roscpp是使用C++实现的库，而rospy则是使用python实现的库，std_msgs是标准消息库，创建ROS功能包时，一般都会依赖这三个库实现。

由于我们本次的程序就是使用ros输出一个hello world，所以我在自定义ROS包名出命名为：hello_world

这时我们会看到在文件夹中，出现了下面的两个文件：

![](https://github.com/Alexbeast-CN/Notes2ROS/blob/main/Notes/pics/3.png)

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
删除#号键，取消注释，并成以下内容，任意名称处我使用的是haha。

```
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


![](https://github.com/Alexbeast-CN/Notes2ROS/blob/main/Notes/pics/4.png)

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
![](https://github.com/Alexbeast-CN/Notes2ROS/blob/main/Notes/pics/5.png)

## 第二节 Code in VSCode

### 2.1 下载 vscode
下载vscode的方式有很多种，其中比较简单是直接从ubuntu software里下载

![](https://github.com/Alexbeast-CN/Notes2ROS/blob/main/Notes/pics/6.png)


如果在ubuntu software中找不到vscode，可以从官网下载.deb包。
![](https://github.com/Alexbeast-CN/Notes2ROS/blob/main/Notes/pics/7.png)

下载好后，在`.deb`包所在的目录处，打开终端输入:
![](https://github.com/Alexbeast-CN/Notes2ROS/blob/main/Notes/pics/8.png)

```
$ sudo dpkg -i code_1.58.2-1626302803_amd64.deb
```

之后就可以在应用里看到vscode了，右键可以将其添加到左边的收藏夹。
![](https://github.com/Alexbeast-CN/Notes2ROS/blob/main/Notes/pics/9.png)

## 2.2 搭建 ROS 的开发环境
进入VSCode后可以下载如图所示的一些插件。

![](https://github.com/Alexbeast-CN/Notes2ROS/blob/main/Notes/pics.png)

其中Jupyter 和 Pylance 非必须。

有了插件后还需要修改配置文件，但在这之前我们先创建一个工作空间。

```
mkdir -p xxx_ws/src
cd xxx_ws
catkin_make
```

![](https://github.com/Alexbeast-CN/Notes2ROS/blob/main/Notes/pics/11.png)

完成后再使用
```
code .
```
打开vscode

然后我们可以在src处右键 创建一个catkin的包

![](https://github.com/Alexbeast-CN/Notes2ROS/blob/main/Notes/pics/12.png)

随后两步是命名和创建包的依赖

![](https://github.com/Alexbeast-CN/Notes2ROS/blob/main/Notes/pics/13.png)

包的依赖就是上一节讲的` roscpp`, `rospy`, `std_msgs`。

这时会生成一个`.xml`文件，其中包含了我们刚刚创建的包的信息。

然后我们使用快捷键`ctrl`+`shift`+`b`，点击catkin_make:build 后面的齿轮，进入配置文件。

![](https://github.com/Alexbeast-CN/Notes2ROS/blob/main/Notes/pics/14.png)

初始的配置文件如图：

![](https://github.com/Alexbeast-CN/Notes2ROS/blob/main/Notes/pics/15.png)


我们需要把它替换为其他的代码：

```json
{
// 有关 tasks.json 格式的文档，请参见
    // https://go.microsoft.com/fwlink/?LinkId=733558
    "version": "2.0.0",
    "tasks": [
        {
            "label": "catkin_make:debug", //代表提示的描述性信息
            "type": "shell",  //可以选择shell或者process,如果是shell代码是在shell里面运行一个命令，如果是process代表作为一个进程来运行
            "command": "catkin_make",//这个是我们需要运行的命令
            "args": [],//如果需要在命令后面加一些后缀，可以写在这里，比如-DCATKIN_WHITELIST_PACKAGES=“pac1;pac2”
            "group": {"kind":"build","isDefault":true},
            "presentation": {
                "reveal": "always"//可选always或者silence，代表是否输出信息
            },
            "problemMatcher": "$msCompile"
        }
    ]
}
```

然后我们再使用快捷键`ctrl`+`shift`+`b`编译一下我们刚刚创建的包，出现以下结果则创建成功，这也意味这我们的ROS环境搭建成功。

![](https://github.com/Alexbeast-CN/Notes2ROS/blob/main/Notes/pics/16.png)

如果报错了也不要慌，那可能是我们的依赖写错了，正确的3个依赖名称是:`rospy`, `roscpp`, `std_msgs`。我们只需要将`CMakeLists.txt`和`package.xml`文件中的全部错误依赖名称修改为正确的就可以了。

## 2.3 开始写cpp程序
在src下创建一个叫`hello_vscode.cpp`的文件。写入一个输出代码：
```cpp
#include "ros/ros.h"

int main(int argc, char *argv[])
{
    // 初始化ros
    ros::init(argc,argv,"hello");
    // 输出信息
    ROS_INFO("Hello_vscode");
    return 0;
}
```
在写的过程中我们会发现，在写与`ros`有关的代码的时候，编辑器并不会出现提示，这是因为我们选择的编译器有点老了，我们需要在修改`.vscode`文件夹中的`c_cpp_properties.json`文件。修改或添加`"cppStandard": "c++20"`就可以了。
![](https://github.com/Alexbeast-CN/Notes2ROS/blob/main/Notes/pics/17.png)

此时，我们再编写就会出现提示了。

代码写好后，我们还需要修改`CMakeLists.txt`文件。方法和之前一样，但多了我们可以使用快捷键`ctrl`+`/`来取消`#`注释了。

```
add_executable(任意名称
  src/步骤3的源文件名.cpp
)
target_link_libraries(任意名称
  ${catkin_LIBRARIES}
)
```


然后我们用`ctrl`+`shift`+`b`编译文件。如图所示就是成功了。

![](https://github.com/Alexbeast-CN/Notes2ROS/blob/main/Notes/pics/18.png)

然后，我们可以在vscode中之间使用快捷键`ctrl`+`shift`+`~`新建终端，vscode中的终端与我们在ubuntu系统中创建的终端使用相同。
现开一个ros master:
```
roscore
```
然后`source`我们的包：
```
source ./devel/setup.bash 
```
最后`rosrun`：
```
rosrun 包名 节点名
```
out：
```
[ INFO] [1627223101.571287843]: Hello_vscode
```
![](https://github.com/Alexbeast-CN/Notes2ROS/blob/main/Notes/pics/19.png)

## 第三节 launch文件的编写

一般大型项目需要一次性启动多个节点，一个一个的启动过于麻烦了，launch文件的用处是可以使我们一次性启动多个节点。

### 3.1 一个ROS工作空间里的内容
![](https://github.com/Alexbeast-CN/Notes2ROS/blob/main/Notes/pics/20.png)

launch文件就储存在launch文件夹下面

### 3.2 launch文件的格式
`luanch`文件是用`xml`写的，但命名的时候需要被叫做`xxx.launch`。

其一般格式如下：

```xml
<launch>
<node pkg="包名" type="节点名" name="为节点名称" output="输出地点"/>
</launch>
```

* node ---> 包含的某个节点

* pkg -----> 功能包

* type ----> 被运行的节点文件

* name --> 为节点命名

* output-> 设置日志的输出目标

其中`name`处如果不写，程序也不会报错，运行的时候编译器会自动给节点命名。我个人习惯直接把`type`后面的节点名复制到`name`。

这里我展示一个例程：
```xml
<launch>
    <node pkg="hello_vscode" type="hello_vscode" name="" output="screen"/>
    <node pkg="turtlesim" type="turtlesim_node" name=""/>
    <node pkg="turtlesim" type="turtle_teleop_key" name=""/>
</launch>
```

### 3.3 启动launch文件
通过下面的代码启动launch文件：
```
roslaunch 包名 xxx.launch
```

例如我上面展示的那个例程的启动指令为：
```
source ./devel/setup.bash
roslaunch hello_vscode start_turtle.launch
```
![](https://github.com/Alexbeast-CN/Notes2ROS/blob/main/Notes/pics/21.png)

out:
![](https://github.com/Alexbeast-CN/Notes2ROS/blob/main/Notes/pics/22.png)

process的[ ]中的内容很多，是因为我`name`后面是空的，这个名称是系统自动命名的结果，如果我们自己命名了，这里就会显示我们命名的节点名称。这可以方便我们看launch过程中有哪些节点扑街了。（大型项目中的节点经常扑街，从GitHub上下载一个项目跑跑全是error）。