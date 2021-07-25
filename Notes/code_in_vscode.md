# Code in VSCode

## 1. 下载 vscode
下载vscode的方式有很多种，其中比较简单是直接从ubuntu software里下载

![](./pics/6.png)


如果在ubuntu software中找不到vscode，可以从官网下载.deb包。
![](./Notes/pics/7.png)

下载好后，在`.deb`包所在的目录处，打开终端输入:
![](./pics/8.png)

```
$ sudo dpkg -i code_1.58.2-1626302803_amd64.deb
```

之后就可以在应用里看到vscode了，右键可以将其添加到左边的收藏夹。
![](./Notes/pics/9.png)

## 2. 搭建 ROS 的开发环境
进入VSCode后可以下载如图所示的一些插件。

![](./pics/10.png)

其中Jupyter 和 Pylance 非必须。

有了插件后还需要修改配置文件，但在这之前我们先创建一个工作空间。

```
mkdir -p xxx_ws/src
cd xxx_ws
catkin_make
```

![](./pics/11.png)

完成后再使用
```
code .
```
打开vscode

然后我们可以在src处右键 创建一个catkin的包

![](./pics/12.png)

随后两步是命名和创建包的依赖

![](./pics/13.png)

包的依赖就是上一节讲的` roscpp`, `rospy`, `std_msgs`。

这时会生成一个`.xml`文件，其中包含了我们刚刚创建的包的信息。

然后我们使用快捷键`ctrl`+`shift`+`b`，点击catkin_make:build 后面的齿轮，进入配置文件。

![](./pics/14.png)

初始的配置文件如图：

![](./pics/15.png)


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

![](./pics/16.png)

如果报错了也不要慌，那可能是我们的依赖写错了，正确的3个依赖名称是:`rospy`, `roscpp`, `std_msgs`。我们只需要将`CMakeLists.txt`和`package.xml`文件中的全部错误依赖名称修改为正确的就可以了。

## 3. 开始写cpp程序
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
![](./pics/17.png)

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

![](./pics/18.png)

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
![](./pics/19.png)

