# 第四章 ROS运行管理

## 4.1 ROS节点管理 launch 文件

之前的文章中我们已经介绍过了 launch 文件的基本使用方法，本节的内容为 launch 的进阶使用方法

### 4.1.1 launch 文件的标签表

|标签名|标签作用|
|:-:|:-:|
| launch|`<launch>`标签是所有 launch 文件的根标签，充当其他标签的容器 |
|**属性**|**属性说明**| 
|`deprecated`|"弃用声明"，继续使用会输出警告日志所有其它标签|
|**子集标签**|
|其他标签都是都是launch的子级|
|---| |
|||
|**标签名**|**标签作用**|
|node|`<node>`标签用于指定 ROS 节点，是最常见的标签，需要注意的是: roslaunch 命令不能保证按照 node 的声明顺序来启动节点(节点的启动是多进程的)|
|**属性**|**属性说明**| 
|`pkg="包名"`|节点所属的包|
|`type="nodeType"`|节点类型(与之相同名称的可执行文件)|
|`name="nodeName"`|节点名称(在 ROS 网络拓扑中节点的名称)|
|`args="xxx xxx xxx" (可选)`|将参数传递给节点|
|`machine="机器名"`|在指定机器上启动节点|
|`respawn="true | false" (可选)`|如果节点退出，是否自动重启|
|`respawn_delay=" N" (可选)`|如果 respawn 为 true, 那么延迟 N 秒后启动节点|
|`required="true | false" (可选)`|该节点是否必须，如果为 true,那么如果该节点退出，将杀死整个 roslaunch|
|`ns="xxx" (可选)`|在指定命名空间 xxx 中启动节点|
|`clear_params="true | false" (慎用)`|在启动前，删除节点的私有空间的所有参数|
|`output="log | screen" (可选)`|日志发送目标，可以设置为 log 日志文件，或 screen 屏幕,默认是 log，但常用 screen|
|**子集标签**||
|env 环境变量设置||
|remap 重映射节点名称||
|rosparam 参数设置||
|param 参数设置||
|---||
|||
|**标签名**|**标签作用**|
|`include`|include标签用于将另一个 xml 格式的 launch 文件导入到当前文件|
|**属性**|**属性说明**| 
|`file="$(find 包名)/xxx/xxx.launch"`|要包含的文件路径|
|`ns="xxx" (可选)`|在指定命名空间导入文件|
|**子集标签**||
|env 环境变量设置||
|arg 将参数传递给被包含的文件||
|---||
|||
|**标签名**|**标签作用**|
|`remap`|用于话题重命名|
|**属性**|**属性说明**| 
|`from="xxx"`|原始话题名称|
|to="yyy"|目标名称|
|**子级标签**||
|无||
|---||
|||
|**标签名**|**标签作用**|
|`param`|`<param>`标签主要用于在参数服务器上设置参数，参数源可以在标签中通过 value 指定，也可以通过外部文件加载，在`<node>`标签中时，相当于私有命名空间。|
|**属性**|**属性说明**| 
|`name="命名空间/参数名"`|参数名称，可以包含命名空间|
|`value="xxx" (可选)`|定义参数值，如果此处省略，必须指定外部文件作为参数源|
|`type="str | int | double | bool | yaml" (可选)`|指定参数类型，如果未指定，roslaunch 会尝试确定参数类型，规则如下:1. 如果包含 '.' 的数字解析未浮点型，否则为整型。2."true" 和 "false" 是 bool 值(不区分大小写)。3.其他是字符串|
|**子级标签**||
|无||
|---||
|||
|**标签名**|**标签作用**|
|`rosparam`|`<rosparam>`标签可以从 YAML 文件导入参数，或将参数导出到 YAML 文件，也可以用来删除参数，`<rosparam>`标签在`<node>`标签中时被视为私有。|
|**属性**|**属性说明**| 
|`command="load | dump | delete" (可选，默认 load)`|加载、导出或删除参数|
|`file="$(find xxxxx)/xxx/yyy...."`|加载或导出到的 yaml 文件|
|`param="参数名称"`|如字面意思|
|`ns="命名空间" (可选)`|与之前相同|
|**子级标签**||
|无||
|---||
|||
|**标签名**|**标签作用**|
|`group`|`<group>`标签可以对节点分组，具有 ns 属性，可以让节点归属某个命名空间|
|**属性**|**属性说明**| 
|`ns="名称空间" (可选)`|-|
|`clear_params="true | false" (可选)`|启动前，是否删除组名称空间的所有参数(慎用....此功能危险)|
|**子级标签**||
|无||
|---||
|||
|**标签名**|**标签作用**|
|arg|`<arg>`标签是用于动态传参，类似于函数的参数，可以增强launch文件的灵活性|
|**属性**|**属性说明**| 
|`name="参数名称"`|-|
|`default="默认值" (可选)`|-|
|`value="数值" (可选)`|不可以与 default 并存|
|`doc="描述"`|参数说明|
|**子级标签**||
|无||


### 4.1.2 一些例子

<font color = green>例1：<font>

```xml
 <!--launch deprecated= "out of date"-->
 <launch>

    <!-- 启动的节点-->
    <!-- respawn= "true" 节点关闭后自动重启-->
    <!-- <node pkg="turtlesim" type="turtlesim_node" name="my_turtle" output="screen" respawn="true"/> -->
    <!-- required="true" 节点推出后关闭整个系统 -->
    <!-- <node pkg="turtlesim" type="turtlesim_node" name="my_turtle" output="screen" required="true"/> -->
    <!-- ns="NS" 设置节点的命名空间 -->
    <!-- <node pkg="turtlesim" type="turtlesim_node" name="my_turtle" output="screen" ns="NS"/> -->

    <!-- param 使用：向参数服务器设置参赛-->
    <!-- 格式1： launch 下， node 外-->
    <param name="param_A" type="int" value="100"/>

    <!-- rosparam 使用： 操作参数服务器数据-->
    <!-- 格式1： launch 下， node 外-->
    <!-- 加载参数-->
    <rosparam command="load" file="$(find launch01_basic)/launch/params.yaml"/>
    <!-- 导出参数-->
    <!-- <rosparam command="dump" file="$(find launch01_basic)/launch/params_out.yaml"/> -->

 
    <node pkg="turtlesim" type="turtlesim_node" name="my_turtle" output="screen">

    <!-- 格式2： launch 下， node 内-->
    <param name="param_B" type="double" value="3.14"/>

    <remap from="/turtle1/cmd_vel"   to="/cmd_vel"/>
    <!-- 格式2： launch 下， node 外-->
    <rosparam command="load" file="$(find launch01_basic)/launch/params.yaml"/>

    </node>
    <!-- 键盘控制节点 -->
    <node pkg="turtlesim" type="turtle_teleop_key" name="my_key" output="screen" />



</launch>
```

<font color = green>例2：<font>

```xml
<!-- 需要复用 start_turtle.launch -->
<launch>

    <include file="$(find launch01_basic)/launch/start_turtle.launch" />
    <!-- 启动其他自己需要的节点-->
</launch>
```

<font color = green>例2：<font>

```xml
<launch>
    <rosparam command="dump" file="$(find launch01_basic)/launch/params_out.yaml"/>

    <!-- 删除参数-->
    <rosparam command="delete" param="bg_B"/>
    

</launch>
```

<font color = green>例3：<font>

```xml
<launch>
    <!-- 启动两对乌龟gui 与 键盘控制节点-->
    <group ns="A" >
        <node pkg="turtlesim" type="turtlesim_node" name="my_turtle" output="screen"/>
        <node pkg="turtlesim" type="turtle_teleop_key" name="my_key" output="screen" />

    </group>


    <group ns="B" >
        <node pkg="turtlesim" type="turtlesim_node" name="my_turtle" output="screen"/>
        <node pkg="turtlesim" type="turtle_teleop_key" name="my_key" output="screen" />

    </group>
</launch>
```


<font color = green>例4：<font>

```xml
<launch>
    <!-- 需求： 演示 arg 的使用，需要设置多个参数，这县参数使用的是同一个指（小车的长度），怎么设置？-->
    <!-- <param name="A" value="0.5"/>
    <param name="B" value="0.5"/>
    <param name="C" value="0.5"/> -->

    <arg name="car_width" dafault="0.32"/>
    <param name="A" value="$(arg car_width)"/>
    <param name="B" value="$(arg car_width)"/>
    <param name="V" value="$(arg car_width)"/>

    
</launch>
```

