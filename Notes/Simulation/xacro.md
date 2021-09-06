#! https://zhuanlan.zhihu.com/p/407369184
# 基于 ROS 的机器人学习日志 -- 18

# 第六章 机器人仿真
# 第二节 URDF 的优化语法 Xacro

在使用 `URDF` 的过程中存在诸多不便，主要的问题有：

1. 无法进行代码复用
2. 无法进行数学运算

而在`Xacro`中，这些问题都可以解决。`Xacro`为用户提供了可编成的接口，用于可以使用类似编程语言的方法对于`参数`和`部件`进行`变量化`和`函数化`的操作。

## 2.1 Xacro 语法详解

### 2.1.1 空间声明

在使用`Xacro`时，文件的第一行必须包含一个空间声明：

```xml
xmlns:xacro="http://wiki.ros.org/xacro"
```

该声明可以放在标签`<robot>`之中，比如：

```xml
<robot name="xxx" xmlns:xacro="http://wiki.ros.org/xacro">
...
</robot>
```

### 2.1.2 属性与算术运算

属性类似于编成语言中的变量声明。

**属性定义**

```xml
<xacro:property name="xxxx" value="yyyy" />
```

**属性调用**

```xml
${属性名称}
```

例如：

```xml
<xacro:property name="wheel_radius" value="3.2" />
<xacro:property name="wheel_length" value="3.1" />
...
<geometry>
    <cylinder radius="${wheel_radius}" length="${wheel_length}" />
</geometry>
```

**算术运算**

```xml
${数学表达式}
```

例如：

```xml
<xacro:property name="PI" value="3.1415927" />

<origin xyz="0 0 0" rpy="${PI / 2} 0 0" />
```
>在将 `Xacro`解析为`URDF`的时候，并不会对属性进行解析。

### 2.1.3 宏

宏类似于编程中的函数，可以提高代码的复用性。

**宏定义**

```xml
<xacro::macro name="宏名称" params="参数列表（多参数之间使用空格分隔">

...

</xacro::macro>
```

**宏调用**

```xml
<xacro:宏名词 参数1=xxx 参数2=xxx>
```

例如：

```xml
<!-- 宏名称 -->
<xacro:macro name="xxx_func" params="xxx flag" >
        <link name="xxx">
            <visual>
                <geometry>
                </geometry>
            </visual>
        </link>
    
        <joint name="xxx" type="xxx">
        ...
        </joint>
</xacro:macro>

<!-- 宏调用 -->
<xacro:xxx_func xxx="" flag=""/>
```

>在将 `Xacro`解析为`URDF`的时候，并不会对宏定义解析，只对调用进行解析。

### 2.1.4 文件包含

机器人由多部件组成，不同部件可能封装为单独的 `Xacro` 文件，最后再将不同的文件集成，组合为完整机器人，可以使用文件包含实现。

语法：

```xml
<xacro:include filename="xxx" />
```

## 2.2 示例

```xml
<robot name="mycar" xmlns:xacro="http://wiki.ros.org/xacro">
    <!-- 属性封装 -->
    <xacro:property name="wheel_radius" value="0.0325" />
    <xacro:property name="wheel_length" value="0.0015" />
    <xacro:property name="PI" value="3.1415927" />
    <xacro:property name="base_link_length" value="0.08" />
    <xacro:property name="lidi_space" value="0.015" />

    <!-- 宏 -->
    <xacro:macro name="wheel_func" params="wheel_name flag" >
        <link name="${wheel_name}_wheel">
            <visual>
                <geometry>
                    <cylinder radius="${wheel_radius}" length="${wheel_length}" />
                </geometry>

                <origin xyz="0 0 0" rpy="${PI / 2} 0 0" />

                <material name="wheel_color">
                    <color rgba="0 0 0 0.3" />
                </material>
            </visual>
        </link>

        <!-- 3-2.joint -->
        <joint name="${wheel_name}2link" type="continuous">
            <parent link="base_link"  />
            <child link="${wheel_name}_wheel" />
            <!-- 
                x 无偏移
                y 车体半径
                z z= 车体高度 / 2 + 离地间距 - 车轮半径

            -->
            <origin xyz="0 ${0.1 * flag} ${(base_link_length / 2 + lidi_space - wheel_radius) * -1}" rpy="0 0 0" />
            <axis xyz="0 1 0" />
        </joint>

    </xacro:macro>
    <xacro:wheel_func wheel_name="left" flag="1" />
    <xacro:wheel_func wheel_name="right" flag="-1" />
</robot>
```

## 2.3 Xarco 的使用

命令行进入 xacro文件 所属目录，执行:

```
rosrun xacro xacro xxx.xacro > xxx.urdf
```

xacro 文件 就会被解析为 urdf 文件，其内容不再展示。

当然，如果不想转为 `URDF`， `ROS`也提供了直接使用 `Xacro`的方法。用户可以在`launch`文件中使用语句：

```xml
<param name="robot_description" command="$(find xacro)/xacro $(find 功能包)/<xacro文件的路径>/xxx.xacro" />
```

与使用`URDF`文件的区别在于`param`标签后面的属性：

```xml
<param name = "robot_description" textfile = "$(find 功能包)/<路径>/xxx.urdf"/>
```
