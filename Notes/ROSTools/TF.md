# 第五章：ROS常用组件

## 5.1 TF坐标变换

概念性的东西赵老师的讲义写的非常的清楚，这里就不再重复了。讲义链接如下：

[赵老师的讲义](http://www.autolabor.com.cn/book/ROSTutorials/di-5-zhang-ji-qi-ren-dao-hang/51-tfzuo-biao-bian-huan.html)

下面用一个例子来演示 TF 的使用方法。

例程需求：
    创建一个发布者节点，发布两个坐标系以及他们之间的空间变换关系
    创建一个接收者节点，用来接收并打印变换后的坐标数据
    两个节点之间的话题消息使用 geometry_msgs


例程


### 5.1.1 坐标 msg 消息



### 5.1.2 发布者节点

代码如下：
```cpp
#include "ros/ros.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"

/*
    需求：发布坐标系之间的相对关系

    流程：
        1. 包含头文件；
        2. 初始化（设置编码，节点初始化，创建节点句柄）；
        3. 创建发布对象；
        4. 组织被发布的消息；
        5. 发布数据；
        6. spin();
*/

int main(int argc, char *argv[])
{
        // 2. 初始化（设置编码，节点初始化，创建节点句柄）；
        setlocale(LC_ALL,"");
        ros::init(argc,argv,"static_pub");
        ros::NodeHandle nh;
        // 3. 创建发布对象；
        tf2_ros::StaticTransformBroadcaster pub;
        // 4. 组织被发布的消息；
        geometry_msgs::TransformStamped tfs;
        tfs.header.stamp = ros::Time::now();
        tfs.header.frame_id = "base_link";    // 相对坐标系关系中被参考的那一个
        tfs.child_frame_id = "laser";
        tfs.transform.translation.x = 0.2;
        tfs.transform.translation.y = 0.0;
        tfs.transform.translation.z = 0.5;

        // 需要根据欧拉角转换
        tf2::Quaternion qtn;        // 创建四元数对象
        // 向该对象设置欧拉角，这个对象可以将欧拉角转化成四元数
        qtn.setRPY(0.0, 0.0, 0.0);  // 欧拉角的单位是弧度
        tfs.transform.rotation.x = qtn.getX();
        tfs.transform.rotation.y = qtn.getY();
        tfs.transform.rotation.z = qtn.getZ();
        tfs.transform.rotation.w = qtn.getW();
        
        // 5. 发布数据；
        pub.sendTransform(tfs);
        // 6. spin();
        ros::spin();
    
    return 0;
}
```

### 5.1.3 接收者节点

代码如下：

```cpp
#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
/*
    订阅方：订阅发布的坐标系相对关系，传入一个坐标点，调用 tf 实现转换

    流程：
        1. 包含头文件；
        2. 初始化（编码，节点，NodeHandle）
        3. 创建订阅对象； --> 订阅坐标系相对关系
        4. 组织一个座标点数据；
        5. 转化算法，调用tf内置实现；
        6. 最后输出。
*/

int main(int argc, char *argv[])
{
    // 2. 初始化（编码，节点，NodeHandle）
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"Static_sub");
    ros::NodeHandle nh;
    // 3. 创建订阅对象； --> 订阅坐标系相对关系
    // 3-1. 创建一个 buffer 缓存
    tf2_ros::Buffer buffer;
    // 3-2. 创建一个监听对象（将订阅的数据缓存到 buffer）
    tf2_ros::TransformListener listener(buffer);
    // 4. 组织一个座标点数据；
    geometry_msgs::PointStamped ps;
    ps.header.frame_id = "laser";
    ps.header.stamp = ros::Time::now();
    ps.point.x = 2.0;
    ps.point.y = 3.0;
    ps.point.z = 5.0;

    // 休眠
    // ros::Duration(2).sleep();
    // 5. 转化算法，调用tf内置实现；
    ros::Rate rate(10);
    while(ros::ok())
    {
        // 核心代码 -- 将 ps 转换成相对于 base_link 的坐标点
        geometry_msgs::PointStamped ps_out;
        /*
            调用了 buffer 的转换函数 transform
            参数1：被转化的座标点
            参数2：目标坐标系
            返回值：输出的坐标点

            PS1：调用时必须包含头文件 tf2_geometry_msgs/tf2_geometry_msgs.h
            PS2: 运行时存在的问题，抛出异常 base_link 不存在
                 原因： 订阅数据是一个耗时操作，可能在调用 transform 转换函数时，
                        坐标系的相对关系还没订阅到，因此出现异常
                 解决：
                       方案1：在调用转换函数前，执行休眠；
                       方案2： 进行异常处理(建议使用)。


        */

       try
       {
           
        ps_out =  buffer.transform(ps,"base_link");
        // 6. 最后输出
        ROS_INFO("转换后的坐标值:(%.2f,%.2f,%.2f),参考坐标系: %s",
                        ps_out.point.x,
                        ps_out.point.y,
                        ps_out.point.z,
                        ps_out.header.frame_id.c_str()
                        );
       }
       catch(const std::exception& e)
       {
        //    std::cerr << e.what() << '\n';
        ROS_INFO("异常消息：%s",e.what());
       }
       
       
        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}

```