# receive_INS_data
根据UDP协议接收"Trimble BD992"组合导航发送的数据;
接收数据需要的一些操作步骤：
(1):组合导航接通电源;
(2):通过网线或路由将组合导航与电脑相连;
(3):修改电脑本机IP地址，在Ubuntu系统下:setting-->Network-->Wired-->右上角"+"-->IPV4-->Manual-->Address:192.168.1.22-->Netmask:255.255.255.0-->Add,点击刚刚自定义的Profile连接;
(4):谷歌浏览器打开组合导航网址：192.168.1.202(如果打不开说明连接的IP地址有问题)，在这个网页上可以修改组合导航发送数据的格式、频率，以及查看IMU的状态是否融合，GPS的状态是否差分(RTK)
 

现在解析的数据格式是Gsof,如果需要其他的格式则还需修改源码重新解析，一般Gsof的数据格式已经够用了
程序接收UDP发送的原始数据，经过解析以后发送了两个/topic:"/PoseInfoFromGps"和"/OriginalGsofData",前者的格式为自定义消息类型：gsof.msg,后者消息类型为ros标准的PoseStamped.

"/PoseInfoFromGps"包括转换为UTM坐标系，单位为m的x,y,z坐标和roll,pitch,heading转换的四元数：x,y,z,w;
"/OriginalGsofData"包括字符串形式的原始数据，以及Gsof解析以后的数据
## Dependency

需要安装QT，用到了Qt5Network
 
## Compile and run

cd ~/catkin_ws/src
git clone 
cd ..
catkin_make
source devel/setup.bash
roslaunch receive_INS_data_node run.launch

