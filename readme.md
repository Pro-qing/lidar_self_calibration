<!-- 

<v1.0 仅实现了16线雷达的自标定功能>
< 这是最终放入4号车中的16线雷达自标定代码。

订阅话题： /points_16 
16线雷达数据直接输入即可，不要过滤成单线雷达数据。

使用前提：叉车停在三面墙之间。最好墙面干净整洁没有坑洞，然后离前墙近一些

base_link_y正方向指向的墙为左墙     actual_left_distance
base_link_y负方向指向的墙为右墙     actual_right_distance
base_link_x正方向指向的墙为前墙     actual_front_distance

使用方法：人工测量base_link到三面墙的距离。 
然后在launch文件中填入测量的数据
内容：
车间墙面的实际距离（必须根据实际车间测量设置）
<param name="actual_left_distance" type="double" value="1.0" />
<param name="actual_right_distance" type="double" value="2.0" />
<param name="actual_front_distance" type="double" value="3.0" />

数据读取：
当运行的时候，终端会输出数据。
大约运行10-20秒之后，
稳定帧数会输出为50，当环境诊断显示的结果跟实际结果相似的时候，可以记录当前结果。
当终端中多次出现近似数据，即可采用。


若雷达距离地面或房顶较近，需要进行z值的设定。 

<v2.0 增加补盲雷达的自标定部分>
通过修改launch文件，可以切换使用16线雷达或补盲雷达。
参数：lidar_type和mid_type.


-->


<!-- 
策略：
先定义基准墙面，可以通过三面墙构成一个空间。然后通过雷达获取的数据，拟合出三面墙，然后通过对比两个空间，可以得出x,y,yaw,roll,pitch的值，z需要手动测量。

-->