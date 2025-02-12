# Kernal Manual

## 一、基本信息

### 1、基本参数

|项目|值|说明|
|---|---|---|
|游戏运行频率|200Hz|一个周期称为一个epoch，即200epoch/s|
|决策（操作）最高频率|20Hz|即20次/s|
|尺寸比例|10mm/pixel|即8m×5m的地图对应800p×500p的游戏屏幕|
|车的尺寸|60p×45p|即600mm×450mm|
|车前进后退最大速度|1.5p/epoch|即3m/s|
|车左右平移最大速度|1p/epoch|即2m/s|
|底盘旋转最大速度|200°/s|尚未进行精确测量，有待确定|
|云台旋转最大速度|600°/s|尚未进行精确测量，有待确定|
|子弹飞行速度|12.5p/epoch|即25m/s，可设置|
|枪口热度结算及其造成的扣血频率|10Hz||

### 2、须知

a. 底盘和云台的速度在按下相应的指令后会逐渐增加到最大，指令停止后速度会逐渐减小。具体控制指令请参考[operation.md](./operation.md)

b. 默认的队伍划分为，红方：car1, car3，蓝方：car2，car4，这样设计的原因是，可以只需通过改变车的数量，就能改变对抗模式。例如，当 car_num = 3 时，比赛模式为：2v1

c. `pygame`的作用只是可视化，即逻辑运算不依靠`pygame`，训练网络时不用依靠`pygame`

d. 手动操作时，比赛时间不以实际时间为准，以游戏时间为准

e. 手动操作时，一次只能控制一辆车

f. 有反弹效果，但不完全符合物理定律

g. 在距补给点的一定距离内才能触发补给，触发补给后会有3s无法控制

h. 当车的中心点在防御加成区的方形区域内时，会进行防御加成计时

## 二、可改变参数

在`kernal`类的`__init__`函数中，有一些可以视场地等环境改变的量，如下

```python
        self.bullet_speed = 12.5 # 子弹速度，单位为pixel
        self.motion = 6 # 移动的惯性感大小
        self.rotate_motion = 4 # 底盘旋转的惯性感大小
        self.yaw_motion = 1 # 云台旋转的惯性感大小
        self.camera_angle = 75 / 2 # 摄像头的视野范围
        self.lidar_angle = 120 / 2 # 激光雷达的视野视野范围
        self.move_discount = 0.6 # 撞墙之后反弹的强度大小
```

## 三、小功能

### 1、调用地图信息

`kernal.get_map()`，返回`g_map`，参数格式见：[params.md](./params.md/#g_map)

### 2、设置车的坐标

`kernal.set_car_loc(n, loc)`，`n`为车的编号，0~3，`loc`为车的坐标，二维数组，无返回
