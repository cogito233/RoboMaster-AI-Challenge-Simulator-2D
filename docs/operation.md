# 控制指令说明

## 机器人动作指令

单个指令为一维数组，类型为：`int`，具体如下

|引索|名称|范围|解释|手控按键|
|---|---|---|---|---|
|0|x|-1~1|-1：后退，0：不动，1：前进|s/w|
|1|y|-1~1|-1：左移，0：不动，1：右移|q/e|
|2|rotate|-1~1|底盘，-1：左转，0：不动，1：右转|a/d|
|3|yaw|-1~1|云台，-1：左转，0：不动，1：右转|b/m|
|4|shoot|0~1|是否射击，0：否，1：是|space|
|5|supply|0~1|时候触发补给，0：否，1：是|f|
|6|shoot_mode|0~1|射击模式，0：单发，1：连发|r|
|7|auto_aim|0~1|是否启用自瞄，0：否，1：是|n|

## 辅助按键

|按键|解释|举例|
|---|---|---|
|键盘上方的数字|切换操作对象|比如按`2`，控制`car2`|
|Tab|展示更多信息|-|

## Record Player的按键

`Record Player`可以用来复现之前的游戏，更多信息请参考[record_player.md](./record_player.md)

|按键|解释|
|---|---|
|Tab|展示更多信息|
|←|后退|
|→|快进|
|space|暂停|