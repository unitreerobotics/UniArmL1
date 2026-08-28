<div align="center">

   <h1 align="center"> Open UniArmL1 </h1>
  <a href="https://www.unitree.com/" target="_blank">
    <img src="https://www.unitree.com/images/0079f8938336436e955ea3a98c4e1e59.svg" alt="Unitree LOGO" width="25%">
  </a>
  <!-- <p align="center">
    <a href="README.md"> English </a> | <a>中文</a>
  </p> -->
</div>

# 0. 📖 介绍
UniArmL1 是宇树科技设计与开发的 6-DOF 机械臂，现已开源，欢迎大家使用与进一步开发，一起帮助让面向机器人领域的端到端AI技术更加易于获取和使用！
<div align="center">
   <img
      src="../media/readme/real_follower.png?raw=true"
      alt="UniArmL1 follower"
      title="UniArmL1 follower"
      style="width: 42.1%;"
    />
   <img
      src="../media/readme/real_leader.png?raw=true"
      alt="UniArmL1 leader"
      title="UniArmL1 leader"
      style="width: 40%;"
    />
</div>

通过标准化软硬件接口，UniArmL1 能快速接入现有宇树平台，降低集成与二次开发门槛，缩短从原型到应用的周期。


# 1. 构建你的UniArmL1
我们目前仅支持自行构建的方式，步骤如下：

1. 首先需要购买非打印零部件，购买清单见[1.1 采购零件](#11-采购零件)
2. 3D打印机械臂主体零件，具体说明见[1.2 打印零件](#12-打印零件)
3. 请参照我们的电机配置方法配置电机[1.3 电机配置](#13-电机配置)
4. 请跟随我们的[1.4 装配指导](#14-装配指导)


## 1.1 采购零件

如需搭建遥操作系统，请购买以下两套机械臂所需的全部部件。

### 主从机械臂零件清单：

| 部件 | 数量 | 单价 (RMB) | 购买链接 |
| ---- | ---- |----------| ------- |
| J288舵机| 16 | 599 | [Alibaba](https://detail.tmall.com/item.htm?app=chrome&bxsign=scdz2f0snPj9vmk_AgV-zO3sLQEPFOK6-qCQ_pbcOg28FLIdlV4KcqE-ElChcgXl_ajb0ftzEknjg3moi7xdRPkixF1vgzs914wrqi-mXl_DLVOkwQ_4pfwdENqKzn6Nk2d&cpp=1&h5_spm=a-tb-item.b-tb-item&id=1013064765085&share_crt_v=1&shareurl=true&short_name=h.iH9q7JQ33hJUSic&sp_tk=SUtKZzVqWFJaSlM%3D&spm=a2159r.13376460.0.0&tbSocialPopKey=shareItem&tk=IKJg5jXRZJSHU287&un=80f1c3ec9848a15f0a62cb2c046d31df&un_site=0&ut_sk=1.aPJDlV7XQ34DAFwSBD1M3rTL_21646297_1777025038443.TaoPassword-WeiXin.1&wxsign=tbwnYmcZ8qSnI5hBqnSXjxB8FJgKhTIwlmp-ztXX2anJ0hQkn9DQ5400DsjLagVrgu1H18lj8UNZ-uxLbbvjZlV-8qDDuA_F7VgL0XOlFeH4gjEYhbYOce3MJvitg8XLuqzz5L2-Og3Ykvvgf8h7-P6CA&x-ssr=true&skuId=6181458503257) |
| 电机控制板 | 2 | 99 | [Alibaba](https://detail.tmall.com/item.htm?app=chrome&bxsign=scdsFfl08q1Uys4UlipmXu-jQGY7v4pCkQdEoSoXXuon--FO2l3IVPX_JyK2JTxN3AJV8y7i9d-9rnL21btzcooI-7RU0TRv9xN8OKFIgT6ieJ3bF1HAOPWj0pbrSHvQyGA&cpp=1&h5_spm=a-tb-item.b-tb-item&id=1013064765085&share_crt_v=1&shareurl=true&short_name=h.iH9q7JQ33hJUSic&sp_tk=SUtKZzVqWFJaSlM%3D&spm=a2159r.13376460.0.0&tbSocialPopKey=shareItem&tk=IKJg5jXRZJSHU287&un=80f1c3ec9848a15f0a62cb2c046d31df&un_site=0&ut_sk=1.aPJDlV7XQ34DAFwSBD1M3rTL_21646297_1777025038443.TaoPassword-WeiXin.1&wxsign=tbwnYmcZ8qSnI5hBqnSXjxB8FJgKhTIwlmp-ztXX2anJ0hQkn9DQ5400DsjLagVrgu1H18lj8UNZ-uxLbbvjZlV-8qDDuA_F7VgL0XOlFeH4gjEYhbYOce3MJvitg8XLuqzz5L2-Og3Ykvvgf8h7-P6CA&x-ssr=true&skuId=6181458503258)| 
| 腕部相机 | 1 | 190 | [Alibaba](https://item.taobao.com/item.htm?id=739033087984&skuId=5267839634724)|
| 全局相机 | 1 | 183 | [Alibaba](https://detail.tmall.com/item.htm?id=631167230453&skuId=6105245094571)| 
| USB-C 数据线 | 2 | 23.5 | [Alibaba](https://e.tb.cn/h.itebBTDTzUCVMnm?tk=Ia1Z5jXFDnAMF278)| 
| 24V-5A电源适配器 | 2 | 30 | [Alibaba](https://item.taobao.com/item.htm?abbucket=10&id=769661452640&mi_id=0000lPJIlFPmQ5qQpd5ZGiE5ihEapK61h68lGSSAgUbv0vI&ns=1&priceTId=2150472e17688106605562084e1bef&skuId=5623956616060&spm=a21n57.1.hoverItem.7&utparam=%7B%22aplus_abtest%22%3A%2280ebc6dc211e261f18bac44dbc47a7b0%22%7D&xxc=taobaoSearch)|
| 电源转换线 | 2 | 9.9 | [Alibaba](https://item.taobao.com/item.htm?id=944933463975&skuId=5850600264594)|
| 固定器 | 4 | 7.8 | [Alibaba](https://detail.tmall.com/item.htm?id=801399113134&skuId=5817329226938) |

请额外准备紧固件若干：72× M2×10 螺栓，92× M2×8 螺栓，14× M2×5 螺栓，4× M3×6 螺栓，4× M3 螺母

## 1.2 打印零件
### 步骤1：选择打印机
我们提供的 STL 文件已可直接用于多种 FDM 3D 打印机进行打印。下面列出了经过测试和建议的打印设置，不过其他设置也可能同样适用。
1. 材料：PLA+
2. 喷嘴直径与打印精度：使用 0.4 mm 喷嘴时，层高为 0.2 mm
3. 填充密度：30%
4. 示例打印机：[Bambu Lab A/P/X-series](https://bambulab.com)，[Prusa MINI+](https://www.prusa3d.com/product/original-prusa-mini-semi-assembled-3d-printer-4/), [UP Plus 2](https://shop.tiertime.com/product/tiertime-up-plus-2-3d-printer/) 等

### 步骤 2：设置打印机
1. 按照打印机对应的说明，确保打印机已完成校准，并且打印床已正确调平 
2. 清洁打印床，确保表面没有灰尘或油污。如果用水或其他液体清洁了打印床，请先将其彻底擦干
3. 如果你的打印机有此建议，可使用普通固体胶，在打印床的打印区域均匀涂上一层薄薄的胶水，避免局部堆积或涂抹不均
4. 按照打印机对应的说明装入打印耗材
5. 确保打印机设置与上文建议的参数一致；大多数打印机会提供多个选项，选择最接近建议值的设置即可
6. 将支撑设置为“开启支撑”，阈值角度为30°
7. 水平轴向的螺丝孔内部不应生成支撑

### 步骤3：打印部件
用于 leader 和 follower 的所有零件都已经整合在单个文件中，并且已为 3D 打印做好优化处理；零件朝向也已正确设置，以尽量减少支撑结构并确保打印精度。
对于 220 mm × 220 mm 打印床尺寸的打印机，请打印以下文件：
- [打印文件](STL/UniArmL1.3mf)


>注意！末端由于Follower需要加装相机，Leader需要加装把手，模型并不相同，但是都在一个打印文件中，可以参考部件朝向。

如果有不同的打印或替换需求，下面的表格包含了所有单独的文件：
1. 通用部分：

| Part | Link  |
|------|-------|
| Base.stl         | [Base.stl](STL/common/Base.stl)       |
| Base_motor_holder.stl        | [Base_motor_holder.stl](STL/common/Base_motor_holder.stl) |
| Rotation_Pitch.stl   | [Rotation_Pitch.stl](STL/common/Rotation_Pitch.stl) |
| Under_arm.stl  | [Under_arm.stl](STL/common/Under_arm.stl) |
| Upper_arm.stl  | [Upper_arm.stl](STL/common/Upper_arm.stl) |
| Under_motor_holder.stl       | [Under_motor_holder.stl](STL/common/Under_motor_holder.stl) |
| Wrist_roll_pitch.stl        | [Wrist_roll_pitch.stl](STL/common/Wrist_roll_pitch.stl) |


2. Leader-专有部分

| Part | Link  |
|------|-------|
| Handle.stl | [Handle.stl](STL/leader/handle/handel.stl)       |
| Trigger.stl | [Trigger.stl](STL/leader/handle/Trigger.stl) |



3. Follower-专有部分

| Part | Link  |
|------|-------|
| Finger.stl | [Finger.stl](STL/follower/eef/Finger.stl) |
| Gripper_gear.stl | [Gripper_gear.stl](STL/follower/eef/Gripper_gear.stl) |
| Gripper_shell.stl | [Gripper_shell.stl](STL/follower/eef/Gripper_shell.stl) |
| Guide_rail.stl | [Guide_rail.stl](STL/follower/eef/Guide_rail.stl) |

4. 相机支架

| Part | Link  |
|------|-------|
| Camera_mount_up.stl | [Camera_mount_up.stl](STL/follower/camera/Camera_mount_up.stl) |
| Camera_mount_down.stl | [Camera_mount_down.stl](STL/follower/camera/Camera_mount_down.stl) |


### 步骤4：移除支撑
1. 打印完成后，用小铲将零件从打印床上铲下来。
2. 然后去除零件上的所有支撑材料。

## 1.3 电机配置
各个关节分别需要使用哪些电机，如下表所示：
| 关节名称 | 数量  | 下标 |
|------|-------|-------|
|Shoulder Pan(base)| 1 | 0 |
|Shoulder Lift| 2 | 1&6 |
|Elbow Flex| 2 | 2&7 |
|Wrist Flex| 1 | 3 |
|Wrist Roll| 1 | 4 |
|Gripper| 1 | 5 |

首先下载上位机软件 [电机调试助手](doc/Motor_Assistant_V1.1.0.zip)，目前只支持win10和win11系统。

控制板一端连接PC，一端连接电机，在功能区可以选择电机型号、选择通信串口、控制的电机 ID 号。

修改 ID：将当前电机 ID 更改为指定的目标 ID。操作成功后，电机 ID 将更新为目标值，同时上位机软件中控制的电机 ID 也将同步切换至新 ID。

按照上表所示依次修改电机ID，主从臂同理。
## 1.4 装配指导
此为 Follower 在 solidworks 中的总装图，仅供参考：
<div align="center">
   <img
      src="media/assembly/assembled.jpg?raw=true"
      alt="follower"
      title="follower"
      style="width: 70%;"
    />
</div>

以下为分步装配过程：

1. 基座与 0 号电机：注意在 0 号电机爆炸图左侧连接通往 1 号电机的排线，并使用 4 颗 M2×10 螺栓固定。
<div align="center">
   <img
      src="media/base+motor.gif?raw=true"
      alt="base and motor"
      title="base and motor"
      style="width: 70%;"
    />
</div>

2. 在基座上安装电机套，并在电机转轴处固定 Rotation_Pitch；两侧使用 M2×8 螺栓，电机侧使用 M2×10 螺栓。
<div align="center">
   <img
      src="media/base_holder.gif?raw=true"
      alt="base holder"
      title="base holder"
      style="width: 80%;"
    />
</div>

3. 注意，**ID 为 1 和 2 的电机输出轴需朝向机械臂右侧，请先接入排线再固定电机**。将双电机固定在 Rotation_Pitch 上，上臂部分同理；电机编号对应关系为 [2, 7; 1, 6]，电机两侧使用 M2×5 螺栓。
<div align="center">
   <img
      src="media/Upperarm+motor.gif?raw=true"
      alt="upperarm and motor"
      title="upperarm and motor"
      style="width: 80%;"
    />
</div>

4. 将电机装入 arm 后加装固定保护套，再固定 Wrist_roll_pitch。ID 为 3 的电机输出轴朝左；4 号电机使用 M2×10 螺栓，其余位置使用 M2×8 螺栓。
<div align="center">
   <img
      src="media/under_arm+wrist_pitch.gif?raw=true"
      alt="under arm and wrist pitch"
      title="under arm and wrist pitch"
      style="width: 80%;"
    />
</div>

5. 最后进行末端夹爪装配，请注意以下两点：
    - **先固定外壳和 4 号电机**，再将 5 号电机装入外壳并固定，随后安装齿轮连接件。
    - **先将平行夹爪手指与导轨配合后**，再放置到外壳上固定。
<div align="center">
   <img
      src="media/Gripper.gif?raw=true"
      alt="gripper assembly"
      title="gripper assembly"
      style="width: 80%;"
    />
</div>

6. 安装 Leader 握把：将扳机机构与 5 号电机输出轴连接。扳机处使用 6 颗 M2×8 螺栓，其余位置使用 8 颗 M2×10 螺栓。
<div align="center">
<table>
  <tr>
    <td>
      <img
        src="media/leader_handle_assembly_1.jpg?raw=true"
        alt="leader handle assembly 1"
        title="leader handle assembly 1"
        style="max-width: 100%;"
      />
    </td>
    <td>
      <img
        src="media/leader_handle_assembly_2.jpg?raw=true"
        alt="leader handle assembly 2"
        title="leader handle assembly 2"
        style="max-width: 100%;"
      />
    </td>
  </tr>
</table>
</div>



## 1.5 校准过程
请根据自己查找到的端口修改 --port 参数。

### 从臂（Follower）
1. 校准指令：
```bash
python teleop/scripts/calibrate_uniarml1.py --port /dev/ttyACM1 --id follower
```
2. 将从臂放置在零位，确认姿态后回车。
<div align="center">
   <img
      src="media/zero_follower.jpg?raw=true"
      alt="follower zero pose"
      title="follower zero pose"
      style="width: 42%;"
    />
</div>
3. 依次旋转每个关节到左右最大的可移动范围，完成后回车。
<div align="center">
   <img
      src="media/follower_cali.gif?raw=true"
      alt="rotation reference"
      title="rotation reference"
      style="width: 60%;"
    />
</div>

### 主臂（Leader）
1. 校准指令：
```bash
python teleop/scripts/calibrate_uniarml1.py --port /dev/ttyACM3 --id leader
```
2. 将主臂放置在零位，确认姿态后回车（两种姿态，按使用习惯选择其一）。
<div align="center">
<table>
  <tr>
    <td>
      <img
        src="media/zero_leader_1.jpg?raw=true"
        alt="leader zero pose 1"
        title="leader zero pose 1"
        style="max-width: 100%;"
      />
    </td>
    <td>
      <img
        src="media/zero_leader_2.jpg?raw=true"
        alt="leader zero pose 2"
        title="leader zero pose 2"
        style="max-width: 100%;"
      />
    </td>
  </tr>
</table>
</div>
3. 按提示依次旋转每个关节到左右最大的可移动范围，腕部关节主臂与从臂应左右旋转相同范围，完成后回车。
<div align="center">
   <img
      src="media/leader_cali.gif?raw=true"
      alt="rotation reference"
      title="rotation reference"
      style="width: 60%;"
    />
</div>

校准数据保存在 `~/.cache/unitree/calibration/` 目录下。

