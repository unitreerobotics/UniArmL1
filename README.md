<div align="center">

# UniArmL1

  <a href="https://www.unitree.com/" target="_blank">
    <img src="https://www.unitree.com/images/0079f8938336436e955ea3a98c4e1e59.svg" alt="Unitree LOGO" width="25%">
  </a>

**UniArmL1** 开源 6-DOF 机械臂及遥操作框架

<!-- [![English](https://img.shields.io/badge/English-README-blue)](./README.md) [![中文](https://img.shields.io/badge/中文-README-green)](./README_zh.md) -->

</div>

<div align="center">
<table>
  <tr>
    <td><img src="media/readme/vr_demo.gif?raw=true" alt="VR demo" title="VR demo" style="max-width: 100%;" /></td>
    <td><img src="media/readme/leader_demo.gif?raw=true" alt="Leader demo" title="Leader demo" style="max-width: 100%;" /></td>
  </tr>
</table>
</div>

## ✳️ 概述

当前项目 UniArmL1 是一个轻量化的开源 6-DOF 机械臂及遥操作框架，支持三种控制模式与标准化数据采集，采集数据可无缝对接 [unitree_lerobot](https://github.com/unitreerobotics/unitree_lerobot) 进行模仿学习训练与部署。

**核心功能：**

- 🎮 **遥操作控制** — VR 手柄 / 键盘 / 主从臂三种模式实时控制 UniArmL1
- 📦 **数据采集** — 固定频率录制关节角度与相机图像，输出标准格式数据集
- 🔄 **训练对接** — 采集数据可直接用于 [unitree_lerobot](https://github.com/unitreerobotics/unitree_lerobot) 模仿学习训练

**完整流程：**

`硬件准备` → `校准` → `配置环境` → `遥操作 & 数据采集` → `训练` → `部署`

## 🔧 硬件准备

UniArmL1 机械臂的BOM 清单、组装指南与 3D 打印文件请参考：

👉 [unitree_UniArmL1_hardware](hardware/README_zh-CN.md)

<div align="center">
<table>
  <tr>
    <td style="width: 50%; text-align: center;">
      <img src="media/readme/real_follower.png?raw=true" alt="Follower" title="Follower" style="width: 98%; height: auto; object-fit: contain; display: block;" />
    </td>
    <td style="width: 50%; text-align: center;">
      <img src="media/readme/real_leader.png?raw=true" alt="Leader" title="Leader" style="width: 100%; height: auto; object-fit: contain; display: block;" />
    </td>
  </tr>
</table>
</div>

## 📥 配置环境

### **1. 克隆仓库**

```bash
git clone https://github.com/unitreerobotics/UniArmL1
cd UniArmL1
```

### **2. 安装依赖**

该脚本会安装全部依赖（含VR模式）：
```bash
bash setup.sh
```

### **3. VR环境配置**

使用VR遥操需额外配置以下步骤。VR 手柄模式需要安装 xrobotoolkit SDK，请参考 [xrobotoolkit 文档](https://github.com/XR-Robotics) 进行配置。

#### 3.1 PC 上安装 XRoboToolkit-PC-Service
```bash
wget https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases/download/v1.0.0/XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb

sudo dpkg -i XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb
```

#### 3.2 Pico 上安装客户端
- 下载 APK: https://github.com/XR-Robotics/XRoboToolkit-Unity-Client/releases/download/v1.1.1/XRoboToolkit-PICO-1.1.1.apk
- 将 Pico 连接电脑，把 APK 拖入 Pico Download 文件夹
- 在 Pico 中安装 APK

可以在资源库中找到 XRoboToolkit app。


## 🛠️ 遥操作

### **1. 检查端口**

```bash
conda activate UniArmL1
# 查看串口设备
python teleop/scripts/check_port_all.py
```

如果串口权限不足：

```bash
sudo chmod 777 /dev/ttyACM*
```

### **2. 校准**

首次使用或更换电机后，需要先完成机械臂校准。校准前请先确认主臂和从臂对应的串口号，并根据实际情况修改 `--port` 参数。

完整校准指令、零位姿态和关节旋转步骤请参考：[[校准过程]](hardware/README_zh-CN.md#15-校准过程)

> ⚠️ **未校准将无法正常控制机械臂！** 校准前请确保机械臂已正确组装并通电。

### **3. 修改配置文件**

默认配置保存在 `teleop/teleop_config.yaml`，可直接修改，也可通过命令行参数覆盖，详见[参数说明](#parameters)。


### **4. 启动遥操作**
#### 4.1 VR 手柄模式

<div align="center">
   <img
      src="media/readme/vr_demo.gif?raw=true"
      alt="VR demo"
      title="VR demo"
      style="width: 75%;"
   />
</div>

```bash
python teleop/teleop.py -i vr
```
VR 头戴打开 XRoboToolkit app，输入 PC 端IP，点击连接，勾选控制面板中的 controller、send；电脑端打开 XRoboToolkit-PC-Service软件，按住手柄侧键即可开始遥操作。

#### 4.2 键盘模式
启动之后按住某个方向键机械臂会对应缓慢移动。
```bash
python teleop/teleop.py -i keyboard
```
具体操控机械臂的按键会在终端中说明，也可以在代码中查看。

#### 4.3 主从遥操模式
<div align="center">
   <img
      src="media/readme/leader_demo.gif?raw=true"
      alt="Leader demo"
      title="Leader demo"
      style="width: 75%;"
   />
</div>
使用另一条 UniArmL1 作为主臂，直接映射关节角度：

```bash
python teleop/teleop.py -i leader --port /dev/ttyACM1 --leader-port /dev/ttyACM2
```

主臂处于零阻尼模式，从臂直接跟随主臂关节角度。


## 📦 数据采集

添加 `--record` 即可在遥操作过程中录制数据：

```bash
python teleop/teleop.py -i vr --record --task-dir ./data/pick_place --task-goal "pick up the cup"
```

## 🧑‍✈ 参数说明 <a id="parameters"></a>

以下是 `teleop.py` 支持的命令行参数及其说明：

- `--input`, `-i`: 输入源模式，可选值：`vr` (VR 手柄)、`keyboard` (键盘)、`leader` (主从臂)。默认值：`vr`。
- `--port`, `-p`: 从臂串口端口。默认值：`/dev/ttyACM1`。
- `--leader-port`: 主臂串口端口，仅在主从模式下使用。默认值：`/dev/ttyACM2`。
- `--urdf`: URDF 文件路径。默认值：`assets/uniarml1/urdf/UniArmL1.urdf`。
- `--mesh-dir`: 网格目录路径。默认值：`assets/uniarml1/urdf/`。
- `--cameras`, `-c`: 相机配置，格式为 `name:id`，例如 `head:0 wrist:2`。可以指定多个。
- `--no-camera`: 禁用相机显示。默认不启用。
- `--record`, `-r`: 启用数据录制。默认不启用。
- `--task-dir`: 录制数据目录。默认值：`./data/teleop`。
- `--task-goal`: 任务目标描述。默认值：空字符串。
- `--record-hz`: 录制频率 (Hz)。默认值：50。
- `--meshcat`: 启用 Meshcat 可视化。默认不启用。
- `--vr-scale`: VR 增量缩放因子。默认值：1.2。


## 🕊 训练部署
采集的数据可直接用于 [unitree_lerobot](https://github.com/unitreerobotics/unitree_lerobot) 的模仿学习训练与部署，详见 [unitree_lerobot 文档](https://github.com/unitreerobotics/unitree_lerobot)。

<div align="center">
   <img src="media/readme/vla_demo1.gif?raw=true" alt="UniArmL1 策略推理演示 1" title="UniArmL1 策略推理演示 1" style="width: 70%; height: auto; object-fit: contain; display: block; margin: 0 auto 20px auto;" />
   <img src="media/readme/vla_demo2.gif?raw=true" alt="UniArmL1 策略推理演示 2" title="UniArmL1 策略推理演示 2" style="width: 70%; height: auto; object-fit: contain; display: block; margin: 0 auto;" />
</div>

## 🎉 致谢

- [lerobot](https://github.com/huggingface/lerobot) — 开源机器人模仿学习框架，提供数据采集、训练和部署的完整工具链
- [SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100.git) — 开源机械臂硬件设计与组装指南
- [xr_teleoperate](https://github.com/unitreerobotics/xr_teleoperate.git) — Unitree XR 遥操作框架，提供了多种控制模式和数据采集功能
- [unitree_lerobot](https://github.com/unitreerobotics/unitree_lerobot.git) — Unitree 基于 LeRobot 的模仿学习实现，用于训练和部署机器人策略

## 许可证

本项目采用 Apache License 2.0 许可证 - 详见 [LICENSE](LICENSE) 文件。