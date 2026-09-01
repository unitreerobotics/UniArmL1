#!/bin/bash

set -e
conda config --set ssl_verify false
conda config --set remote_read_timeout_secs 300
conda config --set remote_connect_timeout_secs 120
conda config --remove-key channels
conda config --add channels http://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/main/
conda config --add channels http://mirrors.tuna.tsinghua.edu.cn/anaconda/cloud/conda-forge/
CONDA_FORGE_MIRROR="http://mirrors.tuna.tsinghua.edu.cn/anaconda/cloud/conda-forge/"
YELLOW='\033[1;33m'
RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m'

echo -e "${GREEN}=== XR Teleoperate 环境安装脚本 ===${NC}"

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO_ROOT="$SCRIPT_DIR"
ENV_NAME="${1:-UniArmL1}"
XROBO_PYBIND_DIR=""
OS_NAME="$(uname -s)"

retry_git_clone() {
    local repo_url="$1"
    local dest_dir="$2"
    local attempts=3
    local i

    for i in $(seq 1 "$attempts"); do
        if git clone --depth 1 "$repo_url" "$dest_dir"; then
            return 0
        fi
        echo "git clone failed ($i/$attempts): $repo_url"
        rm -rf "$dest_dir"
        if [[ "$i" -lt "$attempts" ]]; then
            echo "Retrying clone..."
        fi
    done

    echo "Error: failed to clone $repo_url after $attempts attempts."
    return 1
}

echo -e "${YELLOW}使用环境名: $ENV_NAME${NC}"
echo ""

echo -e "${YELLOW}[1/6] 创建/激活 Conda 环境${NC}"
cd "$REPO_ROOT/teleop"

mkdir -p xrobotoolkit
cd xrobotoolkit

PYTHON_VERSION="3.12"
echo "Forcing Python version: $PYTHON_VERSION"

if [ -f "$HOME/miniconda3/etc/profile.d/conda.sh" ]; then
    . "$HOME/miniconda3/etc/profile.d/conda.sh"
elif [ -f "$HOME/anaconda3/etc/profile.d/conda.sh" ]; then
    . "$HOME/anaconda3/etc/profile.d/conda.sh"
elif [ -f "/home/unitree/conda/yes/etc/profile.d/conda.sh" ]; then
    . "/home/unitree/conda/yes/etc/profile.d/conda.sh"
else
    echo "Conda initialization script not found. Please install Miniconda or Anaconda."
    exit 1
fi

# 自动接受 Conda ToS，避免非交互模式下 conda create 失败
if conda tos --help >/dev/null 2>&1; then
    conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/main >/dev/null 2>&1 || true
    conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/r >/dev/null 2>&1 || true
fi

if conda env list | awk '{print $1}' | grep -qx "$ENV_NAME"; then
    echo "Conda environment '$ENV_NAME' already exists, skipping creation."
else
    conda create -n "$ENV_NAME" python=$PYTHON_VERSION -y
fi

echo "Conda environment '$ENV_NAME' created with Python $PYTHON_VERSION"

conda activate "$ENV_NAME"

# pinocchio 由 placo (xrobotoolkit_teleop 的依赖) 通过 pip 的 pin 包自动安装
# placo 0.9.20 要求 pin (pinocchio 3.8.0)，与 conda 的 pinocchio 4.0.0 不兼容


echo -e "${YELLOW}[2/6] 安装 xrobotoolkit_teleop（通过 setup_conda.sh）${NC}"

if [[ -z "${CONDA_DEFAULT_ENV}" ]]; then
    echo "Error: No conda environment is currently activated."
    echo "Please activate a conda environment first with: conda activate <env_name>"
    exit 1
fi
ENV_NAME=${CONDA_DEFAULT_ENV}

CURRENT_PYTHON_VERSION=$(python --version 2>&1 | grep -oP '\d+\.\d+')
if [[ "$CURRENT_PYTHON_VERSION" != "3.12" ]]; then
    echo "Warning: Current Python version is $CURRENT_PYTHON_VERSION, but Python 3.12 is required."
    echo "Please activate a conda environment with Python 3.12."
    exit 1
fi

if [[ "$OS_NAME" == "Linux" ]]; then
    conda install -c "$CONDA_FORGE_MIRROR" libstdcxx-ng -y
fi

pip install --upgrade pip

# cmake 缺失会导致 setup.py 阶段报错退出
if ! command -v cmake >/dev/null 2>&1; then
    echo "cmake not found in current environment, installing via conda-forge..."
    conda install -c "$CONDA_FORGE_MIRROR" cmake -y
fi

if ! command -v cmake >/dev/null 2>&1; then
    echo "Error: cmake is still unavailable after installation attempt."
    exit 1
fi

# make 和 C++ 编译器是 CMake 构建所必需的
if ! command -v make >/dev/null 2>&1; then
    echo "make not found in current environment, installing via conda-forge..."
    conda install -c "$CONDA_FORGE_MIRROR" make -y
fi

if ! command -v g++ >/dev/null 2>&1; then
    echo "g++ (C++ compiler) not found, installing via conda-forge..."
    conda install -c "$CONDA_FORGE_MIRROR" cxx-compiler -y
fi

DEPS_DIR="$REPO_ROOT/teleop/xrobotoolkit/dependencies"
LOCAL_PYBIND_REPO_DIR="$REPO_ROOT/XRoboToolkit-PC-Service-Pybind-main"
PYBIND_REPO_DIR="$DEPS_DIR/XRoboToolkit-PC-Service-Pybind-main"

# 优先使用本地仓库，缺失时才克隆
if [[ -d "$LOCAL_PYBIND_REPO_DIR" && -f "$LOCAL_PYBIND_REPO_DIR/setup.py" ]]; then
    PYBIND_REPO_DIR="$LOCAL_PYBIND_REPO_DIR"
    echo "Using local XRoboToolkit-PC-Service-Pybind: $PYBIND_REPO_DIR"
else
    mkdir -p "$DEPS_DIR"
    if [[ -d "$PYBIND_REPO_DIR" && -f "$PYBIND_REPO_DIR/setup.py" ]]; then
        echo "Using existing XRoboToolkit-PC-Service-Pybind directory: $PYBIND_REPO_DIR"
    elif [[ ! -d "$PYBIND_REPO_DIR/.git" ]]; then
        retry_git_clone https://github.com/XR-Robotics/XRoboToolkit-PC-Service-Pybind.git "$PYBIND_REPO_DIR"
    else
        echo "Using existing cloned XRoboToolkit-PC-Service-Pybind: $PYBIND_REPO_DIR"
    fi
fi

SERVICE_REPO_DIR="$PYBIND_REPO_DIR/tmp/XRoboToolkit-PC-Service-main"
SDK_DIR="$SERVICE_REPO_DIR/RoboticsService/PXREARobotSDK"
XROBO_PYBIND_DIR="$PYBIND_REPO_DIR"

cd "$PYBIND_REPO_DIR"

mkdir -p tmp
cd tmp
if [[ -d "$SERVICE_REPO_DIR/RoboticsService/PXREARobotSDK" ]]; then
    echo "Using existing XRoboToolkit-PC-Service directory: $SERVICE_REPO_DIR"
elif [[ ! -d "$SERVICE_REPO_DIR/.git" ]]; then
    retry_git_clone https://github.com/XR-Robotics/XRoboToolkit-PC-Service.git "$SERVICE_REPO_DIR"
else
    echo "Using existing cloned XRoboToolkit-PC-Service: $SERVICE_REPO_DIR"
fi
bash "$SDK_DIR/build.sh"

mkdir -p "$PYBIND_REPO_DIR/lib" "$PYBIND_REPO_DIR/include"

HEADER_SRC="$SDK_DIR/PXREARobotSDK.h"
# fallback 路径：上游仓库结构可能变化
if [[ ! -f "$HEADER_SRC" ]]; then
    HEADER_SRC="$SERVICE_REPO_DIR/RoboticsService/SDK/include/PXREARobotSDK.h"
fi

NLOHMANN_SRC="$SDK_DIR/nlohmann"
if [[ ! -d "$NLOHMANN_SRC" ]]; then
    NLOHMANN_SRC="$SERVICE_REPO_DIR/RoboticsService/SDK/include/nlohmann"
fi

LIB_SRC="$SDK_DIR/build/libPXREARobotSDK.so"
# so 位置因分支/平台不同，用 find 兜底探测
if [[ ! -f "$LIB_SRC" ]]; then
    LIB_SRC=$(find "$SERVICE_REPO_DIR/RoboticsService" -type f -name libPXREARobotSDK.so | head -n 1)
fi

if [[ ! -f "$HEADER_SRC" ]]; then
    echo "Error: PXREARobotSDK.h not found."
    exit 1
fi

if [[ ! -d "$NLOHMANN_SRC" ]]; then
    echo "Error: nlohmann include directory not found."
    exit 1
fi

if [[ -z "$LIB_SRC" || ! -f "$LIB_SRC" ]]; then
    echo "Error: libPXREARobotSDK.so not found."
    exit 1
fi

cp "$HEADER_SRC" "$PYBIND_REPO_DIR/include/"
rm -rf "$PYBIND_REPO_DIR/include/nlohmann"
cp -r "$NLOHMANN_SRC" "$PYBIND_REPO_DIR/include/"
cp "$LIB_SRC" "$PYBIND_REPO_DIR/lib/"
cd "$PYBIND_REPO_DIR"

if [[ -n "$CONDA_DEFAULT_ENV" ]]; then
    conda install -c "$CONDA_FORGE_MIRROR" pybind11 -y
else
    pip install pybind11
fi

pip uninstall -y xrobotoolkit_sdk || true
python setup.py install
cd "$REPO_ROOT"


echo -e "${YELLOW}[4/6] 安装 xrobotoolkit_teleop${NC}"
TELEOP_SAMPLE_DIR="$DEPS_DIR/XRoboToolkit-Teleop-Sample-Python"

if [[ -d "$REPO_ROOT/xrobotoolkit_teleop" ]]; then
    pip install -e "$REPO_ROOT" || { echo "Failed to install xrobotoolkit_teleop from repo root"; exit 1; }
    echo -e "[INFO] xrobotoolkit_teleop installed from repo root in conda environment '$ENV_NAME'."
elif [[ -f "$REPO_ROOT/teleop/pyproject.toml" || -f "$REPO_ROOT/teleop/setup.py" ]]; then
    pip install -e "$REPO_ROOT/teleop" || { echo "Failed to install project from teleop directory"; exit 1; }
    echo -e "[INFO] Python project installed from teleop directory in conda environment '$ENV_NAME'."
else
    # 本地没有 xrobotoolkit_teleop，从 GitHub 克隆上游仓库
    if [[ -d "$TELEOP_SAMPLE_DIR/xrobotoolkit_teleop" ]]; then
        echo "Using existing XRoboToolkit-Teleop-Sample-Python: $TELEOP_SAMPLE_DIR"
    else
        echo "Cloning XRoboToolkit-Teleop-Sample-Python..."
        retry_git_clone https://github.com/XR-Robotics/XRoboToolkit-Teleop-Sample-Python.git "$TELEOP_SAMPLE_DIR"
    fi
    pip install -e "$TELEOP_SAMPLE_DIR" || { echo "Failed to install xrobotoolkit_teleop"; exit 1; }
    echo -e "[INFO] xrobotoolkit_teleop installed from $TELEOP_SAMPLE_DIR in conda environment '$ENV_NAME'."
fi
echo -e "\n"

echo -e "${YELLOW}[5/6] 安装缺失的依赖${NC}"
pip install scipy==1.17.1
pip install draccus==0.10.0
pip install rerun-sdk==0.27.1
pip install logging_mp==0.1.6
pip install pyserial==3.5
pip install opencv-python
pip install pyyaml
pip install --upgrade --force-reinstall placo

echo -e "${YELLOW}[6/6] 配置环境变量${NC}"
# CONDA_PREFIX/lib 必须在 LD_LIBRARY_PATH 前面，
# 因为系统 libstdc++ 太旧（CXXABI 最高 1.3.13），pinocchio 4.0 需要 1.3.15
CONDA_LIB="\$CONDA_PREFIX/lib"
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:$XROBO_PYBIND_DIR/lib:$LD_LIBRARY_PATH"
export PYTHONPATH="$HOME/unitree/lerobot/src:$PYTHONPATH"

echo -e "${GREEN}=== 安装完成！ ===${NC}"

# 幂等写入 ~/.bashrc
ENV_LINES=(
    "export LD_LIBRARY_PATH=\$CONDA_PREFIX/lib:$XROBO_PYBIND_DIR/lib:\$LD_LIBRARY_PATH"
    "export PYTHONPATH=\$HOME/unitree/lerobot/src:\$PYTHONPATH"
)

for line in "${ENV_LINES[@]}"; do
    if ! grep -qF "$line" "$HOME/.bashrc" 2>/dev/null; then
        echo "$line" >> "$HOME/.bashrc"
    fi
done

echo ""
echo -e "${YELLOW}验证安装：${NC}"
echo "conda activate $ENV_NAME"
