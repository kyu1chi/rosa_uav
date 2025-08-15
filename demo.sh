#!/usr/bin/env bash
# Copyright (c) 2024. Jet Propulsion Laboratory. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# This script launches the ROSA demo in Docker

# Check if Docker is installed
if ! command -v docker &> /dev/null; then
    echo "Error: Docker is not installed. Please install Docker and try again."
    exit 1
fi

# Set default headless mode
HEADLESS=${HEADLESS:-false}
DEVELOPMENT=${DEVELOPMENT:-false}

# Function to detect environment
detect_environment() {
    if grep -qi microsoft /proc/version 2>/dev/null; then
        echo "WSL2"
    elif [ -n "$SSH_CLIENT" ]; then
        echo "SSH"
    elif [ -n "$DISPLAY" ] && [ "$DISPLAY" != "host.docker.internal:0" ]; then
        echo "LOCAL"
    else
        echo "UNKNOWN"
    fi
}

# Enable X11 forwarding based on OS and environment
echo "🔧 检测系统环境..."
ENV_TYPE=$(detect_environment)
echo "环境类型: $ENV_TYPE"

case "$(uname)" in
    Linux*)
        echo "Enabling X11 forwarding for Linux..."
        
        case $ENV_TYPE in
            "WSL2")
                echo "检测到WSL2环境"
                # 获取Windows主机IP地址
                WINDOWS_IP=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}' | head -1)
                export DISPLAY=$WINDOWS_IP:0.0
                echo "设置DISPLAY为: $DISPLAY"
                echo "⚠️ 请确保Windows上运行了X11服务器 (如VcXsrv, X410, 或Xming)"
                ;;
            "SSH")
                echo "检测到SSH连接"
                if [ -z "$DISPLAY" ]; then
                    echo "❌ SSH X11转发未启用，请使用 'ssh -X' 连接"
                    exit 1
                fi
                ;;
            "LOCAL"|*)
                echo "本地Linux环境"
                if [ -z "$DISPLAY" ]; then
                    export DISPLAY=:0
                fi
                ;;
        esac
        
        # 测试X11连接
        echo "🧪 测试X11连接..."
        if timeout 5 xdpyinfo >/dev/null 2>&1; then
            echo "✅ X11连接成功"
        elif timeout 5 xset q >/dev/null 2>&1; then
            echo "✅ X11连接成功 (通过xset)"
        else
            echo "❌ X11连接失败"
            echo "故障排除建议:"
            echo "1. 确保在图形桌面环境中运行"
            echo "2. WSL2用户: 安装并启动VcXsrv (允许公共访问)"
            echo "3. SSH用户: 使用 'ssh -X' 连接"
            echo "4. 本地用户: 确保X11服务器运行"
            echo "5. 当前DISPLAY: $DISPLAY"
            
            # 尝试常见的DISPLAY设置
            echo "🔄 尝试其他DISPLAY设置..."
            for display_option in ":0" ":1" "localhost:10.0"; do
                export DISPLAY=$display_option
                echo "尝试DISPLAY=$DISPLAY"
                if timeout 3 xset q >/dev/null 2>&1; then
                    echo "✅ 找到工作的DISPLAY: $DISPLAY"
                    break
                fi
            done
            
            # 最后检查
            if ! timeout 3 xset q >/dev/null 2>&1; then
                echo "❌ 无法建立X11连接，继续以无头模式运行..."
                export HEADLESS=true
            fi
        fi
        
        # 启用Docker X11访问
        if [ "$HEADLESS" != "true" ]; then
            echo "🔓 启用Docker X11访问..."
            xhost +local:docker 2>/dev/null || xhost + 2>/dev/null || echo "⚠️ xhost命令失败"
        fi
        ;;
        
    Darwin*)
        echo "Enabling X11 forwarding for macOS..."
        # macOS需要XQuartz
        if ! command -v xquartz &> /dev/null; then
            echo "⚠️ 请安装XQuartz: brew install --cask xquartz"
        fi
        
        # 获取本机IP
        IP=$(ifconfig en0 | grep inet | awk '$1=="inet" {print $2}' | head -1)
        if [ -z "$IP" ]; then
            IP="localhost"
        fi
        export DISPLAY=$IP:0
        
        echo "设置DISPLAY为: $DISPLAY"
        xhost + $IP 2>/dev/null || echo "⚠️ xhost命令失败"
        ;;
        
    MINGW*|CYGWIN*|MSYS*)
        echo "Enabling X11 forwarding for Windows..."
        export DISPLAY=host.docker.internal:0
        echo "⚠️ Windows用户请确保运行X11服务器"
        ;;
        
    *)
        echo "Error: Unsupported operating system."
        exit 1
        ;;
esac

echo "最终DISPLAY设置: $DISPLAY"

# Build and run the Docker container
CONTAINER_NAME="rosa-turtlesim-demo"
echo "Building the $CONTAINER_NAME Docker image..."
docker build --build-arg DEVELOPMENT=$DEVELOPMENT -t $CONTAINER_NAME -f Dockerfile . || { echo "Error: Docker build failed"; exit 1; }

echo "Running the Docker container..."

# 构建Docker运行参数
DOCKER_ARGS="-it --rm --name $CONTAINER_NAME"
DOCKER_ARGS="$DOCKER_ARGS -e HEADLESS=$HEADLESS"
DOCKER_ARGS="$DOCKER_ARGS -e DEVELOPMENT=$DEVELOPMENT"
DOCKER_ARGS="$DOCKER_ARGS -v $PWD/src:/app/src"
DOCKER_ARGS="$DOCKER_ARGS -v $PWD/tests:/app/tests"
DOCKER_ARGS="$DOCKER_ARGS --network host"

# 添加X11相关参数（如果不是无头模式）
if [ "$HEADLESS" != "true" ]; then
    DOCKER_ARGS="$DOCKER_ARGS -e DISPLAY=$DISPLAY"
    DOCKER_ARGS="$DOCKER_ARGS -v /tmp/.X11-unix:/tmp/.X11-unix:rw"
    
    # 添加Xauthority文件（如果存在）
    if [ -f "$HOME/.Xauthority" ]; then
        DOCKER_ARGS="$DOCKER_ARGS -v $HOME/.Xauthority:/root/.Xauthority:rw"
    fi
    
    # 添加其他X11环境变量
    DOCKER_ARGS="$DOCKER_ARGS -e QT_X11_NO_MITSHM=1"
fi

# 运行容器
eval "docker run $DOCKER_ARGS $CONTAINER_NAME"

# Disable X11 forwarding
if [ "$HEADLESS" != "true" ]; then
    echo "🧹 清理X11权限..."
    xhost -local:docker 2>/dev/null || xhost - 2>/dev/null || true
fi

exit 0
