#!/bin/bash

# 检查Docker是否安装
if ! command -v docker &> /dev/null; then
    echo "Docker未安装，正在安装Docker..."
    
    # 更新包索引
    sudo apt-get update
    
    # 安装依赖
    sudo apt-get install -y \
        apt-transport-https \
        ca-certificates \
        curl \
        gnupg \
        lsb-release
    
    # 添加Docker官方GPG密钥
    curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg
    
    # 设置稳定版仓库
    echo \
      "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu \
      $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
    
    # 安装Docker Engine
    sudo apt-get update
    sudo apt-get install -y docker-ce docker-ce-cli containerd.io
    
    # 启动Docker服务
    sudo systemctl start docker
    sudo systemctl enable docker
    
    # 将当前用户添加到docker组
    sudo usermod -aG docker $USER
    
    echo "Docker安装完成！请重新登录以应用组权限更改。"
fi

# 检查Docker Compose是否安装
if ! command -v docker-compose &> /dev/null; then
    echo "Docker Compose未安装，正在安装..."
    
    # 下载Docker Compose
    sudo curl -L "https://github.com/docker/compose/releases/download/v2.20.2/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose
    
    # 赋予执行权限
    sudo chmod +x /usr/local/bin/docker-compose
    
    echo "Docker Compose安装完成！"
fi

# 创建必要的目录
mkdir -p workspace models

# 设置X11转发权限
xhost +local:docker

echo "环境准备完成！"
echo "使用以下命令启动容器："
echo "docker-compose up -d"
echo "docker-compose exec drone_sim /bin/bash"
