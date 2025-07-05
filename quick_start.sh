#!/bin/bash

# 快速启动脚本
echo "快速启动无人机仿真环境..."

# 检查Docker是否运行
if ! docker info > /dev/null 2>&1; then
    echo "错误：Docker未运行"
    exit 1
fi

# 允许X11连接
xhost +local:docker 2>/dev/null

# 启动容器
docker-compose up -d

# 进入容器
docker-compose exec drone_sim /bin/bash
