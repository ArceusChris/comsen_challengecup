#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
YOLOv12推理脚本，与yolo12_bridge_node.py配合使用

使用示例：
    python3 yolo12_inference.py --model yolov12n.pt --device cpu --conf 0.45 --iou 0.45
    
    # 使用不同端口（适用于同时运行多个推理节点）
    python3 yolo12_inference.py --model yolov12n.pt --device cpu --sender-port 5555 --receiver-port 5556
    python3 yolo12_inference.py --model yolov12n.pt --device cpu --sender-port 5557 --receiver-port 5558
"""

import cv2
import numpy as np
import zmq
import json
import time
import argparse
from ultralytics import YOLO
import os
import sys

# 加载YOLO模型
def load_yolo(model_path="yolov12n.pt", device="cpu"):
    """加载YOLO模型"""
    try:
        # 尝试多个可能的模型路径
        model_paths = [
            model_path,                              # 原始指定路径
            os.path.join(os.getcwd(), model_path),   # 当前工作目录
            os.path.join(os.path.dirname(os.path.abspath(__file__)), model_path),  # 脚本所在目录
            os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "models", os.path.basename(model_path)),  # 包目录下的models文件夹
            os.path.join("/home/xylin/comsen_challengecup", model_path),  # 工作空间根目录
            os.path.join("/home/xylin/comsen_challengecup/models", os.path.basename(model_path)),  # 工作空间中的models目录
        ]
        
        # 查找第一个存在的模型文件
        found_model_path = None
        for path in model_paths:
            print(f"尝试查找模型文件: {path}")
            if os.path.exists(path):
                found_model_path = path
                print(f"找到模型文件: {path}")
                break
        
        if found_model_path is None:
            print("无法找到任何有效的模型文件路径")
            sys.exit(1)
        
        # 更新模型路径
        model_path = found_model_path
        
        # 加载模型
        model = YOLO(model_path)
        model.to(device)
        return model
    except Exception as e:
        print(f"加载YOLO模型出错: {e}")
        raise

def main():
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='YOLOv12推理脚本，与ROS桥接节点通信')
    parser.add_argument('--model', type=str, default='yolov12n.pt', help='YOLO模型路径')
    parser.add_argument('--device', type=str, default='cpu', help='运行推理的设备 (cpu/cuda)')
    parser.add_argument('--conf', type=float, default=0.6, help='置信度阈值')
    parser.add_argument('--iou', type=float, default=0.45, help='IOU阈值')
    parser.add_argument('--sender-port', type=int, default=5555, help='从桥接节点接收图像的端口(与桥接节点的sender_port对应)')
    parser.add_argument('--receiver-port', type=int, default=5556, help='向桥接节点发送结果的端口(与桥接节点的receiver_port对应)')
    args = parser.parse_args()

    # 设置ZeroMQ
    context = zmq.Context()

    # 用于从ROS节点接收图像的Socket
    receiver = context.socket(zmq.PULL)
    receiver.connect(f"tcp://127.0.0.1:{args.sender_port}")
    print(f"连接到图像发送端口: {args.sender_port}")

    # 用于向ROS节点发送结果的Socket
    sender = context.socket(zmq.PUSH)
    sender.connect(f"tcp://127.0.0.1:{args.receiver_port}")
    print(f"连接到结果接收端口: {args.receiver_port}")

    print("正在加载YOLO模型...")
    model = load_yolo(args.model, args.device)
    print(f"YOLO模型 {args.model} 已在 {args.device} 上加载完成")
    print("等待从ROS节点接收图像...")

    # 处理图像
    try:
        while True:
            # 从ROS节点接收图像
            frames = receiver.recv_multipart()
            
            if len(frames) >= 3 and frames[0] == b"image":
                img_data = frames[1]
                frame_id = frames[2].decode('utf-8')
                
                # 解码图像
                img_array = np.frombuffer(img_data, dtype=np.uint8)
                img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
                
                # 运行YOLO推理
                results = model(img, conf=args.conf, iou=args.iou, verbose=False)
                
                # 将检测结果转换为可序列化的格式
                detections_list = []
                for r in results:
                    boxes = r.boxes
                    for box in boxes:
                        x1, y1, x2, y2 = box.xyxy[0].tolist()
                        conf = float(box.conf[0])
                        cls = int(box.cls[0])
                        cls_name = model.names[cls]
                        
                        detections_list.append({
                            "xmin": x1,
                            "ymin": y1, 
                            "xmax": x2,
                            "ymax": y2,
                            "confidence": conf,
                            "class": cls,
                            "name": cls_name
                        })
                
                detections = json.dumps(detections_list)
                
                # 获取标注图像
                annotated_img = results[0].plot()
                
                # 向ROS节点发送结果
                sender.send_multipart([b"result", bytes(detections, 'utf-8'), bytes(frame_id, 'utf-8')])
                
                # 编码并发送标注图像
                _, img_encoded = cv2.imencode('.jpg', annotated_img)
                sender.send(img_encoded.tobytes())
                
                print(f"处理了图像，检测到 {len(detections_list)} 个目标")
            
            time.sleep(0.001)  # 小的延迟以避免占用过多CPU
            
    except KeyboardInterrupt:
        print("停止YOLO推理脚本")
    finally:
        receiver.close()
        sender.close()
        context.term()

if __name__ == "__main__":
    main()
