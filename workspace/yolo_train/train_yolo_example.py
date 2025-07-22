#!/usr/bin/env python3
"""
使用转换后的数据集训练YOLO11模型的示例脚本
"""

# 注意：运行此脚本前请确保已安装ultralytics
# pip install ultralytics

from ultralytics import YOLO
import os


def train_yolo_model():
    """训练YOLO11模型"""
    
    print("=" * 50)
    print("YOLO11 模型训练")
    print("=" * 50)
    
    # 检查数据集配置文件
    config_file = "yolo_dataset/dataset.yaml"
    if not os.path.exists(config_file):
        print(f"错误: 找不到数据集配置文件 {config_file}")
        return
    
    print(f"使用数据集配置: {config_file}")
    
    # 加载预训练模型
    print("加载YOLO11预训练模型...")
    model = YOLO('yolo11n.yaml')  # 使用nano版本，速度较快
    
    # 训练模型
    print("开始训练...")
    results = model.train(
        data=config_file,           # 数据集配置文件
        epochs=100,                 # 训练轮数
        imgsz=640,                  # 图像尺寸
        batch=16,                   # 批次大小
        workers=4,                  # 数据加载进程数
        project="runs/detect",      # 项目目录
        name="yolo11_custom",       # 实验名称
        save=True,                  # 保存模型
        save_period=10,             # 每10轮保存一次
        val=True,                   # 启用验证
        plots=True,                 # 生成训练图表
        device='auto',              # 自动选择设备（GPU/CPU）
        verbose=True,               # 详细输出
        
        # 数据增强参数
        hsv_h=0.015,               # 色调增强
        hsv_s=0.7,                 # 饱和度增强
        hsv_v=0.4,                 # 明度增强
        degrees=0.0,               # 旋转角度
        translate=0.1,             # 平移比例
        scale=0.5,                 # 缩放比例
        shear=0.0,                 # 剪切变换
        perspective=0.0,           # 透视变换
        flipud=0.0,                # 上下翻转概率
        fliplr=0.5,                # 左右翻转概率
        mosaic=1.0,                # 马赛克增强概率
        mixup=0.0,                 # mixup增强概率
        
        # 优化器参数
        optimizer='SGD',           # 优化器类型
        lr0=0.01,                  # 初始学习率
        lrf=0.01,                  # 最终学习率比例
        momentum=0.937,            # 动量
        weight_decay=0.0005,       # 权重衰减
        warmup_epochs=3.0,         # 预热轮数
        warmup_momentum=0.8,       # 预热动量
        warmup_bias_lr=0.1,        # 预热偏置学习率
    )
    
    print("\n训练完成!")
    print(f"最佳模型保存在: {results.save_dir}/weights/best.pt")
    print(f"最后模型保存在: {results.save_dir}/weights/last.pt")
    
    # 验证模型
    print("\n验证模型性能...")
    metrics = model.val()
    print(f"mAP50: {metrics.box.map50:.3f}")
    print(f"mAP50-95: {metrics.box.map:.3f}")
    
    return model, results


def test_model_inference(model_path="runs/detect/yolo11_custom/weights/best.pt"):
    """测试模型推理"""
    
    print("\n" + "=" * 50)
    print("模型推理测试")
    print("=" * 50)
    
    if not os.path.exists(model_path):
        print(f"模型文件不存在: {model_path}")
        return
    
    # 加载训练好的模型
    model = YOLO(model_path)
    
    # 使用验证集中的图像进行测试
    test_images = "yolo_dataset/images/val"
    if os.path.exists(test_images):
        results = model.predict(
            source=test_images,
            save=True,
            project="runs/detect", 
            name="predict",
            conf=0.25,              # 置信度阈值
            iou=0.7,                # NMS IoU阈值
            show_labels=True,       # 显示标签
            show_conf=True,         # 显示置信度
            save_txt=True,          # 保存预测结果为txt
            save_crop=True,         # 保存裁剪的检测结果
        )
        
        print(f"推理结果保存在: runs/detect/predict/")
    else:
        print(f"测试图像目录不存在: {test_images}")


if __name__ == "__main__":
    try:
        # 检查是否安装了ultralytics
        import ultralytics
        print(f"使用 ultralytics 版本: {ultralytics.__version__}")
        
        # 训练模型
        model, results = train_yolo_model()
        
        # 测试推理
        test_model_inference()
        
    except ImportError:
        print("错误: 未安装 ultralytics 库")
        print("请运行: pip install ultralytics")
    except Exception as e:
        print(f"训练过程中发生错误: {e}")
        print("请检查数据集和配置文件是否正确")
