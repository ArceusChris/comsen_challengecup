#!/usr/bin/env python3
"""
将labelme格式的数据集转换为YOLO11格式
"""

import os
import json
import shutil
from datetime import datetime
from pathlib import Path
import argparse


class LabelmeToYOLO:
    def __init__(self, dataset_path, output_path=None, backup=True):
        self.dataset_path = Path(dataset_path)
        self.output_path = Path(output_path) if output_path else self.dataset_path.parent / "yolo_dataset"
        self.backup = backup
        
        # 创建类别映射
        self.classes = []
        self.class_mapping = {}
        
    def backup_original_dataset(self):
        """备份原始数据集"""
        if not self.backup:
            return
            
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        backup_path = self.dataset_path.parent / f"dataset_backup_{timestamp}"
        
        print(f"正在备份原始数据集到: {backup_path}")
        shutil.copytree(self.dataset_path, backup_path)
        print(f"备份完成!")
        
    def scan_classes(self):
        """扫描所有标注文件获取类别信息"""
        label_dir = self.dataset_path / "label"
        classes_set = set()
        
        if not label_dir.exists():
            raise FileNotFoundError(f"标注目录不存在: {label_dir}")
            
        print("正在扫描类别信息...")
        for json_file in label_dir.glob("*.json"):
            try:
                with open(json_file, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                    
                for shape in data.get("shapes", []):
                    label = shape.get("label", "")
                    if label:
                        classes_set.add(label)
            except Exception as e:
                print(f"警告: 无法处理文件 {json_file}: {e}")
                
        self.classes = sorted(list(classes_set))
        self.class_mapping = {cls: idx for idx, cls in enumerate(self.classes)}
        
        print(f"发现 {len(self.classes)} 个类别: {self.classes}")
        
    def create_output_structure(self):
        """创建YOLO数据集目录结构"""
        # 创建主目录
        self.output_path.mkdir(parents=True, exist_ok=True)
        
        # 创建子目录
        (self.output_path / "images" / "train").mkdir(parents=True, exist_ok=True)
        (self.output_path / "labels" / "train").mkdir(parents=True, exist_ok=True)
        
        print(f"YOLO数据集目录结构已创建: {self.output_path}")
        
    def convert_bbox_to_yolo(self, points, img_width, img_height):
        """将labelme的矩形坐标转换为YOLO格式"""
        # labelme的points格式: [[x1, y1], [x2, y2]]
        x1, y1 = points[0]
        x2, y2 = points[1]
        
        # 确保坐标顺序正确
        x_min = min(x1, x2)
        x_max = max(x1, x2)
        y_min = min(y1, y2)
        y_max = max(y1, y2)
        
        # 计算中心点坐标和宽高（归一化）
        center_x = (x_min + x_max) / 2.0 / img_width
        center_y = (y_min + y_max) / 2.0 / img_height
        width = (x_max - x_min) / img_width
        height = (y_max - y_min) / img_height
        
        return center_x, center_y, width, height
        
    def convert_annotations(self):
        """转换标注文件"""
        label_dir = self.dataset_path / "label"
        image_dir = self.dataset_path / "image"
        
        output_images_dir = self.output_path / "images" / "train"
        output_labels_dir = self.output_path / "labels" / "train"
        
        converted_count = 0
        error_count = 0
        
        print("正在转换标注文件...")
        
        for json_file in label_dir.glob("*.json"):
            try:
                # 读取labelme标注文件
                with open(json_file, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                
                # 获取图像信息
                img_width = data.get("imageWidth", 0)
                img_height = data.get("imageHeight", 0)
                
                if img_width == 0 or img_height == 0:
                    print(f"警告: {json_file} 中缺少图像尺寸信息，跳过")
                    error_count += 1
                    continue
                
                # 查找对应的图像文件
                image_path = data.get("imagePath", "")
                if image_path.startswith("../image/"):
                    image_name = image_path.replace("../image/", "")
                else:
                    image_name = json_file.stem + ".jpg"  # 假设是jpg格式
                
                source_image_path = image_dir / image_name
                if not source_image_path.exists():
                    # 尝试其他常见的图像格式
                    for ext in ['.jpg', '.jpeg', '.png', '.bmp']:
                        alt_path = image_dir / (json_file.stem + ext)
                        if alt_path.exists():
                            source_image_path = alt_path
                            image_name = alt_path.name
                            break
                    else:
                        print(f"警告: 找不到对应的图像文件 {image_name}，跳过")
                        error_count += 1
                        continue
                
                # 复制图像文件
                target_image_path = output_images_dir / image_name
                shutil.copy2(source_image_path, target_image_path)
                
                # 转换标注
                yolo_annotations = []
                for shape in data.get("shapes", []):
                    if shape.get("shape_type") != "rectangle":
                        continue  # 只处理矩形标注
                        
                    label = shape.get("label", "")
                    if label not in self.class_mapping:
                        print(f"警告: 未知类别 '{label}'，跳过")
                        continue
                        
                    class_id = self.class_mapping[label]
                    points = shape.get("points", [])
                    
                    if len(points) != 2:
                        print(f"警告: 矩形标注点数不正确，跳过")
                        continue
                    
                    # 转换为YOLO格式
                    center_x, center_y, width, height = self.convert_bbox_to_yolo(
                        points, img_width, img_height
                    )
                    
                    yolo_annotations.append(f"{class_id} {center_x:.6f} {center_y:.6f} {width:.6f} {height:.6f}")
                
                # 保存YOLO标注文件
                yolo_label_path = output_labels_dir / (json_file.stem + ".txt")
                with open(yolo_label_path, 'w', encoding='utf-8') as f:
                    f.write('\n'.join(yolo_annotations))
                
                converted_count += 1
                
            except Exception as e:
                print(f"错误: 处理文件 {json_file} 时出错: {e}")
                error_count += 1
                
        print(f"转换完成! 成功转换: {converted_count} 个文件，错误: {error_count} 个文件")
        
    def create_dataset_yaml(self):
        """创建YOLO数据集配置文件"""
        yaml_content = f"""# YOLO数据集配置文件
# 由labelme格式转换而来

# 数据集路径
path: {self.output_path.absolute()}  # 数据集根目录
train: images/train  # 训练图像路径（相对于path）
val: images/train    # 验证图像路径（当前与训练集相同，请根据需要分割）

# 类别数量
nc: {len(self.classes)}

# 类别名称
names:
"""
        
        for idx, class_name in enumerate(self.classes):
            yaml_content += f"  {idx}: {class_name}\n"
            
        yaml_path = self.output_path / "dataset.yaml"
        with open(yaml_path, 'w', encoding='utf-8') as f:
            f.write(yaml_content)
            
        print(f"数据集配置文件已创建: {yaml_path}")
        
    def create_classes_txt(self):
        """创建classes.txt文件"""
        classes_path = self.output_path / "classes.txt"
        with open(classes_path, 'w', encoding='utf-8') as f:
            for class_name in self.classes:
                f.write(f"{class_name}\n")
                
        print(f"类别文件已创建: {classes_path}")
        
    def convert(self):
        """执行完整的转换流程"""
        print("=" * 50)
        print("Labelme to YOLO 数据集转换工具")
        print("=" * 50)
        
        # 1. 备份原始数据集
        if self.backup:
            self.backup_original_dataset()
            
        # 2. 扫描类别
        self.scan_classes()
        
        # 3. 创建输出目录结构
        self.create_output_structure()
        
        # 4. 转换标注文件
        self.convert_annotations()
        
        # 5. 创建配置文件
        self.create_dataset_yaml()
        self.create_classes_txt()
        
        print("=" * 50)
        print("转换完成!")
        print(f"YOLO数据集保存在: {self.output_path}")
        print("注意: 当前所有图像都放在train目录中，请根据需要手动分割训练集和验证集")
        print("=" * 50)


def main():
    parser = argparse.ArgumentParser(description="将labelme格式数据集转换为YOLO11格式")
    parser.add_argument("--dataset", "-d", type=str, default="dataset", 
                       help="labelme数据集路径 (默认: dataset)")
    parser.add_argument("--output", "-o", type=str, default=None,
                       help="输出YOLO数据集路径 (默认: yolo_dataset)")
    parser.add_argument("--no-backup", action="store_true",
                       help="不备份原始数据集")
    
    args = parser.parse_args()
    
    converter = LabelmeToYOLO(
        dataset_path=args.dataset,
        output_path=args.output,
        backup=not args.no_backup
    )
    
    converter.convert()


if __name__ == "__main__":
    main()
