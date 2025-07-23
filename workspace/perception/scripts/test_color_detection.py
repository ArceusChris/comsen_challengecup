#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
颜色检测测试脚本
用于测试和调试HSV颜色阈值参数
"""

import cv2
import numpy as np

def test_color_detection():
    """测试颜色检测功能"""
    
    # HSV颜色范围（与主程序保持一致）
    color_ranges = {
        'red': {
            'lower1': np.array([0, 50, 50]),
            'upper1': np.array([10, 255, 255]),
            'lower2': np.array([170, 50, 50]),
            'upper2': np.array([180, 255, 255])
        },
        'yellow': {
            'lower': np.array([20, 50, 50]),
            'upper': np.array([30, 255, 255])
        },
        'white': {
            'lower': np.array([0, 0, 200]),
            'upper': np.array([180, 30, 255])
        }
    }
    
    def create_color_mask(hsv, color_range):
        """创建颜色掩码"""
        if 'lower1' in color_range and 'upper1' in color_range:
            # 红色有两个范围
            mask1 = cv2.inRange(hsv, color_range['lower1'], color_range['upper1'])
            mask2 = cv2.inRange(hsv, color_range['lower2'], color_range['upper2'])
            mask = cv2.bitwise_or(mask1, mask2)
        else:
            # 其他颜色只有一个范围
            mask = cv2.inRange(hsv, color_range['lower'], color_range['upper'])
        return mask
    
    # 创建测试图像或使用摄像头
    use_camera = input("使用摄像头测试? (y/n): ").lower() == 'y'
    
    if use_camera:
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("无法打开摄像头")
            return
    else:
        # 创建测试图像
        test_image = np.zeros((480, 640, 3), dtype=np.uint8)
        # 添加红色圆形
        cv2.circle(test_image, (150, 150), 50, (0, 0, 255), -1)
        # 添加黄色矩形
        cv2.rectangle(test_image, (300, 100), (400, 200), (0, 255, 255), -1)
        # 添加白色三角形
        pts = np.array([[500, 100], [450, 200], [550, 200]], np.int32)
        cv2.fillPoly(test_image, [pts], (255, 255, 255))
    
    print("颜色检测测试")
    print("键盘控制:")
    print("  'q' - 退出")
    print("  'r' - 显示红色掩码")
    print("  'y' - 显示黄色掩码")
    print("  'w' - 显示白色掩码")
    print("  'a' - 显示所有掩码")
    print("  's' - 显示原图")
    
    current_mode = 'original'
    
    while True:
        if use_camera:
            ret, frame = cap.read()
            if not ret:
                break
        else:
            frame = test_image.copy()
        
        # 转换到HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # 根据当前模式显示不同内容
        if current_mode == 'red':
            mask = create_color_mask(hsv, color_ranges['red'])
            result = cv2.bitwise_and(frame, frame, mask=mask)
            cv2.putText(result, "Red Detection", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        elif current_mode == 'yellow':
            mask = create_color_mask(hsv, color_ranges['yellow'])
            result = cv2.bitwise_and(frame, frame, mask=mask)
            cv2.putText(result, "Yellow Detection", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
        elif current_mode == 'white':
            mask = create_color_mask(hsv, color_ranges['white'])
            result = cv2.bitwise_and(frame, frame, mask=mask)
            cv2.putText(result, "White Detection", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        elif current_mode == 'all':
            # 显示所有颜色的检测结果
            red_mask = create_color_mask(hsv, color_ranges['red'])
            yellow_mask = create_color_mask(hsv, color_ranges['yellow'])
            white_mask = create_color_mask(hsv, color_ranges['white'])
            
            # 创建彩色掩码
            result = np.zeros_like(frame)
            result[red_mask > 0] = [0, 0, 255]      # 红色区域
            result[yellow_mask > 0] = [0, 255, 255] # 黄色区域
            result[white_mask > 0] = [255, 255, 255] # 白色区域
            
            cv2.putText(result, "All Colors", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        else:
            result = frame.copy()
            cv2.putText(result, "Original Image", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        cv2.imshow('Color Detection Test', result)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('r'):
            current_mode = 'red'
        elif key == ord('y'):
            current_mode = 'yellow'
        elif key == ord('w'):
            current_mode = 'white'
        elif key == ord('a'):
            current_mode = 'all'
        elif key == ord('s'):
            current_mode = 'original'
    
    if use_camera:
        cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    test_color_detection()
