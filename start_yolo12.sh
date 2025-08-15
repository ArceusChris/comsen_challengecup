#!/bin/bash
source ~/.bashrc
conda activate yolov12
python workspace/src/perception_yolov12/scripts/yolo12_inference.py --model yolo_models/yolov12n.pt --device cuda --sender-port 5555 --receiver-port 5556 &
python workspace/src/perception_yolov12/scripts/yolo12_inference.py --model yolo_models/yolov12n.pt --device cuda --sender-port 5557 --receiver-port 5558 &
wait