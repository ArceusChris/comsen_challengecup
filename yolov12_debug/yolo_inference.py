#!/usr/bin/env python3
import cv2
import numpy as np
import zmq
import json
import time
import argparse
from ultralytics import YOLO

# Function to load YOLO model using ultralytics
def load_yolo(model_path="yolov12n.pt", device="cuda"):
    """Load YOLO model using ultralytics library"""
    try:
        model = YOLO(model_path)
        model.to(device)
        return model
    except Exception as e:
        print(f"Error loading YOLO model: {e}")
        raise

# Parse arguments
parser = argparse.ArgumentParser(description='YOLO inference script that communicates with ROS')
parser.add_argument('--model', type=str, default='yolov12n.pt', help='YOLO model path')
parser.add_argument('--device', type=str, default='cuda', help='Device to run inference on (cpu/cuda)')
parser.add_argument('--conf', type=float, default=0.45, help='Confidence threshold')
parser.add_argument('--iou', type=float, default=0.45, help='IOU threshold')
args = parser.parse_args()

# Set up ZeroMQ
context = zmq.Context()

# Socket to receive images from ROS node
receiver = context.socket(zmq.PULL)
receiver.connect("tcp://127.0.0.1:5555")

# Socket to send results back to ROS node
sender = context.socket(zmq.PUSH)
sender.connect("tcp://127.0.0.1:5556")

print("Loading YOLO model...")
model = load_yolo(args.model, args.device)
print(f"YOLO model {args.model} loaded on {args.device}")
print("Waiting for images from ROS node...")

# Process images
try:
    while True:
        # Receive image from ROS node
        frames = receiver.recv_multipart()
        
        if len(frames) >= 3 and frames[0] == b"image":
            img_data = frames[1]
            frame_id = frames[2].decode('utf-8')
            
            # Decode image
            img_array = np.frombuffer(img_data, dtype=np.uint8)
            img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
            
            # Run YOLO inference
            results = model(img, conf=args.conf, iou=args.iou, verbose=False)
            
            # Convert detections to a serializable format
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
            
            # Get annotated image
            annotated_img = results[0].plot()
            
            # Send results back to ROS node
            sender.send_multipart([b"result", bytes(detections, 'utf-8'), bytes(frame_id, 'utf-8')])
            
            # Encode and send the annotated image
            _, img_encoded = cv2.imencode('.jpg', annotated_img)
            sender.send(img_encoded.tobytes())
            
            print(f"Processed image, found {len(detections_list)} objects")
        
        time.sleep(0.001)  # Small sleep to prevent CPU hogging
        
except KeyboardInterrupt:
    print("Stopping YOLO inference script")
    receiver.close()
    sender.close()
    context.term()