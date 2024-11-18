import cv2
import torch
import pyrealsense2 as rs
from yolov5_linux.utils.general import non_max_suppression, scale_boxes
from yolov5_linux.utils.plots import Annotator
from yolov5_linux.models.common import DetectMultiBackend
import pathlib
import platform

if platform.system() == 'Windows':
    pathlib.PosixPath = pathlib.WindowsPath
else:
    pathlib.WindowsPath = pathlib.PosixPath

class YOLO:
    def __init__(self):
        self.weights = "runs/train/exp4/weights/best.pt"
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model = DetectMultiBackend(self.weights, device=self.device)
        self.stride, self.names = self.model.stride, self.model.names
        self.imgsz = (640, 640)
        
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(self.config)
        
    def detect(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            return None
        img0 = cv2.resize(np.asanyarray(color_frame.get_data()), self.imgsz)
        img = torch.from_numpy(img0).to(self.device).float() / 255.0
        img = img.permute(2, 0, 1).unsqueeze(0)
        
        pred = self.model(img)
        pred = non_max_suppression(pred, conf_thres=0.25, iou_thres=0.45)
        
        annotator = Annotator(img0)
        if pred[0] is not None and len(pred[0]):
            best_det = max(pred[0], key=lambda x: x[4])
            xyxy, conf, cls = best_det[:4], best_det[4], int(best_det[5])
            
            resize_ratio_w = img0.shape[1] / self.imgsz[0]
            resize_ratio_h = img0.shape[0] / self.imgsz[1]
            xyxy[0::2] *= resize_ratio_w
            xyxy[1::2] *= resize_ratio_h

            label = f"{self.names[cls]} {conf:.2f}"
            annotator.box_label(xyxy, label)

            center_x = int((xyxy[0] + xyxy[2]) / 2)
            center_y = int((xyxy[1] + xyxy[3]) / 2)
            # print(f"Best Detection - Center: ({center_x}, {center_y}), Label: {label}")
            return center_x

        else:
            return None

    def __del__(self):
        self.pipeline.stop()