from ultralytics import YOLO

def main():
    # Use YOLOv11 nano (smallest, best for Pi inference)
    model = YOLO("yolov11n.pt")

    model.train(
        data="urc_objects/data.yaml",
        epochs=60,
        imgsz=416,
        batch=16,          # lower if you hit memory limits
        workers=4,         # adjust for your CPU
        device=0,          # GPU if available; set to "cpu" if no GPU
        pretrained=True,
        patience=15,       # early stop if not improving
        optimizer="AdamW",
        lr0=0.002,
        weight_decay=0.0005,
        augment=True,
        close_mosaic=10,   # stabilizes late training
        project="runs",
        name="urc_objects_yolov11n",
    )

    # Evaluate
    model.val(data="urc_objects/data.yaml", imgsz=416)

    # Export for fast Pi inference
    best = "runs/urc_objects_yolov11n/weights/best.pt"

    m2 = YOLO(best)
    m2.export(format="onnx", imgsz=416, opset=12)   # good path for Pi (OpenCV DNN / ONNX Runtime)

if __name__ == "__main__":
    main()
