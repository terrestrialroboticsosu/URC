import argparse
import time

import cv2
from ultralytics import YOLO


def pick_single_detection(result, target_class_ids=None, conf_min=0.25):
    """
    Returns index of the ONE detection to highlight: highest confidence among allowed classes.
    If target_class_ids is None, consider all classes.
    """
    boxes = result.boxes
    if boxes is None or len(boxes) == 0:
        return None

    best_i = None
    best_conf = -1.0

    for i in range(len(boxes)):
        cls_id = int(boxes.cls[i].item())
        conf = float(boxes.conf[i].item())

        if conf < conf_min:
            continue
        if target_class_ids is not None and cls_id not in target_class_ids:
            continue

        if conf > best_conf:
            best_conf = conf
            best_i = i

    return best_i


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", default="0",
                        help="0 for webcam, or path to video file (e.g. test.mp4)")
    parser.add_argument("--model", default="yolov8n.pt",
                        help="YOLO model path (e.g. yolov8n.pt or your custom .pt)")
    parser.add_argument("--imgsz", type=int, default=416,
                        help="Inference image size (smaller = faster on Pi)")
    parser.add_argument("--conf", type=float, default=0.25,
                        help="Confidence threshold")
    parser.add_argument("--only-one", action="store_true",
                        help="Highlight only ONE detection (highest confidence)")
    parser.add_argument("--classes", default="",
                        help="Comma-separated class IDs to allow (e.g. '0,39'). Empty = all")
    parser.add_argument("--show-fps", action="store_true",
                        help="Overlay FPS")
    args = parser.parse_args()

    # Parse source
    if args.source.isdigit():
        source = int(args.source)
    else:
        source = args.source

    # Parse allowed class ids
    target_class_ids = None
    if args.classes.strip() != "":
        target_class_ids = set(int(x.strip()) for x in args.classes.split(",") if x.strip() != "")

    # Load model
    model = YOLO(args.model)

    cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open source: {args.source}")

    prev_time = time.time()
    fps = 0.0

    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            break

        # Run inference
        results = model.predict(
            source=frame,
            imgsz=args.imgsz,
            conf=args.conf,
            verbose=False
        )
        r = results[0]

        annotated = frame.copy()

        if r.boxes is not None and len(r.boxes) > 0:
            # If only-one: draw exactly one bbox. Otherwise draw all.
            if args.only_one:
                best_i = pick_single_detection(r, target_class_ids=target_class_ids, conf_min=args.conf)
                if best_i is not None:
                    x1, y1, x2, y2 = map(int, r.boxes.xyxy[best_i].tolist())
                    cls_id = int(r.boxes.cls[best_i].item())
                    conf = float(r.boxes.conf[best_i].item())
                    label = f"{model.names.get(cls_id, cls_id)} {conf:.2f}"

                    cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 3)
                    cv2.putText(annotated, label, (x1, max(20, y1 - 10)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            else:
                # Draw all detections (optionally filtered by class)
                for i in range(len(r.boxes)):
                    cls_id = int(r.boxes.cls[i].item())
                    conf = float(r.boxes.conf[i].item())
                    if conf < args.conf:
                        continue
                    if target_class_ids is not None and cls_id not in target_class_ids:
                        continue

                    x1, y1, x2, y2 = map(int, r.boxes.xyxy[i].tolist())
                    label = f"{model.names.get(cls_id, cls_id)} {conf:.2f}"

                    cv2.rectangle(annotated, (x1, y1), (x2, y2), (255, 0, 0), 2)
                    cv2.putText(annotated, label, (x1, max(20, y1 - 10)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

        # FPS overlay
        if args.show_fps:
            now = time.time()
            dt = now - prev_time
            prev_time = now
            if dt > 0:
                fps = 0.9 * fps + 0.1 * (1.0 / dt) if fps > 0 else (1.0 / dt)
            cv2.putText(annotated, f"FPS: {fps:.1f}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)

        cv2.imshow("YOLOv8 Detection", annotated)
        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord('q')):  # ESC or q
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
