import cv2
from ultralytics import YOLO

def main():
    # 1. Load the model you trained
    # Ensure this file is in the same folder as this script!
    model = YOLO("best_object_detection_URCv2.pt")

    # 2. Open the Webcam (0 is usually the default laptop camera)
    cap = cv2.VideoCapture(0)

    # Check if camera opened successfully
    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return

    print("Press 'q' to quit.")

    while True:
        # Read a frame from the camera
        ret, frame = cap.read()
        if not ret:
            print("Error: Can't receive frame (stream end?). Exiting ...")
            break

        # 3. Run YOLO inference on the frame
        # conf=0.6 filters out weak predictions (based on your F1 chart)
        results = model(frame, conf=0.6) 

        # 4. Visualize the results (draw boxes on the image)
        annotated_frame = results[0].plot()

        # --- ROBOT CONTROL LOGIC SECTION ---
        # If you need coordinates for your rover, access them here:
        for r in results:
            for box in r.boxes:
                # Get coordinates: x_center, y_center, width, height
                x, y, w, h = box.xywh[0] 
                confidence = box.conf[0]
                class_id = int(box.cls[0])
                class_name = model.names[class_id]
                
                # Example: Print coordinates to terminal
                # print(f"Found {class_name} at X:{x:.0f} Y:{y:.0f}")
        # -----------------------------------

        # 5. Display the resulting frame
        cv2.imshow('URC Object Detection', annotated_frame)

        # Press 'q' on keyboard to exit
        if cv2.waitKey(1) == ord('q'):
            break

    # Cleanup
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()