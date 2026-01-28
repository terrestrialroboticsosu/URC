On Laptop do:
pip install ultralytics opencv-python

on Raspberry Pi do:
sudo apt-get update
sudo apt-get install -y python3-opencv
pip install ultralytics

Run code:
python3 yolo_test.py --model runs/urc_objects_yolov8n/weights/best.pt --source 0 --only-one

