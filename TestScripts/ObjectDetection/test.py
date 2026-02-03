from ultralytics import YOLO
import cv2
import urllib.request

# 1. Load your trained model
# Make sure this path is exactly correct on your laptop
model = YOLO("best_object_detection_URC.pt")

# 2. Download a test image
url = "https://encrypted-tbn3.gstatic.com/shopping?q=tbn:ANd9GcTxc3UcJx5YyfNvfhjRAlmVFe_7dEXNp6nzKH4Cox6SKPn9bP3U-MQNcv32RXdVC518D94-Gsn0sr4kdfSs_qlVzUvJLVkhDUBfv1r7Fys"
filename = "test_image.jpg"
urllib.request.urlretrieve(url, filename)

# 3. Run prediction
results = model(filename)

# 4. Show results
res_plotted = results[0].plot()

# CHANGE HERE: Use standard cv2.imshow instead of Colab's version
cv2.imshow("Detection Result", res_plotted)

# REQUIRED FOR LOCAL: Wait for a key press to close the window
# '0' means wait indefinitely until a key is pressed
cv2.waitKey(0)  
cv2.destroyAllWindows()