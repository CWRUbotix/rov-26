from ultralytics import YOLO

model = YOLO('runs/detect/train4/weights/best.pt')

results = model('dataset/images/test/frame_470.jpg')
results[0].show()
