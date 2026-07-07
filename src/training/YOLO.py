from ultralytics import YOLO

# Load a pretrained YOLO26n model
model = YOLO('yolo26n.pt')

# Train the model on the COCO8 dataset for 100 epochs
train_results = model.train(
    data='config.yaml',  # Path to dataset configuration file
    epochs=50,  # Number of training epochs
    imgsz=640,  # Image size for training
    device='mps',  # Device to run on (e.g., 'cpu', 0, [0,1,2,3])
)

# Evaluate the model's performance on the validation set
metrics = model.val()

# Perform object detection on an image
results = model('Test Image.png')  # Predict on an image
results[0].show()  # Display results

# Export the model to ONNX format for deployment
path = model.export(format='onnx')  # Returns the path to the exported model
