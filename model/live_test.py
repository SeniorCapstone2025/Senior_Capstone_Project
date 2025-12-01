from ultralytics import YOLO
import cv2

# Load a pretrained YOLO model (you can use your trained model instead)
model = YOLO("yolo11n.pt")

# Open the webcam (0 = default camera)
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

# Loop through frames from the webcam
while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to capture frame.")
        break

    # Run YOLO detection on the current frame
    results = model(frame, stream=True)

    # Process and display the results
    for r in results:
        # Draw boxes and labels directly on the frame
        annotated_frame = r.plot()

        # Show the frame
        cv2.imshow("YOLO Live Detection", annotated_frame)

    # Quit if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Clean up
cap.release()
cv2.destroyAllWindows()
