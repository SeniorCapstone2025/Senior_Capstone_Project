from qreader import QReader
import cv2
import requests
import time

capture = cv2.VideoCapture(0)

if not capture.isOpened():
    print("Error: Could not open webcam.")
    exit()
    
qreader = QReader()

while True:
    ret, frame = capture.read()

    if not ret:
        break
    
    
    image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    detected = qreader.detect_and_decode(image=image)
    
    response = None

    if detected:
        if detected[0]:
            print("QR Code detected:", detected[0])
            try:
                response = requests.get(detected[0])
            except requests.exceptions.RequestException as e:
                print("Error during HTTP request:", e)
            break
    else:
        print("No QR Code detected.")
        
    cv2.imshow("QR Code Scanner", frame)
        
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
if response:
    response = response.json()
    print()
    print("Item: " , response.get("Item"))
    print("Location: " , response.get("location"))
    print("On Hand Count: " , response.get("onHandCount"))

capture.release()
cv2.destroyAllWindows()