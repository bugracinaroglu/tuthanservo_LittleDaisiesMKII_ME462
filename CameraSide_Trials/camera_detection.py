import cv2

camera_index = 2  # 0, 1, 2 diye dene

cap = cv2.VideoCapture(camera_index)

if not cap.isOpened():
    print(f"Camera index {camera_index} açılamadı.")
    exit()

while True:
    ret, frame = cap.read()

    if not ret:
        print("Frame alınamadı.")
        break

    cv2.imshow("Camera Test", frame)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()