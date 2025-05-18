import cv2

def find_available_cameras(max_index=10):
    available = []
    for i in range(max_index):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            ret, frame = cap.read()
            if ret:
                print(f"[INFO] Camera found at index {i}")
                available.append(i)
            cap.release()
    return available

def test_camera(index):
    cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        print(f"[ERROR] Cannot open camera at index {index}")
        return

    print(f"[INFO] Testing camera at index {index}. Press 'q' to quit.")
    while True:
        ret, frame = cap.read()
        if not ret:
            print("[ERROR] Failed to grab frame")
            break
        cv2.imshow(f"Camera {index}", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    print("Searching for available cameras...")
    cams = find_available_cameras(10)

    if not cams:
        print("No cameras found.")
    else:
        for cam_index in cams:
            test_camera(cam_index)
