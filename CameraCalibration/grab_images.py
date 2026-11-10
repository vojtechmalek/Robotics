import cv2

cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
image_num = 0
win_name = "stream"

cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
# cv2.resizeWindow(win_name, 1600, 900)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    cv2.imshow(win_name, frame)

    key = cv2.waitKey(1)
    if key == 27: # ESC press
        break
    elif key == 32: # SPACEBAR press
        cv2.imwrite(f"unmarked/{image_num}.png", frame)
        image_num += 1

cap.release()
cv2.destroyAllWindows()