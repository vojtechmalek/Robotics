import cv2
from ultralytics import YOLO


def main():
    model_path = r"../runs/detect/train-3/weights/best.pt"
    model = YOLO(model_path)

    source_path = r"real_keyboard_images/keyboard_1.jpg"  # Replace with your test image path

    # Run prediction
    results = model.predict(
        source=source_path,
        conf=0.25,
        device=0
    )

    # Display the result using OpenCV and keep it open
    for r in results:
        # Get annotated image with drawn bounding boxes (returns a numpy array)
        annotated_frame = r.plot()

        # Show the image in a persistent window
        cv2.imshow("YOLO Key Recognition", annotated_frame)

        # Wait until any key is pressed before closing
        print("Press any key in the image window to close it...")
        cv2.waitKey(0)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()