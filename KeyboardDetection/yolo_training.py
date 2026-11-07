from ultralytics import YOLO

def main():
    model = YOLO("yolo26n.pt")

    results = model.train(
        data=r"dataset/data.yaml",
        epochs=50,
        imgsz=640,  # Keeps resolution standard
        batch=16,  # Increase batch size to keep GPU saturated (try 16 or 32)
        device=0,  # Force RTX 5070
        workers=2,  # Try 2 workers now that it's inside main()
        cache=True  # Caches images in RAM to bypass slow drive disk read times
    )

if __name__ == "__main__":
    main()