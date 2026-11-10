import glob
import os
import re

import cv2
import numpy as np

# ---------------- Settings ----------------
IMAGE_DIR = "unmarked"
OUTPUT_DIR = "marked"
CALIB_FILE = "calib.npz"

SQUARES_X = 5            # number of squares (not inner corners) in X
SQUARES_Y = 7            # number of squares in Y
SQUARE_LENGTH = 0.040    # [m] measure your printed board!
MARKER_LENGTH = 0.024    # [m]
DICTIONARY = cv2.aruco.DICT_6X6_250

MIN_CORNERS = 6          # minimum Charuco corners per image to use it
# ------------------------------------------

aruco_dict = cv2.aruco.getPredefinedDictionary(DICTIONARY)
board = cv2.aruco.CharucoBoard(
    (SQUARES_X, SQUARES_Y), SQUARE_LENGTH, MARKER_LENGTH, aruco_dict
)
detector = cv2.aruco.CharucoDetector(board)

# 3D positions of all Charuco corners in the board frame, shape (M, 3)
board_corners_3d = np.asarray(board.getChessboardCorners(), dtype=np.float32).reshape(-1, 3)

os.makedirs(OUTPUT_DIR, exist_ok=True)


def natural_key(path):
    """Sort 2.png before 10.png."""
    return [int(t) if t.isdigit() else t for t in re.split(r"(\d+)", path)]


image_paths = sorted(
    glob.glob(os.path.join(IMAGE_DIR, "*.png"))
    + glob.glob(os.path.join(IMAGE_DIR, "*.jpg")),
    key=natural_key,
)
if not image_paths:
    raise SystemExit(f"No images found in '{IMAGE_DIR}'")

all_obj_pts = []
all_img_pts = []
img_size = None

for path in image_paths:
    name = os.path.basename(path)
    img = cv2.imread(path)
    if img is None:
        print(f"{name}: could not be read, skipping.")
        continue

    size = (img.shape[1], img.shape[0])  # (width, height)
    if img_size is None:
        img_size = size
    elif size != img_size:
        print(f"{name}: resolution {size} differs from {img_size}, skipping.")
        continue

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    charuco_corners, charuco_ids, marker_corners, marker_ids = detector.detectBoard(gray)

    # ---------- Visualization (manual drawing, version independent) ----------
    vis = img.copy()
    if marker_ids is not None and len(marker_ids) > 0:
        for mc in marker_corners:
            pts = np.asarray(mc).reshape(-1, 2).astype(np.int32)
            cv2.polylines(vis, [pts], True, (0, 255, 0), 2)

    n_corners = 0
    if charuco_ids is not None and len(charuco_ids) > 0:
        pts = np.asarray(charuco_corners).reshape(-1, 2)
        ids = np.asarray(charuco_ids).reshape(-1)
        n_corners = len(ids)
        for (x, y), cid in zip(pts, ids):
            cv2.circle(vis, (int(x), int(y)), 4, (0, 0, 255), -1)
            cv2.putText(vis, str(cid), (int(x) + 5, int(y) - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)
    cv2.imwrite(os.path.join(OUTPUT_DIR, name), vis)

    # ---------- Collect calibration data ----------
    if n_corners < MIN_CORNERS:
        print(f"{name}: only {n_corners} Charuco corners, skipping.")
        continue

    img_pts = np.asarray(charuco_corners, dtype=np.float32).reshape(-1, 1, 2)
    ids = np.asarray(charuco_ids, dtype=np.int32).reshape(-1)
    obj_pts = board_corners_3d[ids].reshape(-1, 1, 3)

    all_obj_pts.append(obj_pts)
    all_img_pts.append(img_pts)
    print(f"{name}: {n_corners} corners OK")

# ---------- Calibration ----------
if len(all_obj_pts) < 3:
    raise SystemExit(f"Only {len(all_obj_pts)} usable images, need at least ~10-20.")

rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(
    all_obj_pts, all_img_pts, img_size, None, None
)

print(f"\nUsed {len(all_obj_pts)} images, image size {img_size}")
print("RMS reprojection error [px]:", rms)
print("Camera matrix K:\n", K)
print("Distortion coefficients:\n", dist.ravel())

np.savez(CALIB_FILE, K=K, dist=dist, img_size=np.array(img_size))
print(f"Saved to {CALIB_FILE}")

# Load later with:
# data = np.load("calib.npz"); K = data["K"]; dist = data["dist"]