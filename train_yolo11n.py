from pathlib import Path
from ultralytics import YOLO

BASE_DIR = Path(__file__).resolve().parent
DATA_YAML = BASE_DIR / "data.yaml"
RUNS_DIR = BASE_DIR / "runs"

model = YOLO("yolo11n.pt")

model.train(
    data=str(DATA_YAML),
    epochs=100,
    imgsz=640,
    batch=16,
    patience=20,
    seed=0,
    deterministic=True,
    project=str(RUNS_DIR),
    name="yolo11n_comparison",
    save=True,
)

print(f"완료: {RUNS_DIR / 'yolo11n_comparison/weights/best.pt'}")