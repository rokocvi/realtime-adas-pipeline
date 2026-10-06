"""Export YOLOv8n (PyTorch) -> ONNX i provjera modela za C++ ONNX Runtime."""
from pathlib import Path
import shutil

import onnxruntime as ort
from ultralytics import YOLO

ROOT = Path(__file__).resolve().parent.parent
MODELS_DIR = ROOT / "models"
MODELS_DIR.mkdir(exist_ok=True)

# 1) Učitaj pretrenirani model (sam se skine pri prvom pokretanju)
model = YOLO("yolov8n.pt")

# 2) Export: fiksni ulaz 640x640, batch 1 -> predvidljiva latencija (bitno za embedded)
onnx_path = Path(model.export(format="onnx", imgsz=640, dynamic=False, simplify=True))

# 3) Premjesti u models/
target = MODELS_DIR / "yolov8n.onnx"
shutil.move(str(onnx_path), target)
print(f"\nModel spremljen: {target}")

# 4) Provjera: učitaj ga u ONNX Runtime i ispiši ulaze/izlaze
session = ort.InferenceSession(str(target), providers=["CPUExecutionProvider"])
for i in session.get_inputs():
    print(f"INPUT  name={i.name}  shape={i.shape}  type={i.type}")
for o in session.get_outputs():
    print(f"OUTPUT name={o.name}  shape={o.shape}  type={o.type}")