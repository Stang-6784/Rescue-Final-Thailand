# -*- coding: utf-8 -*-
# สคริปต์เทรน YOLO11 จาก Roboflow dataset บนเครื่อง Windows (CPU)
#
# วิธีใช้:
#   1) pip install ultralytics
#   2) วาง dataset ที่ export จาก Roboflow ไว้ที่ AI_Trainng/dataset/
#      (ต้องมี data.yaml + โฟลเดอร์ train/ valid/ ภายใน)
#   3) cd AI_Trainng
#      python train.py
#
# ผลลัพธ์: runs/rescue_train/weights/best.pt

from pathlib import Path

from ultralytics import YOLO

# ===== config =====
HERE       = Path(__file__).resolve().parent          # โฟลเดอร์ AI_Trainng
DATA_YAML  = HERE / "dataset" / "data.yaml"           # path ไปยัง data.yaml ของ Roboflow
BASE_MODEL = "yolo11n.pt"                             # base model เล็กสุด (เหมาะกับ CPU)
EPOCHS     = 50
IMGSZ      = 640
BATCH      = 4                                        # CPU ใช้ batch เล็ก
DEVICE     = "cpu"
WORKERS    = 2


def main():
    if not DATA_YAML.exists():
        raise FileNotFoundError(
            f"ไม่พบ data.yaml ที่ {DATA_YAML}\n"
            "กรุณาวาง dataset ที่ export จาก Roboflow ไว้ที่ AI_Trainng/dataset/ ก่อน"
        )

    print(f"[train] base model : {BASE_MODEL}")
    print(f"[train] data.yaml  : {DATA_YAML}")
    print(f"[train] device     : {DEVICE} | epochs={EPOCHS} imgsz={IMGSZ} batch={BATCH}")

    model = YOLO(BASE_MODEL)              # ครั้งแรกจะดาวน์โหลด weight base อัตโนมัติ
    model.train(
        data=str(DATA_YAML),
        epochs=EPOCHS,
        imgsz=IMGSZ,
        batch=BATCH,
        device=DEVICE,
        workers=WORKERS,
        project=str(HERE / "runs"),
        name="rescue_train",
    )

    best = HERE / "runs" / "rescue_train" / "weights" / "best.pt"
    print(f"\n[train] เสร็จสิ้น! โมเดลที่ดีที่สุดอยู่ที่:\n  {best}")
    print("[train] นำไปใช้ inference โดยก๊อปไปทับ AI_Detection/X7.pt "
          "หรือโหลด path นี้ผ่านปุ่มในหน้าเว็บ (/ai/load)")


# สำคัญบน Windows: ต้องครอบด้วย __main__ ป้องกัน multiprocessing error
if __name__ == "__main__":
    main()
