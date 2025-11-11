# file: map_obstacle_extractor.py
# -*- coding: utf-8 -*-
import os, json, cv2, yaml
import numpy as np
import pytesseract   # OCR 추가

class MapObstacleExtractor:
    """
    PGM + YAML 기반으로 장애물 검출 + 이미지 내 글씨(OCR) 인식하여 name 지정
    """

    def __init__(self, pgm_path: str, yaml_path: str, output_json: str):
        self.pgm_path = pgm_path
        self.yaml_path = yaml_path
        self.output_json = output_json

    def run(self):
        # 1️⃣ 파일 로드
        img = cv2.imread(self.pgm_path, cv2.IMREAD_UNCHANGED)
        if img is None:
            raise FileNotFoundError(f"Cannot read: {self.pgm_path}")

        with open(self.yaml_path, "r") as f:
            meta = yaml.safe_load(f)

        resolution = float(meta.get("resolution", 0.05))
        origin = meta.get("origin", [0.0, 0.0, 0.0])
        h, w = img.shape

        # 2️⃣ 어두운 영역을 장애물로 판단
        mask = (img < 128).astype(np.uint8) * 255
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask)

        # 3️⃣ OCR 준비 (글씨는 어두운 배경에 밝은 글씨라고 가정)
        ocr_img = cv2.bitwise_not(img)  # 색 반전 (OCR이 잘 읽게)
        ocr_img = cv2.cvtColor(ocr_img, cv2.COLOR_GRAY2BGR)

        # pytesseract로 전체 이미지 텍스트 인식
        # boxes=True → 각 글자/단어의 위치도 함께 반환
        data = pytesseract.image_to_data(ocr_img, lang='kor+eng', output_type=pytesseract.Output.DICT)

        text_regions = []
        for i in range(len(data["text"])):
            txt = data["text"][i].strip()
            if txt == "" or len(txt) < 2:
                continue
            x, y, bw, bh = data["left"][i], data["top"][i], data["width"][i], data["height"][i]
            cx, cy = x + bw / 2, y + bh / 2
            text_regions.append({"text": txt, "cx": cx, "cy": cy})

        # 4️⃣ 장애물 영역 추출
        regions = []
        id_counter = 101

        for i in range(1, num_labels):
            area = stats[i, cv2.CC_STAT_AREA]
            if area < 50:
                continue

            cx_px, cy_px = centroids[i]
            w_px, h_px = stats[i, cv2.CC_STAT_WIDTH], stats[i, cv2.CC_STAT_HEIGHT]

            # 픽셀 → 맵 좌표 변환
            mx = origin[0] + cx_px * resolution
            my = origin[1] + (h - cy_px) * resolution
            mw = w_px * resolution
            mh = h_px * resolution

            # 5️⃣ OCR 결과 중 영역 안에 포함된 텍스트 찾기
            name = None
            for t in text_regions:
                if abs(t["cx"] - cx_px) < w_px / 2 and abs(t["cy"] - cy_px) < h_px / 2:
                    name = t["text"]
                    break

            regions.append({
                "id": id_counter,
                "name": name if name else f"obstacle_{id_counter}",
                "type": "obstacle",
                "position": [round(mx, 3), round(my, 3)],
                "size": [round(mw, 3), round(mh, 3)],
                "orientation": 0.0
            })
            id_counter += 1

        # 6️⃣ JSON 저장
        out = {"regions": regions}
        with open(self.output_json, "w", encoding="utf-8") as f:
            json.dump(out, f, indent=2, ensure_ascii=False)

        print(f"✅ {len(regions)} obstacles saved with OCR names → {self.output_json}")
        return out


def main():
    extractor = MapObstacleExtractor(
        pgm_path="/home/changwoo/ros2_ws/src/bcr_bot/config/bcr_map.pgm",
        yaml_path="/home/changwoo/ros2_ws/src/bcr_bot/config/bcr_map.yaml",
        output_json="/home/changwoo/ros2_ws/src/bcr_bot/config/obstacle.json"
    )
    extractor.run()

if __name__ == "__main__":
    main()
