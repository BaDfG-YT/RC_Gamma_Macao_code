from flask import Flask, Response, request
import cv2
import numpy as np
from picamera2 import Picamera2

app = Flask(__name__)

# Максимальное разрешение сенсора IMX708 (Camera Module 3): 4608x2592.
# Съёмка на полном разрешении - лучший вариант для подбора HSV, видно
# мельчайшие детали границ цвета. Если Pi не тянет плавный стрим на таком
# размере - опустись до половинного бина 2304x1296 (тоже нативный режим
# сенсора, читается быстрее) или временно до .
FRAME_W, FRAME_H = 2592, 1944

picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"size": (FRAME_W, FRAME_H)}))
picam2.start()

# значения по умолчанию
H_MIN, S_MIN, V_MIN = 3, 155, 150
H_MAX, S_MAX, V_MAX = 16, 210, 255

MIN_AREA_DEFAULT = 20
MIN_AREA_MAX = 5000  # верхняя граница ползунка

# Качество JPEG для стрима (0-100). 80 -> 92: заметно меньше артефактов
# сжатия, тоже полезно при подборе HSV по краям объектов.
JPEG_QUALITY = 92

# --- Круговой ROI: цвет ищем только внутри этого круга ---
# по умолчанию - центр кадра, радиус - четверть меньшей стороны
ROI_CX_DEFAULT = FRAME_W // 2
ROI_CY_DEFAULT = FRAME_H // 2
ROI_R_DEFAULT = min(FRAME_W, FRAME_H) // 4
ROI_R_MAX = min(FRAME_W, FRAME_H) // 2  # радиус не больше половины меньшей стороны

PAGE = """
<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <title>HSV tuner</title>
  <style>
    body { font-family: sans-serif; margin: 16px; }
    .row { display:flex; gap:16px; flex-wrap:wrap; }
    .panel { min-width: 320px; }
    label { display:block; margin: 8px 0 2px; }
    input[type=range]{ width: 320px; }
    code{ background:#f3f3f3; padding:2px 6px; border-radius:6px; }
    hr { border:none; border-top:1px solid #ddd; margin:12px 0; }
  </style>
</head>
<body>
  <h3>HSV tuner</h3>
  <div class="row">
    <div class="panel">
      <img id="img" src="/video" width="640">
      <div style="margin-top:8px">
        <button onclick="setMode('overlay')">overlay</button>
        <button onclick="setMode('mask')">mask</button>
      </div>
      <div style="margin-top:8px">
        <div>LOWER: <code id="lowerv"></code></div>
        <div>UPPER: <code id="upperv"></code></div>
        <div>MIN AREA: <code id="minareav"></code></div>
        <div>ROI: <code id="roiv"></code></div>
      </div>
    </div>

    <div class="panel">
      <label>H min: <span id="hminv"></span></label>
      <input id="hmin" type="range" min="0" max="179" value="__H_MIN__">
      <label>H max: <span id="hmaxv"></span></label>
      <input id="hmax" type="range" min="0" max="179" value="__H_MAX__">

      <label>S min: <span id="sminv"></span></label>
      <input id="smin" type="range" min="0" max="255" value="__S_MIN__">
      <label>S max: <span id="smaxv"></span></label>
      <input id="smax" type="range" min="0" max="255" value="__S_MAX__">

      <label>V min: <span id="vminv"></span></label>
      <input id="vmin" type="range" min="0" max="255" value="__V_MIN__">
      <label>V max: <span id="vmaxv"></span></label>
      <input id="vmax" type="range" min="0" max="255" value="__V_MAX__">

      <label>Min area (px): <span id="minareav"></span></label>
      <input id="minarea" type="range" min="0" max="__MIN_AREA_MAX__" value="__MIN_AREA_DEFAULT__">

      <hr>
      <b>ROI (круг поиска)</b>

      <label>Center X: <span id="roicxv"></span></label>
      <input id="roicx" type="range" min="0" max="__FRAME_W__" value="__ROI_CX_DEFAULT__">
      <label>Center Y: <span id="roicyv"></span></label>
      <input id="roicy" type="range" min="0" max="__FRAME_H__" value="__ROI_CY_DEFAULT__">
      <label>Radius: <span id="roirv"></span></label>
      <input id="roir" type="range" min="1" max="__ROI_R_MAX__" value="__ROI_R_DEFAULT__">
    </div>
  </div>

<script>
let mode = "overlay";
function val(id){ return document.getElementById(id).value; }
function updLabels(){
  ["hmin","hmax","smin","smax","vmin","vmax","minarea","roicx","roicy","roir"].forEach(id=>{
    const el = document.getElementById(id+"v");
    if (el) el.textContent = val(id);
  });
  document.getElementById("lowerv").textContent = `[${val("hmin")}, ${val("smin")}, ${val("vmin")}]`;
  document.getElementById("upperv").textContent = `[${val("hmax")}, ${val("smax")}, ${val("vmax")}]`;
  document.getElementById("roiv").textContent = `cx=${val("roicx")} cy=${val("roicy")} r=${val("roir")}`;
}
function refresh(){
  updLabels();
  const url = `/video?hmin=${val("hmin")}&hmax=${val("hmax")}&smin=${val("smin")}&smax=${val("smax")}`
            + `&vmin=${val("vmin")}&vmax=${val("vmax")}&minarea=${val("minarea")}`
            + `&roicx=${val("roicx")}&roicy=${val("roicy")}&roir=${val("roir")}`
            + `&mode=${mode}&_=${Date.now()}`;
  document.getElementById("img").src = url;
}
function setMode(m){ mode=m; refresh(); }

["hmin","hmax","smin","smax","vmin","vmax","minarea","roicx","roicy","roir"].forEach(id=>{
  document.getElementById(id).addEventListener("input", refresh);
});
updLabels();
refresh();
setInterval(refresh, 200); // обновление 5 fps для тюнинга
</script>
</body>
</html>
""".replace("__H_MIN__", str(H_MIN)).replace("__H_MAX__", str(H_MAX)) \
   .replace("__S_MIN__", str(S_MIN)).replace("__S_MAX__", str(S_MAX)) \
   .replace("__V_MIN__", str(V_MIN)).replace("__V_MAX__", str(V_MAX)) \
   .replace("__MIN_AREA_MAX__", str(MIN_AREA_MAX)).replace("__MIN_AREA_DEFAULT__", str(MIN_AREA_DEFAULT)) \
   .replace("__FRAME_W__", str(FRAME_W)).replace("__FRAME_H__", str(FRAME_H)) \
   .replace("__ROI_CX_DEFAULT__", str(ROI_CX_DEFAULT)).replace("__ROI_CY_DEFAULT__", str(ROI_CY_DEFAULT)) \
   .replace("__ROI_R_DEFAULT__", str(ROI_R_DEFAULT)).replace("__ROI_R_MAX__", str(ROI_R_MAX))


def parse_int(name, default, lo, hi):
    try:
        v = int(request.args.get(name, default))
        return max(lo, min(hi, v))
    except Exception:
        return default


@app.route("/")
def index():
    return PAGE


@app.route("/video")
def video():
    hmin = parse_int("hmin", H_MIN, 0, 179)
    hmax = parse_int("hmax", H_MAX, 0, 179)
    smin = parse_int("smin", S_MIN, 0, 255)
    smax = parse_int("smax", S_MAX, 0, 255)
    vmin = parse_int("vmin", V_MIN, 0, 255)
    vmax = parse_int("vmax", V_MAX, 0, 255)
    min_area = parse_int("minarea", MIN_AREA_DEFAULT, 0, MIN_AREA_MAX)

    roi_cx = parse_int("roicx", ROI_CX_DEFAULT, 0, FRAME_W)
    roi_cy = parse_int("roicy", ROI_CY_DEFAULT, 0, FRAME_H)
    roi_r = parse_int("roir", ROI_R_DEFAULT, 1, ROI_R_MAX)

    mode = request.args.get("mode", "overlay")

    frame = picam2.capture_array()
    bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

    color_mask = cv2.inRange(hsv, np.array([hmin, smin, vmin]), np.array([hmax, smax, vmax]))

    # круговая маска ROI - белый круг на чёрном фоне того же размера,
    # что и кадр; цвет ищем только там, где обе маски пересекаются
    roi_mask = np.zeros(color_mask.shape, dtype=np.uint8)
    cv2.circle(roi_mask, (roi_cx, roi_cy), roi_r, 255, -1)

    mask = cv2.bitwise_and(color_mask, roi_mask)

    if mode == "mask":
        out = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        cv2.circle(out, (roi_cx, roi_cy), roi_r, (0, 255, 255), 3)
    else:
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)
            if area > min_area:
                (x, y), r = cv2.minEnclosingCircle(c)
                cx, cy = int(x), int(y)
                cv2.circle(bgr, (cx, cy), int(r), (0, 255, 0), 2)
                cv2.circle(bgr, (cx, cy), 3, (0, 0, 255), -1)
                cv2.putText(bgr, f"H[{hmin},{hmax}] S[{smin},{smax}] V[{vmin},{vmax}]",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                cv2.putText(bgr, f"center=({cx},{cy}) r={int(r)} area={int(area)} minarea={min_area}",
                            (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        # рамка ROI на overlay - всегда видна, чтобы понимать рабочую зону
        cv2.circle(bgr, (roi_cx, roi_cy), roi_r, (255, 255, 0), 3)
        out = bgr

    ok, jpg = cv2.imencode(".jpg", out, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
    if not ok:
        return ("encode error", 500)

    return Response(jpg.tobytes(), mimetype="image/jpeg")


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, threaded=True)