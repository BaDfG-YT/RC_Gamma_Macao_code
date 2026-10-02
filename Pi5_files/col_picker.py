import threading

from flask import Flask, Response, request, jsonify
import cv2
import numpy as np
from picamera2 import Picamera2

from config_io import load_config, save_detection, save_zone

app = Flask(__name__)

CFG = load_config()

# Maximum resolution of the IMX708 sensor (Camera Module 3): 4608x2592.
# Shooting at full resolution is the best option for tuning HSV, since it
# shows the finest details of color edges. If the Pi can't handle a smooth
# stream at that size, drop to the half-binned 2304x1296 (also a native
# sensor mode, reads faster) or temporarily lower still.
FRAME_W, FRAME_H = int(CFG["vision"]["frame"]["high"]["w"]), int(CFG["vision"]["frame"]["high"]["h"])

# upper bound for both radius sliders
RAD_MAX = max(FRAME_W, FRAME_H)

picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"size": (FRAME_W, FRAME_H)}))
picam2.start()

# Flask runs with threaded=True, but there's only one camera - don't let
# requests call capture_array() at the same time
cam_lock = threading.Lock()

# targets that can be tuned (keys of vision.detection in the config)
TARGETS = ["ball", "gateB", "gateY"]
DEFAULT_TARGET = "ball"


def target_values(target):
    """Initial slider values from the config (defaults + local)."""
    det = CFG["vision"]["detection"][target]
    (h0, s0, v0), (h1, s1, v1) = det["hsv"]
    zones = CFG["vision"]["accessible zone"]
    xc, yc, rad_sml, rad_big = zones["xc"], zones["yc"], zones["rad_sml"], zones["rad_big"]
    return h0, s0, v0, h1, s1, v1, int(det["min_area"]), xc, yc, rad_sml, rad_big


# startup values - from the config for the default target
H_MIN, S_MIN, V_MIN, H_MAX, S_MAX, V_MAX, MIN_AREA_DEFAULT, ROI_CX, ROI_CY, ROI_R_SML, ROI_R_BIG = target_values(DEFAULT_TARGET)
MIN_AREA_MAX = 5000  # upper bound for the slider

# JPEG quality for the stream (0-100). 80 -> 92: noticeably fewer
# compression artifacts, also helpful when tuning HSV around object edges.
JPEG_QUALITY = 92

PAGE_TMPL = """
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
  <div style="margin-bottom:8px">
    Target:
    <select id="target" onchange="location.href='/?target='+this.value">__TARGET_OPTIONS__</select>
    (values loaded from config)
  </div>
  <div class="row">
    <div class="panel">
      <img id="img" src="/video" width="640">
      <div style="margin-top:8px">
        <button onclick="setMode('overlay')">overlay</button>
        <button onclick="setMode('mask')">mask</button>
      </div>
      <div style="margin-top:8px">
        <div><b>HSV + min area</b> (for the selected target)</div>
        <button onclick="saveCfg('defaults')">Save to default config</button>
        <button onclick="saveCfg('local')">Save to local config</button>
      </div>
      <div style="margin-top:8px">
        <div><b>ROI</b> (shared by all targets)</div>
        <button onclick="saveZone('defaults')">Save ROI to default config</button>
        <button onclick="saveZone('local')">Save ROI to local config</button>
      </div>
      <div id="savemsg" style="margin-top:6px"></div>
      <div style="margin-top:8px">
        <div>LOWER: <code id="lowerv"></code></div>
        <div>UPPER: <code id="upperv"></code></div>
        <div>MIN AREA: <code id="minareasum"></code></div>
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
      <b>ROI (search circle)</b>

      <label>Center X: <span id="roicxv"></span></label>
      <input id="roicx" type="range" min="0" max="__FRAME_W__" value="__ROI_CX__">
      <label>Center Y: <span id="roicyv"></span></label>
      <input id="roicy" type="range" min="0" max="__FRAME_H__" value="__ROI_CY__">
      <label>Radius small (rad_sml): <span id="roirsv"></span></label>
      <input id="roirs" type="range" min="1" max="__RAD_MAX__" value="__ROI_R_SML__">
      <label>Radius big (rad_big): <span id="roirbv"></span></label>
      <input id="roirb" type="range" min="1" max="__RAD_MAX__" value="__ROI_R_BIG__">
    </div>
  </div>

<script>
const IDS = ["hmin","hmax","smin","smax","vmin","vmax","minarea","roicx","roicy","roirs","roirb"];
let mode = "overlay";
function val(id){ return document.getElementById(id).value; }
function updLabels(){
  IDS.forEach(id=>{
    const el = document.getElementById(id+"v");
    if (el) el.textContent = val(id);
  });
  document.getElementById("lowerv").textContent = `[${val("hmin")}, ${val("smin")}, ${val("vmin")}]`;
  document.getElementById("upperv").textContent = `[${val("hmax")}, ${val("smax")}, ${val("vmax")}]`;
  document.getElementById("minareasum").textContent = val("minarea");
  document.getElementById("roiv").textContent =
    `cx=${val("roicx")} cy=${val("roicy")} r_sml=${val("roirs")} r_big=${val("roirb")}`;
}
function refresh(){
  updLabels();
  const url = `/video?hmin=${val("hmin")}&hmax=${val("hmax")}&smin=${val("smin")}&smax=${val("smax")}`
            + `&vmin=${val("vmin")}&vmax=${val("vmax")}&minarea=${val("minarea")}`
            + `&roicx=${val("roicx")}&roicy=${val("roicy")}&roirs=${val("roirs")}&roirb=${val("roirb")}`
            + `&mode=${mode}&_=${Date.now()}`;
  document.getElementById("img").src = url;
}
function setMode(m){ mode=m; refresh(); }

async function post(url, okText){
  const msg = document.getElementById("savemsg");
  try {
    const r = await fetch(url, {method: "POST"});
    const j = await r.json();
    msg.textContent = j.ok ? okText(j) : `Error: ${j.error}`;
  } catch (e) { msg.textContent = "Error: " + e; }
}

// HSV + min area for the selected target (ROI is NOT sent here)
function saveCfg(where){
  const q = `target=${document.getElementById("target").value}&where=${where}`
          + `&hmin=${val("hmin")}&smin=${val("smin")}&vmin=${val("vmin")}`
          + `&hmax=${val("hmax")}&smax=${val("smax")}&vmax=${val("vmax")}`
          + `&minarea=${val("minarea")}`;
  return post("/save?" + q, j => `Saved (${where}): ${j.target} -> ${j.path}`);
}

// ROI - sent as a separate request since it's shared by all targets
function saveZone(where){
  const q = `where=${where}&roicx=${val("roicx")}&roicy=${val("roicy")}`
          + `&roirs=${val("roirs")}&roirb=${val("roirb")}`;
  return post("/save_zone?" + q, j =>
    `ROI saved (${where}): cx=${j.xc} cy=${j.yc} r_sml=${j.rad_sml} r_big=${j.rad_big} -> ${j.path}`);
}

IDS.forEach(id=>{
  document.getElementById(id).addEventListener("input", refresh);
});
updLabels();
refresh();
setInterval(refresh, 200); // refresh at 5 fps for tuning
</script>
</body>
</html>
"""


def render_page(target):
    h0, s0, v0, h1, s1, v1, min_area, roi_cx, roi_cy, roi_rsml, roi_rbig = target_values(target)
    options = "".join(
        f'<option value="{t}"{" selected" if t == target else ""}>{t}</option>'
        for t in TARGETS
    )
    return (PAGE_TMPL
            .replace("__TARGET_OPTIONS__", options)
            .replace("__H_MIN__", str(h0)).replace("__H_MAX__", str(h1))
            .replace("__S_MIN__", str(s0)).replace("__S_MAX__", str(s1))
            .replace("__V_MIN__", str(v0)).replace("__V_MAX__", str(v1))
            .replace("__MIN_AREA_MAX__", str(MIN_AREA_MAX))
            .replace("__MIN_AREA_DEFAULT__", str(min_area))
            .replace("__FRAME_W__", str(FRAME_W)).replace("__FRAME_H__", str(FRAME_H))
            .replace("__RAD_MAX__", str(RAD_MAX))
            .replace("__ROI_CX__", str(roi_cx))
            .replace("__ROI_CY__", str(roi_cy))
            .replace("__ROI_R_SML__", str(roi_rsml))
            .replace("__ROI_R_BIG__", str(roi_rbig)))


def parse_int(name, default, lo, hi):
    try:
        v = int(request.args.get(name, default))
        return max(lo, min(hi, v))
    except Exception:
        return default


def reload_cfg():
    """Reload the config after saving so the page shows fresh values."""
    global CFG
    CFG = load_config()


@app.route("/")
def index():
    target = request.args.get("target", DEFAULT_TARGET)
    if target not in TARGETS:
        target = DEFAULT_TARGET
    return render_page(target)


@app.route("/save", methods=["POST"])
def save():
    """Saves HSV and min_area of the selected target."""
    target = request.args.get("target", "")
    where = request.args.get("where", "")
    if target not in TARGETS or where not in ("defaults", "local"):
        return jsonify(ok=False, error="bad target/where"), 400
    lo = [parse_int("hmin", 0, 0, 179), parse_int("smin", 0, 0, 255), parse_int("vmin", 0, 0, 255)]
    hi = [parse_int("hmax", 179, 0, 179), parse_int("smax", 255, 0, 255), parse_int("vmax", 255, 0, 255)]
    min_area = parse_int("minarea", 0, 0, MIN_AREA_MAX)
    try:
        path = save_detection(target, lo, hi, min_area, where)
        reload_cfg()
    except Exception as e:
        return jsonify(ok=False, error=str(e)), 500
    return jsonify(ok=True, target=target, where=where, path=str(path))


@app.route("/save_zone", methods=["POST"])
def save_zone_route():
    """Saves ROI (xc, yc, rad_sml, rad_big) in vision.accessible zone."""
    where = request.args.get("where", "")
    if where not in ("defaults", "local"):
        return jsonify(ok=False, error="bad where"), 400
    xc = parse_int("roicx", FRAME_W // 2, 0, FRAME_W)
    yc = parse_int("roicy", FRAME_H // 2, 0, FRAME_H)
    rad_sml = parse_int("roirs", ROI_R_SML, 1, RAD_MAX)
    rad_big = parse_int("roirb", ROI_R_BIG, 1, RAD_MAX)
    if rad_sml > rad_big:
        return jsonify(ok=False, error=f"rad_sml ({rad_sml}) is greater than rad_big ({rad_big})"), 400
    try:
        path = save_zone(xc, yc, rad_sml, rad_big, where)
        reload_cfg()
    except Exception as e:
        return jsonify(ok=False, error=str(e)), 500
    return jsonify(ok=True, where=where, path=str(path),
                   xc=xc, yc=yc, rad_sml=rad_sml, rad_big=rad_big)


@app.route("/video")
def video():
    hmin = parse_int("hmin", H_MIN, 0, 179)
    hmax = parse_int("hmax", H_MAX, 0, 179)
    smin = parse_int("smin", S_MIN, 0, 255)
    smax = parse_int("smax", S_MAX, 0, 255)
    vmin = parse_int("vmin", V_MIN, 0, 255)
    vmax = parse_int("vmax", V_MAX, 0, 255)
    min_area = parse_int("minarea", MIN_AREA_DEFAULT, 0, MIN_AREA_MAX)

    roi_cx = parse_int("roicx", ROI_CX, 0, FRAME_W)
    roi_cy = parse_int("roicy", ROI_CY, 0, FRAME_H)
    roi_rs = parse_int("roirs", ROI_R_SML, 1, RAD_MAX)
    roi_rb = parse_int("roirb", ROI_R_BIG, 1, RAD_MAX)

    mode = request.args.get("mode", "overlay")

    with cam_lock:
        frame = picam2.capture_array()
    bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

    color_mask = cv2.inRange(hsv, np.array([hmin, smin, vmin]), np.array([hmax, smax, vmax]))

    # ring-shaped ROI mask: a white ring between rad_sml (inner radius)
    # and rad_big (outer) on a black background the same size as the frame;
    # color is only searched for where both masks overlap
    roi_mask = np.zeros(color_mask.shape, dtype=np.uint8)
    cv2.circle(roi_mask, (roi_cx, roi_cy), roi_rb, 255, -1)  # fill the big circle
    cv2.circle(roi_mask, (roi_cx, roi_cy), roi_rs, 0, -1)    # cut out the small one

    mask = cv2.bitwise_and(color_mask, roi_mask)

    if mode == "mask":
        out = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
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
        out = bgr

    # ROI outlines - always visible: small circle (cyan) and big (magenta)
    cv2.circle(out, (roi_cx, roi_cy), roi_rs, (255, 255, 0), 3)
    cv2.circle(out, (roi_cx, roi_cy), roi_rb, (255, 0, 255), 3)

    ok, jpg = cv2.imencode(".jpg", out, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
    if not ok:
        return ("encode error", 500)

    return Response(jpg.tobytes(), mimetype="image/jpeg")


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, threaded=True)