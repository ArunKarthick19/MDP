"""
Flask web app for testing YOLO image recognition (based on model.py).
Run: python web_app.py
Then open: http://localhost:5000
"""

import io
import os
import time
import logging
import base64
from pathlib import Path
from datetime import datetime

import cv2
import torch
import numpy as np
from PIL import Image
from flask import Flask, request, jsonify, render_template_string
from ultralytics import YOLO

# ── Config ──────────────────────────────────────────────────────────────────
MODEL_PATH = Path(__file__).parent / "SYJ_noob.pt"
CONF_THRESHOLD = 0.3
DEVICE = "cuda" if torch.cuda.is_available() else "cpu"

# ID map from model.py
ID_MAP = {
    "10": 10,  # Bullseye
    "11": 11,  # 1
    "12": 12,  # 2
    "13": 13,  # 3
    "14": 14,  # 4
    "15": 15,  # 5
    "16": 16,  # 6
    "17": 17,  # 7
    "18": 18,  # 8
    "19": 19,  # 9
    "20": 20,  # a
    "21": 21,  # b
    "22": 22,  # c
    "23": 23,  # d
    "24": 24,  # e
    "25": 25,  # f
    "26": 26,  # g
    "27": 27,  # h
    "28": 28,  # s
    "29": 29,  # t
    "30": 30,  # u
    "31": 31,  # v
    "32": 32,  # w
    "33": 33,  # x
    "34": 34,  # y
    "35": 35,  # z
    "36": 36,  # Up Arrow
    "37": 37,  # Down Arrow
    "38": 38,  # Right Arrow
    "39": 39,  # Left Arrow
    "40": 40,  # target
}

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# ── Load model once at startup ───────────────────────────────────────────────
logger.info(f"Loading YOLO model from {MODEL_PATH} on {DEVICE} ...")
model = YOLO(MODEL_PATH)
model.to(DEVICE)
logger.info("Model loaded successfully!")

app = Flask(__name__)


# ── Helpers (mirrored from model.py) ────────────────────────────────────────

def find_largest_or_central_bbox(bboxes, signal="C"):
    """Select the best bounding box (from model.py logic)."""
    if not bboxes:
        return "NA", 0.0

    valid_bboxes = [b for b in bboxes if b["label"] != "10" and b["confidence"] > CONF_THRESHOLD]
    if not valid_bboxes:
        return "NA", 0.0

    if len(valid_bboxes) == 1:
        return valid_bboxes[0]["label"], valid_bboxes[0]["bbox_area"]

    max_area = max(b["bbox_area"] for b in valid_bboxes)
    threshold = 0.1
    largest_bboxes = [b for b in valid_bboxes if b["bbox_area"] >= (1 - threshold) * max_area]

    if signal == "L":
        chosen = min(largest_bboxes, key=lambda x: x["xywh"][0])
    elif signal == "R":
        chosen = max(largest_bboxes, key=lambda x: x["xywh"][0])
    else:
        chosen = max(largest_bboxes, key=lambda x: x["bbox_area"])

    return chosen["label"], chosen["bbox_area"]



def run_inference(pil_image: Image.Image, conf: float, signal: str):
    """
    Run YOLO on a PIL image and return:
      - annotated image (numpy BGR)
      - list of detection dicts
      - selected label / image_id
      - timing info
    """
    t0 = time.time()

    # Convert PIL → numpy (RGB)
    img_np = np.array(pil_image.convert("RGB"))

    results = model.predict(
        source=img_np,
        conf=conf,
        imgsz=640,
        device=DEVICE,
        verbose=False,
    )
    inference_time = time.time() - t0

    bboxes = []
    if results[0].boxes:
        for result in results:
            for box in result.boxes:
                cls_index = int(box.cls.tolist()[0])
                label = result.names[cls_index]
                xywh = box.xywh.tolist()[0]
                bbox_area = xywh[2] * xywh[3]
                confidence = box.conf.tolist()[0]
                bboxes.append(
                    {
                        "label": label,
                        "xywh": xywh,
                        "bbox_area": bbox_area,
                        "confidence": confidence,
                        "class_name": result.names[cls_index],
                    }
                )

    selected_label, selected_area = find_largest_or_central_bbox(bboxes, signal)
    image_id = ID_MAP.get(selected_label, "NA")

    # Build annotated image
    annotated = results[0].plot()  # BGR numpy

    return annotated, bboxes, selected_label, image_id, inference_time


def numpy_to_base64(img_bgr: np.ndarray) -> str:
    """Encode a BGR numpy image to a base64 PNG string."""
    success, buf = cv2.imencode(".jpg", img_bgr, [cv2.IMWRITE_JPEG_QUALITY, 90])
    if not success:
        raise RuntimeError("Failed to encode image")
    return base64.b64encode(buf.tobytes()).decode("utf-8")


# ── Routes ───────────────────────────────────────────────────────────────────

HTML_TEMPLATE = """<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1.0" />
  <title>YOLO Image Tester</title>
  <style>
    *, *::before, *::after { box-sizing: border-box; margin: 0; padding: 0; }

    body {
      font-family: 'Segoe UI', system-ui, sans-serif;
      background: #0f1117;
      color: #e2e8f0;
      min-height: 100vh;
      display: flex;
      flex-direction: column;
      align-items: center;
      padding: 2rem 1rem;
    }

    h1 {
      font-size: 1.8rem;
      font-weight: 700;
      background: linear-gradient(135deg, #60a5fa, #a78bfa);
      -webkit-background-clip: text;
      -webkit-text-fill-color: transparent;
      margin-bottom: 0.3rem;
    }
    .subtitle { color: #94a3b8; font-size: 0.9rem; margin-bottom: 2rem; }

    .container {
      width: 100%;
      max-width: 1100px;
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 1.5rem;
    }
    @media (max-width: 700px) { .container { grid-template-columns: 1fr; } }

    .card {
      background: #1e2130;
      border: 1px solid #2d3148;
      border-radius: 14px;
      padding: 1.4rem;
    }
    .card h2 { font-size: 1rem; font-weight: 600; color: #93c5fd; margin-bottom: 1rem; }

    /* Drop zone */
    #dropzone {
      border: 2px dashed #3b4278;
      border-radius: 10px;
      padding: 2.5rem 1rem;
      text-align: center;
      cursor: pointer;
      transition: border-color 0.2s, background 0.2s;
      position: relative;
    }
    #dropzone.drag-over { border-color: #60a5fa; background: #1a2540; }
    #dropzone .icon { font-size: 2.8rem; margin-bottom: 0.5rem; }
    #dropzone p { color: #94a3b8; font-size: 0.9rem; }
    #dropzone span { color: #60a5fa; font-weight: 600; cursor: pointer; }
    #fileInput { display: none; }

    #preview-wrap { margin-top: 1rem; display: none; text-align: center; }
    #preview-wrap img { max-width: 100%; max-height: 280px; border-radius: 8px; border: 1px solid #2d3148; }
    #preview-name { font-size: 0.78rem; color: #64748b; margin-top: 0.4rem; }

    /* Controls */
    .ctrl-row { display: flex; align-items: center; gap: 0.8rem; margin-bottom: 0.9rem; }
    .ctrl-row label { font-size: 0.85rem; color: #94a3b8; min-width: 120px; }
    input[type=range] { flex: 1; accent-color: #60a5fa; }
    #confValue { font-size: 0.85rem; color: #e2e8f0; min-width: 2.5rem; text-align: right; }

    select {
      background: #0f1117;
      border: 1px solid #2d3148;
      color: #e2e8f0;
      border-radius: 6px;
      padding: 0.35rem 0.6rem;
      font-size: 0.85rem;
      flex: 1;
    }

    #detectBtn {
      width: 100%;
      padding: 0.75rem;
      background: linear-gradient(135deg, #3b82f6, #7c3aed);
      border: none;
      border-radius: 8px;
      color: #fff;
      font-size: 1rem;
      font-weight: 600;
      cursor: pointer;
      transition: opacity 0.2s;
      margin-top: 0.5rem;
    }
    #detectBtn:disabled { opacity: 0.5; cursor: not-allowed; }
    #detectBtn:hover:not(:disabled) { opacity: 0.9; }

    /* Spinner */
    .spinner { display: none; width: 20px; height: 20px; border: 3px solid #fff3; border-top-color: #fff; border-radius: 50%; animation: spin 0.7s linear infinite; margin: 0 auto; }
    @keyframes spin { to { transform: rotate(360deg); } }

    /* Result */
    #resultImg { width: 100%; border-radius: 8px; border: 1px solid #2d3148; display: none; }
    #noResult { color: #64748b; font-size: 0.9rem; text-align: center; padding: 3rem 0; }

    .badge-wrap { display: flex; flex-wrap: wrap; gap: 0.5rem; margin-top: 1rem; }
    .badge {
      background: #1a2540;
      border: 1px solid #2d3148;
      border-radius: 20px;
      padding: 0.3rem 0.8rem;
      font-size: 0.8rem;
      display: flex;
      align-items: center;
      gap: 0.4rem;
    }
    .badge .conf { color: #60a5fa; font-weight: 700; }
    .badge .lbl  { color: #e2e8f0; }
    .badge .id   { color: #a78bfa; font-size: 0.75rem; }

    #selected-box {
      margin-top: 1rem;
      padding: 0.8rem 1rem;
      background: #0f1117;
      border-left: 3px solid #60a5fa;
      border-radius: 0 8px 8px 0;
      font-size: 0.9rem;
    }
    #selected-box .key { color: #64748b; }
    #selected-box .val { color: #f1f5f9; font-weight: 700; font-size: 1rem; }
    #selected-box .img-id { color: #a78bfa; font-size: 0.85rem; }

    #timing { color: #475569; font-size: 0.78rem; margin-top: 0.8rem; }

    .error { color: #f87171; font-size: 0.88rem; margin-top: 0.5rem; }
  </style>
</head>
<body>
  <h1>🎯 YOLO Image Tester</h1>
  <p class="subtitle">Based on model.py · drag &amp; drop or click to upload</p>

  <div class="container">
    <!-- Left: Upload + controls -->
    <div class="card">
      <h2>📤 Upload Image</h2>

      <div id="dropzone">
        <div class="icon">🖼️</div>
        <p>Drag &amp; drop an image here, or <span id="browseBtn">browse</span></p>
        <p style="margin-top:0.4rem; font-size:0.78rem;">JPG · PNG · JPEG · WEBP</p>
        <input type="file" id="fileInput" accept="image/*" />
      </div>

      <div id="preview-wrap">
        <img id="previewImg" src="" alt="preview" />
        <div id="preview-name"></div>
      </div>

      <div style="margin-top:1.2rem;">
        <div class="ctrl-row">
          <label>Confidence</label>
          <input type="range" id="confSlider" min="0" max="1" step="0.05" value="0.30" />
          <span id="confValue">0.30</span>
        </div>
        <div class="ctrl-row">
          <label>Signal (tie-break)</label>
          <select id="signalSelect">
            <option value="C">C — Central (default)</option>
            <option value="L">L — Obstacle Left</option>
            <option value="R">R — Obstacle Right</option>
          </select>
        </div>
      </div>

      <button id="detectBtn" disabled>🔍 Detect</button>
      <div class="spinner" id="spinner"></div>
      <div class="error" id="errorMsg"></div>
    </div>

    <!-- Right: Result -->
    <div class="card">
      <h2>📊 Detection Result</h2>
      <p id="noResult">Upload an image and click Detect to see results.</p>
      <img id="resultImg" src="" alt="result" />
      <div class="badge-wrap" id="badges"></div>
      <div id="selected-box" style="display:none">
        <div class="key">Selected detection</div>
        <div class="val" id="selLabel">—</div>
        <div class="img-id" id="selId"></div>
      </div>
      <div id="timing"></div>
    </div>
  </div>

  <script>
    const dropzone   = document.getElementById('dropzone');
    const fileInput  = document.getElementById('fileInput');
    const browseBtn  = document.getElementById('browseBtn');
    const previewImg = document.getElementById('previewImg');
    const previewWrap= document.getElementById('preview-wrap');
    const previewName= document.getElementById('preview-name');
    const confSlider = document.getElementById('confSlider');
    const confValue  = document.getElementById('confValue');
    const signalSel  = document.getElementById('signalSelect');
    const detectBtn  = document.getElementById('detectBtn');
    const spinner    = document.getElementById('spinner');
    const errorMsg   = document.getElementById('errorMsg');
    const resultImg  = document.getElementById('resultImg');
    const noResult   = document.getElementById('noResult');
    const badges     = document.getElementById('badges');
    const selBox     = document.getElementById('selected-box');
    const selLabel   = document.getElementById('selLabel');
    const selId      = document.getElementById('selId');
    const timing     = document.getElementById('timing');

    let currentFile = null;

    // Confidence slider
    confSlider.addEventListener('input', () => {
      confValue.textContent = parseFloat(confSlider.value).toFixed(2);
    });

    // Browse click
    browseBtn.addEventListener('click', () => fileInput.click());
    dropzone.addEventListener('click', (e) => { if (e.target !== browseBtn) fileInput.click(); });

    // File input change
    fileInput.addEventListener('change', () => {
      if (fileInput.files[0]) loadFile(fileInput.files[0]);
    });

    // Drag & drop
    dropzone.addEventListener('dragover', (e) => { e.preventDefault(); dropzone.classList.add('drag-over'); });
    dropzone.addEventListener('dragleave', () => dropzone.classList.remove('drag-over'));
    dropzone.addEventListener('drop', (e) => {
      e.preventDefault();
      dropzone.classList.remove('drag-over');
      if (e.dataTransfer.files[0]) loadFile(e.dataTransfer.files[0]);
    });

    function loadFile(file) {
      if (!file.type.startsWith('image/')) {
        showError('Please upload a valid image file.');
        return;
      }
      currentFile = file;
      const reader = new FileReader();
      reader.onload = (ev) => {
        previewImg.src = ev.target.result;
        previewWrap.style.display = 'block';
        previewName.textContent = file.name + ' (' + (file.size / 1024).toFixed(1) + ' KB)';
      };
      reader.readAsDataURL(file);
      detectBtn.disabled = false;
      clearResults();
      clearError();
    }

    // Detect button
    detectBtn.addEventListener('click', async () => {
      if (!currentFile) return;
      setLoading(true);
      clearResults();
      clearError();

      const formData = new FormData();
      formData.append('image', currentFile);
      formData.append('conf', confSlider.value);
      formData.append('signal', signalSel.value);

      try {
        const resp = await fetch('/predict', { method: 'POST', body: formData });
        const data = await resp.json();

        if (!resp.ok || data.error) {
          showError(data.error || 'Server error');
          return;
        }

        // Show annotated image
        resultImg.src = 'data:image/jpeg;base64,' + data.annotated_image;
        resultImg.style.display = 'block';
        noResult.style.display  = 'none';

        // Badges
        badges.innerHTML = '';
        data.detections.forEach(d => {
          const b = document.createElement('div');
          b.className = 'badge';
          b.innerHTML = `<span class="lbl">${d.class_name}</span>
                         <span class="conf">${(d.confidence * 100).toFixed(1)}%</span>
                         <span class="id">id:${d.label}</span>`;
          badges.appendChild(b);
        });

        // Selected result
        if (data.selected_label !== 'NA') {
          selLabel.textContent = data.selected_label;
          selId.textContent    = `Image ID: ${data.image_id}`;
          selBox.style.display = 'block';
        } else {
          selLabel.textContent = '⚠️ No valid detection';
          selId.textContent    = '';
          selBox.style.display = 'block';
        }

        // Timing
        timing.textContent = `⚡ Inference: ${data.inference_time_ms} ms  |  Device: ${data.device}`;

      } catch (err) {
        showError('Network error: ' + err.message);
      } finally {
        setLoading(false);
      }
    });

    function setLoading(on) {
      detectBtn.disabled    = on;
      spinner.style.display = on ? 'block' : 'none';
      detectBtn.textContent = on ? '' : '🔍 Detect';
    }

    function clearResults() {
      resultImg.style.display = 'none';
      noResult.style.display  = 'block';
      badges.innerHTML        = '';
      selBox.style.display    = 'none';
      timing.textContent      = '';
    }

    function showError(msg) { errorMsg.textContent = '❌ ' + msg; }
    function clearError()   { errorMsg.textContent = ''; }
  </script>
</body>
</html>
"""


@app.route("/")
def index():
    return render_template_string(HTML_TEMPLATE)


@app.route("/predict", methods=["POST"])
def predict():
    if "image" not in request.files:
        return jsonify({"error": "No image uploaded"}), 400

    file = request.files["image"]
    if file.filename == "":
        return jsonify({"error": "Empty filename"}), 400

    conf   = float(request.form.get("conf", CONF_THRESHOLD))
    signal = request.form.get("signal", "C").upper()

    try:
        pil_image = Image.open(io.BytesIO(file.read())).convert("RGB")
    except Exception as e:
        return jsonify({"error": f"Cannot read image: {e}"}), 400

    try:
        annotated_bgr, bboxes, selected_label, image_id, inference_time = run_inference(
            pil_image, conf, signal
        )
    except Exception as e:
        logger.exception("Inference error")
        return jsonify({"error": f"Inference failed: {e}"}), 500

    annotated_b64 = numpy_to_base64(annotated_bgr)

    return jsonify(
        {
            "annotated_image": annotated_b64,
            "detections": [
                {
                    "label": b["label"],
                    "class_name": b["class_name"],
                    "confidence": round(b["confidence"], 4),
                    "bbox_area": round(b["bbox_area"], 2),
                }
                for b in bboxes
            ],
            "selected_label": selected_label,
            "image_id": image_id,
            "inference_time_ms": round(inference_time * 1000, 1),
            "device": DEVICE,
        }
    )


if __name__ == "__main__":
    print("\n" + "=" * 50)
    print("  YOLO Web Tester  —  based on model.py")
    print("  Open: http://localhost:5001")
    print("=" * 50 + "\n")
    app.run(host="0.0.0.0", port=5001, debug=False)
