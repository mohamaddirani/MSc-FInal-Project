# This script writes an updated HTML viewer that matches the *exact* A* indexing math:
#   grid_to_meters(cx, cy): x=(cx-W//2)*res, y=(cy-H//2)*res
#   meters_to_grid(x,y):    cx=int(W//2 + x/res), cy=int(H//2 + y/res)
#
# It also keeps your prior "world edges" mode as an option, but defaults to the A* mode.
# Extras:
#  - Inputs to jump to a specific (cx,cy) cell and to convert (x,y) meters -> (cx,cy)
#  - Paste your A* log to highlight all cells in the path
#
# The file is saved to `sim_app/npz_grid_viewer_astar.html`.

import numpy as np
from pathlib import Path
import json

npz_path = Path("sim_app/map_memory.npz")  # fallback if user-provided path isn't mounted
# Try project path first (as per user's code); if missing, fallback to /mnt/data
proj_npz_path = Path("sim_app/map_memory.npz")
use_path = proj_npz_path if proj_npz_path.exists() else npz_path

npz = np.load(use_path, allow_pickle=True)

two_d_arrays = {}
for k in npz.files:
    arr = npz[k]
    if isinstance(arr, np.ndarray) and arr.ndim == 2:
        two_d_arrays[k] = arr.astype(float)

# common names
if not two_d_arrays and "arr_0" in npz.files:
    arr0 = npz["arr_0"]
    if isinstance(arr0, np.ndarray) and arr0.ndim == 2:
        two_d_arrays["arr_0"] = arr0.astype(float)

if not two_d_arrays:
    two_d_arrays["NO_2D_ARRAYS_FOUND"] = np.zeros((2,2), dtype=float)

meta = {k: {"shape": list(v.shape), "min": float(np.nanmin(v)), "max": float(np.nanmax(v))} for k, v in two_d_arrays.items()}
serializable = {k: v.tolist() for k, v in two_d_arrays.items()}

# Pull resolution if present
res_val = float(npz["res"]) if "res" in npz.files else 0.2  # default matches sim_app.shared.MAP_RESOLUTION

def export_free_grids_within_rectangle(arr, rect, out_path):
  x_min = min(rect[0][0], rect[1][0], rect[2][0], rect[3][0])
  x_max = max(rect[0][0], rect[1][0], rect[2][0], rect[3][0])
  y_min = min(rect[0][1], rect[1][1], rect[2][1], rect[3][1])
  y_max = max(rect[0][1], rect[1][1], rect[2][1], rect[3][1])
  with open(out_path, "w") as f:
    for r in range(arr.shape[0]):
      for c in range(arr.shape[1]):
        if arr[r, c] == 0:
          if x_min <= c <= x_max and y_min <= r <= y_max:
            f.write(f"({c},{r})\n")

def export_occupied_grids_within_rectangle(arr, rect, out_path):
  x_min = min(rect[0][0], rect[1][0], rect[2][0], rect[3][0])
  x_max = max(rect[0][0], rect[1][0], rect[2][0], rect[3][0])
  y_min = min(rect[0][1], rect[1][1], rect[2][1], rect[3][1])
  y_max = max(rect[0][1], rect[1][1], rect[2][1], rect[3][1])
  with open(out_path, "w") as f:
    for r in range(arr.shape[0]):
      for c in range(arr.shape[1]):
        if arr[r, c] == 1:
          if x_min <= c <= x_max and y_min <= r <= y_max:
            f.write(f"({c},{r})\n")

# Rectangle corners as per user: (80,54), (159,54), (135,80), (135,159)
rect = [(80,54), (159,54), (80,135), (159,135)]
for k, arr in two_d_arrays.items():
  out_path_free = f"sim_app/free_grids_{k}_rect.txt"
  export_free_grids_within_rectangle(arr, rect, out_path_free)
  print(f"Exported free grids for {k} to {out_path_free} (rectangle filtered)")

  out_path_occ = f"sim_app/occupied_grids_{k}_rect.txt"
  export_occupied_grids_within_rectangle(arr, rect, out_path_occ)
  print(f"Exported occupied grids for {k} to {out_path_occ} (rectangle filtered)")

template = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8" />
<meta name="viewport" content="width=device-width, initial-scale=1" />
<title>NPZ Grid Viewer — A* Accurate Indices</title>
<style>
  :root { --bg:#0f1115; --panel:#151823; --text:#e6e6e6; --muted:#9aa4b2; --accent:#5b9cff; --border:#272a36; }
  html, body { margin:0; padding:0; background:var(--bg); color:var(--text); font-family:ui-sans-serif,system-ui,-apple-system,Segoe UI,Roboto,Arial; }
  .wrap { max-width:1280px; margin:24px auto; padding:0 16px; }
  .card { background:var(--panel); border:1px solid var(--border); border-radius:14px; padding:12px 14px; box-shadow:0 6px 20px rgba(0,0,0,0.25); }
  .header { display:flex; align-items:center; justify-content:space-between; gap:12px; flex-wrap:wrap; margin-bottom:12px; }
  .controls { display:grid; grid-template-columns: repeat(auto-fit, minmax(160px,1fr)); gap:12px; align-items:end; }
  label { font-size:13px; color:var(--muted); }
  select, input, textarea { background:#0e1118; border:1px solid var(--border); color:var(--text); border-radius:10px; padding:8px 10px; width:100%; }
  input[type="checkbox"] { accent-color:var(--accent); width:auto; }
  input[type="range"] { width: 200px; }
  .viewer { position:relative; margin-top:14px; border-radius:16px; overflow:hidden; border:1px solid var(--border); }
  canvas { display:block; image-rendering: pixelated; }
  .hovercard { position:absolute; pointer-events:none; background:rgba(0,0,0,0.85); color:#fff; padding:8px 10px; border-radius:10px; font-size:12px; transform:translate(12px,12px); white-space:nowrap; border:1px solid rgba(255,255,255,0.12); }
  .info { margin-top:10px; font-size:13px; color:var(--muted); }
  .legend { display:flex; gap:8px; align-items:center; }
  .swatch { width:14px; height:14px; border-radius:4px; border:1px solid #1f1f1f; display:inline-block; }
  .row { display:flex; gap:8px; align-items:center; flex-wrap:wrap; }
  .hl { position:absolute; border:2px solid rgba(91,156,255,0.9); pointer-events:none; }
</style>
</head>
<body>
<div class="wrap">
  <div class="header card">
    <div>
      <div style="font-weight:700; font-size:18px;">NPZ Grid Viewer — A* Accurate Indices</div>
      <div style="color:var(--muted); font-size:13px;">Matches A* math exactly: x=(cx-W//2)*res, y=(cy-H//2)*res. Hover, jump, and paste your A* log to highlight cells.</div>
    </div>
    <div class="legend">
      <span class="swatch" style="background:#ffffff;"></span><span>0 (free)</span>
      <span class="swatch" style="background:#000000;"></span><span>1 (occupied)</span>
      <span class="swatch" style="background:#808080;"></span><span>-1 (unknown)</span>
    </div>
  </div>

  <div class="card controls">
    <div>
      <label>Array</label><br/>
      <select id="keySel"></select>
    </div>
    <div>
      <label>Zoom</label><br/>
      <input id="zoom" type="range" min="1" max="28" step="1" value="4"/>
    </div>
    <div>
      <label>Flip L/R (visual only)</label><br/>
      <input id="flipLR" type="checkbox" />
    </div>
    <div>
      <label>Flip U/D (visual only)</label><br/>
      <input id="flipUD" type="checkbox" />
    </div>
    <div>
      <label>Rotate CW (visual only)</label><br/>
      <select id="rot">
        <option value="0">0°</option>
        <option value="1">90°</option>
        <option value="2">180°</option>
        <option value="3">270°</option>
      </select>
    </div>

    <div>
      <label>Coordinate Mode</label><br/>
      <select id="coordMode">
        <option value="astar" selected>A* center-origin (res only)</option>
        <option value="edges">World edges</option>
      </select>
    </div>
    <div id="edgesBox" style="display:none">
      <label>Edges (x_min, x_max, y_min, y_max)</label><br/>
      <div class="row">
        <input id="x_min" type="number" step="0.01" value="-6" />
        <input id="x_max" type="number" step="0.01" value="9" />
        <input id="y_min" type="number" step="0.01" value="-11" />
        <input id="y_max" type="number" step="0.01" value="4" />
      </div>
    </div>
    <div>
      <label>Resolution (m/cell)</label><br/>
      <input id="res" type="number" step="0.001" value="__RES__" />
    </div>
    <div>
      <label>Gridlines</label><br/>
      <input id="showGrid" type="checkbox" />
    </div>
    <div>
      <label>Snap hover to cell center</label><br/>
      <input id="snap" type="checkbox" checked />
    </div>
  </div>

  <div class="card controls">
    <div>
      <label>Jump to cell (cx, cy)</label><br/>
      <div class="row">
        <input id="jump_cx" type="number" step="1" value="137" />
        <input id="jump_cy" type="number" step="1" value="110" />
        <button id="btnJump">Highlight</button>
      </div>
    </div>
    <div>
      <label>Convert meters → cell</label><br/>
      <div class="row">
        <input id="mx" type="number" step="0.01" placeholder="x m" />
        <input id="my" type="number" step="0.01" placeholder="y m" />
        <button id="btnConvert">Compute (cx,cy)</button>
      </div>
      <div id="convOut" class="info"></div>
    </div>
    <div>
      <label>Paste A* log (highlights all "Grid: (cx, cy)")</label><br/>
      <textarea id="logBox" rows="4" placeholder='Paste lines like: "Grid: (137, 110) => ..."'></textarea>
      <div class="row"><button id="btnParseLog">Highlight Path</button><button id="btnClearHL">Clear</button></div>
    </div>
  </div>

  <div class="viewer">
    <canvas id="base"></canvas>
    <canvas id="overlay" style="position:absolute; left:0; top:0;"></canvas>
    <div id="hover" class="hovercard" style="display:none;"></div>
    <div id="hl" class="hl" style="display:none;"></div>
  </div>

  <div id="info" class="info"></div>
</div>

<script>
  const RAW = __DATA_JSON__;
  const META = __META_JSON__;

  const keySel = document.getElementById('keySel');
  const zoomEl = document.getElementById('zoom');
  const flipLR = document.getElementById('flipLR');
  const flipUD = document.getElementById('flipUD');
  const rot = document.getElementById('rot');
  const coordMode = document.getElementById('coordMode');
  const edgesBox = document.getElementById('edgesBox');
  const x_min_el = document.getElementById('x_min');
  const x_max_el = document.getElementById('x_max');
  const y_min_el = document.getElementById('y_min');
  const y_max_el = document.getElementById('y_max');
  const resEl = document.getElementById('res');
  const showGridEl = document.getElementById('showGrid');
  const snapEl = document.getElementById('snap');

  const base = document.getElementById('base');
  const overlay = document.getElementById('overlay');
  const hover = document.getElementById('hover');
  const baseCtx = base.getContext('2d');
  const overlayCtx = overlay.getContext('2d');
  const hl = document.getElementById('hl');

  const jump_cx = document.getElementById('jump_cx');
  const jump_cy = document.getElementById('jump_cy');
  const btnJump = document.getElementById('btnJump');
  const mx = document.getElementById('mx');
  const my = document.getElementById('my');
  const btnConvert = document.getElementById('btnConvert');
  const convOut = document.getElementById('convOut');
  const logBox = document.getElementById('logBox');
  const btnParseLog = document.getElementById('btnParseLog');
  const btnClearHL = document.getElementById('btnClearHL');

  let zoom = parseInt(zoomEl.value, 10);
  let rows = 0, cols = 0;       // of rendered (after transform)
  let rawRows = 0, rawCols = 0; // original

  // Populate arrays
  Object.keys(RAW).forEach(k => {
    const opt = document.createElement('option');
    const shape = META[k] ? META[k].shape : null;
    opt.value = k; opt.textContent = k + (shape ? `  (${shape[0]}×${shape[1]})` : '');
    keySel.appendChild(opt);
  });
  let currentKey = Object.keys(RAW)[0] || null;
  if (currentKey) keySel.value = currentKey;

  function clone2D(a) { return a.map(row => row.slice()); }
  function toBinary(a) {
    const r = a.length, c = a[0].length;
    const out = new Array(r);
    for (let i=0;i<r;i++){ out[i]=new Array(c); for (let j=0;j<c;j++) out[i][j]=(a[i][j]>0)?1:0; }
    return out;
  }
  function flipLeftRight(a) {
    const r=a.length, c=a[0].length, out=new Array(r);
    for (let i=0;i<r;i++){ out[i]=new Array(c); for(let j=0;j<c;j++) out[i][j]=a[i][c-1-j]; }
    return out;
  }
  function flipUpDown(a) {
    const r=a.length, c=a[0].length, out=new Array(r);
    for (let i=0;i<r;i++){ out[i]=a[r-1-i].slice(); }
    return out;
  }
  function rotCW(a,k){
    k=((k%4)+4)%4;
    let out=a;
    for(let t=0;t<k;t++){
      const r=out.length, c=out[0].length;
      const nxt=new Array(c);
      for(let i=0;i<c;i++){ nxt[i]=new Array(r); for(let j=0;j<r;j++) nxt[i][j]=out[r-1-j][i]; }
      out=nxt;
    }
    return out;
  }

  function getWorkingGrid() {
    const raw = RAW[currentKey];
    rawRows = raw.length; rawCols = raw[0].length;
    let g = clone2D(raw);
    // Visual transforms only
    // (Viewer always reports raw indices by inverting these)
    if (flipLR.checked) g = flipLeftRight(g);
    if (flipUD.checked) g = flipUpDown(g);
    const k = parseInt(rot.value,10)||0;
    if (k) g = rotCW(g, k);
    return g;
  }

  function colorForValue(v, minV, maxV) {
    if (Number.isFinite(v)) {
      if (v===0) return [255,255,255,255];
      if (v===1) return [0,0,0,255];
      if (v===-1) return [128,128,128,255];
      let t=0; if (maxV>minV){ t=(v-minV)/(maxV-minV); }
      t=Math.max(0,Math.min(1,t)); const g=Math.round(255*(1-t));
      return [g,g,g,255];
    }
    return [200,200,200,255];
  }

  function drawBase() {
    const g = getWorkingGrid();
    rows = g.length; cols = g[0].length;

    base.width = cols; base.height = rows;
    overlay.width = cols*zoom; overlay.height = rows*zoom;
    base.style.width = (cols*zoom)+'px';
    base.style.height = (rows*zoom)+'px';
    overlay.style.width = (cols*zoom)+'px';
    overlay.style.height = (rows*zoom)+'px';

    // stats for grayscale
    let minV=Infinity, maxV=-Infinity;
    for (let r=0;r<rows;r++){ for (let c=0;c<cols;c++){ const v=g[r][c]; if(Number.isFinite(v)){ if(v<minV)minV=v; if(v>maxV)maxV=v; } } }
    if (!Number.isFinite(minV)) { minV=0; maxV=1; }

    const imgData = baseCtx.createImageData(cols, rows);
    const data = imgData.data; let i=0;
    for (let r=0;r<rows;r++){
      for (let c=0;c<cols;c++){
        const rgba = colorForValue(g[r][c], minV, maxV);
        data[i++]=rgba[0]; data[i++]=rgba[1]; data[i++]=rgba[2]; data[i++]=rgba[3];
      }
    }
    baseCtx.putImageData(imgData, 0, 0);
    base.style.imageRendering = 'pixelated';

    document.getElementById('info').textContent =
      `Array: ${keySel.value} | raw: ${rawRows}×${rawCols} | rendered: ${rows}×${cols} | res=${parseFloat(resEl.value).toFixed(4)} m/cell`;

    overlayCtx.clearRect(0,0,overlay.width,overlay.height);
    drawGridlines();
    clearHighlights();
  }

  function drawGridlines() {
    if (!showGridEl.checked || zoom < 8) return;
    overlayCtx.save();
    overlayCtx.strokeStyle = 'rgba(255,255,255,0.12)';
    overlayCtx.lineWidth = 1;
    for (let x=0; x<=cols; x++){ overlayCtx.beginPath(); overlayCtx.moveTo(x*zoom+0.5,0); overlayCtx.lineTo(x*zoom+0.5, rows*zoom); overlayCtx.stroke(); }
    for (let y=0; y<=rows; y++){ overlayCtx.beginPath(); overlayCtx.moveTo(0,y*zoom+0.5); overlayCtx.lineTo(cols*zoom, y*zoom+0.5); overlayCtx.stroke(); }
    overlayCtx.restore();
  }

  function drawHoverCell(r,c) {
    overlayCtx.save();
    overlayCtx.strokeStyle='rgba(91,156,255,0.9)';
    overlayCtx.lineWidth=2;
    overlayCtx.strokeRect(c*zoom+1, r*zoom+1, zoom-2, zoom-2);
    overlayCtx.restore();
  }

  function invIndex(rVis, cVis) {
    // invert visual transforms back to RAW indices
    const k = ((parseInt(rot.value,10)||0)%4+4)%4;
    let r=rVis, c=cVis, R=rows, C=cols;
    if (k===1){ const nr=C, nc=R; const r2=c; const c2=(nc-1)-r; r=r2; c=c2; R=nr; C=nc; }
    else if (k===2){ r=(R-1)-r; c=(C-1)-c; }
    else if (k===3){ const nr=C, nc=R; const r2=(nr-1)-c; const c2=r; r=r2; c=c2; R=nr; C=nc; }
    if (document.getElementById('flipUD').checked){ r = (rawRows-1) - r; }
    if (document.getElementById('flipLR').checked){ c = (rawCols-1) - c; }
    return [r, c];
  }

  function metersFromRaw(rcRaw) {
    const res = parseFloat(resEl.value);
    const [rRaw, cRaw] = rcRaw;
    const mode = coordMode.value;
    if (mode === 'astar') {
      const x = (cRaw - Math.floor(rawCols/2)) * res;
      const y = (rRaw - Math.floor(rawRows/2)) * res;
      return [x, y];
    } else {
      const xMin=parseFloat(x_min_el.value), xMax=parseFloat(x_max_el.value);
      const yMin=parseFloat(y_min_el.value), yMax=parseFloat(y_max_el.value);
      const cellW = (xMax - xMin) / rawCols;
      const cellH = (yMax - yMin) / rawRows;
      const xw = xMin + (cRaw + 0.5) * cellW;
      const yw = yMax - (rRaw + 0.5) * cellH;
      return [xw, yw];
    }
  }

  function rawFromMeters(x, y) {
    const res = parseFloat(resEl.value);
    const mode = coordMode.value;
      if (mode === 'astar') {
        const cx = Math.trunc(rawCols/2 + x/res);
        const cy = Math.trunc(rawRows/2 + y/res);
        return [cx, cy]; // returns (cRaw, rRaw)
    } else {
      const xMin=parseFloat(x_min_el.value), xMax=parseFloat(x_max_el.value);
      const yMin=parseFloat(y_min_el.value), yMax=parseFloat(y_max_el.value);
      const cellW = (xMax - xMin) / rawCols;
      const cellH = (yMax - yMin) / rawRows;
      const cRaw = Math.floor((x - xMin) / cellW);
      const rRaw = Math.floor((yMax - y) / cellH);
      return [rRaw, cRaw];
    }
  }

  function placeHL(cx, cy) {
    // highlight raw (cx,cy) cell in current visual transform
    // Convert raw->vis using forward transforms:
    let r = cy, c = cx; // careful: raw index pair is (row, col) => (r,c). User passes cx,cy; we need r=cy, c=cx
    // Actually we want to highlight by raw indices (rRaw,cRaw):
    r = cy; c = cx;
    // Apply flips
    if (flipLR.checked){ c = (rawCols-1) - c; }
    if (flipUD.checked){ r = (rawRows-1) - r; }
    // Apply rotation
    const k = ((parseInt(rot.value,10)||0)%4+4)%4;
    if (k===1){ const r2=c; const c2=(rawRows-1)-r; r=r2; c=c2; }
    else if (k===2){ r=(rawRows-1)-r; c=(rawCols-1)-c; }
    else if (k===3){ const r2=(rawCols-1)-c; const c2=r; r=r2; c=c2; }

    // draw rectangle
    hl.style.display='block';
    hl.style.left = (c*zoom+1)+'px';
    hl.style.top  = (r*zoom+1)+'px';
    hl.style.width  = (zoom-2)+'px';
    hl.style.height = (zoom-2)+'px';
  }

  function clearHighlights(){
    hl.style.display='none';
    overlayCtx.clearRect(0,0,overlay.width,overlay.height);
    drawGridlines();
  }

  overlay.addEventListener('mousemove', (e) => {
    const rect = overlay.getBoundingClientRect();
    const x = e.clientX - rect.left;
    const y = e.clientY - rect.top;
    const cVis = Math.floor(x / zoom);
    const rVis = Math.floor(y / zoom);

    if (rVis<0 || rVis>=rows || cVis<0 || cVis>=cols) {
      hover.style.display='none';
      overlayCtx.clearRect(0,0,overlay.width,overlay.height);
      drawGridlines();
      return;
    }

    overlayCtx.clearRect(0,0,overlay.width,overlay.height);
    drawGridlines();
    drawHoverCell(rVis, cVis);

    const [rRaw, cRaw] = invIndex(rVis, cVis);
    const val = RAW[keySel.value][rRaw][cRaw];

    const [xw, yw] = metersFromRaw([rRaw, cRaw]);
    hover.style.display='block';
    hover.innerHTML = `(row,col) vis=(${rVis},${cVis}) → raw=(${rRaw},${cRaw})<br/>value=${val}<br/>x=${xw.toFixed(3)} m, y=${yw.toFixed(3)} m`;

    let hx=x, hy=y;
    if (document.getElementById('snap').checked){ hx=(cVis+0.5)*zoom; hy=(rVis+0.5)*zoom; }
    hover.style.left = hx+'px';
    hover.style.top  = hy+'px';
  });

  overlay.addEventListener('mouseleave', () => {
    hover.style.display='none';
    overlayCtx.clearRect(0,0,overlay.width,overlay.height);
    drawGridlines();
  });

  coordMode.addEventListener('input', () => {
    edgesBox.style.display = coordMode.value === 'edges' ? '' : 'none';
    drawBase();
  });

  [keySel, zoomEl, flipLR, flipUD, rot, resEl, x_min_el, x_max_el, y_min_el, y_max_el, showGridEl].forEach(el => {
    el.addEventListener('input', () => {
      zoom = parseInt(zoomEl.value,10);
      drawBase();
    });
  });

  btnJump.addEventListener('click', () => {
    const cx = parseInt(jump_cx.value,10);
    const cy = parseInt(jump_cy.value,10);
    if (isNaN(cx) || isNaN(cy)) return;
    placeHL(cx, cy);
  });

  btnConvert.addEventListener('click', () => {
    const x = parseFloat(mx.value), y = parseFloat(my.value);
    if (isNaN(x) || isNaN(y)) return;
    const [rRaw, cRaw] = rawFromMeters(x, y);
    convOut.textContent = `Meters (${x.toFixed(3)}, ${y.toFixed(3)}) → cell raw=(${rRaw}, ${cRaw})`;
    placeHL(cRaw, rRaw);
  });

  btnParseLog.addEventListener('click', () => {
    const text = logBox.value || "";
    const re = /Grid:\s*\((\d+)\s*,\s*(\d+)\)/g;
    clearHighlights();
    // We'll draw lightweight highlights on overlay for each match
    overlayCtx.save();
    overlayCtx.strokeStyle='rgba(255, 215, 0, 0.9)';
    overlayCtx.lineWidth=2;
    let m;
    while ((m = re.exec(text)) !== null) {
      const cx = parseInt(m[1],10);
      const cy = parseInt(m[2],10);
      // convert raw (r=cy, c=cx) to current visual pos
      let r = cy, c = cx;
      if (flipLR.checked){ c = (rawCols-1) - c; }
      if (flipUD.checked){ r = (rawRows-1) - r; }
      const k = ((parseInt(rot.value,10)||0)%4+4)%4;
      if (k===1){ const r2=c; const c2=(rawRows-1)-r; r=r2; c=c2; }
      else if (k===2){ r=(rawRows-1)-r; c=(rawCols-1)-c; }
      else if (k===3){ const r2=(rawCols-1)-c; const c2=r; r=r2; c=c2; }
      overlayCtx.strokeRect(c*zoom+1, r*zoom+1, zoom-2, zoom-2);
    }
    overlayCtx.restore();
  });

  btnClearHL.addEventListener('click', clearHighlights);

  // Init
  (function init(){
    Object.keys(RAW).forEach((k,i)=>{ if(/costmap/i.test(k)) keySel.value=k; });
    drawBase();
  })();
</script>
</body>
</html>
"""

html = template.replace("__DATA_JSON__", json.dumps(serializable)).replace("__META_JSON__", json.dumps(meta)).replace("__RES__", f"{res_val:.4f}")

out_html = Path("sim_app/npz_grid_viewer_astar.html")
out_html.parent.mkdir(parents=True, exist_ok=True)
out_html.write_text(html, encoding="utf-8")

print({"html_file": str(out_html), "note": "Defaults to A* center-origin mapping; also supports world-edges mode; includes jump/convert and path-highlighting."})
