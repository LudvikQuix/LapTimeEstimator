// Shared helpers: API fetch, SSE, tab switching, health probe.

export async function apiGet(path) {
  const r = await fetch(path);
  if (!r.ok) throw new Error(`GET ${path} -> ${r.status}: ${await r.text()}`);
  return r.json();
}

export async function apiPost(path, body) {
  const r = await fetch(path, {
    method: "POST",
    headers: {"Content-Type": "application/json"},
    body: JSON.stringify(body),
  });
  if (!r.ok) throw new Error(`POST ${path} -> ${r.status}: ${await r.text()}`);
  return r.json();
}

// Open an SSE-style stream from a POST endpoint.
// `onEvent({type, data})` is called per parsed event; `onError(err)` on
// transport / parse failures. Returns a function that aborts the stream.
export function openSSEPost(path, body, onEvent, onError) {
  const ctrl = new AbortController();
  (async () => {
    try {
      const resp = await fetch(path, {
        method: "POST",
        headers: {"Content-Type": "application/json", "Accept": "text/event-stream"},
        body: JSON.stringify(body),
        signal: ctrl.signal,
      });
      if (!resp.ok) {
        throw new Error(`POST ${path} -> ${resp.status}: ${await resp.text()}`);
      }
      const reader = resp.body.getReader();
      const decoder = new TextDecoder("utf-8");
      let buf = "";
      while (true) {
        const {value, done} = await reader.read();
        if (done) break;
        buf += decoder.decode(value, {stream: true});
        let idx;
        while ((idx = buf.indexOf("\n\n")) >= 0) {
          const block = buf.slice(0, idx);
          buf = buf.slice(idx + 2);
          parseSSEBlock(block, onEvent);
        }
      }
    } catch (e) {
      if (e.name !== "AbortError") onError(e);
    }
  })();
  return () => ctrl.abort();
}

function parseSSEBlock(block, onEvent) {
  let event = "message";
  let data = "";
  for (const raw of block.split("\n")) {
    const line = raw.trim();
    if (!line) continue;
    if (line.startsWith(":")) continue; // keep-alive comment
    if (line.startsWith("event:")) event = line.slice(6).trim();
    else if (line.startsWith("data:")) data += line.slice(5).trim();
  }
  if (!data) return;
  let parsed;
  try { parsed = JSON.parse(data); } catch { parsed = {raw: data}; }
  onEvent({type: event, data: parsed});
}

// --- tab switcher ---
function showTab(name) {
  document.querySelectorAll(".tab-panel").forEach((p) => p.classList.add("hidden"));
  const panel = document.getElementById(`tab-${name}`);
  if (panel) panel.classList.remove("hidden");
  document.querySelectorAll(".tab-btn").forEach((b) => {
    b.classList.remove("bg-slate-700", "active");
    if (b.dataset.tab === name) b.classList.add("bg-slate-700", "active");
  });
  window.dispatchEvent(new CustomEvent("tab-shown", {detail: {name}}));
}

document.addEventListener("DOMContentLoaded", () => {
  document.querySelectorAll(".tab-btn").forEach((b) => {
    b.addEventListener("click", () => showTab(b.dataset.tab));
  });
  showTab("sim");
  refreshLakeBadge();
  setInterval(refreshLakeBadge, 30_000);
});

async function refreshLakeBadge() {
  const el = document.getElementById("lake-state");
  if (!el) return;
  try {
    const h = await apiGet("/api/health");
    el.textContent = h.lake;
    el.className = "px-2 py-0.5 rounded text-xs " + (
      h.lake === "arrow" ? "bg-emerald-700" :
      h.lake === "csv"   ? "bg-amber-700"   :
                           "bg-rose-800"
    );
  } catch {
    el.textContent = "offline";
    el.className = "px-2 py-0.5 rounded text-xs bg-rose-800";
  }
}

// Small util: load cars + tracks + drivers in parallel for tab pickers.
export async function loadEnvLists() {
  const [cars, tracks, drivers] = await Promise.all([
    apiGet("/api/cars").catch(() => []),
    apiGet("/api/tracks").catch(() => []),
    apiGet("/api/drivers").catch(() => []),
  ]);
  return {cars, tracks, drivers};
}

// Populate a <select> from a list of {value, label} entries.
export function fillSelect(sel, items, {placeholder = null} = {}) {
  sel.innerHTML = "";
  if (placeholder !== null) {
    const o = document.createElement("option");
    o.value = "";
    o.textContent = placeholder;
    sel.appendChild(o);
  }
  for (const it of items) {
    const o = document.createElement("option");
    o.value = it.value;
    o.textContent = it.label;
    sel.appendChild(o);
  }
}

// Render a small "stage chip strip" for SSE progress.
export function renderStageStrip(parent, stages) {
  parent.innerHTML = "";
  for (const s of stages) {
    const span = document.createElement("span");
    span.className = "stage-chip " + (s.state || "");
    span.textContent = s.label;
    parent.appendChild(span);
  }
}

// Fetch CSV and parse with naive splitter (Plotly accepts arrays directly).
export async function fetchCsv(url) {
  const r = await fetch(url);
  if (!r.ok) throw new Error(`CSV fetch failed: ${r.status} ${await r.text()}`);
  const text = await r.text();
  return parseCsv(text);
}

function parseCsv(text) {
  const lines = text.split(/\r?\n/).filter(Boolean);
  if (!lines.length) return {columns: [], rows: []};
  const header = lines[0].split(",");
  const rows = lines.slice(1).map((line) => {
    const cells = line.split(",");
    const obj = {};
    header.forEach((h, i) => obj[h] = cells[i]);
    return obj;
  });
  return {columns: header, rows};
}

export function columnFloat(rows, key) {
  return rows.map(r => {
    const v = parseFloat(r[key]);
    return Number.isFinite(v) ? v : null;
  });
}

export function columnInt(rows, key) {
  return rows.map(r => {
    const v = parseInt(r[key], 10);
    return Number.isFinite(v) ? v : null;
  });
}
