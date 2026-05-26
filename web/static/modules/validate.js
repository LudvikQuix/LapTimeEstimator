// Validate tab: sim vs real lap overlay.

import {
  apiPost, fillSelect, loadEnvLists, fetchCsv, columnFloat,
} from "/static/modules/common.js";

let _booted = false;

window.addEventListener("tab-shown", (e) => {
  if (e.detail.name === "validate" && !_booted) { boot(); _booted = true; }
});

async function boot() {
  const panel = document.getElementById("tab-validate");
  panel.innerHTML = `
    <h2 class="text-xl font-semibold mb-4">Validate sim vs real lap</h2>
    <div class="grid grid-cols-1 md:grid-cols-3 gap-4 mb-4 text-sm">
      <label class="flex flex-col">Car
        <select id="va-car" class="mt-1"></select>
      </label>
      <label class="flex flex-col">Track
        <select id="va-track" class="mt-1"></select>
      </label>
      <label class="flex flex-col">Layout
        <select id="va-layout" class="mt-1"></select>
      </label>
      <label class="flex flex-col">Driver
        <select id="va-driver" class="mt-1"></select>
      </label>
      <label class="flex flex-col">Compound
        <select id="va-compound" class="mt-1"></select>
      </label>
      <label class="flex flex-col">Real-lap path (samples/aclog/...)
        <input id="va-real-path" type="text" placeholder="samples/aclog/Tomas_Lap2.csv" class="mt-1" />
      </label>
      <label class="flex flex-col">Bin (m)
        <input id="va-bin" type="number" min="10" max="500" value="100" class="mt-1" />
      </label>
      <label class="flex items-center gap-2"><input id="va-per-corner" type="checkbox" /> Per-corner</label>
    </div>
    <button id="va-run" class="primary">Run validate</button>
    <span id="va-status" class="ml-3 text-sm text-slate-400"></span>
    <div id="va-summary" class="my-3 text-sm"></div>
    <div id="va-plot" style="height:600px;"></div>
  `;
  const {cars, tracks, drivers} = await loadEnvLists();
  fillSelect(document.getElementById("va-car"),
    cars.map(c => ({value: c.name, label: c.name})));
  fillSelect(document.getElementById("va-track"),
    tracks.map(t => ({value: t.track, label: t.track})));
  fillSelect(document.getElementById("va-driver"),
    drivers.map(d => ({value: d.name, label: d.name})));
  function refreshLayouts() {
    const t = document.getElementById("va-track").value;
    const layouts = (tracks.find(x => x.track === t) || {}).layouts || [];
    fillSelect(document.getElementById("va-layout"),
      layouts.map(l => ({value: `${t}/${l.full}`, label: l.name})));
  }
  function refreshCompounds() {
    const carName = document.getElementById("va-car").value;
    const car = cars.find(c => c.name === carName);
    fillSelect(document.getElementById("va-compound"),
      (car?.compounds || []).map(c => ({value: c.name, label: c.name})),
      {placeholder: "(default)"});
  }
  document.getElementById("va-track").addEventListener("change", refreshLayouts);
  document.getElementById("va-car").addEventListener("change", refreshCompounds);
  refreshLayouts(); refreshCompounds();
  document.getElementById("va-run").addEventListener("click", run);
}

async function run() {
  const btn = document.getElementById("va-run");
  const status = document.getElementById("va-status");
  btn.disabled = true;
  status.textContent = "running...";
  document.getElementById("va-summary").innerHTML = "";
  document.getElementById("va-plot").innerHTML = "";
  const body = {
    car: document.getElementById("va-car").value,
    track: document.getElementById("va-layout").value,
    driver: document.getElementById("va-driver").value,
    compound: document.getElementById("va-compound").value || null,
    real_source: {
      kind: "local",
      path: document.getElementById("va-real-path").value.trim(),
    },
    bin_m: parseInt(document.getElementById("va-bin").value, 10),
    per_corner: document.getElementById("va-per-corner").checked,
  };
  try {
    const r = await apiPost("/api/validate", body);
    const colour = r.verdict === "GOOD" ? "bg-emerald-900" :
                   r.verdict === "LOOSE" ? "bg-amber-800" : "bg-rose-900";
    document.getElementById("va-summary").innerHTML = `
      <span class="px-2 py-0.5 rounded ${colour}">${r.verdict}</span>
      real: <strong>${r.real_lap_time_s}s</strong> |
      sim: <strong>${r.sim_lap_time_s}s</strong> |
      delta: <strong>${r.delta_s}s (${r.delta_pct}%)</strong>
    `;
    status.textContent = "done.";

    const {rows} = await fetchCsv(r.validation_bins_csv_url);
    const start = columnFloat(rows, "bin_start_m");
    const dlt = columnFloat(rows, "delta_s");
    const vsim = columnFloat(rows, "v_avg_sim_kmh");
    const vreal = columnFloat(rows, "v_avg_real_kmh");
    Plotly.newPlot("va-plot", [
      {x: start, y: vsim, name: "sim km/h", xaxis: "x1", yaxis: "y1", mode: "lines"},
      {x: start, y: vreal, name: "real km/h", xaxis: "x1", yaxis: "y1", mode: "lines"},
      {x: start, y: dlt, name: "delta s", xaxis: "x1", yaxis: "y2", type: "bar"},
    ], {
      grid: {rows: 2, columns: 1, pattern: "independent"},
      template: "plotly_dark",
      paper_bgcolor: "#0f172a", plot_bgcolor: "#0f172a",
      font: {color: "#e2e8f0"},
      yaxis:  {domain: [0.5, 1.0], title: "km/h"},
      yaxis2: {domain: [0.0, 0.42], title: "delta (s)"},
      xaxis:  {title: "distance (m)"},
      height: 600, margin: {t: 30, l: 60, r: 20, b: 50},
    }, {responsive: true});
  } catch (e) {
    status.textContent = `error: ${e.message}`;
  } finally {
    btn.disabled = false;
  }
}
