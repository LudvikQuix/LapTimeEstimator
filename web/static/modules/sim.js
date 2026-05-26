// Sim tab: car/track/driver picker -> POST /api/sim -> stacked Plotly subplots.

import {
  apiGet, apiPost, fillSelect, columnFloat, columnInt, fetchCsv, loadEnvLists,
} from "/static/modules/common.js";

let _booted = false;

window.addEventListener("tab-shown", (e) => {
  if (e.detail.name === "sim" && !_booted) {
    boot();
    _booted = true;
  }
});

async function boot() {
  const panel = document.getElementById("tab-sim");
  panel.innerHTML = `
    <h2 class="text-xl font-semibold mb-4">Simulate a stint</h2>
    <div class="grid grid-cols-1 md:grid-cols-3 gap-4 mb-4">
      <label class="flex flex-col text-sm">Car
        <select id="sim-car" class="mt-1"></select>
      </label>
      <label class="flex flex-col text-sm">Track
        <select id="sim-track" class="mt-1"></select>
      </label>
      <label class="flex flex-col text-sm">Layout
        <select id="sim-layout" class="mt-1"></select>
      </label>
      <label class="flex flex-col text-sm">Driver
        <select id="sim-driver" class="mt-1"></select>
      </label>
      <label class="flex flex-col text-sm">Compound
        <select id="sim-compound" class="mt-1"></select>
      </label>
      <label class="flex flex-col text-sm">Model
        <select id="sim-model" class="mt-1">
          <option value="point-mass" selected>Point-mass (v2 — default, fast)</option>
          <option value="slip" title="Phase 4: preview-target slip-aware driver + RK4 ODE + per-wheel Pacejka. Slower wall-clock than v2; honest physics.">Slip-based dynamics (v3 — Phase 4, slip-aware driver)</option>
        </select>
      </label>
      <label class="flex flex-col text-sm">Laps
        <input id="sim-laps" type="number" min="1" max="50" value="2" class="mt-1" />
      </label>
      <label class="flex flex-col text-sm">FL psi
        <input id="sim-fl" type="number" step="0.1" value="33" class="mt-1" />
      </label>
      <label class="flex flex-col text-sm">FR psi
        <input id="sim-fr" type="number" step="0.1" value="33" class="mt-1" />
      </label>
      <label class="flex flex-col text-sm">RL psi
        <input id="sim-rl" type="number" step="0.1" value="34" class="mt-1" />
      </label>
      <label class="flex flex-col text-sm">RR psi
        <input id="sim-rr" type="number" step="0.1" value="34" class="mt-1" />
      </label>
      <label class="flex flex-col text-sm">Ambient C
        <input id="sim-amb" type="number" step="0.5" value="26" class="mt-1" />
      </label>
      <label class="flex flex-col text-sm">ds (m)
        <input id="sim-ds" type="number" step="0.5" value="2.0" class="mt-1" />
      </label>
    </div>
    <div class="flex items-center gap-3 mb-4">
      <button id="sim-run" class="primary">Run sim</button>
      <span id="sim-status" class="text-sm text-slate-400"></span>
    </div>
    <div id="sim-summary" class="mb-4 text-sm text-slate-300"></div>
    <div id="sim-lap-pick-wrap" class="mb-2 hidden">
      <label class="text-sm">Show lap:
        <select id="sim-lap-pick" class="ml-2"></select>
      </label>
    </div>
    <div id="sim-plot" class="w-full" style="height:780px;"></div>
    <h3 class="text-lg font-semibold mt-6 mb-2">Per-lap stint summary</h3>
    <div id="sim-stint-table"></div>
  `;

  const {cars, tracks, drivers} = await loadEnvLists();
  fillSelect(document.getElementById("sim-car"),
    cars.map(c => ({value: c.name, label: c.name})));
  fillSelect(document.getElementById("sim-track"),
    tracks.map(t => ({value: t.track, label: t.track})));
  fillSelect(document.getElementById("sim-driver"),
    drivers.map(d => ({value: d.name, label: `${d.name}  (skill=${d.skill_pct?.toFixed?.(2) ?? "?"})`})));

  function refreshLayouts() {
    const t = document.getElementById("sim-track").value;
    const layouts = (tracks.find(x => x.track === t) || {}).layouts || [];
    fillSelect(document.getElementById("sim-layout"),
      layouts.map(l => ({value: `${t}/${l.full}`, label: l.name})));
  }
  function refreshCompounds() {
    const carName = document.getElementById("sim-car").value;
    const car = cars.find(c => c.name === carName);
    const comps = (car && car.compounds) || [];
    fillSelect(document.getElementById("sim-compound"),
      comps.map(c => ({value: c.name, label: `${c.name} (idx ${c.index})`})),
      {placeholder: "(default)"});
  }
  document.getElementById("sim-track").addEventListener("change", refreshLayouts);
  document.getElementById("sim-car").addEventListener("change", refreshCompounds);
  refreshLayouts();
  refreshCompounds();

  document.getElementById("sim-run").addEventListener("click", runSim);
}

async function runSim() {
  const btn = document.getElementById("sim-run");
  const status = document.getElementById("sim-status");
  btn.disabled = true;
  status.textContent = "Running sim...";
  document.getElementById("sim-summary").innerHTML = "";
  document.getElementById("sim-plot").innerHTML = "";
  document.getElementById("sim-stint-table").innerHTML = "";

  const body = {
    car: document.getElementById("sim-car").value,
    track: document.getElementById("sim-layout").value,
    driver: document.getElementById("sim-driver").value,
    compound: document.getElementById("sim-compound").value || null,
    pressures_psi: {
      FL: parseFloat(document.getElementById("sim-fl").value),
      FR: parseFloat(document.getElementById("sim-fr").value),
      RL: parseFloat(document.getElementById("sim-rl").value),
      RR: parseFloat(document.getElementById("sim-rr").value),
    },
    ambient_temp_c: parseFloat(document.getElementById("sim-amb").value),
    n_laps: parseInt(document.getElementById("sim-laps").value, 10),
    ds: parseFloat(document.getElementById("sim-ds").value),
    model: document.getElementById("sim-model").value,
  };
  try {
    const t0 = performance.now();
    const resp = await apiPost("/api/sim", body);
    const elapsed = ((performance.now() - t0) / 1000).toFixed(2);
    status.textContent = `Done in ${elapsed}s.`;
    renderSummary(resp);
    await renderTelemetry(resp);
    await renderStintTable(resp);
  } catch (e) {
    status.textContent = `Error: ${e.message}`;
    console.error(e);
  } finally {
    btn.disabled = false;
  }
}

function renderSummary(resp) {
  const el = document.getElementById("sim-summary");
  const times = resp.lap_times_s.map((t, i) => `L${i + 1}: ${t.toFixed(3)}s`).join("  |  ");
  // v3 Phase 4: util_p85 is the headline slip-utilisation metric (spec
  // §11.55). When present and >0, surface it under the lap times.
  let slipBlock = "";
  if (typeof resp.util_p85 === "number" && resp.util_p85 > 0) {
    const slipDeg = (typeof resp.slip_target_deg === "number") ?
      `, target ${resp.slip_target_deg.toFixed(2)} deg` : "";
    const verdict = resp.util_p85 <= 1.0 ? " (within tyre peak)" :
      ` (${((resp.util_p85 - 1) * 100).toFixed(0)}% over peak)`;
    slipBlock = `<div><strong>util_p85:</strong> ${resp.util_p85.toFixed(3)}${slipDeg}${verdict}</div>`;
  }
  let mcBlock = "";
  if (typeof resp.mc_n_runs === "number" && resp.mc_n_runs > 1) {
    mcBlock = `<div><strong>MC:</strong> ${resp.mc_n_runs} runs, sigma=${(resp.mc_sigma_s ?? 0).toFixed(3)}s</div>`;
  }
  el.innerHTML = `
    <div><strong>Lap times:</strong> ${times}</div>
    ${slipBlock}
    ${mcBlock}
    <div><strong>Compound:</strong> ${resp.compound_resolved} (${resp.compound_source})</div>
    <div><strong>Setup:</strong> ${resp.setup_line || "—"}</div>
  `;
}

async function renderTelemetry(resp) {
  const {rows} = await fetchCsv(resp.telemetry_csv_url);
  if (!rows.length) {
    document.getElementById("sim-plot").textContent = "(no telemetry rows)";
    document.getElementById("sim-lap-pick-wrap").classList.add("hidden");
    return;
  }
  const cols = {
    dist:  columnFloat(rows, "distanceTraveled"),
    gas:   columnFloat(rows, "gas"),
    brake: columnFloat(rows, "brake"),
    speed: columnFloat(rows, "speedKmh"),
    lap:   columnInt(rows, "lap"),
    tFL: columnFloat(rows, "tempFL"),
    tFR: columnFloat(rows, "tempFR"),
    tRL: columnFloat(rows, "tempRL"),
    tRR: columnFloat(rows, "tempRR"),
    wFL: columnFloat(rows, "wearFL"),
    wFR: columnFloat(rows, "wearFR"),
    wRL: columnFloat(rows, "wearRL"),
    wRR: columnFloat(rows, "wearRR"),
    pFL: columnFloat(rows, "pressureFL"),
    pFR: columnFloat(rows, "pressureFR"),
    pRL: columnFloat(rows, "pressureRL"),
    pRR: columnFloat(rows, "pressureRR"),
  };

  // Build the lap dropdown from distinct values in the `lap` column.
  const lapWrap = document.getElementById("sim-lap-pick-wrap");
  const lapSel = document.getElementById("sim-lap-pick");
  const distinct = [...new Set(cols.lap)].filter(v => Number.isFinite(v)).sort((a, b) => a - b);
  if (distinct.length > 1) {
    lapSel.innerHTML = distinct.map(v => `<option value="${v}">Lap ${v}</option>`).join("");
    lapSel.value = String(distinct[distinct.length - 1]);  // default: last lap (flying)
    lapWrap.classList.remove("hidden");
    lapSel.onchange = () => drawForLap(cols, parseInt(lapSel.value, 10));
  } else {
    lapWrap.classList.add("hidden");
  }
  const initial = distinct.length > 1 ? distinct[distinct.length - 1] : (distinct[0] ?? null);
  drawForLap(cols, initial);
}

function drawForLap(cols, lapValue) {
  // Indices to keep; if lapValue is null, keep everything.
  const keep = (cols.lap || []).map((v, i) => (lapValue == null || v === lapValue) ? i : -1).filter(i => i >= 0);
  const pick = arr => keep.map(i => arr[i]);
  const traces = [
    {x: pick(cols.dist), y: pick(cols.speed), name: "speed (km/h)", xaxis: "x1", yaxis: "y1", mode: "lines"},
    {x: pick(cols.dist), y: pick(cols.gas),   name: "gas",   xaxis: "x1", yaxis: "y2", mode: "lines"},
    {x: pick(cols.dist), y: pick(cols.brake), name: "brake", xaxis: "x1", yaxis: "y2", mode: "lines"},
    {x: pick(cols.dist), y: pick(cols.tFL), name: "tempFL", xaxis: "x1", yaxis: "y3", mode: "lines"},
    {x: pick(cols.dist), y: pick(cols.tFR), name: "tempFR", xaxis: "x1", yaxis: "y3", mode: "lines"},
    {x: pick(cols.dist), y: pick(cols.tRL), name: "tempRL", xaxis: "x1", yaxis: "y3", mode: "lines"},
    {x: pick(cols.dist), y: pick(cols.tRR), name: "tempRR", xaxis: "x1", yaxis: "y3", mode: "lines"},
    {x: pick(cols.dist), y: pick(cols.wFL), name: "wearFL", xaxis: "x1", yaxis: "y4", mode: "lines"},
    {x: pick(cols.dist), y: pick(cols.wFR), name: "wearFR", xaxis: "x1", yaxis: "y4", mode: "lines"},
    {x: pick(cols.dist), y: pick(cols.wRL), name: "wearRL", xaxis: "x1", yaxis: "y4", mode: "lines"},
    {x: pick(cols.dist), y: pick(cols.wRR), name: "wearRR", xaxis: "x1", yaxis: "y4", mode: "lines"},
    {x: pick(cols.dist), y: pick(cols.pFL), name: "pressFL", xaxis: "x1", yaxis: "y5", mode: "lines"},
    {x: pick(cols.dist), y: pick(cols.pFR), name: "pressFR", xaxis: "x1", yaxis: "y5", mode: "lines"},
    {x: pick(cols.dist), y: pick(cols.pRL), name: "pressRL", xaxis: "x1", yaxis: "y5", mode: "lines"},
    {x: pick(cols.dist), y: pick(cols.pRR), name: "pressRR", xaxis: "x1", yaxis: "y5", mode: "lines"},
  ];
  const layout = {
    grid: {rows: 5, columns: 1, pattern: "independent"},
    template: "plotly_dark",
    paper_bgcolor: "#0f172a",
    plot_bgcolor: "#0f172a",
    font: {color: "#e2e8f0"},
    margin: {t: 30, l: 60, r: 20, b: 50},
    height: 780,
    xaxis: {anchor: "y1", title: "distance (m)"},
    yaxis:  {domain: [0.82, 1.00], title: "km/h"},
    yaxis2: {domain: [0.64, 0.80], title: "throttle / brake", range: [0, 1.05]},
    yaxis3: {domain: [0.46, 0.62], title: "temp (C)"},
    yaxis4: {domain: [0.28, 0.44], title: "wear (%)"},
    yaxis5: {domain: [0.00, 0.26], title: "pressure (psi)"},
    showlegend: true,
    legend: {orientation: "h", y: -0.05},
  };
  Plotly.newPlot("sim-plot", traces, layout, {responsive: true});
}

async function renderStintTable(resp) {
  const {rows, columns} = await fetchCsv(resp.stint_summary_csv_url);
  const wrap = document.getElementById("sim-stint-table");
  if (!rows.length) {
    wrap.textContent = "(no stint summary rows)";
    return;
  }
  let html = `<table class="grid"><thead><tr>`;
  for (const c of columns) html += `<th>${c}</th>`;
  html += `</tr></thead><tbody>`;
  for (const r of rows) {
    html += `<tr>`;
    for (const c of columns) html += `<td>${r[c] ?? ""}</td>`;
    html += `</tr>`;
  }
  html += `</tbody></table>`;
  wrap.innerHTML = html;
}
