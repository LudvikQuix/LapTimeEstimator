// Stint solver tab.

import {
  apiPost, fillSelect, loadEnvLists,
} from "/static/modules/common.js";

let _booted = false;

window.addEventListener("tab-shown", (e) => {
  if (e.detail.name === "stint" && !_booted) { boot(); _booted = true; }
});

async function boot() {
  const panel = document.getElementById("tab-stint");
  panel.innerHTML = `
    <h2 class="text-xl font-semibold mb-4">Inverse PSI solver</h2>
    <div class="grid grid-cols-1 md:grid-cols-3 gap-4 mb-4 text-sm">
      <label class="flex flex-col">Car
        <select id="st-car" class="mt-1"></select>
      </label>
      <label class="flex flex-col">Track
        <select id="st-track" class="mt-1"></select>
      </label>
      <label class="flex flex-col">Layout
        <select id="st-layout" class="mt-1"></select>
      </label>
      <label class="flex flex-col">Driver
        <select id="st-driver" class="mt-1"></select>
      </label>
      <label class="flex flex-col">Compound
        <select id="st-compound" class="mt-1"></select>
      </label>
      <label class="flex flex-col">Target wheel
        <select id="st-target-wheel" class="mt-1">
          <option>max</option><option>min</option><option>avg</option>
          <option>FL</option><option>FR</option><option>RL</option><option>RR</option>
        </select>
      </label>
      <label class="flex flex-col">Target wear (0..1)
        <input id="st-target-wear" type="number" step="0.05" min="0" max="1" value="0.50" class="mt-1" />
      </label>
      <label class="flex flex-col">Target lap
        <input id="st-target-lap" type="number" min="2" max="50" value="10" class="mt-1" />
      </label>
      <label class="flex flex-col">Ambient C
        <input id="st-amb" type="number" step="0.5" value="26" class="mt-1" />
      </label>
      <label class="flex items-center text-sm gap-2 md:col-span-3">
        <input id="st-uniform" type="checkbox" /> Uniform pressure (single PSI for all 4 wheels)
      </label>
    </div>
    <button id="st-run" class="primary">Run solver</button>
    <span id="st-status" class="ml-3 text-sm text-slate-400"></span>
    <div id="st-result" class="mt-4"></div>
  `;
  const {cars, tracks, drivers} = await loadEnvLists();
  fillSelect(document.getElementById("st-car"),
    cars.map(c => ({value: c.name, label: c.name})));
  fillSelect(document.getElementById("st-track"),
    tracks.map(t => ({value: t.track, label: t.track})));
  fillSelect(document.getElementById("st-driver"),
    drivers.map(d => ({value: d.name, label: d.name})));
  function refreshLayouts() {
    const t = document.getElementById("st-track").value;
    const layouts = (tracks.find(x => x.track === t) || {}).layouts || [];
    fillSelect(document.getElementById("st-layout"),
      layouts.map(l => ({value: `${t}/${l.full}`, label: l.name})));
  }
  function refreshCompounds() {
    const carName = document.getElementById("st-car").value;
    const car = cars.find(c => c.name === carName);
    fillSelect(document.getElementById("st-compound"),
      (car?.compounds || []).map(c => ({value: c.name, label: c.name})),
      {placeholder: "(default)"});
  }
  document.getElementById("st-track").addEventListener("change", refreshLayouts);
  document.getElementById("st-car").addEventListener("change", refreshCompounds);
  refreshLayouts(); refreshCompounds();

  document.getElementById("st-run").addEventListener("click", run);
}

async function run() {
  const btn = document.getElementById("st-run");
  const status = document.getElementById("st-status");
  const out = document.getElementById("st-result");
  btn.disabled = true;
  status.textContent = "solving...";
  out.innerHTML = "";

  const body = {
    car: document.getElementById("st-car").value,
    track: document.getElementById("st-layout").value,
    driver: document.getElementById("st-driver").value,
    compound: document.getElementById("st-compound").value || null,
    target_wear: parseFloat(document.getElementById("st-target-wear").value),
    target_lap: parseInt(document.getElementById("st-target-lap").value, 10),
    target_wheel: document.getElementById("st-target-wheel").value,
    uniform_pressure: document.getElementById("st-uniform").checked,
    ambient_temp_c: parseFloat(document.getElementById("st-amb").value),
  };
  try {
    const r = await apiPost("/api/solve", body);
    status.textContent = r.converged ? "converged" : "did not converge";
    const rows = r.verification_stint;
    let html = `
      <div class="grid grid-cols-2 md:grid-cols-4 gap-4 my-3 text-sm">
        <div>FL: <strong>${r.recommended_pressures_psi.FL.toFixed(2)} psi</strong></div>
        <div>FR: <strong>${r.recommended_pressures_psi.FR.toFixed(2)} psi</strong></div>
        <div>RL: <strong>${r.recommended_pressures_psi.RL.toFixed(2)} psi</strong></div>
        <div>RR: <strong>${r.recommended_pressures_psi.RR.toFixed(2)} psi</strong></div>
      </div>
      <div class="text-sm mb-2">target wheel: ${r.target_wheel_resolved} | observed wear: ${(r.observed_wear*100).toFixed(1)}% | iters: ${r.iterations}</div>
      <table class="grid"><thead><tr><th>Lap</th><th>Time (s)</th><th>FL</th><th>FR</th><th>RL</th><th>RR</th><th>Temp avg</th><th>Press avg</th></tr></thead><tbody>
    `;
    for (const row of rows) {
      html += `<tr><td>${row.lap}</td><td>${row.lap_time_s}</td><td>${row.wear_FL}</td><td>${row.wear_FR}</td><td>${row.wear_RL}</td><td>${row.wear_RR}</td><td>${row.temp_avg_C}</td><td>${row.pressure_avg_psi}</td></tr>`;
    }
    html += "</tbody></table>";
    out.innerHTML = html;
  } catch (e) {
    status.textContent = `error: ${e.message}`;
  } finally {
    btn.disabled = false;
  }
}
