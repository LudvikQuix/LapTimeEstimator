// Driver tab: view existing fit OR fit a new driver from lake/local CSVs.

import {
  apiGet, fillSelect, loadEnvLists, openSSEPost, renderStageStrip,
} from "/static/modules/common.js";

let _booted = false;

window.addEventListener("tab-shown", (e) => {
  if (e.detail.name === "driver" && !_booted) {
    boot();
    _booted = true;
  }
});

async function boot() {
  const panel = document.getElementById("tab-driver");
  panel.innerHTML = `
    <h2 class="text-xl font-semibold mb-4">Driver</h2>
    <div class="flex gap-2 mb-4 text-sm">
      <button class="mode-btn px-3 py-1.5 rounded bg-slate-700" data-mode="view">View existing</button>
      <button class="mode-btn px-3 py-1.5 rounded hover:bg-slate-700" data-mode="fit">Fit new driver</button>
    </div>
    <div id="driver-view-pane">
      <label class="text-sm">Pick a driver:
        <select id="driver-pick" class="ml-2"></select>
      </label>
      <div id="driver-summary" class="mt-4 grid grid-cols-1 md:grid-cols-2 gap-4"></div>
      <h3 class="mt-4 text-base font-semibold">tyre_calibration block</h3>
      <pre id="driver-tyre-cal" class="json-viewer"></pre>
      <h3 class="mt-4 text-base font-semibold">profile.dynamic block</h3>
      <pre id="driver-profile" class="json-viewer"></pre>
      <h3 class="mt-4 text-base font-semibold">Full JSON</h3>
      <pre id="driver-raw" class="json-viewer"></pre>
    </div>
    <div id="driver-fit-pane" class="hidden">
      <div class="grid grid-cols-1 md:grid-cols-3 gap-4 mb-4">
        <label class="flex flex-col text-sm">Driver name (output JSON)
          <input id="fit-name" type="text" placeholder="e.g. ludvik_test" class="mt-1" />
        </label>
        <label class="flex flex-col text-sm">Car
          <select id="fit-car" class="mt-1"></select>
        </label>
        <label class="flex flex-col text-sm">Track
          <select id="fit-track" class="mt-1"></select>
        </label>
        <label class="flex flex-col text-sm">Layout
          <select id="fit-layout" class="mt-1"></select>
        </label>
        <label class="flex flex-col text-sm">Source
          <select id="fit-source-kind" class="mt-1">
            <option value="lake">From lake (newest N)</option>
            <option value="local">From local CSVs</option>
          </select>
        </label>
        <label class="flex flex-col text-sm">N newest (lake mode)
          <input id="fit-n-newest" type="number" min="2" value="5" class="mt-1" />
        </label>
        <label class="flex flex-col text-sm">Lake driver (lake mode)
          <input id="fit-lake-driver" type="text" placeholder="e.g. tomas" class="mt-1" />
        </label>
        <label class="flex flex-col text-sm">Lake car (lake mode)
          <input id="fit-lake-car" type="text" placeholder="e.g. bmw_1m" class="mt-1" />
        </label>
        <label class="flex flex-col text-sm">Lake track (lake mode)
          <input id="fit-lake-track" type="text" placeholder="e.g. ks_nurburgring" class="mt-1" />
        </label>
        <label class="flex flex-col text-sm md:col-span-3">CSV paths (local mode, one per line)
          <textarea id="fit-csv-paths" rows="3" placeholder="samples/aclog/Lap1.csv"></textarea>
        </label>
        <label class="flex items-center text-sm gap-2 md:col-span-3">
          <input id="fit-overwrite" type="checkbox" />
          Overwrite drivers/&lt;name&gt;.json if it already exists
        </label>
      </div>
      <div class="flex items-center gap-3 mb-4">
        <button id="fit-run" class="primary">Run fit</button>
        <span id="fit-status" class="text-sm text-slate-400"></span>
      </div>
      <div id="fit-stages" class="mb-4"></div>
      <div id="fit-result" class="text-sm"></div>
    </div>
  `;

  // Mode toggle.
  panel.querySelectorAll(".mode-btn").forEach(b => {
    b.addEventListener("click", () => {
      const mode = b.dataset.mode;
      panel.querySelectorAll(".mode-btn").forEach(x => x.classList.remove("bg-slate-700"));
      b.classList.add("bg-slate-700");
      document.getElementById("driver-view-pane").classList.toggle("hidden", mode !== "view");
      document.getElementById("driver-fit-pane").classList.toggle("hidden", mode !== "fit");
    });
  });

  const {cars, tracks, drivers} = await loadEnvLists();
  fillSelect(document.getElementById("driver-pick"),
    drivers.map(d => ({value: d.name, label: d.name})),
    {placeholder: "Select driver"});
  document.getElementById("driver-pick").addEventListener("change", onPickDriver);

  fillSelect(document.getElementById("fit-car"),
    cars.map(c => ({value: c.name, label: c.name})));
  fillSelect(document.getElementById("fit-track"),
    tracks.map(t => ({value: t.track, label: t.track})));
  function refreshLayouts() {
    const t = document.getElementById("fit-track").value;
    const layouts = (tracks.find(x => x.track === t) || {}).layouts || [];
    fillSelect(document.getElementById("fit-layout"),
      layouts.map(l => ({value: `${t}/${l.full}`, label: l.name})));
  }
  document.getElementById("fit-track").addEventListener("change", refreshLayouts);
  refreshLayouts();

  document.getElementById("fit-run").addEventListener("click", runFit);
}

async function onPickDriver() {
  const name = document.getElementById("driver-pick").value;
  if (!name) return;
  const j = await apiGet(`/api/driver/${encodeURIComponent(name)}`);
  const summary = document.getElementById("driver-summary");
  summary.innerHTML = `
    <div>skill_pct: <strong>${j.skill_pct ?? "?"}</strong></div>
    <div>consistency_sigma: <strong>${j.consistency_sigma ?? "?"}</strong></div>
    <div>trail_brake_m: <strong>${j.trail_brake_m ?? "?"}</strong></div>
    <div>throttle_ramp_m: <strong>${j.throttle_ramp_m ?? "?"}</strong></div>
    <div>driver_tau_s: <strong>${j.driver_tau_s ?? "?"}</strong></div>
    <div>fit_version: <strong>${j.source?.fit_version ?? "?"}</strong></div>
    <div>sim_lap_time_s: <strong>${j.source?.sim_lap_time_s ?? "?"}</strong></div>
    <div>real_lap_time_s: <strong>${j.source?.real_lap_time_s ?? "?"}</strong></div>
    <div>delta_s: <strong>${j.source?.delta_s ?? "?"}</strong></div>
  `;
  document.getElementById("driver-tyre-cal").textContent =
    JSON.stringify(j.tyre_calibration ?? {}, null, 2);
  document.getElementById("driver-profile").textContent =
    JSON.stringify(j.profile?.dynamic ?? {}, null, 2);
  document.getElementById("driver-raw").textContent =
    JSON.stringify(j, null, 2);
}

function runFit() {
  const btn = document.getElementById("fit-run");
  const status = document.getElementById("fit-status");
  const stagesEl = document.getElementById("fit-stages");
  const resultEl = document.getElementById("fit-result");

  const name = document.getElementById("fit-name").value.trim();
  if (!name) { status.textContent = "name required"; return; }

  const sourceKind = document.getElementById("fit-source-kind").value;
  let source;
  if (sourceKind === "lake") {
    source = {
      kind: "lake",
      driver: document.getElementById("fit-lake-driver").value.trim(),
      car: document.getElementById("fit-lake-car").value.trim(),
      track: document.getElementById("fit-lake-track").value.trim(),
      n_newest: parseInt(document.getElementById("fit-n-newest").value, 10),
    };
  } else {
    const paths = document.getElementById("fit-csv-paths").value
      .split("\n").map(s => s.trim()).filter(Boolean);
    source = {kind: "local", csv_paths: paths};
  }

  const body = {
    car: document.getElementById("fit-car").value,
    track: document.getElementById("fit-layout").value,
    driver_name: name,
    source,
    overwrite: document.getElementById("fit-overwrite").checked,
  };

  const stages = [];
  btn.disabled = true;
  status.textContent = "fitting...";
  stagesEl.innerHTML = "";
  resultEl.innerHTML = "";

  openSSEPost("/api/fit_driver", body, (evt) => {
    if (evt.type === "stage") {
      const name = evt.data?.stage || "stage";
      stages.push({label: name, state: "done"});
      renderStageStrip(stagesEl, stages);
    } else if (evt.type === "written") {
      stages.push({label: `wrote ${evt.data?.path || ""}`, state: "done"});
      renderStageStrip(stagesEl, stages);
      resultEl.innerHTML = `
        <div class="p-3 rounded bg-emerald-900">
          <strong>Saved to container.</strong>
          Pull the file locally and commit to persist.
          <pre class="mt-2">${JSON.stringify(evt.data, null, 2)}</pre>
        </div>`;
    } else if (evt.type === "done") {
      status.textContent = "fit complete.";
      btn.disabled = false;
    } else if (evt.type === "error") {
      stages.push({label: "ERROR", state: "err"});
      renderStageStrip(stagesEl, stages);
      resultEl.innerHTML = `<div class="p-3 rounded bg-rose-900">${evt.data?.detail || JSON.stringify(evt.data)}</div>`;
      status.textContent = "error";
      btn.disabled = false;
    }
  }, (err) => {
    status.textContent = `transport error: ${err.message}`;
    btn.disabled = false;
  });
}
