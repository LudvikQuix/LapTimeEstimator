// Lake tab: walk the QuixLake partition tree and render driver/car/track/session.

import {apiGet} from "/static/modules/common.js";

let _booted = false;

window.addEventListener("tab-shown", (e) => {
  if (e.detail.name === "lake" && !_booted) { boot(); _booted = true; }
});

async function boot() {
  const panel = document.getElementById("tab-lake");
  panel.innerHTML = `
    <div class="flex items-center gap-3 mb-4">
      <h2 class="text-xl font-semibold">Lake partition tree</h2>
      <button id="lake-refresh" class="primary text-sm">Refresh</button>
      <span id="lake-meta" class="text-sm text-slate-400"></span>
    </div>
    <div id="lake-tree"></div>
  `;
  document.getElementById("lake-refresh").addEventListener("click", () => load({force: 1}));
  await load({force: 0});
}

async function load({force}) {
  const meta = document.getElementById("lake-meta");
  const out = document.getElementById("lake-tree");
  meta.textContent = "loading...";
  out.innerHTML = "";
  try {
    const t = await apiGet(`/api/lake/tree?force=${force}`);
    meta.textContent = `transport=${t.transport} | cache=${t.cache} | ${t.drivers?.length || 0} drivers`;
    out.innerHTML = renderTree(t);
  } catch (e) {
    meta.textContent = `error: ${e.message}`;
  }
}

function renderTree(t) {
  if (!t.drivers || !t.drivers.length) return `<div class="text-slate-400">No partitions.</div>`;
  let html = "";
  for (const d of t.drivers) {
    html += `<details open class="mb-3">
      <summary class="cursor-pointer text-base font-semibold">
        ${d.driver}
        <span class="ml-2 text-xs ${d.fitted_locally ? "text-emerald-400" : "text-amber-400"}">${d.fitted_locally ? "fitted locally" : "not fitted"}</span>
      </summary>
      <div class="pl-4 mt-2">`;
    for (const c of d.cars) {
      html += `<details class="mb-2"><summary class="cursor-pointer">car: ${c.car}</summary><div class="pl-4 mt-1">`;
      for (const tr of c.tracks) {
        html += `<details class="mb-1"><summary class="cursor-pointer">track: ${tr.track}</summary><table class="grid mt-2"><thead><tr><th>session_id</th><th>laps</th><th>laps list</th></tr></thead><tbody>`;
        for (const s of tr.sessions) {
          html += `<tr><td>${s.session_id}</td><td>${s.lap_count}</td><td>${(s.laps || []).join(", ")}</td></tr>`;
        }
        html += `</tbody></table></details>`;
      }
      html += `</div></details>`;
    }
    html += `</div></details>`;
  }
  return html;
}
