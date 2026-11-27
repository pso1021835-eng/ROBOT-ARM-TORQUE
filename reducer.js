/* 관절 감속기 배치 · 충격 판정 — 화면 (reducer.html) */
(function () {
  "use strict";
  const $ = (id) => document.getElementById(id);
  const RC = window.ReducerCore, PRE = window.REDUCER_PRESETS;
  const D2R = Math.PI / 180, ARC = RC.ARCMIN;
  const SHORT = ["어깨 pitch", "어깨 roll", "어깨 yaw", "팔꿈치", "손목 roll", "손목 pitch", "손목 yaw"];
  const REF_POSE = [0, 0, 0, 0, 0, 0, 0], EXT_POSE = [-90, 0, 0, 88, 0, 0, 0];
  const H2X_MAP = ["A", "B", "B", "B", "B", "C", "C"];   // URDF effort 111 / 53.7 / 24.8 N·m
  const DIRS = [[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]];
  let model = null, last = null, sel = 0;
  const kinds = {};       // A/B/C → {label, Jm, src, D, info}
  const asg = [];         // 관절별 {kind, K(N·m/rad), bl(rad), Jm}

  const status = (t, alert) => { $("status").textContent = t; $("status").classList.toggle("alert", !!alert); };
  const f = (v, d = 1) => (Number.isFinite(v) ? v.toFixed(d) : "—");
  const cls = (k) => (k < 1 ? "b" : k < 1.5 ? "w" : "g");

  // ---------- 초기화 ----------
  fetch("model.json").then((r) => r.json()).then((m) => {
    model = m;
    for (const k of Object.keys(PRE)) kinds[k] = Object.assign({}, PRE[k]);
    for (const k of Object.keys(kinds)) loadKind(k, kinds[k].D);
    buildPose(EXT_POSE);
    for (let j = 0; j < 7; j++) asg[j] = {};
    setMap(H2X_MAP);
    showPresets();
    wire();
    status("준비됨. 시나리오와 배치를 정하고 [계산]을 누르세요.");
  }).catch((e) => status("model.json 불러오기 실패: " + e, true));

  function loadKind(k, D) {
    const info = RC.reducerInfo(D);
    if (!info.ok) throw new Error(info.errors.join(" · "));
    kinds[k].D = D; kinds[k].info = info;
  }
  function buildPose(v) {
    $("pose").innerHTML = ["어깨P", "어깨R", "어깨Y", "팔꿈치", "손목R", "손목P", "손목Y"].map((n) => `<span>${n}</span>`).join("") +
      v.map((x, i) => `<input type="number" id="q${i}" value="${x}" step="5">`).join("");
  }
  const setPose = (v) => v.forEach((x, i) => { $("q" + i).value = x; });
  function setMap(map) { map.forEach((k, j) => setKind(j, k)); drawAsg(); }
  function setKind(j, k) {
    const info = kinds[k].info;
    Object.assign(asg[j], { kind: k, K: info.K, bl: info.backlash, Jm: kinds[k].Jm });
  }
  function drawAsg() {
    $("asg").innerHTML = asg.map((a, j) => `<tr><td>${SHORT[j]}</td>
      <td><select data-j="${j}" class="kind">${Object.keys(kinds).map((k) => `<option${k === a.kind ? " selected" : ""}>${k}</option>`).join("")}</select></td>
      <td>${f(kinds[a.kind].info.ratio, 2)}</td>
      <td><input data-j="${j}" data-k="K" type="number" step="1" value="${+(a.K * ARC).toFixed(2)}"></td>
      <td><input data-j="${j}" data-k="bl" type="number" step="0.5" value="${+(a.bl / ARC).toFixed(2)}"></td>
      <td><input data-j="${j}" data-k="Jm" type="number" step="0.001" value="${a.Jm}"></td></tr>`).join("");
    document.querySelectorAll("#asg select.kind").forEach((s) => s.addEventListener("change", () => { setKind(+s.dataset.j, s.value); drawAsg(); }));
    document.querySelectorAll("#asg input").forEach((inp) => inp.addEventListener("change", () => {
      const j = +inp.dataset.j, k = inp.dataset.k, v = +inp.value;
      if (!(v >= 0)) return;
      asg[j][k] = k === "K" ? v / ARC : k === "bl" ? v * ARC : v;
    }));
  }
  function showPresets() {
    $("presetInfo").innerHTML = Object.entries(kinds).map(([k, o]) => {
      const i = o.info, ob = o.D.outBearing;
      const st = i.stages.map((s, n) => `${n + 1}단 m${s.m} S${s.s}/P${s.p}/R${s.r}×${s.n} b${s.b} (i ${f(s.ratio, 3)})`).join("<br>");
      return `<div class="card" style="margin:6px 0;padding:7px 9px"><b>${k}</b> — ${o.label}<br><span class="note">${o.D.name || ""}</span><br>
        <span style="font:11px ui-monospace,monospace">${st}<br>감속비 ${f(i.ratio, 3)}${i.sharedRing ? " · 링 공유" : ""} · 정적 한계 토크 ${f(i.staticCap)} N·m · 수명 기준 ${f(i.capLife)} N·m<br>
        강성 ${f(i.KarcMin, 1)} N·m/′ · 백래시 ${f(i.backlashArcmin, 2)}′ · 출력 베어링 C0 ${ob.C0} N, dp ${f(ob.dp)} mm<br>Jm ${o.Jm} kg·m² — ${o.src}</span></div>`;
    }).join("");
    $("tSH").textContent = f(kinds.A.info.target.SH, 1); $("tSF").textContent = f(kinds.A.info.target.SF, 1); $("tS0").textContent = f(kinds.A.info.target.s0, 1);
  }

  function wire() {
    $("poseRef").onclick = () => setPose(REF_POSE);
    $("poseExt").onclick = () => setPose(EXT_POSE);
    $("asgDef").onclick = () => setMap(H2X_MAP);
    $("asgEng").onclick = () => { asg.forEach((a) => { const i = kinds[a.kind].info; a.K = i.K; a.bl = i.backlash; }); drawAsg(); };
    $("pasteBtn").onclick = () => {
      const k = $("pasteTo").value;
      try {
        const D = JSON.parse($("paste").value);
        if (!D || !Array.isArray(D.stages) || !D.req) throw new Error("설계 JSON 형식이 아닙니다 (stages · req 필요)");
        const prev = kinds[k].D; try { loadKind(k, D); } catch (e) { kinds[k].D = prev; throw e; }
        kinds[k].label = "붙여 넣은 설계"; kinds[k].src = "반사관성은 이전 값 유지 — 표에서 고치세요";
        asg.forEach((a, j) => { if (a.kind === k) setKind(j, k); });
        drawAsg(); showPresets(); status(`${k} 종류를 붙여 넣은 설계로 바꿨습니다.`);
      } catch (e) { status("설계 적용 실패: " + e.message, true); }
    };
    $("run").onclick = run;
    window.addEventListener("resize", () => { if (last) { drawPlots(); drawDetail(); } });
  }

  // ---------- 계산 ----------
  function run() {
    const btn = $("run"); btn.disabled = true; status("계산 중…");
    setTimeout(() => {
      try {
        const t0 = performance.now();
        const pose = Array.from({ length: 7 }, (_, i) => (+$("q" + i).value || 0) * D2R);
        const sc = { pose, payload: Math.max(0, +$("pay").value || 0), off: [+$("payX").value || 0, 0, 0],
          impact: { F: Math.max(0, +$("impF").value || 0), T: Math.max(1, +$("impT").value || 30) / 1000, dir: DIRS[+$("impDir").value], link: +$("impAt").value },
          tPre: 0.05, tPost: Math.min(3, Math.max(0.1, +$("tPost").value || 0.5)),
          cfg: { armature: asg.map((a) => a.Jm) } };   // 강체 계산의 반사관성도 배치한 감속기의 Jm으로 맞춤
        const rec = RC.runRigid(model, sc);
        const kScale = Math.max(0.01, +$("kScale").value || 1), zetaG = Math.max(0, +$("zetaG").value || 0);
        const res = asg.map((a, j) => {
          const base = kinds[a.kind].info;
          const info = Object.assign({}, base, { K: a.K, backlash: a.bl });
          return RC.evaluateJoint(rec, j, { info, Jm: a.Jm, kScale, zetaG });
        });
        last = { rec, res, sc, impT: sc.impact.T };
        const sat = rec.sat.some((s) => s.some(Boolean));
        drawTable(); drawPlots(); drawDetail();
        const gov = res.reduce((x, y) => (y.gov.k < x.gov.k ? y : x));
        status(`계산 ${f(performance.now() - t0, 0)} ms · 최약: ${SHORT[gov.j]} ${gov.gov.part} (여유 ${f(gov.gov.k, 2)})` +
          (sat ? "\n주의: 강체 계산 중 구동 토크가 URDF 한계에서 잘린 관절이 있습니다 (모터 포화)." : ""), gov.gov.k < 1 || sat);
      } catch (e) { status("계산 실패: " + e.message, true); console.error(e); }
      btn.disabled = false;
    }, 20);
  }

  function drawTable() {
    const rows = last.res.map((r) => {
      const g = r.gComp, pl = g.planetMin, kPl = pl.s0 / r.info.target.s0, kOb = r.obS0 / r.info.target.s0;
      return `<tr data-j="${r.j}" class="${r.j === sel ? "sel" : ""}"><td>${SHORT[r.j]}</td><td>${asg[r.j].kind}</td>
        <td>${f(Math.abs(r.rigPk))}</td><td>${f(Math.abs(r.cPk))}</td><td class="${r.daf > 1.5 ? "w" : ""}">${f(r.daf, 2)}</td><td>${r.comp.sep}</td>
        <td>${f(Math.abs(r.cPk) / r.effort * 100, 0)}%</td>
        <td class="${cls(g.ratio)}">${f(g.ratio, 2)}</td><td class="${cls(kPl)}">${f(kPl, 2)}</td>
        <td>${f(r.obW.Fa, 0)} / ${f(r.obW.Fr, 0)} / ${f(r.obW.M, 1)}</td><td class="${cls(kOb)}">${f(kOb, 2)}</td>
        <td style="text-align:left" class="${cls(r.gov.k)}">${r.gov.part}</td></tr>`;
    }).join("");
    $("resWrap").innerHTML = `<table id="res"><thead><tr><th>관절</th><th>종류</th><th>강체 피크<br>N·m</th><th>유연 피크<br>N·m</th><th>증폭</th><th>이 떨어짐<br>횟수</th><th>피크 /<br>URDF 한계</th>
      <th>기어<br>여유</th><th>유성 베어링<br>여유</th><th>출력 베어링 최대 하중<br>Fa / Fr [N] / M [N·m]</th><th>출력 베어링<br>여유</th><th style="text-align:left">가장 먼저 닿는 부품</th></tr></thead><tbody>${rows}</tbody></table>`;
    document.querySelectorAll("#res tbody tr").forEach((tr) => tr.addEventListener("click", () => select(+tr.dataset.j)));
  }
  function select(j) { sel = j; drawTable(); drawPlots(); drawDetail(); }

  // ---------- 그래프 ----------
  function canvasFit(c) {
    const r = c.getBoundingClientRect(), dpr = window.devicePixelRatio || 1;
    c.width = Math.max(10, r.width * dpr); c.height = Math.max(10, r.height * dpr);
    const g = c.getContext("2d"); g.setTransform(dpr, 0, 0, dpr, 0, 0); return { g, W: r.width, H: r.height };
  }
  function plot(c, title, t, series, opt = {}) {
    const { g, W, H } = canvasFit(c);
    const L = 46, R = 8, T = 20, B = 22, w = W - L - R, h = H - T - B;
    let lo = Infinity, hi = -Infinity;
    series.forEach((s) => s.y.forEach((v) => { if (v < lo) lo = v; if (v > hi) hi = v; }));
    (opt.hlines || []).forEach((v) => { if (Math.abs(v) < 3 * Math.max(Math.abs(lo), Math.abs(hi), 1e-9)) { lo = Math.min(lo, v); hi = Math.max(hi, v); } });
    if (!(hi > lo)) { hi = lo + 1; } const pad = 0.08 * (hi - lo); lo -= pad; hi += pad;
    const t0 = t[0], t1 = t[t.length - 1];
    const X = (v) => L + (v - t0) / (t1 - t0) * w, Y = (v) => T + (hi - v) / (hi - lo) * h;
    g.fillStyle = "#1c1c1d"; g.fillRect(0, 0, W, H);
    if (opt.band) { g.fillStyle = "rgba(255,255,255,0.06)"; g.fillRect(X(opt.band[0]), T, X(opt.band[1]) - X(opt.band[0]), h); }
    g.strokeStyle = "#2a2a2a"; g.lineWidth = 1; g.font = "10px system-ui"; g.fillStyle = "#8a8a8a";
    for (let k = 0; k <= 4; k++) { const v = lo + (hi - lo) * k / 4, y = Y(v); g.beginPath(); g.moveTo(L, y); g.lineTo(L + w, y); g.stroke(); g.fillText(v.toFixed(Math.abs(hi - lo) < 10 ? 1 : 0), 4, y + 3); }
    for (let k = 0; k <= 4; k++) { const v = t0 + (t1 - t0) * k / 4; g.fillText(v.toFixed(2) + " s", X(v) - 12, H - 6); }
    if (lo < 0 && hi > 0) { g.strokeStyle = "#444"; g.beginPath(); g.moveTo(L, Y(0)); g.lineTo(L + w, Y(0)); g.stroke(); }
    g.setLineDash([4, 3]); g.strokeStyle = "#c8371e";
    (opt.hlines || []).forEach((v) => { if (v >= lo && v <= hi) { g.beginPath(); g.moveTo(L, Y(v)); g.lineTo(L + w, Y(v)); g.stroke(); } });
    g.setLineDash([]);
    if (opt.ticks) { g.strokeStyle = "#c9962a"; opt.ticks.forEach((tt) => { g.beginPath(); g.moveTo(X(tt), T + h - 6); g.lineTo(X(tt), T + h); g.stroke(); }); }
    const step = Math.max(1, Math.floor(t.length / (w * 2)));
    series.forEach((s) => {
      g.strokeStyle = s.color; g.lineWidth = s.width || 1.4; g.beginPath();
      for (let k = 0; k < t.length; k += step) { const x = X(t[k]), y = Y(s.y[k]); k ? g.lineTo(x, y) : g.moveTo(x, y); }
      g.stroke();
    });
    g.fillStyle = "#d6d6d6"; g.font = "600 11.5px system-ui"; g.fillText(title, L, 13);
  }
  function drawPlots() {
    const { rec, res, impT } = last;
    $("plots").innerHTML = res.map((r) => `<div class="pl${r.j === sel ? " sel" : ""}" data-j="${r.j}"><canvas></canvas></div>`).join("");
    document.querySelectorAll("#plots .pl").forEach((d) => d.addEventListener("click", () => select(+d.dataset.j)));
    res.forEach((r) => {
      const c = document.querySelector(`#plots .pl[data-j="${r.j}"] canvas`);
      const rig = rec.tauOut.map((v) => v[r.j]);
      const ticks = []; for (let k = 1; k < rec.t.length; k++) if (r.comp.gap[k] && !r.comp.gap[k - 1]) ticks.push(rec.t[k]);
      plot(c, `${SHORT[r.j]} (${asg[r.j].kind}) — 토크 N·m`, rec.t, [{ y: rig, color: "#8a8a8a" }, { y: Array.from(r.comp.tg), color: "#7d9cb8" }],
        { band: [0, impT], hlines: [r.effort, -r.effort], ticks });
    });
  }
  function drawDetail() {
    const r = last.res[sel], rec = last.rec, i = r.info, g = r.gComp, w = r.obW;
    const pl = g.planet.map((p) => `${p.stage}단: 핀 하중 ${f(p.pinForce, 0)} N · C0 ${p.C0} N → s0 ${f(p.s0, 2)}`).join("<br>");
    const ob = i.ob, util = Array.from(r.P0, (v) => v / ob.C0 * 100);
    $("detail").innerHTML = `<div class="card"><b>${SHORT[sel]} — 출력 베어링 하중 P0 / C0 [%]</b><div class="big"><canvas id="obPlot"></canvas></div>
        <div class="note">P0 = Fr + 2000·M/dp + 0.44·Fa · 최대 ${f(r.obPk, 0)} N (t = ${f(r.obT, 3)} s): Fa ${f(w.Fa, 0)} N · Fr ${f(w.Fr, 0)} N · M ${f(w.M, 2)} N·m · 축 토크 ${f(w.T, 1)} N·m</div></div>
      <div class="card" style="font-size:12px;line-height:1.7"><b>${SHORT[sel]} · 감속기 ${asg[sel].kind}</b> (i ${f(i.ratio, 2)})<br>
        강체 피크 ${f(r.rigPk)} N·m (t ${f(r.rigT, 3)} s) → 유연·백래시 피크 <b>${f(r.cPk)} N·m</b> (t ${f(r.cT, 3)} s), 증폭 ${f(r.daf, 2)}<br>
        모델: K ${f(r.comp.K * ARC, 1)} N·m/′ (배율 포함) · 백래시 ±${f(r.comp.delta / ARC, 2)}′ · Jm ${r.comp.Jm} · Jl ${f(r.comp.Jl, 4)} kg·m² · 고유진동수 ${f(r.comp.fn, 0)} Hz · 이 떨어짐 ${r.comp.sep}회<br>
        <b>기어</b> (유연 피크 ${f(Math.abs(g.T))} N·m): 여유 <span class="${cls(g.ratio)}">${f(g.ratio, 2)}</span> — 최약 ${g.worst ? `${g.worst.stage}단 ${g.worst.mesh} ${g.worst.part} ${g.worst.mode} S ${f(g.worst.S, 2)} (목표 ${g.worst.target})` : "—"}<br>
        강체 피크 기준 여유 ${f(r.gRig.ratio, 2)} · 감속기 정적 한계 토크 ${f(i.staticCap)} N·m<br>
        <b>유성 베어링</b><br>${pl}<br>
        <b>출력 베어링</b> ${ob.name}: C0 ${ob.C0} N, dp ${f(ob.dp)} mm → s0 <span class="${cls(r.obS0 / i.target.s0)}">${f(r.obS0, 2)}</span><br>
        <span class="note">산정 기준: ${i.basis === "ideal" ? "오차 0 (ideal)" : "ISO 방법 C"} · 정적 판정 (rpm 0)</span></div>`;
    plot($("obPlot"), "", rec.t, [{ y: util, color: "#c9962a" }], { band: [0, last.impT], hlines: [100 / i.target.s0] });
  }
})();
