/* 관절 감속기 실시간 판정 — reducer.html 전용 (기본 시뮬레이터 app.js 위에 겹쳐 동작) */
(function () {
  "use strict";
  const $ = (id) => document.getElementById(id);
  const RC = window.ReducerCore, PRE = window.REDUCER_PRESETS, ARC = RC.ARCMIN;
  const SHORT = ["어깨 pitch", "어깨 roll", "어깨 yaw", "팔꿈치", "손목 roll", "손목 pitch", "손목 yaw"];
  const H2X_MAP = ["A", "B", "B", "B", "B", "C", "C"];   // URDF effort 111 / 53.7 / 24.8 N·m
  const kinds = {}, asg = [];
  let H, sim, live, rings = [], plots = [], lastUi = 0, holdK = new Array(7).fill(Infinity), holdT = new Array(7).fill(0);
  const f = (v, d = 1) => (Number.isFinite(v) ? v.toFixed(d) : "—");
  const kc = (k) => (!Number.isFinite(k) ? "kg" : k < 1 ? "kb" : k < 1.5 ? "kw" : "kg");
  const kf = (k) => (Number.isFinite(k) ? (k > 99 ? ">99" : k.toFixed(2)) : "—");

  // app.js가 3D·시뮬레이터를 다 만들 때까지 기다림
  (function wait() {
    H = window.__h2x;
    if (!H || !H.sim || !H.scene || !H.linkGroups || H.linkGroups.length !== 7) return setTimeout(wait, 100);
    setTimeout(init, 30);
  })();

  function init() {
    sim = H.sim;
    try {
      for (const k of Object.keys(PRE)) { kinds[k] = Object.assign({}, PRE[k]); loadKind(k, kinds[k].D); }
    } catch (e) { $("lvMsg").textContent = "감속기 계산 실패: " + e.message; return; }
    for (let j = 0; j < 7; j++) asg[j] = {};
    H2X_MAP.forEach((k, j) => setKind(j, k)); drawAsg(); showPresets();
    $("asgApply").onclick = apply;
    $("asgDef").onclick = () => { H2X_MAP.forEach((k, j) => setKind(j, k)); drawAsg(); apply(); };
    $("asgEng").onclick = () => { asg.forEach((a) => { const i = kinds[a.kind].info; a.K = i.K; a.bl = i.backlash; }); drawAsg(); apply(); };
    $("lvReset").onclick = () => { live.reset(); holdK.fill(Infinity); };
    $("lvOn").onchange = () => { live.enabled = $("lvOn").checked; if (live.enabled) live.reset(); };
    $("pasteBtn").onclick = paste;
    buildRings(); buildPlots();
    live = new RC.Live(sim, joints(), opts()).attach();
    apply();
    $("lvMsg").textContent = "실시간 판정 동작 중. 충격(외력 펄스)·랜덤 자세·하중을 바꾸면 바로 반영됩니다.";
    requestAnimationFrame(loop);
  }
  function loadKind(k, D) {
    const info = RC.reducerInfo(D);
    if (!info.ok) throw new Error(info.errors.join(" · "));
    kinds[k].D = D; kinds[k].info = info; kinds[k].tab = RC.gearTable(info);
  }
  function setKind(j, k) { const i = kinds[k].info; Object.assign(asg[j], { kind: k, K: i.K, bl: i.backlash, Jm: kinds[k].Jm }); }
  const joints = () => asg.map((a) => ({ info: kinds[a.kind].info, tab: kinds[a.kind].tab, K: a.K, bl: a.bl, Jm: a.Jm }));
  const opts = () => ({ kScale: Math.max(0.01, +$("kScale").value || 1), zetaG: Math.max(0, +$("zetaG").value || 0), histSec: 3.2 });
  function apply() {
    // 강체 계산의 반사관성(구동부 표 J)을 배치한 감속기 Jm으로 맞춤 → app.js의 [적용] 버튼으로 반영
    document.querySelectorAll('#actTable input[data-k="J"]').forEach((el) => { el.value = asg[+el.dataset.i].Jm; });
    $("btnCtrl").click();
    live.set(joints(), opts()); holdK.fill(Infinity);
    rings.forEach((r, j) => { const s = kinds[asg[j].kind].info.ratio > 21 ? 1.25 : kinds[asg[j].kind].info.ratio > 18 ? 1 : 0.8; r.scale.setScalar(s); });
  }
  function drawAsg() {
    $("asg").innerHTML = asg.map((a, j) => `<tr><td>${SHORT[j]}</td>
      <td><select data-j="${j}">${Object.keys(kinds).map((k) => `<option${k === a.kind ? " selected" : ""}>${k}</option>`).join("")}</select></td>
      <td>${f(kinds[a.kind].info.ratio, 2)}</td>
      <td><input data-j="${j}" data-k="K" type="number" step="1" value="${+(a.K * ARC).toFixed(2)}"></td>
      <td><input data-j="${j}" data-k="bl" type="number" step="0.5" value="${+(a.bl / ARC).toFixed(2)}"></td>
      <td><input data-j="${j}" data-k="Jm" type="number" step="0.001" value="${a.Jm}"></td></tr>`).join("");
    document.querySelectorAll("#asg select").forEach((s) => s.addEventListener("change", () => { setKind(+s.dataset.j, s.value); drawAsg(); }));
    document.querySelectorAll("#asg input").forEach((inp) => inp.addEventListener("change", () => {
      const j = +inp.dataset.j, k = inp.dataset.k, v = +inp.value; if (!(v >= 0)) return;
      asg[j][k] = k === "K" ? Math.max(v, 1e-3) / ARC : k === "bl" ? v * ARC : Math.max(v, 1e-5);
    }));
  }
  function showPresets() {
    $("presetInfo").innerHTML = Object.entries(kinds).map(([k, o]) => {
      const i = o.info, ob = o.D.outBearing;
      const st = i.stages.map((s, n) => `${n + 1}단 m${s.m} S${s.s}/P${s.p}/R${s.r}×${s.n} b${s.b}`).join("<br>");
      return `<div class="pcard"><b>${k}</b> ${o.label}<br>${st}<br>i ${f(i.ratio, 3)}${i.sharedRing ? " · 링 공유" : ""} · 정적 한계 ${f(i.staticCap)} N·m · 강성 ${f(i.KarcMin, 1)} N·m/′ · 백래시 ${f(i.backlashArcmin, 2)}′<br>
        출력 베어링 C0 ${ob.C0} N · dp ${f(ob.dp)} mm · Jm ${o.Jm} (${o.src})</div>`;
    }).join("") + `<div class="note">예시 제원은 설계실 추천 엔진으로 자동 생성 (H2-X 실물 아님). 기어 목표 SH ${kinds.A.info.target.SH} · SF ${kinds.A.info.target.SF} (정적), 베어링 s0 ${kinds.A.info.target.s0}</div>`;
  }
  function paste() {
    const k = $("pasteTo").value;
    try {
      const D = JSON.parse($("paste").value);
      if (!D || !Array.isArray(D.stages) || !D.req) throw new Error("설계 JSON 형식이 아닙니다 (stages · req 필요)");
      const prev = { D: kinds[k].D, info: kinds[k].info, tab: kinds[k].tab };
      try { loadKind(k, D); } catch (e) { Object.assign(kinds[k], prev); throw e; }
      kinds[k].label = "붙여 넣은 설계"; kinds[k].src = "Jm은 이전 값 — 표에서 고치세요";
      asg.forEach((a, j) => { if (a.kind === k) setKind(j, k); });
      drawAsg(); showPresets(); apply(); $("lvMsg").textContent = `${k} 종류를 붙여 넣은 설계로 바꿨습니다.`;
    } catch (e) { $("lvMsg").textContent = "설계 적용 실패: " + e.message; }
  }

  // ---------- 3D: 관절 고리 (색 = 여유) ----------
  function buildRings() {
    const T = THREE, model = H.model;
    rings = model.joints.map((jn, j) => {
      const geo = new T.TorusGeometry(0.05, 0.008, 12, 40);
      const mat = new T.MeshBasicMaterial({ color: 0x5fbf86, transparent: true, opacity: 0.85, depthTest: false });
      const m = new T.Mesh(geo, mat); m.renderOrder = 5;
      // 토러스 기본 축 = z → 관절축(자식 링크 좌표계)으로 회전
      m.quaternion.setFromUnitVectors(new T.Vector3(0, 0, 1), new T.Vector3(...jn.axis).normalize());
      H.linkGroups[j].add(m);
      return m;
    });
  }
  function ringColor(k) {
    if (!Number.isFinite(k) || k >= 2) return 0x5fbf86;
    if (k >= 1.5) return 0xc8c84a;
    if (k >= 1) return 0xe08a2a;
    return 0xff3b2a;
  }

  // ---------- 그래프 ----------
  function buildPlots() {
    const box = $("lvPlots");
    plots = SHORT.map((n, j) => { const d = document.createElement("div"); d.className = "pl"; const c = document.createElement("canvas"); d.appendChild(c); box.appendChild(d); return c; });
    const lg = document.createElement("div"); lg.className = "pl"; lg.style.padding = "8px 10px"; lg.style.fontSize = "11px"; lg.style.color = "#8a8a8a"; lg.style.lineHeight = "1.6";
    lg.innerHTML = "<b style='color:#d6d6d6'>읽는 법</b><br>충격이나 급가속 때 파랑(기어 토크)이 회색(강체)보다 튀면 백래시·강성 때문에 이빨이 다시 부딪히는 하중입니다.<br>하중 부호가 바뀌는 순간(0 통과) 주황 눈금이 찍히고 그 직후 튀는 값이 기어 판정에 들어갑니다.";
    box.appendChild(lg);
  }
  function drawPlot(c, j, hist) {
    const r = c.getBoundingClientRect(), dpr = window.devicePixelRatio || 1;
    if (c.width !== Math.round(r.width * dpr) || c.height !== Math.round(r.height * dpr)) { c.width = Math.round(r.width * dpr); c.height = Math.round(r.height * dpr); }
    const g = c.getContext("2d"); g.setTransform(dpr, 0, 0, dpr, 0, 0);
    const W = r.width, Hh = r.height, L = 40, R = 6, T = 20, B = 16, w = W - L - R, h = Hh - T - B;
    g.clearRect(0, 0, W, Hh);
    const n = hist.length; if (n < 2) return;
    const tEnd = hist[n - 1].t, t0 = tEnd - 3, eff = sim.arm.effort[j];
    let m = 1; for (let k = 0; k < n; k++) { m = Math.max(m, Math.abs(hist[k].rig[j]), Math.abs(hist[k].tg[j])); }
    const yl = Math.min(Math.max(m * 1.2, 2), eff * 2.5);
    const X = (t) => L + (t - t0) / 3 * w, Y = (v) => T + (1 - (Math.max(-yl, Math.min(yl, v)) + yl) / (2 * yl)) * h;
    // 외력 구간
    g.fillStyle = "rgba(200,55,30,0.18)";
    let a0 = null;
    for (let k = 0; k < n; k++) { const on = hist[k].ext; if (on && a0 === null) a0 = hist[k].t; if ((!on || k === n - 1) && a0 !== null) { g.fillRect(X(Math.max(a0, t0)), T, Math.max(1, X(hist[k].t) - X(Math.max(a0, t0))), h); a0 = null; } }
    g.strokeStyle = "#2a2b2e"; g.lineWidth = 1; g.font = "10px system-ui"; g.fillStyle = "#8a8a8a"; g.textAlign = "right"; g.textBaseline = "middle";
    for (const v of [-yl, -yl / 2, 0, yl / 2, yl]) { g.beginPath(); g.moveTo(L, Y(v)); g.lineTo(L + w, Y(v)); g.stroke(); g.fillText(Math.abs(v) >= 10 ? v.toFixed(0) : v.toFixed(1), L - 3, Y(v)); }
    g.setLineDash([4, 3]); g.strokeStyle = "#ff5a45";
    for (const s of [1, -1]) if (eff <= yl) { g.beginPath(); g.moveTo(L, Y(s * eff)); g.lineTo(L + w, Y(s * eff)); g.stroke(); }
    g.setLineDash([]);
    g.strokeStyle = "#c9962a";
    for (let k = 1; k < n; k++) if (hist[k].gap[j] && !hist[k - 1].gap[j]) { const x = X(hist[k].t); g.beginPath(); g.moveTo(x, T + h - 7); g.lineTo(x, T + h); g.stroke(); }
    const step = Math.max(1, Math.floor(n / (w * 1.5)));
    for (const [key, col, lw] of [["tg", "#7d9cb8", 2], ["rig", "#9a9a9a", 1]]) {   // 강체를 위에 얇게 → 겹쳐도 둘 다 보임
      g.strokeStyle = col; g.lineWidth = lw; g.beginPath();
      // 구간 최대값 보존 다운샘플 (튀는 값이 사라지지 않게)
      for (let k = 0; k < n; k += step) {
        let v = hist[k][key][j]; for (let q = k + 1; q < Math.min(n, k + step); q++) if (Math.abs(hist[q][key][j]) > Math.abs(v)) v = hist[q][key][j];
        const x = X(hist[k].t), y = Y(v); k ? g.lineTo(x, y) : g.moveTo(x, y);
      }
      g.stroke();
    }
    const now = hist[n - 1], k = live.st ? live.st[j].now : null;
    g.textAlign = "left"; g.textBaseline = "top"; g.font = "600 11px system-ui"; g.fillStyle = "#e4e4e4";
    g.fillText(`${SHORT[j]} (${asg[j].kind})  기어 ${f(now.tg[j], 1)} N·m`, L, 4);
    if (k) { g.textAlign = "right"; g.fillStyle = ({ kg: "#5fbf86", kw: "#c9962a", kb: "#ff6b55" })[kc(k.gov[1])]; g.fillText(`여유 ${kf(k.gov[1])}`, W - R, 4); }
  }

  // ---------- 표 ----------
  function drawTable() {
    const rows = live.peak.map((P, j) => {
      const now = live.st ? live.st[j].now : null; if (!now) return "";
      const amp = Math.abs(P.tg) / Math.max(Math.abs(P.rig), 1e-6);
      return `<tr><td>${SHORT[j]}</td><td>${asg[j].kind}</td>
        <td>${f(now.tg, 1)}</td><td class="${kc(now.gov[1])}">${kf(now.gov[1])}</td>
        <td>${f(Math.abs(P.rig), 1)}</td><td>${f(Math.abs(P.tg), 1)}</td><td class="${amp > 1.5 ? "kw" : ""}">${f(amp, 2)}</td><td>${live.sepCount[j]}</td>
        <td>${f(Math.abs(P.tg) / sim.arm.effort[j] * 100, 0)}%</td>
        <td class="${kc(P.gear)}">${kf(P.gear)}</td><td class="${kc(P.planet)}">${kf(P.planet)}</td><td class="${kc(P.ob)}">${kf(P.ob)}</td>
        <td>${P.obW ? `${f(P.obW.Fa, 0)} / ${f(P.obW.Fr, 0)} / ${f(P.obW.M, 1)}` : "—"}</td>
        <td class="l ${kc(P.k)}">${P.part}${P.part === "기어" && P.gearWorst ? ` (${P.gearWorst.stage}단 ${P.gearWorst.part} ${P.gearWorst.mode})` : ""}</td></tr>`;
    }).join("");
    $("lvTable").innerHTML = `<table class="lv"><thead><tr><th>관절</th><th>종류</th><th>기어 토크<br>현재 N·m</th><th>여유<br>현재</th>
      <th>강체 피크<br>N·m</th><th>기어 피크<br>N·m</th><th>증폭</th><th>이 떨어짐<br>횟수</th><th>피크 /<br>URDF 한계</th>
      <th>기어<br>최저 여유</th><th>유성 베어링<br>최저 여유</th><th>출력 베어링<br>최저 여유</th><th>그때 출력 베어링 하중<br>Fa / Fr [N] / M [N·m]</th><th style="text-align:left">가장 먼저 닿는 부품</th></tr></thead><tbody>${rows}</tbody></table>`;
    const cont = sim.contacts.length > 0;
    $("lvNote").textContent = cont ? "● 몸과 접촉 중 — 접촉력은 출력 베어링 하중에 들어가지 않습니다 (충격 외력과 관성·중력만)." : "";
  }

  // ---------- 루프 ----------
  function loop(now) {
    if (live && live.enabled !== false && live.hist.length) {
      const row = live.hist[live.hist.length - 1];
      // 고리 색: 최근 0.5 s 최저 여유 유지
      for (let j = 0; j < 7; j++) {
        const k = row.k[j];
        if (k < holdK[j] || row.t - holdT[j] > 0.5) { holdK[j] = k; holdT[j] = row.t; }
        rings[j].material.color.setHex(ringColor(holdK[j]));
      }
      if (now - lastUi > 80) {
        lastUi = now;
        const hist = live.hist, t0 = hist[hist.length - 1].t - 3;
        let s = 0; while (s < hist.length && hist[s].t < t0) s++;
        const view = hist.slice(s);
        plots.forEach((c, j) => drawPlot(c, j, view));
        drawTable();
      }
    }
    requestAnimationFrame(loop);
  }
})();
