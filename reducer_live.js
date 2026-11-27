/* 관절 감속기 실시간 판정 — reducer.html 전용 (기본 시뮬레이터 app.js 위에 겹쳐 동작) */
(function () {
  "use strict";
  const $ = (id) => document.getElementById(id);
  const RC = window.ReducerCore, PRE = window.REDUCER_PRESETS, ARC = RC.ARCMIN;
  const SHORT = ["어깨 pitch", "어깨 roll", "어깨 yaw", "팔꿈치", "손목 roll", "손목 pitch", "손목 yaw"];
  const H2X_MAP = ["A", "B", "B", "B", "B", "C", "C"];   // URDF effort 111 / 53.7 / 24.8 N·m
  const kinds = {}, asg = [];
  let H, sim, live, sets = [], plots = [], lastUi = 0, nextId = 1, dzRes = [], shellCount = 0;
  const hold = Array.from({ length: 7 }, () => ({ t: -1, k: {} }));
  const f = (v, d = 1) => (Number.isFinite(v) ? v.toFixed(d) : "—");
  const kc = (k) => (!Number.isFinite(k) ? "kg" : k < 1 ? "kb" : k < 1.5 ? "kw" : "kg");
  const kf = (k) => (Number.isFinite(k) ? (k > 99 ? ">99" : k.toFixed(2)) : "—");
  const msg = (t) => { $("lvMsg").textContent = t; };

  // app.js가 3D·시뮬레이터를 다 만들 때까지 기다림
  (function wait() {
    H = window.__h2x;
    if (!H || !H.sim || !H.scene || !H.linkGroups || H.linkGroups.length !== 7) return setTimeout(wait, 100);
    setTimeout(init, 30);
  })();

  function init() {
    sim = H.sim;
    try { for (const k of Object.keys(PRE)) { kinds[k] = Object.assign({}, PRE[k]); loadKind(k, kinds[k].D); } }
    catch (e) { msg("감속기 계산 실패: " + e.message); return; }
    for (let j = 0; j < 7; j++) asg[j] = {};
    H2X_MAP.forEach((k, j) => setKind(j, k)); drawAsg(); showPresets();
    $("asgApply").onclick = apply;
    $("asgDef").onclick = () => { H2X_MAP.forEach((k, j) => setKind(j, k)); drawAsg(); apply(); };
    $("asgEng").onclick = () => { asg.forEach((a) => { const i = kinds[a.kind].info; a.K = i.K; a.bl = i.backlash; }); drawAsg(); apply(); };
    $("lvReset").onclick = () => { live.reset(); };
    $("lvOn").onchange = () => { live.enabled = $("lvOn").checked; if (live.enabled) live.reset(); };
    $("pasteBtn").onclick = paste;
    // 설계
    $("dzJoint").innerHTML = SHORT.map((n, j) => `<option value="${j}">${n}</option>`).join("");
    $("dzJoint").onchange = () => { const k = kinds[asg[+$("dzJoint").value].kind]; $("dzRatio").value = +k.info.ratio.toFixed(1); $("dzJm").value = k.Jm; };
    $("dzFromPeak").onclick = () => { const j = +$("dzJoint").value, P = live.peak[j]; const v = Math.max(Math.abs(P.tg), Math.abs(P.rig)); if (v > 0) $("dzT").value = +v.toFixed(1); else msg("아직 이 관절 피크 기록이 없습니다. 충격이나 동작을 먼저 주세요."); };
    $("dzRun").onclick = recommend;
    $("edKind").onchange = drawEdit; $("edApply").onclick = applyEdit;
    // 3D
    $("gsShow").onchange = () => sets.forEach((s) => { if (s) s.outer.visible = $("gsShow").checked; });
    $("gsShell").onchange = shell; $("gsScale").onchange = () => sets.forEach((s, j) => place(j));
    buildPlots();
    live = new RC.Live(sim, joints(), opts()).attach();
    apply(); refreshKindLists(); shell(); $("dzJoint").onchange();
    msg("실시간 판정 동작 중. 충격·랜덤 자세·하중을 바꾸면 바로 반영됩니다.");
    requestAnimationFrame(loop);
  }

  // ---------- 감속기 종류 ----------
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
    live.set(joints(), opts());
    buildSets();
  }
  function refreshKindLists() {
    const opt = Object.keys(kinds).map((k) => `<option>${k}</option>`).join("");
    const cur = $("edKind").value;
    $("edKind").innerHTML = opt; $("pasteTo").innerHTML = opt;
    if (kinds[cur]) $("edKind").value = cur;
    drawEdit();
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
  const stageTxt = (i) => i.stages.map((s, n) => `${n + 1}단 m${s.m} S${s.s}/P${s.p}/R${s.r}×${s.n} b${s.b}`).join(" + ");
  function showPresets() {
    $("presetInfo").innerHTML = Object.entries(kinds).map(([k, o]) => {
      const i = o.info, ob = o.D.outBearing;
      return `<div class="pcard"><b>${k}</b> ${o.label}<br>${stageTxt(i).replace(" + ", "<br>")}<br>i ${f(i.ratio, 3)}${i.sharedRing ? " · 링 공유" : ""} · 정적 한계 ${f(i.staticCap)} N·m · 강성 ${f(i.KarcMin, 1)} N·m/′ · 백래시 ${f(i.backlashArcmin, 2)}′<br>
        출력 베어링 C0 ${ob.C0} N · dp ${f(ob.dp)} mm · Jm ${o.Jm} (${o.src})</div>`;
    }).join("") + `<div class="note">A·B·C는 설계실 추천 엔진으로 자동 생성한 예시 (H2-X 실물 아님). 기어 목표 SH ${kinds.A.info.target.SH} · SF ${kinds.A.info.target.SF} (정적), 베어링 s0 ${kinds.A.info.target.s0}</div>`;
  }
  function addKind(D, label, src, Jm) {
    const k = "D" + nextId++;
    kinds[k] = { label, src, Jm, D };
    try { loadKind(k, D); } catch (e) { delete kinds[k]; nextId--; throw e; }
    refreshKindLists(); showPresets();
    return k;
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
      drawAsg(); showPresets(); apply(); refreshKindLists(); msg(`${k} 종류를 붙여 넣은 설계로 바꿨습니다.`);
    } catch (e) { msg("설계 적용 실패: " + e.message); }
  }

  // ---------- 감속비 · 기어 제원 추천 (설계실 recommend.js) ----------
  async function recommend() {
    const btn = $("dzRun"), out = $("dzOut");
    const list = (id) => $(id).value.split(/[,\s]+/).map(Number).filter((v) => v > 0);
    const Treq = +$("dzT").value, ratio = +$("dzRatio").value, OD = +$("dzOD").value, Hh = +$("dzH").value;
    const spec = { OD, H: Hh, ratio, ratioTolPct: Math.max(0, +$("dzTol").value || 0), mods: list("dzMods"), beta: 12, b: null, ns: list("dzNs").map(Math.round),
      modes: { one: $("dzOne").checked, shared: $("dzShared").checked, sep: $("dzSep").checked }, keep: { one: 12, shared: 24, sep: 24 } };
    if (!(Treq > 0 && ratio >= 2 && OD > 0 && Hh > 0 && spec.mods.length && spec.ns.length) || !(spec.modes.one || spec.modes.shared || spec.modes.sep)) { out.innerHTML = '<div class="note" style="color:#ff8a76">입력값을 확인하세요.</div>'; return; }
    const E = window.PGD, base = E.defaultDesign();
    Object.assign(base.req, { Trated: +(Treq / 3).toFixed(3), Tpeak: Treq, Temerg: +(Treq * 1.2).toFixed(3), rpm: 3000, lifeH: 5000 });
    base.loads.basis = "ideal";
    btn.disabled = true; out.innerHTML = '<div class="note">후보 찾는 중… (잇수 조합 → 외형·강도 계산)</div>';
    try {
      const r = await window.PGDRecommend.run(base, spec, E, window.PGD3D.layout, (d, n) => { out.innerHTML = `<div class="note">강도 계산 ${d} / ${n}</div>`; });
      const ok = r.res.filter((x) => x.ok).slice(0, 10);
      dzRes = ok.map((x) => ({ x, info: RC.reducerInfo(x.D) })).filter((z) => z.info.ok);
      dzRes.sort((a, b) => (b.info.staticCap >= Treq) - (a.info.staticCap >= Treq) || a.x.OD * a.x.OD * a.x.H - b.x.OD * b.x.OD * b.x.H);
      if (!dzRes.length) { out.innerHTML = `<div class="note" style="color:#ff8a76">조건을 만족하는 조합이 없습니다 (정수비 조합 ${r.total}개 중 강도 계산 ${r.checked}개). 외경·길이·편차를 늘리거나 모듈·유성 수 후보를 바꿔 보세요.</div>`; return; }
      out.innerHTML = `<div class="note">조합 ${r.total}개 → 강도 계산 ${r.checked}개 → 상위 ${dzRes.length}개 (요구 충족 먼저, 그다음 부피 작은 순)</div>
        <table class="asg" style="margin-top:4px"><thead><tr><th>구성 · 제원</th><th>i</th><th>정적 한계<br>N·m</th><th>요구<br>대비</th><th>외형<br>Ø×L</th><th></th></tr></thead><tbody>` +
        dzRes.map((z, n) => {
          const k = z.info.staticCap / Treq, type = { one: "1단", shared: "2단 공유", sep: "2단 분리" }[z.x.type] || z.x.type;
          return `<tr><td style="text-align:left;white-space:normal">${type}<br>${z.x.stages.map((s) => `m${s.m} ${s.s}/${s.p}/${s.R}×${s.n}`).join(" + ")}</td><td>${f(z.info.ratio, 2)}</td><td>${f(z.info.staticCap)}</td>
            <td class="${kc(k)}">${f(k, 2)}</td><td>${f(z.x.OD, 0)}×${f(z.x.H, 0)}</td><td><button data-n="${n}" style="padding:2px 6px">추가</button></td></tr>`;
        }).join("") + "</tbody></table>";
      out.querySelectorAll("button[data-n]").forEach((b) => { b.onclick = () => pick(+b.dataset.n); });
    } catch (e) { out.innerHTML = `<div class="note" style="color:#ff8a76">추천 실패: ${e.message}</div>`; console.error(e); }
    finally { btn.disabled = false; }
  }
  function pick(n) {
    const z = dzRes[n], j = +$("dzJoint").value;
    try {
      const k = addKind(z.x.D, `추천 · ${SHORT[j]}용 · 요구 ${$("dzT").value} N·m`, "직접 입력", Math.max(1e-5, +$("dzJm").value || 0.01));
      setKind(j, k); drawAsg(); apply();
      msg(`${k} 추가 → ${SHORT[j]}에 배치했습니다. 다른 관절에도 종류 칸에서 고를 수 있습니다.`);
    } catch (e) { msg("추가 실패: " + e.message); }
  }

  // ---------- 기어 제원 직접 수정 ----------
  function drawEdit() {
    const k = $("edKind").value; if (!kinds[k]) { $("edTable").innerHTML = ""; return; }
    const D = kinds[k].D, n = D.config.stages;
    const col = (key, label, step) => `<tr><td>${label}</td>${D.stages.slice(0, n).map((s, i) => `<td><input data-i="${i}" data-k="${key}" type="number" step="${step}" value="${s[key]}"></td>`).join("")}</tr>`;
    $("edTable").innerHTML = `<table class="asg"><thead><tr><th></th>${D.stages.slice(0, n).map((_, i) => `<th>${i + 1}단</th>`).join("")}</tr></thead><tbody>
      ${col("m", "모듈", 0.1)}${col("s", "썬 잇수", 1)}${col("p", "유성 잇수", 1)}${col("n", "유성 개수", 1)}${col("b", "치폭 mm", 0.5)}${col("beta", "비틀림각 °", 1)}</tbody></table>
      <div class="note">현재: ${stageTxt(kinds[k].info)} · i ${f(kinds[k].info.ratio, 3)} · 정적 한계 ${f(kinds[k].info.staticCap)} N·m${D.config.sharedRing ? " · 링 공유(2단 모듈·유성은 1단 링에 맞춰짐)" : ""}</div>`;
  }
  function applyEdit() {
    const k = $("edKind").value, D = JSON.parse(JSON.stringify(kinds[k].D));
    document.querySelectorAll("#edTable input").forEach((el) => { D.stages[+el.dataset.i][el.dataset.k] = +el.value; });
    D.stages.forEach((s) => { s.r = null; s.autoRingShift = true; s.xs = 0; s.xp = 0; s.xr = 0; });   // 링 잇수·전위는 엔진이 다시 정함
    const prev = { D: kinds[k].D, info: kinds[k].info, tab: kinds[k].tab };
    try {
      if (D.config.sharedRing && D.stages[1]) window.PGD.syncSharedRing(D);
      loadKind(k, D);
      asg.forEach((a, j) => { if (a.kind === k) setKind(j, k); });
      drawAsg(); showPresets(); apply(); drawEdit();
      msg(`${k} 제원을 바꿨습니다: ${stageTxt(kinds[k].info)} · i ${f(kinds[k].info.ratio, 3)}`);
    } catch (e) { Object.assign(kinds[k], prev); msg("적용 안 함 — 엔진 판정 오류: " + e.message); }
  }

  // ---------- 3D 기어세트 ----------
  function buildSets() {
    sets.forEach((s, j) => { if (s) { H.linkGroups[j].remove(s.outer); s.dispose(); } });
    const model = H.model;
    sets = asg.map((a, j) => {
      try {
        const s = window.ReducerGears.build(kinds[a.kind].info, kinds[a.kind].D);
        // 출력(캐리어·출력 베어링, 기어세트 +z 끝)은 자식 링크 쪽, 입력(1단 썬·모터)은 부모 쪽을 향하게:
        // 자식 링크가 관절축의 어느 방향에 있는지(다음 관절 위치, 없으면 무게중심을 축에 투영한 부호)로 기어세트 z를 ±축에 맞춤
        const jn = model.joints[j], nx = model.joints[j + 1], dot = (u, v) => u[0] * v[0] + u[1] * v[1] + u[2] * v[2];
        let side = nx ? dot(jn.axis, nx.xyz) : 0;
        if (Math.abs(side) < 1e-3) side = dot(jn.axis, jn.com);
        s.sign = side < 0 ? -1 : 1;
        s.outer.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), new THREE.Vector3(...jn.axis).normalize().multiplyScalar(s.sign));
        s.outer.visible = $("gsShow").checked;
        s.outer.traverse((o) => { o.userData.reducer = true; o.renderOrder = 3; });
        H.linkGroups[j].add(s.outer);
        return s;
      } catch (e) { console.error(e); msg("기어세트 3D 생성 실패: " + e.message); return null; }
    });
    sets.forEach((s, j) => place(j));
    hold.forEach((h) => { h.t = -1; });
  }
  function place(j) { const s = sets[j]; if (s) s.outer.scale.setScalar(0.001 * Math.max(0.5, Math.min(5, +$("gsScale").value || 1))); }
  // 로봇 외피 반투명: 장면의 로봇 메시(기어세트 제외) 재질을 투명으로
  function shell() {
    const on = $("gsShell").checked;
    H.scene.traverse((o) => {
      if (!o.isMesh || o.userData.reducer || !o.material || !o.material.isMeshStandardMaterial) return;
      const m = o.material;
      if (m.userData.baseOpacity === undefined) m.userData.baseOpacity = m.opacity;
      if (m.userData.shell === on) return;
      m.userData.shell = on; m.transparent = on; m.opacity = on ? 0.18 : m.userData.baseOpacity; m.depthWrite = !on; m.needsUpdate = true;
    });
  }

  // ---------- 그래프 ----------
  function buildPlots() {
    const box = $("lvPlots");
    plots = SHORT.map(() => { const d = document.createElement("div"); d.className = "pl"; const c = document.createElement("canvas"); d.appendChild(c); box.appendChild(d); return c; });
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
      for (let k = 0; k < n; k += step) {   // 구간 최대값 보존 다운샘플
        let v = hist[k][key][j]; for (let q = k + 1; q < Math.min(n, k + step); q++) if (Math.abs(hist[q][key][j]) > Math.abs(v)) v = hist[q][key][j];
        const x = X(hist[k].t), y = Y(v); k ? g.lineTo(x, y) : g.moveTo(x, y);
      }
      g.stroke();
    }
    const now = hist[n - 1], k = live.st ? live.st[j].now : null;
    g.textAlign = "left"; g.textBaseline = "top"; g.font = "600 11px system-ui"; g.fillStyle = "#e4e4e4";
    g.fillText(`${SHORT[j]} (${asg[j].kind}, i ${f(kinds[asg[j].kind].info.ratio, 1)})  기어 ${f(now.tg[j], 1)} N·m`, L, 4);
    if (k) { g.textAlign = "right"; g.fillStyle = ({ kg: "#5fbf86", kw: "#c9962a", kb: "#ff6b55" })[kc(k.gov[1])]; g.fillText(`여유 ${kf(k.gov[1])}`, W - R, 4); }
  }

  // ---------- 표 ----------
  function drawTable() {
    const rows = live.peak.map((P, j) => {
      const now = live.st ? live.st[j].now : null; if (!now) return "";
      const amp = Math.abs(P.tg) / Math.max(Math.abs(P.rig), 1e-6);
      return `<tr><td>${SHORT[j]}</td><td>${asg[j].kind}</td><td>${f(kinds[asg[j].kind].info.ratio, 1)}</td>
        <td>${f(now.tg, 1)}</td><td class="${kc(now.gov[1])}">${kf(now.gov[1])}</td>
        <td>${f(Math.abs(P.rig), 1)}</td><td>${f(Math.abs(P.tg), 1)}</td><td class="${amp > 1.5 ? "kw" : ""}">${f(amp, 2)}</td><td>${live.sepCount[j]}</td>
        <td>${f(Math.abs(P.tg) / sim.arm.effort[j] * 100, 0)}%</td>
        <td class="${kc(P.gear)}">${kf(P.gear)}</td><td class="${kc(P.planet)}">${kf(P.planet)}</td><td class="${kc(P.ob)}">${kf(P.ob)}</td>
        <td>${P.obW ? `${f(P.obW.Fa, 0)} / ${f(P.obW.Fr, 0)} / ${f(P.obW.M, 1)}` : "—"}</td>
        <td class="l ${kc(P.k)}">${P.part}${P.part === "기어" && P.gearWorst ? ` (${P.gearWorst.stage}단 ${P.gearWorst.part} ${P.gearWorst.mode})` : ""}</td></tr>`;
    }).join("");
    $("lvTable").innerHTML = `<table class="lv"><thead><tr><th>관절</th><th>종류</th><th>i</th><th>기어 토크<br>현재 N·m</th><th>여유<br>현재</th>
      <th>강체 피크<br>N·m</th><th>기어 피크<br>N·m</th><th>증폭</th><th>이 떨어짐<br>횟수</th><th>피크 /<br>URDF 한계</th>
      <th>기어<br>최저 여유</th><th>유성 베어링<br>최저 여유</th><th>출력 베어링<br>최저 여유</th><th>그때 출력 베어링 하중<br>Fa / Fr [N] / M [N·m]</th><th style="text-align:left">가장 먼저 닿는 부품</th></tr></thead><tbody>${rows}</tbody></table>`;
    $("lvNote").textContent = sim.contacts.length ? "● 몸과 접촉 중 — 접촉력은 출력 베어링 하중에 들어가지 않습니다 (충격 외력과 관성·중력만)." : "";
  }

  // ---------- 루프 ----------
  function loop(now) {
    if (++shellCount % 60 === 0 && $("gsShell").checked) shell();   // 늦게 불러온 로봇 메시(STL)도 반투명
    if (live && live.st) {
      const t = sim.t;
      sets.forEach((s, j) => {
        if (!s) return;
        s.pose(s.sign * sim.q[j]);   // 기어세트 z가 −축이면 관절 회전 방향도 반대로
        const st = live.st[j], jp = live.joints[j];
        if (!st || !st.now) return;
        // 부품별 현재 여유 → 0.5 s 동안 최저값 유지 후 색
        const cur = RC.partsAt(jp.tab, st.now.tg);
        cur.planet = cur.planet.map((v) => v / jp.info.target.s0); cur.ob = st.now.ob;
        const hd = hold[j];
        if (hd.t < 0 || t - hd.t > 0.5) { hd.k = {}; hd.t = t; }
        for (const [key, v] of Object.entries(cur)) {
          if (key === "planet") hd.k.planet = hd.k.planet ? hd.k.planet.map((x, i) => Math.min(x, v[i])) : v.slice();
          else hd.k[key] = Math.min(hd.k[key] ?? Infinity, v);
        }
        s.paint(hd.k);
      });
      if (now - lastUi > 80 && live.hist.length) {
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
