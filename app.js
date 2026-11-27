/* H2-X 오른팔 웹 시뮬레이터 — 화면 (three.js r128 + Canvas 2D 그래프) */
(function () {
  "use strict";
  const D = Math.PI / 180;
  const SHORT = ["1 어깨 pitch", "2 어깨 roll", "3 어깨 yaw", "4 팔꿈치", "5 손목 roll", "6 손목 pitch", "7 손목 yaw"];
  const EXT_POSE = [-90, 0, 0, 88, 0, 0, 0];
  const $ = (id) => document.getElementById(id);
  let model, sim, paused = false, stopMsg = "";

  // ------------------------------------------------------------------ 시작
  fetch("model.json").then((r) => r.json()).then((m) => {
    model = m;
    sim = new H2XDyn.Sim(model);
    buildControls();
    buildPlots();
    init3D();
    $("srcInfo").innerHTML =
      `모델: <a style="color:#7d9cb8" href="${m.source.repo}" target="_blank" rel="noopener">unitree_ros</a> @ ${m.source.commit.slice(0, 7)} · ${m.source.file}<br>` +
      `전신 ${m.total_mass.toFixed(2)} kg · 골반 높이 ${m.base_height.toFixed(4)} m · 나머지 관절 0 rad 고정 · 접촉 없음 · BSD-3-Clause`;
    window.__h2x = { get sim() { return sim; }, get arrow() { return arrow; }, get target() { return targetMesh; }, get scene() { return scene; }, get linkGroups() { return linkGroups; }, get model() { return model; } }; // 점검용 · 감속기 페이지(reducer.html)가 같은 화면에 겹쳐 그릴 때 사용
    requestAnimationFrame(loop);
  }).catch((e) => { $("status").textContent = "model.json 불러오기 실패: " + e; });

  // ------------------------------------------------------------------ 제어판
  const sliders = [], readouts = [];
  function buildControls() {
    const box = $("joints");
    model.joints.forEach((j, i) => {
      const lo = j.lower / D, hi = j.upper / D;
      const el = document.createElement("div");
      el.className = "j";
      el.innerHTML = `<div class="top"><span>${SHORT[i]}</span><span class="rng">${lo.toFixed(1)} ~ ${hi.toFixed(1)}° · 한계 ±${j.effort} N·m</span></div>
        <input type="range" min="${lo}" max="${hi}" step="0.1" value="0"><div class="rd"></div>`;
      box.appendChild(el);
      const s = el.querySelector("input");
      s.addEventListener("input", applyTargets);
      sliders.push(s); readouts.push(el.querySelector(".rd"));
    });
    $("armMass").value = sim.arm.armMass0.toFixed(4);
    $("armMass0").textContent = `원본 7링크 합 ${sim.arm.armMass0.toFixed(4)} kg`;
    $("btnRef").onclick = () => setSliders([0, 0, 0, 0, 0, 0, 0]);
    $("btnExt").onclick = () => setSliders(EXT_POSE);
    $("btnRand").onclick = randomPose;
    // 제어 방식
    document.querySelectorAll('input[name=mode]').forEach((r) => r.addEventListener("change", applyImp));
    $("btnImp").onclick = applyImp;
    ["impKt", "impKr", "impZeta", "impKn"].forEach((id) => $(id).addEventListener("keydown", (e) => { if (e.key === "Enter") applyImp(); }));
    $("btnImpact").onclick = impact;
    applyImp();
    $("btnMass").onclick = applyMass;
    ["armMass", "payMass", "ox", "oy", "oz"].forEach((id) => $(id).addEventListener("keydown", (e) => { if (e.key === "Enter") applyMass(); }));
    $("btnCtrl").onclick = applyCtrl;
    ["vmax", "amax"].forEach((id) => $(id).addEventListener("keydown", (e) => { if (e.key === "Enter") applyCtrl(); }));
    buildActTable(H2XDyn.ACT_DEF);
    $("btnActDef").onclick = () => { buildActTable(H2XDyn.ACT_DEF); applyCtrl(); };
    $("btnActZero").onclick = () => { const z = [0, 0, 0, 0, 0, 0, 0]; buildActTable({ J: z, b: z, tc: z }); applyCtrl(); };
    buildBodyControls();
    $("col").onchange = () => { sim.cfg.collision = $("col").checked; };
    $("showCol").onchange = () => { colGroup.visible = $("showCol").checked; };
    $("colInfo").textContent = `구 ${countSpheres()}개로 근사 · 어깨 관절부(몸통 쪽 반경 ${(sim.cfg.shoulderClear * 100).toFixed(0)} cm)와 인접 링크는 제외 · 비탄성 접촉`;
    $("ff").onchange = applyCtrl;
    $("yfix").onchange = () => { yfix = $("yfix").checked; };
    $("btnPause").onclick = togglePause;
    $("btnClear").onclick = () => { sim.clearRecords(); };
    $("btnCsv").onclick = saveCsv;
    $("btnStatic").onclick = runStatic;
    applyMass();
  }
  function buildActTable(d) {
    let h = "<tr><th>관절</th><th>J kg·m²</th><th>b</th><th>τc N·m</th></tr>";
    for (let i = 0; i < 7; i++) h += `<tr><td>${SHORT[i]}</td><td><input data-k="J" data-i="${i}" type="number" step="0.005" min="0" value="${d.J[i]}"></td><td><input data-k="b" data-i="${i}" type="number" step="0.1" min="0" value="${d.b[i]}"></td><td><input data-k="tc" data-i="${i}" type="number" step="0.05" min="0" value="${d.tc[i]}"></td></tr>`;
    $("actTable").innerHTML = h;
  }
  const BODY_UI = [["waist_yaw_joint", "허리 yaw"], ["waist_roll_joint", "허리 roll"], ["waist_pitch_joint", "허리 pitch"],
    ["head_pitch_joint", "머리 pitch"], ["head_yaw_joint", "머리 yaw"],
    ["left_shoulder_pitch_joint", "왼어깨 pitch"], ["left_shoulder_roll_joint", "왼어깨 roll"], ["left_shoulder_yaw_joint", "왼어깨 yaw"],
    ["left_elbow_joint", "왼팔꿈치"], ["left_wrist_roll_joint", "왼손목 roll"], ["left_wrist_pitch_joint", "왼손목 pitch"], ["left_wrist_yaw_joint", "왼손목 yaw"]];
  const bodySl = {};
  function buildBodyControls() {
    const lim = {}; model.body_joints.forEach((j) => { lim[j.name] = j; });
    const box = $("bodyJoints");
    BODY_UI.forEach(([n, label]) => {
      const j = lim[n], el = document.createElement("div"); el.className = "bj";
      el.innerHTML = `<span>${label}</span><input type="range" min="${(j.lower / D).toFixed(1)}" max="${(j.upper / D).toFixed(1)}" step="0.5" value="0"><span>0.0°</span>`;
      box.appendChild(el);
      const r = el.querySelector("input"), v = el.querySelectorAll("span")[1];
      r.addEventListener("input", () => { v.textContent = (+r.value).toFixed(1) + "°"; applyBody(); });
      bodySl[n] = { r, v };
    });
    const setB = (o) => { Object.entries(bodySl).forEach(([n, x]) => { x.r.value = o[n] || 0; x.v.textContent = (+x.r.value).toFixed(1) + "°"; }); applyBody(); };
    $("btnBodyZero").onclick = () => setB({});
    $("btnBodyLean").onclick = () => setB(Object.assign(curBody(), { waist_pitch_joint: 20 }));
    $("btnLeftDown").onclick = () => setB(Object.assign(curBody(), { left_elbow_joint: 89 }));
  }
  function curBody() { const o = {}; Object.entries(bodySl).forEach(([n, x]) => { o[n] = +x.r.value; }); return o; }
  function applyBody() {
    const o = {}; Object.entries(curBody()).forEach(([n, v]) => { o[n] = v * D; });
    sim.setBodyPose(o);
    if (renderer) updateBodyMeshes();
  }
  function countSpheres() { return model.links.reduce((a, l) => a + (l.spheres ? l.spheres.length : 0), 0); }
  function randomPose() {
    const q = sim.randomPose();
    if (!q) { alert("조건(충돌 없음·토크 여유)을 만족하는 자세를 찾지 못했습니다. 하중을 줄여 보세요."); return; }
    setSliders(q.map((v) => v / D));
    randArrivedAt = null;
  }
  let randArrivedAt = null;
  function autoRandomTick() {
    if (!$("autoRand").checked || paused) return;
    let arrived = true;
    for (let i = 0; i < 7; i++) if (Math.abs(sim.traj.target[i] - sim.traj.q[i]) > 1e-6 || Math.abs(sim.traj.v[i]) > 1e-6) arrived = false;
    if (!arrived) { randArrivedAt = null; return; }
    if (randArrivedAt === null) randArrivedAt = sim.t;
    if (sim.t - randArrivedAt >= (parseFloat($("randWait").value) || 0)) randomPose();
  }
  function applyImp() {
    try {
      const mode = document.querySelector('input[name=mode]:checked').value;
      sim.cfg.mode = mode;
      sim.cfg.impKt = num("impKt", 0); sim.cfg.impKr = num("impKr", 0); sim.cfg.impZeta = num("impZeta", 0); sim.cfg.impKnull = num("impKn", 0);
      $("impBox").style.opacity = mode === "impedance" ? 1 : 0.45;
      if (targetMesh) targetMesh.visible = mode === "impedance";
    } catch (e) { alert(e.message); }
  }
  const DIRS = [[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]];
  function impact() {
    try {
      const f = num("impF", 0), T = num("impT", 1) / 1000, sel = +$("impDir").value, link = +$("impAt").value;
      let d = sel < 6 ? DIRS[sel] : (() => { const v = [Math.random() - 0.5, Math.random() - 0.5, Math.random() - 0.5], n = Math.hypot(...v); return v.map((x) => x / n); })();
      // 몸통 기준 방향 → 월드
      const R = sim.arm.torsoR; d = [R[0] * d[0] + R[1] * d[1] + R[2] * d[2], R[3] * d[0] + R[4] * d[1] + R[5] * d[2], R[6] * d[0] + R[7] * d[1] + R[8] * d[2]];
      if (paused) togglePause();
      sim.applyImpulse(d.map((x) => x * f), T, link, link === 6 ? null : [0, 0, 0]);
      arrowHold = { until: performance.now() + 1200, F: d.map((x) => x * f), link };
    } catch (e) { alert(e.message); }
  }
  let arrowHold = null;
  function setSliders(deg) { sliders.forEach((s, i) => { s.value = deg[i]; }); applyTargets(); }
  function applyTargets() { sim.setTarget(sliders.map((s) => +s.value * D)); }
  function num(id, min) {
    const v = parseFloat($(id).value);
    if (!Number.isFinite(v) || (min !== undefined && v < min)) throw new Error("입력값을 확인하세요: " + ($(id).closest(".row") ? $(id).closest(".row").querySelector("label").textContent.trim() : id));
    return v;
  }
  function applyMass() {
    try {
      const arm = num("armMass", 1e-6), pay = num("payMass", 0), off = [num("ox"), num("oy"), num("oz")];
      if (arm <= 0) throw new Error("팔 질량은 0보다 커야 합니다");
      sim.setArmMass(arm);
      sim.setPayload(pay, off, true);
      const k = arm / sim.arm.armMass0;
      $("massInfo").textContent = `적용: 팔 ${arm.toFixed(4)} kg (질량·관성 ×${k.toFixed(3)}, 무게중심 유지) · 하중 ${pay} kg @ (${off.join(", ")}) m — 1초에 걸쳐 실림`;
      if (payloadMesh) payloadMesh.position.set(off[0], off[1], off[2]);
    } catch (e) { alert(e.message); }
  }
  function applyCtrl() {
    try {
      const v = num("vmax", 0.1), a = num("amax", 0.1);
      const act = { J: [], b: [], tc: [] };
      document.querySelectorAll("#actTable input").forEach((el) => {
        const x = parseFloat(el.value); if (!(x >= 0)) throw new Error("구동부 값은 0 이상이어야 합니다");
        act[el.dataset.k][+el.dataset.i] = x;
      });
      sim.cfg.vmaxDeg = v; sim.cfg.amaxDeg = a;
      sim.cfg.armature = act.J; sim.cfg.viscous = act.b; sim.cfg.coulomb = act.tc;
      sim.cfg.feedforward = $("ff").checked;
      sim.applyLimits();
    } catch (e) { alert(e.message); }
  }
  function togglePause() {
    paused = !paused;
    $("btnPause").textContent = paused ? "재개" : "일시정지";
    if (!paused) { stopMsg = ""; sim.satDur = 0; }
  }
  function saveCsv() {
    const blob = new Blob(["﻿" + sim.csv()], { type: "text/csv;charset=utf-8" });
    const a = document.createElement("a");
    const t = new Date(), p = (n) => String(n).padStart(2, "0");
    a.download = `h2x_right_arm_${t.getFullYear()}${p(t.getMonth() + 1)}${p(t.getDate())}_${p(t.getHours())}${p(t.getMinutes())}${p(t.getSeconds())}.csv`;
    a.href = URL.createObjectURL(blob); a.click(); setTimeout(() => URL.revokeObjectURL(a.href), 2000);
  }
  function runStatic() {
    const off = [num("ox"), num("oy"), num("oz")];
    $("staticOut").innerHTML = '<div class="note">계산 중…</div>';
    setTimeout(() => {
      const r = H2XDyn.staticPayloadTest(model, EXT_POSE.map((v) => v * D), 1.0, off, sim.arm.armMass, sim.bodyQ,
        { armature: sim.cfg.armature, viscous: sim.cfg.viscous, coulomb: sim.cfg.coulomb, collision: sim.cfg.collision });
      let h = `<div class="note" style="margin-top:8px">팔 뻗은 자세(어깨 pitch −90°, 팔꿈치 +88°) · 1 kg @ 손 (${off.join(", ")}) m · 팔 ${sim.arm.armMass.toFixed(3)} kg · 현재 몸 자세<br>Δτ 이론 = −m·aᵢ·((c−pᵢ)×g), 하중 중력모멘트를 실제 관절축에 투영</div>`;
      h += '<table class="st"><tr><th>관절</th><th>τ 0kg</th><th>τ 1kg</th><th>Δτ 시뮬</th><th>Δτ 이론</th><th>추종오차°</th><th>포화</th></tr>';
      for (let i = 0; i < 7; i++) h += `<tr><td>${SHORT[i]}</td><td>${r.tau0[i].toFixed(3)}</td><td>${r.tau1[i].toFixed(3)}</td><td>${r.dsim[i].toFixed(4)}</td><td>${r.theory[i].toFixed(4)}</td><td>${(Math.max(Math.abs(r.track0[i]), Math.abs(r.track1[i])) / D).toExponential(0)}</td><td>${r.sat0[i] || r.sat1[i] ? "Y" : "N"}</td></tr>`;
      $("staticOut").innerHTML = h + "</table>";
    }, 30);
  }

  // ------------------------------------------------------------------ 3D
  let renderer, scene, camera, linkGroups = [], allGroups = [], colGroup = null, payloadMesh = null, view, bodyT = null, targetMesh = null, arrow = null;
  function updateBodyMeshes() {
    bodyT = sim.body.fk(sim.bodyQ, sim.q);
    allGroups.forEach((g, i) => { g.matrix.copy(mat4(bodyT[i].R, bodyT[i].p)); g.matrixWorldNeedsUpdate = true; });
  }
  const orbit = { tgt: new THREE.Vector3(0.15, -0.2, 1.15), r: 2.1, az: -0.75, el: 0.2 };
  function init3D() {
    view = $("view");
    renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.outputEncoding = THREE.sRGBEncoding;
    renderer.shadowMap.enabled = true;
    view.appendChild(renderer.domElement);
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x0f0f10);
    camera = new THREE.PerspectiveCamera(38, 1, 0.02, 50);
    camera.up.set(0, 0, 1);
    scene.add(new THREE.HemisphereLight(0xdfe6ef, 0x26221e, 0.75));
    const key = new THREE.DirectionalLight(0xffffff, 1.0); key.position.set(2, -2.5, 4); key.castShadow = true;
    key.shadow.mapSize.set(2048, 2048); Object.assign(key.shadow.camera, { left: -1.5, right: 1.5, top: 1.5, bottom: -1.5 });
    scene.add(key);
    const rim = new THREE.DirectionalLight(0x9fb6d0, 0.55); rim.position.set(-2.5, 2, 2); scene.add(rim);
    const floor = new THREE.Mesh(new THREE.PlaneGeometry(6, 6), new THREE.MeshStandardMaterial({ color: 0x1d1e20, roughness: 0.95 }));
    floor.receiveShadow = true; scene.add(floor);
    const grid = new THREE.GridHelper(6, 30, 0x34363a, 0x26282b); grid.rotation.x = Math.PI / 2; grid.position.z = 0.001; scene.add(grid);

    // 전신 링크별 그룹 (오른팔 7개는 매 프레임, 나머지는 몸 자세가 바뀔 때 갱신)
    model.links.forEach(() => { const g = new THREE.Group(); g.matrixAutoUpdate = false; scene.add(g); allGroups.push(g); });
    const armLi = model.joints.map((j) => model.links.findIndex((l) => l.name === j.child));
    linkGroups = armLi.map((i) => allGroups[i]);
    payloadMesh = new THREE.Mesh(new THREE.SphereGeometry(0.03, 32, 16), new THREE.MeshStandardMaterial({ color: 0xe0642a, metalness: 0.2, roughness: 0.35 }));
    payloadMesh.position.set(0.12, 0, 0); payloadMesh.castShadow = true; linkGroups[6].add(payloadMesh);
    // 임피던스 목표(손 기준점) 표시, 충격 화살표
    targetMesh = new THREE.Mesh(new THREE.SphereGeometry(0.022, 24, 12), new THREE.MeshBasicMaterial({ color: 0x5fbf86, transparent: true, opacity: 0.55, depthWrite: false }));
    targetMesh.visible = false; scene.add(targetMesh);
    arrow = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(), 0.3, 0xff3b2a, 0.07, 0.04); arrow.visible = false; arrow.renderOrder = 10;
    [arrow.line.material, arrow.cone.material].forEach((m) => { m.depthTest = false; m.transparent = true; }); scene.add(arrow);
    // 충돌 구 표시
    colGroup = new THREE.Group(); colGroup.visible = false; scene.add(colGroup);
    const sg = new THREE.SphereGeometry(1, 14, 10);
    const matArm = new THREE.MeshBasicMaterial({ color: 0x5fbf86, transparent: true, opacity: 0.28, depthWrite: false });
    const matBody = new THREE.MeshBasicMaterial({ color: 0xc9962a, transparent: true, opacity: 0.18, depthWrite: false });
    const armSet = new Set(armLi);
    model.links.forEach((l, li) => (l.spheres || []).forEach((sp) => {
      const m = new THREE.Mesh(sg, armSet.has(li) ? matArm : matBody); m.position.set(sp[0], sp[1], sp[2]); m.scale.setScalar(sp[3]);
      const holder = new THREE.Group(); holder.matrixAutoUpdate = false; holder.add(m); holder.userData.li = li; colGroup.add(holder);
    }));
    const all = [];
    model.links.forEach((l, li) => l.meshes.forEach((m) => all.push(Object.assign({ li, arm: armSet.has(li) }, m))));
    let done = 0;
    all.forEach((m) => {
      fetch("meshes/" + m.file).then((r) => r.arrayBuffer()).then((buf) => {
        const geo = parseSTL(buf);
        const col = new THREE.Color(m.rgba[0], m.rgba[1], m.rgba[2]);
        if (m.arm) col.lerp(new THREE.Color(0x9fc3e6), 0.35);
        const mat = new THREE.MeshStandardMaterial({ color: col, metalness: m.arm ? 0.55 : 0.35, roughness: m.arm ? 0.32 : 0.45 });
        const mesh = new THREE.Mesh(geo, mat);
        mesh.castShadow = true; mesh.receiveShadow = true; mesh.matrixAutoUpdate = false;
        mesh.matrix.copy(mat4(m.R, m.p));
        allGroups[m.li].add(mesh);
      }).catch(() => {}).finally(() => {
        done++;
        $("loading").textContent = `3D 모델 불러오는 중… ${done}/${all.length}`;
        if (done === all.length) $("loading").style.display = "none";
      });
    });
    updateBodyMeshes();
    setupOrbit(renderer.domElement);
    window.addEventListener("resize", resize); resize();
  }
  function mat4(R, p) { return new THREE.Matrix4().set(R[0], R[1], R[2], p[0], R[3], R[4], R[5], p[1], R[6], R[7], R[8], p[2], 0, 0, 0, 1); }
  function parseSTL(buf) {
    const dv = new DataView(buf), n = dv.getUint32(80, true);
    const geo = new THREE.BufferGeometry();
    if (84 + n * 50 === buf.byteLength) {
      const pos = new Float32Array(n * 9), nor = new Float32Array(n * 9);
      for (let i = 0; i < n; i++) {
        const o = 84 + i * 50, nx = dv.getFloat32(o, true), ny = dv.getFloat32(o + 4, true), nz = dv.getFloat32(o + 8, true);
        for (let v = 0; v < 3; v++) {
          const b = o + 12 + v * 12, k = i * 9 + v * 3;
          pos[k] = dv.getFloat32(b, true); pos[k + 1] = dv.getFloat32(b + 4, true); pos[k + 2] = dv.getFloat32(b + 8, true);
          nor[k] = nx; nor[k + 1] = ny; nor[k + 2] = nz;
        }
      }
      geo.setAttribute("position", new THREE.BufferAttribute(pos, 3));
      geo.setAttribute("normal", new THREE.BufferAttribute(nor, 3));
      if (!(Math.abs(nor[0]) + Math.abs(nor[1]) + Math.abs(nor[2]) > 0.5)) geo.computeVertexNormals();
    } else { // ASCII
      const txt = new TextDecoder().decode(buf), v = [];
      txt.replace(/vertex\s+(\S+)\s+(\S+)\s+(\S+)/g, (_, a, b, c) => { v.push(+a, +b, +c); });
      geo.setAttribute("position", new THREE.BufferAttribute(new Float32Array(v), 3)); geo.computeVertexNormals();
    }
    return geo;
  }
  function setupOrbit(el) {
    let drag = null;
    el.addEventListener("contextmenu", (e) => e.preventDefault());
    el.addEventListener("pointerdown", (e) => { drag = { x: e.clientX, y: e.clientY, b: e.button }; el.setPointerCapture(e.pointerId); });
    el.addEventListener("pointerup", () => { drag = null; });
    el.addEventListener("pointermove", (e) => {
      if (!drag) return;
      const dx = e.clientX - drag.x, dy = e.clientY - drag.y; drag.x = e.clientX; drag.y = e.clientY;
      if (drag.b === 0 && !e.shiftKey) { orbit.az -= dx * 0.006; orbit.el = Math.max(-1.4, Math.min(1.45, orbit.el + dy * 0.005)); }
      else {
        const s = orbit.r * 0.0012, f = new THREE.Vector3(); camera.getWorldDirection(f);
        const right = new THREE.Vector3().crossVectors(f, camera.up).normalize(), up = new THREE.Vector3().crossVectors(right, f).normalize();
        orbit.tgt.addScaledVector(right, -dx * s).addScaledVector(up, dy * s);
      }
    });
    el.addEventListener("wheel", (e) => { e.preventDefault(); orbit.r = Math.max(0.4, Math.min(8, orbit.r * Math.exp(e.deltaY * 0.001))); }, { passive: false });
  }
  function resize() {
    const w = view.clientWidth, h = view.clientHeight;
    renderer.setSize(w, h); camera.aspect = w / Math.max(h, 1); camera.updateProjectionMatrix();
    plots.forEach((p) => p.resize());
  }
  function render3D() {
    const F = sim.arm.fk(sim.q);
    F.forEach((f, i) => { linkGroups[i].matrix.copy(mat4(f.R, f.p)); linkGroups[i].matrixWorldNeedsUpdate = true; });
    if (colGroup.visible) {
      const armLiMap = new Map(); linkGroups.forEach((g, i) => armLiMap.set(allGroups.indexOf(g), i));
      colGroup.children.forEach((h) => {
        const li = h.userData.li, ai = armLiMap.get(li);
        const t = ai !== undefined ? F[ai] : bodyT[li];
        h.matrix.copy(mat4(t.R, t.p)); h.matrixWorldNeedsUpdate = true;
      });
    }
    if (sim.cfg.mode === "impedance" && sim.imp.xd) { targetMesh.visible = true; targetMesh.position.set(sim.imp.xd[0], sim.imp.xd[1], sim.imp.xd[2]); } else targetMesh.visible = false;
    if (arrowHold && performance.now() < arrowHold.until) {
      const l = arrowHold.link, off = l === 6 ? sim.tcpLocal() : [0, 0, 0];
      const p = [0, 1, 2].map((k) => F[l].p[k] + F[l].R[3 * k] * off[0] + F[l].R[3 * k + 1] * off[1] + F[l].R[3 * k + 2] * off[2]);
      const Fm = Math.hypot(...arrowHold.F), dir = new THREE.Vector3(...arrowHold.F).normalize(), len = 0.12 + 0.25 * Math.min(1, Fm / 300);
      arrow.position.set(p[0] - dir.x * len, p[1] - dir.y * len, p[2] - dir.z * len); arrow.setDirection(dir); arrow.setLength(len, 0.07, 0.04); arrow.visible = true;
    } else arrow.visible = false;
    const m = sim.payloadMass;
    payloadMesh.visible = m > 1e-6;
    payloadMesh.scale.setScalar(Math.max(0.5, Math.min(2.2, Math.cbrt(Math.max(m, 1e-6)))));
    const ce = Math.cos(orbit.el);
    camera.position.set(orbit.tgt.x + orbit.r * ce * Math.cos(orbit.az), orbit.tgt.y + orbit.r * ce * Math.sin(orbit.az), orbit.tgt.z + orbit.r * Math.sin(orbit.el));
    camera.lookAt(orbit.tgt);
    renderer.render(scene, camera);
  }

  // ------------------------------------------------------------------ 그래프
  const plots = [];
  let yfix = false;
  function buildPlots() {
    const box = $("plots");
    for (let i = 0; i < 7; i++) {
      const d = document.createElement("div"); d.className = "pl";
      const c = document.createElement("canvas"); d.appendChild(c); box.appendChild(d);
      plots.push(new Plot(c, i));
    }
    const lg = document.createElement("div"); lg.className = "pl legend";
    lg.innerHTML = `<b>가로</b> 기준 자세(URDF 영점) 대비 실제 각변위 [°]<br><b>세로</b> 감속기 출력축 토크 [N·m]<br>
      = 구동토크 − 로터 반사관성×각가속도 − 구동부 마찰 (링크에 전달되는 토크)<br><br>
      궤적: <span style="color:#3b5b7a">어두움=과거</span> → <span style="color:#9fd0ff">밝음=최근</span> (최근 6 s)<br>
      <span style="color:#ff5a45">●</span> 현재 &nbsp; <span style="color:#5fbf86">┆</span> 목표각 &nbsp; <span style="color:#ff5a45">┄</span> 토크 한계(URDF effort)<br>
      제목이 빨갛고 SAT = 명령이 한계에서 잘림`;
    box.appendChild(lg);
  }
  class Plot {
    constructor(c, i) { this.c = c; this.i = i; this.ctx = c.getContext("2d"); this.ylim = 2; this.resize(); }
    resize() { const r = this.c.getBoundingClientRect(), d = window.devicePixelRatio || 1; this.w = r.width; this.h = r.height; this.c.width = r.width * d; this.c.height = r.height * d; this.ctx.setTransform(d, 0, 0, d, 0, 0); }
    draw(trail) {
      const i = this.i, ctx = this.ctx, W = this.w, H = this.h, eff = sim.arm.effort[i];
      const lo = (sim.arm.lower[i] - sim.ref[i]) / D, hi = (sim.arm.upper[i] - sim.ref[i]) / D, span = hi - lo;
      const x0 = lo - 0.03 * span, x1 = hi + 0.03 * span;
      if (yfix) this.ylim = eff * 1.08;
      else if (trail.length) {
        let m = 0.5; for (const r of trail) m = Math.max(m, Math.abs(r.y[i]));
        if (m > this.ylim * 0.95 || m < this.ylim * 0.3) this.ylim = nice(m * 1.4);
      }
      const yl = this.ylim, L = 44, R = 8, T = 22, B = 26, pw = W - L - R, ph = H - T - B;
      const X = (x) => L + (x - x0) / (x1 - x0) * pw, Y = (y) => T + (1 - (y + yl) / (2 * yl)) * ph;
      ctx.clearRect(0, 0, W, H);
      ctx.font = "10px Pretendard, system-ui, sans-serif"; ctx.lineWidth = 1;
      // 격자
      ctx.strokeStyle = "#2a2b2e"; ctx.fillStyle = "#8a8a8a"; ctx.textAlign = "right"; ctx.textBaseline = "middle";
      for (const t of ticks(-yl, yl, 5)) { ctx.beginPath(); ctx.moveTo(L, Y(t)); ctx.lineTo(L + pw, Y(t)); ctx.stroke(); ctx.fillText(fmt(t), L - 4, Y(t)); }
      ctx.textAlign = "center"; ctx.textBaseline = "top";
      for (const t of ticks(x0, x1, 6)) { ctx.beginPath(); ctx.moveTo(X(t), T); ctx.lineTo(X(t), T + ph); ctx.stroke(); ctx.fillText(fmt(t), X(t), T + ph + 4); }
      ctx.strokeStyle = "#4a4c50"; ctx.beginPath(); ctx.moveTo(L, Y(0)); ctx.lineTo(L + pw, Y(0)); ctx.moveTo(X(0), T); ctx.lineTo(X(0), T + ph); ctx.stroke();
      // 토크 한계
      ctx.save(); ctx.beginPath(); ctx.rect(L, T, pw, ph); ctx.clip();
      ctx.setLineDash([4, 3]); ctx.strokeStyle = "#ff5a45";
      for (const s of [1, -1]) if (eff <= yl * 1.001) { ctx.beginPath(); ctx.moveTo(L, Y(s * eff)); ctx.lineTo(L + pw, Y(s * eff)); ctx.stroke(); }
      // 목표각
      const tg = (sim.traj.target[i] - sim.ref[i]) / D;
      ctx.strokeStyle = "#5fbf86"; ctx.setLineDash([5, 4]); ctx.beginPath(); ctx.moveTo(X(tg), T); ctx.lineTo(X(tg), T + ph); ctx.stroke();
      ctx.setLineDash([]);
      // 궤적 (나이에 따른 색)
      const n = trail.length;
      if (n > 1) {
        const tEnd = trail[n - 1].t, Tw = sim.cfg.trailSec;
        ctx.lineWidth = 1.6;
        for (let k = 1; k < n; k++) {
          const a = 1 - Math.min(1, (tEnd - trail[k].t) / Tw);
          ctx.strokeStyle = `rgba(${Math.round(59 + 100 * a)},${Math.round(91 + 117 * a)},${Math.round(122 + 133 * a)},${0.25 + 0.75 * a})`;
          ctx.beginPath(); ctx.moveTo(X(trail[k - 1].x[i] / D), Y(trail[k - 1].y[i])); ctx.lineTo(X(trail[k].x[i] / D), Y(trail[k].y[i])); ctx.stroke();
        }
      }
      ctx.restore();
      let satRecent = false, cur = null;
      if (n) {
        cur = trail[n - 1];
        for (let k = Math.max(0, n - 50); k < n; k++) satRecent = satRecent || trail[k].s[i];
        ctx.fillStyle = "#ff5a45"; ctx.beginPath(); ctx.arc(X(cur.x[i] / D), Math.max(T, Math.min(T + ph, Y(cur.y[i]))), 4.5, 0, 2 * Math.PI); ctx.fill();
      }
      // 제목
      ctx.textAlign = "left"; ctx.textBaseline = "top"; ctx.font = "600 11.5px Pretendard, system-ui, sans-serif";
      ctx.fillStyle = satRecent ? "#ff6b55" : "#e4e4e4";
      ctx.fillText(`${SHORT[i]}   τ ${cur ? cur.y[i].toFixed(2) : "–"} N·m`, L, 4);
      ctx.textAlign = "right"; ctx.font = "600 10.5px Pretendard, system-ui, sans-serif";
      if (satRecent) { ctx.fillStyle = "#ff5a45"; ctx.fillText(`SAT 한계 ±${eff}`, W - R, 5); }
      else if (sim.satLatched[i]) { ctx.fillStyle = "#c9962a"; ctx.fillText("이전에 한계 도달", W - R, 5); }
    }
  }
  function nice(v) { const e = Math.pow(10, Math.floor(Math.log10(v))); for (const f of [1, 2, 2.5, 5, 10]) if (f * e >= v) return f * e; return 10 * e; }
  function ticks(a, b, n) { const st = nice((b - a) / n), o = []; for (let t = Math.ceil(a / st) * st; t <= b + 1e-9; t += st) o.push(Math.abs(t) < 1e-9 ? 0 : t); return o; }
  function fmt(t) { return Math.abs(t) >= 100 ? t.toFixed(0) : Math.abs(t) >= 10 ? t.toFixed(0) : t.toFixed(1).replace(/\.0$/, ""); }

  // ------------------------------------------------------------------ 루프
  let last = performance.now(), lastUi = 0, rtf = 1;
  function loop(now) {
    const wall = Math.min((now - last) / 1000, 0.05); last = now;
    if (!paused) {
      const n = Math.max(1, Math.round(wall / sim.cfg.dt));
      const t0 = performance.now();
      sim.step(n);
      const cost = (performance.now() - t0) / 1000;
      rtf = 0.9 * rtf + 0.1 * Math.min(1, wall / Math.max(cost, wall));
      if ($("autoStop").checked && sim.satDur > 0.3) {
        const names = sim.sat.map((s, i) => (s ? SHORT[i].slice(2) : "")).filter(Boolean).join(", ");
        paused = true; $("btnPause").textContent = "재개";
        stopMsg = `■ 자동 일시정지: ${names} 토크 한계 도달\n  하중을 줄이거나 자세를 바꾼 뒤 [재개]\n  (계속 보려면 '자동 일시정지' 해제)`;
      }
    }
    autoRandomTick();
    render3D();
    if (now - lastUi > 50) { lastUi = now; updateUi(); }
    requestAnimationFrame(loop);
  }
  function updateUi() {
    const tr = sim.trailWindow();
    plots.forEach((p) => p.draw(tr));
    for (let i = 0; i < 7; i++) {
      readouts[i].textContent = `${(sim.traj.target[i] / D).toFixed(1).padStart(6)} / ${(sim.traj.q[i] / D).toFixed(1).padStart(6)} / ${(sim.q[i] / D).toFixed(1).padStart(6)}°   τ ${sim.tauOut[i].toFixed(2).padStart(7)}${sim.sat[i] ? "  ⚠SAT" : ""}${sim.limHit[i] ? "  ⛔범위끝" : ""}`;
      readouts[i].className = "rd" + (sim.sat[i] ? " sat" : "");
    }
    const nsat = sim.sat.filter(Boolean).length;
    const lines = [];
    if (stopMsg) lines.push(stopMsg);
    lines.push(`t ${sim.t.toFixed(2)} s · ${paused ? "정지" : "실시간"} · 하중 ${sim.payloadMass.toFixed(2)}/${sim.payloadTarget} kg · CSV ${sim.log.length}샘플`);
    lines.push(nsat ? `⚠ 토크 한계: ${sim.sat.map((s, i) => (s ? SHORT[i].slice(2) : "")).filter(Boolean).join(", ")} — 이 하중·자세는 버틸 수 없음` : "토크 한계 미도달");
    if (sim.cfg.mode === "impedance") lines.push(`임피던스: 손 위치 오차 ${(Math.hypot(...sim.imp.ex) * 1000).toFixed(1)} mm · 자세 오차 ${(Math.hypot(...sim.imp.er) / D).toFixed(1)}° · 스프링 힘 ${Math.hypot(...sim.imp.F).toFixed(1)} N`);
    if (Math.hypot(...sim.extForce) > 0.01) lines.push(`➜ 충격 외력 ${Math.hypot(...sim.extForce).toFixed(0)} N`);
    if (sim.contacts.length) {
      const by = {}; sim.contacts.forEach((c) => { const k = `${SHORT[sim.names.indexOf(c.link)].slice(2)} ↔ ${c.other.replace("_link", "").replace("right_", "오른 ").replace("left_", "왼 ")}`; by[k] = (by[k] || 0) + c.force; });
      lines.push("● 접촉: " + Object.entries(by).map(([k, f]) => `${k} ${f.toFixed(0)} N`).join(", "));
      if (nsat) lines.push("  → 목표 자세가 몸에 막혀 팔이 밀어붙이는 중 (목표각을 몸 밖으로)");
    }
    $("status").textContent = lines.join("\n");
    $("status").className = nsat || stopMsg ? "alert" : "";
  }
})();
