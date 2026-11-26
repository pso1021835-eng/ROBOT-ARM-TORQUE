/* H2-X 오른팔 7축 강체 동역학 · 궤적 · 제어 (브라우저/Node 공용, 외부 라이브러리 없음)
 *
 * - 골반 고정, 오른팔 외 관절은 기립 자세(0 rad)로 고정 → 몸통(torso_link) 프레임이 고정 베이스
 * - 링크 질량·무게중심·관성텐서는 URDF 원본 (model.json)
 * - 역동역학: 재귀 뉴턴–오일러(RNEA), 질량행렬: 단위 가속도 RNEA 7회, 적분: 반암시적 오일러 1 ms
 * - 구동 토크 tauAct = 관절(감속기 출력축) 토크, 토크 한계 = URDF effort 로 클립
 * - 가동범위: 범위를 넘을 때만 작동하는 강성·감쇠 벽(제약 토크, 구동 토크와 별도 기록)
 */
(function (root, factory) {
  if (typeof module === "object" && module.exports) module.exports = factory();
  else root.H2XDyn = factory();
})(typeof self !== "undefined" ? self : this, function () {
  "use strict";
  const N = 7;
  const PAYLOAD_R = 0.03;

  // ---------- 3x3 / 3-vector 유틸 (행 우선 배열 길이 9) ----------
  const mm = (A, B) => {
    const C = new Array(9);
    for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++)
      C[3 * i + j] = A[3 * i] * B[j] + A[3 * i + 1] * B[3 + j] + A[3 * i + 2] * B[6 + j];
    return C;
  };
  const mv = (A, v) => [A[0] * v[0] + A[1] * v[1] + A[2] * v[2], A[3] * v[0] + A[4] * v[1] + A[5] * v[2], A[6] * v[0] + A[7] * v[1] + A[8] * v[2]];
  const tr = (A) => [A[0], A[3], A[6], A[1], A[4], A[7], A[2], A[5], A[8]];
  const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
  const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
  const scl = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
  const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
  function axisAngle(a, t) {
    const c = Math.cos(t), s = Math.sin(t), v = 1 - c, [x, y, z] = a;
    return [c + x * x * v, x * y * v - z * s, x * z * v + y * s,
            y * x * v + z * s, c + y * y * v, y * z * v - x * s,
            z * x * v - y * s, z * y * v + x * s, c + z * z * v];
  }
  const clamp = (x, lo, hi) => Math.min(hi, Math.max(lo, x));

  // 대칭 양정치 행렬 풀이 (Cholesky)
  function cholSolve(A, b, n) {
    const L = new Float64Array(n * n);
    for (let i = 0; i < n; i++) for (let j = 0; j <= i; j++) {
      let s = A[i * n + j];
      for (let k = 0; k < j; k++) s -= L[i * n + k] * L[j * n + k];
      if (i === j) L[i * n + i] = Math.sqrt(Math.max(s, 1e-14));
      else L[i * n + j] = s / L[j * n + j];
    }
    const y = new Float64Array(n), x = new Float64Array(n);
    for (let i = 0; i < n; i++) { let s = b[i]; for (let k = 0; k < i; k++) s -= L[i * n + k] * y[k]; y[i] = s / L[i * n + i]; }
    for (let i = n - 1; i >= 0; i--) { let s = y[i]; for (let k = i + 1; k < n; k++) s -= L[k * n + i] * x[k]; x[i] = s / L[i * n + i]; }
    return x;
  }
  function invDiag(A, n) { // diag(A^-1)
    const d = new Float64Array(n);
    for (let j = 0; j < n; j++) { const e = new Float64Array(n); e[j] = 1; d[j] = cholSolve(A, e, n)[j]; }
    return d;
  }

  function invFull(A, n) {
    const X = new Float64Array(n * n);
    for (let j = 0; j < n; j++) { const e = new Float64Array(n); e[j] = 1; const c = cholSolve(A, e, n); for (let i = 0; i < n; i++) X[i * n + j] = c[i]; }
    return X;
  }
  function invFull6(A) {
    const X = new Float64Array(36);
    for (let j = 0; j < 6; j++) { const e = new Float64Array(6); e[j] = 1; const col = cholSolve(A, e, 6); for (let i = 0; i < 6; i++) X[i * 6 + j] = col[i]; }
    return X;
  }
  function boundSphere(sp) {
    if (!sp.length) return [0, 0, 0, -1];
    let c = [0, 0, 0]; for (const s of sp) c = add(c, s); c = scl(c, 1 / sp.length);
    let r = 0; for (const s of sp) r = Math.max(r, Math.hypot(s[0] - c[0], s[1] - c[1], s[2] - c[2]) + s[3]);
    return [c[0], c[1], c[2], r];
  }
  /* 관절별 독립 피드백의 관성 스케일 D.
   * 기본은 diag(M). 폐루프는 q̈ = M⁻¹·D·(Kp e + Kd ė) 이므로 λ = eig(D½ M⁻¹ D½)가 클수록
   * 이산시간(1 ms)에서 불안정해진다. λmax ≤ lambdaMax 가 되도록 D를 일괄 축소한다.
   * 어떤 관절이 포화(개루프)돼도 남은 관절의 행렬은 주부분행렬이라 λ가 더 커지지 않는다. */
  function feedbackInertia(M, Minv, lambdaMax) {
    // 관절별 '나머지 관절이 풀렸을 때의 유효관성' 1/(M⁻¹)ᵢᵢ 기준 → D½M⁻¹D½ 대각이 1 (가벼운 손목 때문에 어깨 게인이 깎이지 않음)
    const D = Array.from({ length: N }, (_, i) => 1 / Minv[i * N + i]);
    const A = new Float64Array(N * N);
    for (let i = 0; i < N; i++) for (let j = 0; j < N; j++) A[i * N + j] = Math.sqrt(D[i] * D[j]) * Minv[i * N + j];
    let v = new Float64Array(N).fill(1), lam = 0;
    for (let it = 0; it < 60; it++) {
      const w = new Float64Array(N);
      for (let i = 0; i < N; i++) { let s = 0; for (let j = 0; j < N; j++) s += A[i * N + j] * v[j]; w[i] = s; }
      const nw = Math.hypot(...w); lam = nw / Math.hypot(...v); for (let i = 0; i < N; i++) v[i] = w[i] / nw;
    }
    const s = Math.min(1, lambdaMax / Math.max(lam * 1.02, 1e-12));
    return D.map((d) => d * s);
  }

  // ---------- 궤적 생성기 (Python traj.py와 동일 규칙) ----------
  class Trajectory {
    constructor(q0, lower, upper) {
      this.q = Float64Array.from(q0); this.v = new Float64Array(N); this.a = new Float64Array(N);
      this.target = Float64Array.from(q0); this.lower = lower; this.upper = upper;
      this.vmax = new Float64Array(N).fill(1); this.amax = new Float64Array(N).fill(1);
    }
    setLimits(vmax, amax) { for (let i = 0; i < N; i++) { this.vmax[i] = vmax[i]; this.amax[i] = amax[i]; } }
    setTarget(t) { for (let i = 0; i < N; i++) this.target[i] = clamp(t[i], this.lower[i], this.upper[i]); }
    reset(q) { for (let i = 0; i < N; i++) { this.q[i] = q[i]; this.v[i] = 0; this.a[i] = 0; this.target[i] = q[i]; } }
    step(dt) {
      for (let i = 0; i < N; i++) {
        const e = this.target[i] - this.q[i];
        const eEff = Math.max(Math.abs(e) - Math.abs(this.v[i]) * dt, 0);
        const vStar = Math.sign(e) * Math.min(this.vmax[i], Math.sqrt(2 * this.amax[i] * eEff));
        let a = clamp((vStar - this.v[i]) / dt, -this.amax[i], this.amax[i]);
        let vNew = this.v[i] + a * dt;
        let qNew = this.q[i] + 0.5 * (this.v[i] + vNew) * dt;
        const crossed = e !== 0 && Math.sign(this.target[i] - qNew) !== Math.sign(e);
        const near = Math.abs(e) < 1e-7 && Math.abs(this.v[i]) < this.amax[i] * dt * 1.5;
        if (crossed || near) { qNew = this.target[i]; a = -this.v[i] / dt; vNew = 0; }
        this.q[i] = clamp(qNew, this.lower[i], this.upper[i]); this.v[i] = vNew; this.a[i] = a;
      }
    }
  }

  // ---------- 팔 모델 ----------
  class Arm {
    constructor(model) {
      this.model = model;
      this.J = model.joints;
      this.lower = Float64Array.from(this.J.map((j) => j.lower));
      this.upper = Float64Array.from(this.J.map((j) => j.upper));
      this.effort = Float64Array.from(this.J.map((j) => j.effort));
      this.urdfVel = Float64Array.from(this.J.map((j) => j.velocity));
      this.torsoR = model.torso.R; this.torsoP = model.torso.p;
      this.g = model.gravity.slice();
      this.mass0 = this.J.map((j) => j.mass);
      this.armMass0 = this.mass0.reduce((a, b) => a + b, 0);
      this.armMass = this.armMass0;
      this.payload = { mass: 0, off: [0.12, 0, 0] };
      this.body = this.J.map((j) => ({ m: j.mass, c: j.com.slice(), I: j.inertia.slice() }));
      this._rebuildBodies();
    }
    // 팔 질량 변경: 링크별 질량·관성텐서 같은 비율, 무게중심 유지
    setArmMass(total) {
      if (!(total > 0)) throw new Error("팔 질량은 0보다 커야 합니다");
      this.armMass = total; this._rebuildBodies();
    }
    setPayload(mass, off) {
      if (!(mass >= 0)) throw new Error("하중 질량은 0 이상");
      this.payload = { mass, off: off.slice() }; this._rebuildBodies();
    }
    _rebuildBodies() {
      const k = this.armMass / this.armMass0;
      this.bodies = this.J.map((j) => ({ m: j.mass * k, c: j.com.slice(), I: j.inertia.map((v) => v * k) }));
      // 손 하중: 손 링크(7번)에 강체로 합침 (균질구 r=3 cm, 평행축 정리)
      const pm = this.payload.mass;
      if (pm > 0) {
        const b = this.bodies[N - 1], o = this.payload.off;
        const M = b.m + pm;
        const c = scl(add(scl(b.c, b.m), scl(o, pm)), 1 / M);
        const shift = (I, m, r) => { // 무게중심 관성 → 점 c 기준
          const d = sub(r, c), dd = dot(d, d);
          return I.map((v, idx) => v + m * ((idx % 4 === 0 ? dd : 0) - d[Math.floor(idx / 3)] * d[idx % 3]));
        };
        const Ip = [1, 0, 0, 0, 1, 0, 0, 0, 1].map((v) => v * 0.4 * pm * PAYLOAD_R * PAYLOAD_R);
        const Ib = shift(b.I, b.m, b.c), Ipp = shift(Ip, pm, o);
        this.bodies[N - 1] = { m: M, c, I: Ib.map((v, i) => v + Ipp[i]) };
      }
    }
    // 순기구학: 링크 프레임 R,p (월드), 관절축 z (월드)
    fk(q) {
      const out = [];
      let Rp = this.torsoR, pp = this.torsoP;
      for (let i = 0; i < N; i++) {
        const j = this.J[i];
        const R = mm(mm(Rp, j.R0), axisAngle(j.axis, q[i]));
        const p = add(pp, mv(Rp, j.xyz));
        out.push({ R, p, z: mv(R, j.axis) });
        Rp = R; pp = p;
      }
      return out;
    }
    // RNEA: τ = M(q)qdd + C(q,qd)qd + G(q)   (grav=false면 중력 제외)
    rnea(q, qd, qdd, grav = true, F = null) {
      F = F || this.fk(q);
      const w = [], al = [], ao = [], comW = [], Iw = [];
      let wp = [0, 0, 0], alp = [0, 0, 0], aop = grav ? scl(this.g, -1) : [0, 0, 0], pp = this.torsoP;
      for (let i = 0; i < N; i++) {
        const { R, p, z } = F[i];
        const wi = add(wp, scl(z, qd[i]));
        const ali = add(add(alp, scl(z, qdd[i])), cross(wp, scl(z, qd[i])));
        const r = sub(p, pp);
        const aoi = add(add(aop, cross(alp, r)), cross(wp, cross(wp, r)));
        w.push(wi); al.push(ali); ao.push(aoi);
        comW.push(add(p, mv(R, this.bodies[i].c)));
        Iw.push(mm(mm(R, this.bodies[i].I), tr(R)));
        wp = wi; alp = ali; aop = aoi; pp = p;
      }
      const tau = new Float64Array(N);
      let fc = [0, 0, 0], nc = [0, 0, 0], pc = null;
      for (let i = N - 1; i >= 0; i--) {
        const { p, z } = F[i], b = this.bodies[i];
        const rc = sub(comW[i], p);
        const ac = add(add(ao[i], cross(al[i], rc)), cross(w[i], cross(w[i], rc)));
        const Fi = scl(ac, b.m);
        const Ni = add(mv(Iw[i], al[i]), cross(w[i], mv(Iw[i], w[i])));
        let f = add(Fi, fc);
        let n = add(add(Ni, cross(rc, Fi)), nc);
        if (pc) n = add(n, cross(sub(pc, p), fc));
        tau[i] = dot(z, n);
        fc = f; nc = n; pc = p;
      }
      return tau;
    }
    massMatrix(q, F) {
      const M = new Float64Array(N * N), z = new Float64Array(N);
      for (let j = 0; j < N; j++) {
        const e = new Float64Array(N); e[j] = 1;
        const col = this.rnea(q, z, e, false, F);
        for (let i = 0; i < N; i++) M[i * N + j] = col[i];
      }
      for (let i = 0; i < N; i++) for (let j = 0; j < i; j++) { const s = 0.5 * (M[i * N + j] + M[j * N + i]); M[i * N + j] = M[j * N + i] = s; }
      return M;
    }
    gravityTorque(q) { return this.rnea(q, new Float64Array(N), new Float64Array(N), true); }
    handPoint(q, off) { const F = this.fk(q); return add(F[N - 1].p, mv(F[N - 1].R, off)); }
  }

  // ---------- 전신 순기구학 (몸 자세 · 충돌 구) ----------
  class Body {
    constructor(model) {
      this.links = model.links;
      this.idx = {}; this.links.forEach((l, i) => { this.idx[l.name] = i; });
      this.armJointIndex = {}; model.joints.forEach((j, i) => { this.armJointIndex[j.name] = i; });
      this.base = [0, 0, model.base_height];
    }
    // qBody: {관절이름: rad}, qArm: 오른팔 7축 → 링크별 {R, p}
    fk(qBody, qArm) {
      const T = new Array(this.links.length);
      this.links.forEach((l, i) => {
        if (!l.joint) { T[i] = { R: [1, 0, 0, 0, 1, 0, 0, 0, 1], p: this.base.slice() }; return; }
        const P = T[this.idx[l.parent]], j = l.joint;
        let R = mm(P.R, j.R0);
        if (j.type === "revolute" || j.type === "continuous") {
          const ai = this.armJointIndex[j.name];
          const q = ai !== undefined ? (qArm ? qArm[ai] : 0) : (qBody[j.name] || 0);
          R = mm(R, axisAngle(j.axis, q));
        }
        T[i] = { R, p: add(P.p, mv(P.R, j.xyz)) };
      });
      return T;
    }
  }

  // ---------- 시뮬레이터 ----------
  // 구동부 가정값(URDF에 없음): 반사관성 J [kg·m²], 점성 b [N·m·s/rad], 쿨롱 τc [N·m]
  const ACT_DEF = { J: [0.03, 0.025, 0.025, 0.025, 0.01, 0.01, 0.01], b: [1, 1, 1, 1, 0.5, 0.5, 0.5], tc: [1.0, 0.5, 0.5, 0.5, 0.25, 0.25, 0.25] };
  const DEF = { dt: 0.001, vmaxDeg: 60, amaxDeg: 240, wn: 60, zeta: 1, eMaxDeg: 10, edMax: 3, lambdaMax: 3,
                feedforward: true, payloadRamp: 1.0, logHz: 100, trailHz: 100, trailSec: 6,
                armature: ACT_DEF.J.slice(), viscous: ACT_DEF.b.slice(), coulomb: ACT_DEF.tc.slice(), vStick: 0.02,
                collision: true, contactMargin: 0.002, shoulderClear: 0.13,
                // 제어 방식: "position"(관절 위치 추종) | "impedance"(손 끝 직교좌표 임피던스)
                mode: "position", impKt: 300, impKr: 10, impZeta: 0.8, impKnull: 5, impDnull: 1, impEmax: 0.1, impRmax: 0.5 };
  const fric = (c, i, v) => c.viscous[i] * v + c.coulomb[i] * Math.tanh(v / c.vStick);

  class Sim {
    constructor(model, cfg = {}) {
      this.cfg = Object.assign({}, DEF, cfg);
      ["armature", "viscous", "coulomb"].forEach((k) => { this.cfg[k] = this.cfg[k].slice(); });
      this.model = model;
      this.arm = new Arm(model);
      this.body = new Body(model);
      this.n = N;
      this.names = model.joints.map((j) => j.name);
      this.ref = new Float64Array(N);
      this.traj = new Trajectory(this.ref, this.arm.lower, this.arm.upper);
      this.payloadTarget = 0; this.payloadMass = 0;
      this.bodyQ = {};
      this._setupCollision();
      this.setBodyPose({});
      this.reset();
      this.applyLimits();
    }
    // ----- 몸 자세 (허리·머리·왼팔·다리; 골반 고정) -----
    setBodyPose(qBody) {
      this.bodyQ = Object.assign({}, qBody);
      const T = this.body.fk(this.bodyQ, null);
      const t = T[this.body.idx[this.model.torso_link]];
      this.arm.torsoR = t.R; this.arm.torsoP = t.p;
      this.bodyT = T;
      this._bodySpheresWorld(T);
    }
    // ----- 충돌: 오른팔(+손 하중) 구 vs 몸 구, 오른팔 자기충돌 -----
    _setupCollision() {
      const B = this.body, armIdx = this.model.joints.map((j) => B.idx[j.child]);
      this.armLinkIdx = armIdx;
      const isArm = new Set(armIdx);
      this.armSph = armIdx.map((li) => (B.links[li].spheres || []).map((s) => s.slice()));
      this.bodyLinks = B.links.map((l, i) => i).filter((i) => !isArm.has(i) && (B.links[i].spheres || []).length);
      // 어깨 관절 하우징: 몸통 구 중 오른어깨 pitch 관절 원점에서 shoulderClear 이내는 오른팔 충돌에서 제외
      const tl = B.idx[this.model.torso_link], sp0 = this.model.joints[0].xyz;
      this.torsoKeep = B.links[tl].spheres.map((s) => Math.hypot(s[0] - sp0[0], s[1] - sp0[1], s[2] - sp0[2]) > this.cfg.shoulderClear);
      this.torsoIdx = tl;
      // 기준 자세(모두 0)에서 이미 겹치는 링크 쌍은 관절부 겹침으로 보고 제외
      const T0 = B.fk({}, new Float64Array(N));
      const W = (li, s, T) => { const t = T[li]; const c = add(t.p, mv(t.R, s)); return [c[0], c[1], c[2], s[3]]; };
      const sphOf = (li) => B.links[li].spheres.filter((s, k) => li !== tl || this.torsoKeep[k]);
      const overlap = (la, lb, T) => {
        for (const a of sphOf(la)) { const A = W(la, a, T);
          for (const b of sphOf(lb)) { const Bb = W(lb, b, T);
            if (Math.hypot(A[0] - Bb[0], A[1] - Bb[1], A[2] - Bb[2]) < A[3] + Bb[3]) return true; } }
        return false;
      };
      this.excl = new Set();
      for (let a = 0; a < N; a++) {
        for (const lb of this.bodyLinks) if (overlap(armIdx[a], lb, T0)) this.excl.add(`${a}|b${lb}`);
        for (let b = 0; b < a; b++) if (a - b <= 1 || overlap(armIdx[a], armIdx[b], T0)) this.excl.add(`${a}|a${b}`);
      }
      this.excludedPairs = Array.from(this.excl).map((k) => { const [a, r] = k.split("|");
        return `${this.names[+a]} ↔ ${r[0] === "b" ? B.links[+r.slice(1)].name : this.names[+r.slice(1)]}`; });
    }
    _bodySpheresWorld(T) {
      this.bodyW = this.bodyLinks.map((li) => {
        const t = T[li], src = this.body.links[li].spheres.filter((s, k) => li !== this.torsoIdx || this.torsoKeep[k]);
        const sp = src.map((s) => { const c = add(t.p, mv(t.R, s)); return [c[0], c[1], c[2], s[3]]; });
        return { li, sp, bound: boundSphere(sp) };
      });
    }
    _armSpheresWorld(F) {
      const out = this.armSph.map((list, a) => list.map((s) => { const c = add(F[a].p, mv(F[a].R, s)); return [c[0], c[1], c[2], s[3]]; }));
      if (this.payloadMass > 1e-6) { // 손 하중 구 (화면과 같은 크기)
        const r = 0.03 * Math.max(0.5, Math.min(2.2, Math.cbrt(this.payloadMass)));
        const c = add(F[N - 1].p, mv(F[N - 1].R, this.arm.payload.off)); out[N - 1] = out[N - 1].concat([[c[0], c[1], c[2], r]]);
      }
      return out;
    }
    _contacts(F) {
      const arm = this._armSpheresWorld(F), bounds = arm.map(boundSphere), res = [], m0 = this.cfg.contactMargin;
      const pairs = (a, A, Blist, key, other, Cb) => {
        let found = [];
        for (const s of A) {
          if (Cb) { const ex = s[0] - Cb[0], ey = s[1] - Cb[1], ez = s[2] - Cb[2], rr = s[3] + Cb[3] + m0; if (ex * ex + ey * ey + ez * ez > rr * rr) continue; }
          for (const t of Blist) {
          const dx = s[0] - t[0], dy = s[1] - t[1], dz = s[2] - t[2], r2 = s[3] + t[3] + m0, q2 = dx * dx + dy * dy + dz * dz;
          if (q2 > r2 * r2) continue;
          const dist = Math.sqrt(q2), d = dist - s[3] - t[3];
          if (d < m0) found.push({ d, n: dist > 1e-9 ? [dx / dist, dy / dist, dz / dist] : [0, 0, 1], pa: [s[0] - dx / dist * s[3], s[1] - dy / dist * s[3], s[2] - dz / dist * s[3]], pb: [t[0] + dx / dist * t[3], t[1] + dy / dist * t[3], t[2] + dz / dist * t[3]] });
        }
        }
        found.sort((x, y) => x.d - y.d);
        for (const f of found.slice(0, 4)) res.push(Object.assign(f, { a, other, key }));
      };
      for (let a = 0; a < N; a++) {
        if (!arm[a].length) continue;
        for (const bw of this.bodyW) {
          if (this.excl.has(`${a}|b${bw.li}`)) continue;
          const B = bounds[a], C = bw.bound;
          if (Math.hypot(B[0] - C[0], B[1] - C[1], B[2] - C[2]) > B[3] + C[3] + m0) continue;
          pairs(a, arm[a], bw.sp, this.body.links[bw.li].name, -1, C);
        }
        for (let b = 0; b < a; b++) {
          if (this.excl.has(`${a}|a${b}`) || !arm[b].length) continue;
          const B = bounds[a], C = bounds[b];
          if (Math.hypot(B[0] - C[0], B[1] - C[1], B[2] - C[2]) > B[3] + C[3] + m0) continue;
          pairs(a, arm[a], arm[b], this.names[b], b, C);
        }
      }
      // 야코비안 행: n·(v_a(p) − v_b(p))
      for (const c of res) {
        c.J = new Float64Array(N);
        for (let j = 0; j <= c.a; j++) c.J[j] += dot(c.n, cross(F[j].z, sub(c.pa, F[j].p)));
        if (c.other >= 0) for (let j = 0; j <= c.other; j++) c.J[j] -= dot(c.n, cross(F[j].z, sub(c.pb, F[j].p)));
      }
      return res;
    }
    reset(q) {
      q = q || this.ref;
      this.q = Float64Array.from(q); this.qd = new Float64Array(N); this.t = 0;
      this.tauAct = new Float64Array(N); this.tauOut = new Float64Array(N); this.tauRaw = new Float64Array(N); this.tauLim = new Float64Array(N);
      this.sat = new Array(N).fill(false); this.satLatched = new Array(N).fill(false); this.satDur = 0;
      this.limHit = new Array(N).fill(false); this.contacts = []; this.contactForce = 0;
      this.pulses = []; this.extForce = [0, 0, 0]; this.extPoint = null; this.tauExt = new Float64Array(N);
      this.imp = { ex: [0, 0, 0], er: [0, 0, 0], F: [0, 0, 0], xd: null, x: null };
      this.traj.reset(this.q);
      this.clearRecords();
    }
    clearRecords() { this.trail = []; this.log = []; this.nextLog = this.t; this.nextTrail = this.t; this.satLatched.fill(false); }
    applyLimits() {
      const v = new Float64Array(N), a = new Float64Array(N);
      for (let i = 0; i < N; i++) { v[i] = Math.min(this.cfg.vmaxDeg * Math.PI / 180, this.arm.urdfVel[i]); a[i] = this.cfg.amaxDeg * Math.PI / 180; }
      this.traj.setLimits(v, a);
    }
    setTarget(t) { this.traj.setTarget(t); }
    setArmMass(kg) { this.arm.setArmMass(kg); }
    setPayload(kg, off, ramp = true) {
      this.payloadTarget = kg; this.arm.payload.off = off.slice();
      if (!ramp || this.cfg.payloadRamp <= 0 || kg < this.payloadMass) this.payloadMass = kg; // 내려놓기는 즉시, 들기는 램프
      this.arm.setPayload(this.payloadMass, off);
    }
    // 링크 a 위의 월드 점 p 에 대한 위치 야코비안 (3×7, 행 우선)
    pointJac(F, a, p) {
      const J = [new Float64Array(N), new Float64Array(N), new Float64Array(N)];
      for (let j = 0; j <= a; j++) { const c = cross(F[j].z, sub(p, F[j].p)); J[0][j] = c[0]; J[1][j] = c[1]; J[2][j] = c[2]; }
      return J;
    }
    tcpLocal() { return this.arm.payload.off; }   // 손 기준점 = 손에 든 물체 무게중심 위치(손 좌표계)
    /* 충격: 링크 link(0~6)의 국부점 off 에 월드 방향 force[N] 를 반-사인파로 dur[s] 동안 가함 (최대값 = |force|) */
    applyImpulse(force, dur, link = N - 1, off = null, shape = "sine") {
      this.pulses.push({ F: force.slice(), dur: Math.max(dur, 0.001), t0: this.t, link, shape, off: (off || (link === N - 1 ? this.tcpLocal() : [0, 0, 0])).slice() });
    }
    /* 임의 자세: 가동범위 안, 몸과 충돌 없음(여유 1 cm), 현재 하중으로 정지 토크 ≤ 한계의 frac */
    randomPose(rand = Math.random, frac = 0.8, tries = 400) {
      for (let k = 0; k < tries; k++) {
        const q = Array.from({ length: N }, (_, i) => { const lo = this.arm.lower[i], hi = this.arm.upper[i], m = 0.08 * (hi - lo); return lo + m + (hi - lo - 2 * m) * rand(); });
        const F = this.arm.fk(q);
        const save = this.cfg.contactMargin; this.cfg.contactMargin = 0.01;
        const hit = this._contacts(F).length > 0; this.cfg.contactMargin = save;
        if (hit) continue;
        const g = this.arm.gravityTorque(q);
        if (g.some((v, i) => Math.abs(v) > frac * this.arm.effort[i])) continue;
        return q;
      }
      return null;
    }
    step(nsteps = 1) {
      const c = this.cfg, dt = c.dt, kp = c.wn * c.wn, kd = 2 * c.zeta * c.wn, eMax = c.eMaxDeg * Math.PI / 180;
      for (let s = 0; s < nsteps; s++) {
        if (this.payloadMass !== this.payloadTarget) {
          const st = dt * Math.max(this.payloadTarget, this.payloadMass, 0.1) / c.payloadRamp;
          this.payloadMass += clamp(this.payloadTarget - this.payloadMass, -st, st);
          if (Math.abs(this.payloadMass - this.payloadTarget) < 1e-12) this.payloadMass = this.payloadTarget;
          this.arm.setPayload(this.payloadMass, this.arm.payload.off);
        }
        this.traj.step(dt);
        const F = this.arm.fk(this.q);
        const Ml = this.arm.massMatrix(this.q, F);            // 링크(출력축 쪽) 질량행렬
        const bias = this.arm.rnea(this.q, this.qd, new Float64Array(N), true, F);
        // 피드포워드: 실제 자세에서 기준 궤적 속도·가속도의 역동역학 + 구동부(반사관성·마찰) 보상
        const ff = this.arm.rnea(this.q, this.traj.v, this.traj.a, true, F);
        for (let i = 0; i < N; i++) ff[i] += c.armature[i] * this.traj.a[i] + fric(c, i, this.traj.v[i]);
        // 구동부 포함 질량행렬 + 마찰의 암시적 선형화(안정): (Ml + J + dt·C) q̈ = τ − τf(q̇) − bias
        const M = Float64Array.from(Ml);
        for (let i = 0; i < N; i++) {
          const ch = Math.cosh(this.qd[i] / c.vStick);
          M[i * N + i] += c.armature[i] + dt * (c.viscous[i] + c.coulomb[i] / c.vStick / (ch * ch));
        }
        const Minv = invFull(M, N);
        const Dg = feedbackInertia(M, Minv, c.lambdaMax);
        // ----- 외부 충격력 → 관절 일반화력 τext = Jpᵀ F -----
        this.tauExt.fill(0); let Fsum = [0, 0, 0], Pext = null;
        this.pulses = this.pulses.filter((pl) => this.t - pl.t0 <= pl.dur);
        for (const pl of this.pulses) {
          const sh = pl.shape === "step" ? 1 : Math.sin(Math.PI * (this.t - pl.t0) / pl.dur), f = scl(pl.F, Math.max(0, sh));
          const p = add(F[pl.link].p, mv(F[pl.link].R, pl.off)), Jp = this.pointJac(F, pl.link, p);
          for (let j = 0; j < N; j++) this.tauExt[j] += Jp[0][j] * f[0] + Jp[1][j] * f[1] + Jp[2][j] * f[2];
          Fsum = add(Fsum, f); Pext = p;
        }
        this.extForce = Fsum; this.extPoint = Pext;
        // ----- 임피던스 제어: τ = bias + Jᵀ[Kt·ex + Dt·ėx ; Kr·er + Dr·ėr] + 영공간 자세 유지 -----
        let tauImp = null;
        if (c.mode === "impedance") {
          const tl = this.tcpLocal(), Fd = this.arm.fk(this.traj.q);
          const xd = add(Fd[N - 1].p, mv(Fd[N - 1].R, tl)), x = add(F[N - 1].p, mv(F[N - 1].R, tl));
          const Jv = this.pointJac(F, N - 1, x);
          const Jw = [0, 1, 2].map((r) => Float64Array.from({ length: N }, (_, j) => F[j].z[r]));
          const J6 = Jv.concat(Jw);
          // 오차 (월드): 위치, 자세(축-각 벡터)
          let ex = sub(xd, x); const nx = Math.hypot(...ex); if (nx > c.impEmax) ex = scl(ex, c.impEmax / nx);
          const Re = mm(Fd[N - 1].R, tr(F[N - 1].R)); // R_d R^T
          const ang = Math.acos(clamp((Re[0] + Re[4] + Re[8] - 1) / 2, -1, 1));
          let er = [Re[7] - Re[5], Re[2] - Re[6], Re[3] - Re[1]];
          er = ang < 1e-9 ? scl(er, 0.5) : scl(er, ang / (2 * Math.sin(ang)));
          const nr = Math.hypot(...er); if (nr > c.impRmax) er = scl(er, c.impRmax / nr);
          // 속도: 실제와 기준(J·v_ref)
          const v6 = J6.map((row) => row.reduce((a, v, j) => a + v * this.qd[j], 0));
          const vd6 = J6.map((row) => row.reduce((a, v, j) => a + v * this.traj.v[j], 0));
          // 작업공간 관성 Λ = (J M⁻¹ Jᵀ)⁻¹ 대각 → 감쇠 D = 2ζ√(KΛ)
          const A = new Float64Array(36);
          for (let r = 0; r < 6; r++) for (let q2 = 0; q2 < 6; q2++) { let sm = 0; for (let i = 0; i < N; i++) { let t2 = 0; for (let j = 0; j < N; j++) t2 += Minv[i * N + j] * J6[q2][j]; sm += J6[r][i] * t2; } A[r * 6 + q2] = sm + (r === q2 ? 1e-6 : 0); }
          const Lam = invFull6(A);
          const K6 = [c.impKt, c.impKt, c.impKt, c.impKr, c.impKr, c.impKr];
          const e6 = ex.concat(er);
          const Fimp = K6.map((k, r) => k * e6[r] + 2 * c.impZeta * Math.sqrt(k * Math.max(Lam[r * 6 + r], 1e-6)) * (vd6[r] - v6[r]));
          // 영공간 투영 N = I − Jᵀ (J Jᵀ + λI)⁻¹ J  (팔꿈치 등 남는 자유도를 기준 궤적 자세로 약하게)
          const JJ = new Float64Array(36);
          for (let r = 0; r < 6; r++) for (let q2 = 0; q2 < 6; q2++) { let sm = 0; for (let j = 0; j < N; j++) sm += J6[r][j] * J6[q2][j]; JJ[r * 6 + q2] = sm + (r === q2 ? 1e-4 : 0); }
          const JJi = invFull6(JJ);
          const tn = Array.from({ length: N }, (_, i) => c.impKnull * (this.traj.q[i] - this.q[i]) - c.impDnull * this.qd[i]);
          const y = Array.from({ length: 6 }, (_, r) => J6[r].reduce((a, v, j) => a + v * tn[j], 0));
          const z6 = Array.from({ length: 6 }, (_, r) => { let sm = 0; for (let q2 = 0; q2 < 6; q2++) sm += JJi[r * 6 + q2] * y[q2]; return sm; });
          tauImp = new Float64Array(N);
          for (let j = 0; j < N; j++) {
            let t2 = bias[j] + tn[j];
            for (let r = 0; r < 6; r++) t2 += J6[r][j] * (Fimp[r] - z6[r]);
            tauImp[j] = t2;
          }
          this.imp = { ex, er, F: Fimp.slice(0, 3), xd, x };
        } else this.imp = { ex: [0, 0, 0], er: [0, 0, 0], F: [0, 0, 0], xd: null, x: null };
        let anySat = false;
        for (let i = 0; i < N; i++) {
          const e = clamp(this.traj.q[i] - this.q[i], -eMax, eMax);
          const ed = clamp(this.traj.v[i] - this.qd[i], -c.edMax, c.edMax);
          const raw = tauImp ? tauImp[i] : (c.feedforward ? ff[i] : 0) + Dg[i] * (kp * e + kd * ed);
          this.tauRaw[i] = raw;
          this.sat[i] = Math.abs(raw) > this.arm.effort[i];
          this.tauAct[i] = clamp(raw, -this.arm.effort[i], this.arm.effort[i]);
          if (this.sat[i]) { anySat = true; this.satLatched[i] = true; }
        }
        this.satDur = anySat ? this.satDur + dt : 0;
        const rhs = new Float64Array(N);
        for (let i = 0; i < N; i++) rhs[i] = this.tauAct[i] + this.tauExt[i] - fric(c, i, this.qd[i]) - bias[i];
        const qdd0 = cholSolve(M, rhs, N);
        const qPrev = Float64Array.from(this.q), qdPrev = Float64Array.from(this.qd), tPrev = this.t;
        for (let i = 0; i < N; i++) this.qd[i] += qdd0[i] * dt;

        // ----- 제약: 가동범위 스토퍼 + 접촉 (비탄성 충격량, 투영 가우스–자이델) -----
        const lam = new Float64Array(N);
        this.limHit.fill(false);
        const cons = c.collision ? this._contacts(F) : [];
        for (const k of cons) { // 유효질량 w = J M⁻¹ Jᵀ, MJ = M⁻¹ Jᵀ
          k.MJ = new Float64Array(N);
          for (let i = 0; i < N; i++) { let s2 = 0; for (let j = 0; j < N; j++) s2 += Minv[i * N + j] * k.J[j]; k.MJ[i] = s2; }
          k.w = 0; for (let i = 0; i < N; i++) k.w += k.J[i] * k.MJ[i];
          k.lam = 0;
        }
        for (let it = 0; it < 10; it++) {
          for (let i = 0; i < N; i++) {
            const lo = this.arm.lower[i], hi = this.arm.upper[i], qn = this.q[i] + this.qd[i] * dt;
            const upperSide = this.q[i] > (lo + hi) / 2;
            let vStar = null;
            if (upperSide && (qn > hi || lam[i] < 0)) vStar = Math.max((hi - this.q[i]) / dt, -0.2);
            else if (!upperSide && (qn < lo || lam[i] > 0)) vStar = Math.min((lo - this.q[i]) / dt, 0.2);
            if (vStar === null) continue;
            let d = (vStar - this.qd[i]) / Minv[i * N + i];
            const newLam = upperSide ? Math.min(0, lam[i] + d) : Math.max(0, lam[i] + d);
            d = newLam - lam[i]; lam[i] = newLam;
            if (d !== 0) for (let k = 0; k < N; k++) this.qd[k] += Minv[k * N + i] * d;
          }
          for (const k of cons) {
            if (k.w < 1e-12) continue;
            let vn = 0; for (let i = 0; i < N; i++) vn += k.J[i] * this.qd[i];
            const vStar = k.d >= 0 ? -k.d / dt : Math.min(-0.2 * k.d / dt, 0.3); // 침투 시 서서히 밀어냄
            const nl = Math.max(0, k.lam + (vStar - vn) / k.w), d = nl - k.lam; k.lam = nl;
            if (d !== 0) for (let i = 0; i < N; i++) this.qd[i] += k.MJ[i] * d;
          }
        }
        let fmax = 0; for (const k of cons) fmax = Math.max(fmax, k.lam / dt);
        this.contacts = cons.filter((k) => k.lam > 0).map((k) => ({ link: this.names[k.a], other: k.key, force: k.lam / dt, depth: k.d }));
        this.contactForce = fmax;
        for (let i = 0; i < N; i++) {
          this.tauLim[i] = lam[i] / dt; this.limHit[i] = lam[i] !== 0;
          const qdd = (this.qd[i] - qdPrev[i]) / dt;
          // 감속기 출력축 토크 = 구동토크 − 로터 반사관성×가속 − 구동부 마찰 (링크에 실제 전달되는 토크)
          this.tauOut[i] = this.tauAct[i] - c.armature[i] * qdd - fric(c, i, this.qd[i]);
          this.q[i] += this.qd[i] * dt;
        }
        this.t += dt;
        this._record(tPrev, qPrev, qdPrev);
      }
    }
    _record(t, q, qd) {
      const c = this.cfg;
      if (t + 1e-12 >= this.nextTrail) {
        this.nextTrail = t + 1 / c.trailHz;
        this.trail.push({ t, x: Array.from(q, (v, i) => v - this.ref[i]), y: Array.from(this.tauOut), s: this.sat.slice() });
        const keep = c.trailSec * c.trailHz + 1;
        if (this.trail.length > keep * 1.5) this.trail = this.trail.slice(-keep);
      }
      if (t + 1e-12 >= this.nextLog) {
        this.nextLog = t + 1 / c.logHz;
        if (this.log.length < 3600 * c.logHz) this.log.push({ t, tgt: Array.from(this.traj.target), ref: Array.from(this.traj.q), q: Array.from(q), qd: Array.from(qd),
          tau: Array.from(this.tauOut), raw: Array.from(this.tauRaw), sat: this.sat.slice(), arm: this.arm.armMass, pay: this.payloadMass, off: this.arm.payload.off.slice(),
          mode: this.cfg.mode, ext: this.extForce.slice(), cf: this.contactForce });
      }
    }
    trailWindow() { const k = this.cfg.trailSec * this.cfg.trailHz + 1; return this.trail.length > k ? this.trail.slice(-k) : this.trail; }
    csv() {
      const d = (r) => (r * 180 / Math.PI);
      const head = ["time_s", "joint", "target_deg", "traj_ref_deg", "actual_deg", "displacement_deg", "velocity_deg_s",
        "output_torque_Nm", "cmd_torque_unclipped_Nm", "torque_limit_Nm", "saturated", "arm_mass_kg", "payload_kg", "payload_off_x_m", "payload_off_y_m", "payload_off_z_m",
        "control_mode", "ext_force_x_N", "ext_force_y_N", "ext_force_z_N", "contact_force_max_N"];
      const rows = [head.join(",")];
      for (const r of this.log) for (let i = 0; i < N; i++) {
        rows.push([r.t.toFixed(4), this.names[i], d(r.tgt[i]).toFixed(4), d(r.ref[i]).toFixed(4), d(r.q[i]).toFixed(4), d(r.q[i] - this.ref[i]).toFixed(4),
          d(r.qd[i]).toFixed(4), r.tau[i].toFixed(5), r.raw[i].toFixed(5), this.arm.effort[i], r.sat[i] ? 1 : 0, r.arm.toFixed(5), r.pay.toFixed(5),
          r.off[0], r.off[1], r.off[2], r.mode, r.ext[0].toFixed(3), r.ext[1].toFixed(3), r.ext[2].toFixed(3), r.cf.toFixed(2)].join(","));
      }
      return rows.join("\n");
    }
    // 정적 시험: 목표 자세로 이동·정착 후 평균
    settle(qTarget, tSettle = 4, tAvg = 0.5) {
      this.setTarget(qTarget);
      this.step(Math.round(tSettle / this.cfg.dt));
      const n = Math.round(tAvg / this.cfg.dt), tau = new Float64Array(N), q = new Float64Array(N);
      let anySat = new Array(N).fill(false);
      for (let k = 0; k < n; k++) { this.step(1); for (let i = 0; i < N; i++) { tau[i] += this.tauAct[i] / n; q[i] += this.q[i] / n; anySat[i] = anySat[i] || this.sat[i]; } }
      return { tau, q, err: Array.from(q, (v, i) => this.traj.target[i] - v), sat: anySat };
    }
  }

  // 0 kg / m kg 정적 시험 + 이론값(하중 중력 모멘트를 실제 관절축에 투영)
  function staticPayloadTest(model, pose, payload = 1.0, off = [0.12, 0, 0], armMass = null, bodyQ = null, cfg = {}) {
    const res = {};
    for (const m of [0, payload]) {
      const sim = new Sim(model, cfg);
      if (bodyQ) sim.setBodyPose(bodyQ);
      if (armMass) sim.setArmMass(armMass);
      sim.setPayload(m, off, false);
      res[m] = sim.settle(pose);
      res[m].sim = sim;
    }
    const a = res[0], b = res[payload], arm = b.sim.arm;
    const F = arm.fk(b.q);
    const c = add(F[N - 1].p, mv(F[N - 1].R, off));
    const gF = scl(model.gravity, payload);
    const theory = F.map(({ p, z }) => -dot(z, cross(sub(c, p), gF)));
    const dsim = Array.from(b.tau, (v, i) => v - a.tau[i]);
    return { pose, payload, off, tau0: Array.from(a.tau), tau1: Array.from(b.tau), dsim, theory,
      err: dsim.map((v, i) => v - theory[i]), track0: a.err, track1: b.err, sat0: a.sat, sat1: b.sat,
      q0: Array.from(a.q), q1: Array.from(b.q), payloadPoint: c, effort: Array.from(arm.effort) };
  }

  return { Sim, Arm, Body, Trajectory, staticPayloadTest, N, ACT_DEF, util: { mm, mv, cross, dot, sub, add, axisAngle, cholSolve } };
});
