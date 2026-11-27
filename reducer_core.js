/* 관절 감속기 배치 · 충격 판정 계산부 (브라우저/Node 공용)
 *
 * 1) 강체 시뮬레이션 (dynamics.js Sim 그대로): 자세 정착 → 충격 펄스 → 관절 출력축 토크 τ(t)
 * 2) 관절 반력(출력 베어링 하중): 같은 상태로 뉴턴–오일러를 다시 돌려 관절마다 자식 쪽 전체가
 *    부모에 주는 힘 f·모멘트 n을 구함 → 관절축 기준 Fa(축방향) · Fr(반경) · M(기울임) · T(축 토크)
 * 3) 관절 유연 모델 (관절별 독립 근사): 모터 측 관성 Jm – 감속기 비틀림 강성 K + 백래시 ±δ – 링크 측 관성 Jl
 *    · 링크 측 하중 τd(t) = τ강체(t) − Jl·q̈(t)  (중력·다른 관절 연성·외력 몫)
 *    · 모터는 강체 구동 토크를 피드포워드로, 모터 각도를 강체 궤적에 PD 추종 (토크 한계 = URDF effort)
 *    · 기어 토크 τg = K·dz(θm − θl, δ) + c·(ωm − ωl) (맞물려 있을 때만), c = 2ζ√(K·Jeq)
 * 4) 판정: 기어 = 설계실 엔진 evaluate(정적, ISO 6336) · 유성 베어링 = 핀 하중 / C0 · 출력 베어링 = C0 / P0
 *    P0 = Fr + 2000·M/dp + 0.44·Fa  (설계실 outputBearing과 같은 식, 단위 N · N·m · mm)
 */
(function (root, factory) {
  if (typeof module === "object" && module.exports) module.exports = factory(require("./dynamics.js"), require("./vendor/pgd/engine.js"));
  else root.ReducerCore = factory(root.H2XDyn, root.PGD);
})(typeof self !== "undefined" ? self : this, function (Dyn, PGD) {
  "use strict";
  const N = 7, D2R = Math.PI / 180, ARCMIN = Math.PI / 180 / 60;
  const { mv, cross, dot, sub, add } = Dyn.util;
  const scl = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
  const mm3 = Dyn.util.mm;
  const tr3 = (A) => [A[0], A[3], A[6], A[1], A[4], A[7], A[2], A[5], A[8]];
  const clone = (o) => JSON.parse(JSON.stringify(o));

  // ---------------- 감속기 제원 → 판정에 쓰는 값 ----------------
  // 엔진 analyze 한 번으로 감속비 · 비틀림 강성 · 백래시 · 정적 한계 토크를 얻고, 이후 토크별 정적 판정은 evaluate로 함
  function reducerInfo(D) {
    const R = PGD.analyze(clone(D));
    if (R.errors && R.errors.length) return { ok: false, errors: R.errors };
    const st = R.ctx.stages.map((s) => ({ s: s.st.s, p: s.st.p, r: s.geo.r, n: s.st.n, m: s.st.m, b: s.st.b, beta: s.st.beta, ratio: s.geo.ratio, bearing: s.st.bearing }));
    return {
      ok: true, R, ctx: R.ctx, D: clone(D), ratio: R.ratio, stages: st, sharedRing: !!D.config.sharedRing,
      K: R.stiffness.NmPerArcmin / ARCMIN,          // N·m/rad, 출력축 기준 (맞물림 + 핀 + 캐리어, 하우징·베어링 제외)
      KarcMin: R.stiffness.NmPerArcmin,
      backlash: R.backlash.arcmin * ARCMIN,         // rad, 출력축 기준 전체 유격 (= 2δ)
      backlashArcmin: R.backlash.arcmin,
      staticCap: R.staticCap, capLife: R.capacityLife, ob: D.outBearing,
      target: { SH: D.req.SHstat, SF: D.req.SFstat, s0: D.req.s0 || 1 }, basis: R.ctx.basis,
    };
  }
  // 토크 T[N·m]에서 정적 판정 (기어 접촉·굽힘, 유성 베어링 정정격)
  function gearCheck(info, T) {
    const T0 = Math.max(Math.abs(T), 1e-6);
    const ev = PGD.evaluate(info.ctx, { Tout: T0, rpm: 0, static: true });
    const chk = PGD.staticCheck(ev, info.target);
    let worst = null;
    ev.rows.forEach((r) => [["접촉(피팅)", r.SH, info.target.SH], ["굽힘", r.SF, info.target.SF]].forEach(([mode, S, t]) => {
      const k = S / t; if (!worst || k < worst.k) worst = { k, S, target: t, mode, stage: r.stage, mesh: r.mesh, part: r.partName };
    }));
    const planet = ev.stages.map((so, i) => {
      const br = info.ctx.stages[i].st.bearing || {}, P = so.pinForce / Math.max(1, br.count || 1);
      return { stage: i + 1, pinForce: so.pinForce, C0: br.C0, s0: br.C0 / Math.max(P, 1e-9), name: br.name };
    });
    return { T: T0, ratio: chk.ratio, worst, planet, planetMin: planet.reduce((a, b) => (b.s0 < a.s0 ? b : a)) };
  }
  const P0of = (ob, w) => w.Fr + 2000 * w.M / ob.dp + 0.44 * w.Fa;

  // ---------------- 관절 반력 (뉴턴–오일러, 외력 포함) ----------------
  // ext: {link, p(월드), F(월드)} 또는 null. 반환: 관절마다 {f, n} (월드, 관절 원점 기준) + 관절축 분해
  function jointWrench(arm, q, qd, qdd, ext) {
    const F = arm.fk(q);
    const w = [], al = [], ao = [], comW = [], Iw = [];
    let wp = [0, 0, 0], alp = [0, 0, 0], aop = scl(arm.g, -1), pp = arm.torsoP;
    for (let i = 0; i < N; i++) {
      const { R, p, z } = F[i];
      const wi = add(wp, scl(z, qd[i]));
      const ali = add(add(alp, scl(z, qdd[i])), cross(wp, scl(z, qd[i])));
      const r = sub(p, pp);
      const aoi = add(add(aop, cross(alp, r)), cross(wp, cross(wp, r)));
      w.push(wi); al.push(ali); ao.push(aoi);
      comW.push(add(p, mv(R, arm.bodies[i].c)));
      Iw.push(mm3(mm3(R, arm.bodies[i].I), tr3(R)));
      wp = wi; alp = ali; aop = aoi; pp = p;
    }
    const out = new Array(N);
    let fc = [0, 0, 0], nc = [0, 0, 0], pc = null;
    for (let i = N - 1; i >= 0; i--) {
      const { p, z } = F[i], b = arm.bodies[i];
      const rc = sub(comW[i], p);
      const ac = add(add(ao[i], cross(al[i], rc)), cross(w[i], cross(w[i], rc)));
      const Fi = scl(ac, b.m);
      const Ni = add(mv(Iw[i], al[i]), cross(w[i], mv(Iw[i], w[i])));
      let f = add(Fi, fc);
      let n = add(add(Ni, cross(rc, Fi)), nc);
      if (pc) n = add(n, cross(sub(pc, p), fc));
      if (ext && ext.link === i) { f = sub(f, ext.F); n = sub(n, cross(sub(ext.p, p), ext.F)); }
      const T = dot(z, n), Fa = dot(z, f);
      const fr = sub(f, scl(z, Fa)), nr = sub(n, scl(z, T));
      out[i] = { T, Fa: Math.abs(Fa), Fr: Math.hypot(...fr), M: Math.hypot(...nr) };
      fc = f; nc = n; pc = p;
    }
    return out;
  }

  // ---------------- 1+2) 강체 시뮬레이션 + 반력 기록 ----------------
  // sc: {pose[rad 7], payload kg, off[3], armMass?, bodyQ?, impact:{F N, T s, dir[3] 단위, link 0..6}, tPre s, tPost s, cfg}
  function runRigid(model, sc) {
    const sim = new Dyn.Sim(model, Object.assign({ collision: false, payloadRamp: 0 }, sc.cfg || {}));
    if (sc.bodyQ) sim.setBodyPose(sc.bodyQ);
    if (sc.armMass) sim.setArmMass(sc.armMass);
    sim.setPayload(sc.payload || 0, sc.off || [0.12, 0, 0], false);
    sim.reset(sc.pose); sim.setTarget(sc.pose);
    sim.step(Math.round((sc.tSettle ?? 1.0) / sim.cfg.dt));
    const dt = sim.cfg.dt, nPre = Math.round((sc.tPre ?? 0.05) / dt), nPost = Math.round((sc.tPost ?? 0.5) / dt);
    const rec = { dt, t: [], q: [], qd: [], qdd: [], tauOut: [], tauAct: [], wrench: [], ext: [], sat: [], names: sim.names, effort: Array.from(sim.arm.effort) };
    const F0 = sim.arm.fk(sim.q), Ml = sim.arm.massMatrix(sim.q, F0);
    rec.Jl = Array.from({ length: N }, (_, i) => Ml[i * N + i]);   // 다른 관절을 잠근 상태의 링크 측 관성
    rec.armature = sim.cfg.armature.slice(); rec.viscous = sim.cfg.viscous.slice(); rec.coulomb = sim.cfg.coulomb.slice();
    rec.wn = sim.cfg.wn; rec.zeta = sim.cfg.zeta;
    const im = sc.impact;
    for (let k = 0; k < nPre + nPost; k++) {
      if (k === nPre && im && im.F > 0) sim.applyImpulse(scl(im.dir, im.F), im.T, im.link ?? N - 1);
      const qd0 = Float64Array.from(sim.qd), t = sim.t;
      sim.step(1);
      const qdd = Array.from(sim.qd, (v, i) => (v - qd0[i]) / dt);
      // 이번 스텝에 쓰인 외력 (sim.extForce/extPoint는 step 안에서 갱신된 값)
      const pl = sim.pulses[0];
      const ext = sim.extPoint && Math.hypot(...sim.extForce) > 0 ? { link: pl ? pl.link : (im ? im.link : N - 1), p: sim.extPoint, F: sim.extForce } : null;
      // 반력은 스텝 시작 상태(q, qd)와 이번 스텝 가속도로 계산 → τ = 기록된 출력축 토크와 일치
      const qPrev = Array.from(sim.q, (v, i) => v - sim.qd[i] * dt);
      const wr = jointWrench(sim.arm, qPrev, qd0, qdd, ext);
      rec.t.push(t - nPre * dt - (rec.t0 || 0)); rec.q.push(qPrev); rec.qd.push(Array.from(qd0)); rec.qdd.push(qdd);
      rec.tauOut.push(Array.from(sim.tauOut)); rec.tauAct.push(Array.from(sim.tauAct)); rec.wrench.push(wr);
      rec.ext.push(ext ? Math.hypot(...ext.F) : 0); rec.sat.push(sim.sat.slice());
    }
    const t0 = rec.t[nPre] ?? 0; rec.t = rec.t.map((v) => v - t0);
    return rec;
  }

  // ---------------- 3) 관절 유연 모델 (2관성 + 비틀림 강성 + 백래시) ----------------
  // j: 관절 번호, p: {K N·m/rad, backlash rad(전체), Jm kg·m², zetaG, kScale}
  function runCompliant(rec, j, p) {
    const K = p.K * (p.kScale ?? 1), delta = 0.5 * p.backlash, Jm = Math.max(p.Jm, 1e-6), Jl = Math.max(rec.Jl[j], 1e-6);
    const Jeq = Jm * Jl / (Jm + Jl), c = 2 * (p.zetaG ?? 0.05) * Math.sqrt(K * Jeq);
    const wnG = Math.sqrt(K / Jeq), dtR = rec.dt, sub = Math.max(1, Math.ceil(dtR / Math.min(dtR, 0.04 / wnG)));
    const h = dtR / sub, effort = rec.effort[j], bv = rec.viscous[j];
    const Jt = Jm + Jl, kp = Jt * rec.wn * rec.wn, kd = 2 * rec.zeta * rec.wn * Jt;
    const dz = (x) => (x > delta ? x - delta : x < -delta ? x + delta : 0);
    const n = rec.t.length;
    const tauD = new Float64Array(n);
    for (let k = 0; k < n; k++) tauD[k] = rec.tauOut[k][j] - Jl * rec.qdd[k][j];
    // 초기: 강체 첫 상태, 기어는 하중 쪽 치면에 붙어 정적 평형
    const T0 = rec.tauOut[0][j];
    let thL = rec.q[0][j], wL = rec.qd[0][j];
    let thM = thL + T0 / K + Math.sign(T0) * delta, wM = wL;
    const tg = new Float64Array(n), gap = new Uint8Array(n), tm = new Float64Array(n);
    let sep = 0, inContact = true;
    for (let k = 0; k < n; k++) {
      const k1 = Math.min(k + 1, n - 1);
      let tgMax = 0, tgLast = 0, open = false, tmLast = 0;
      for (let s = 0; s < sub; s++) {
        const a = s / sub;
        const qRef = rec.q[k][j] + a * (rec.q[k1][j] - rec.q[k][j]), vRef = rec.qd[k][j] + a * (rec.qd[k1][j] - rec.qd[k][j]);
        const td = tauD[k] + a * (tauD[k1] - tauD[k]), ff = rec.tauAct[k][j];
        let um = ff + kp * (qRef - thM) + kd * (vRef - wM);
        um = Math.max(-effort, Math.min(effort, um));
        const x = thM - thL, d = dz(x), touching = d !== 0;
        const g = touching ? K * d + c * (wM - wL) : 0;
        // 반암시적 오일러
        wM += h * (um - g - bv * wM) / Jm; wL += h * (g - td) / Jl;
        thM += h * wM; thL += h * wL;
        if (Math.abs(g) > Math.abs(tgMax)) tgMax = g;
        tgLast = g; tmLast = um; if (!touching) open = true;
      }
      tg[k] = Math.abs(tgMax) > Math.abs(tgLast) ? tgMax : tgLast; tm[k] = tmLast; gap[k] = open ? 1 : 0;
      if (open && inContact) sep++; inContact = !open;
    }
    return { tg, gap, tm, sep, K, delta, Jm, Jl, c, fn: wnG / (2 * Math.PI), sub };
  }

  // ---------------- 4) 관절별 종합 ----------------
  // assign[j] = {info(reducerInfo), Jm, kScale, zetaG}
  function evaluateJoint(rec, j, a) {
    const info = a.info, n = rec.t.length;
    let rigPk = 0, rigK = 0;
    for (let k = 0; k < n; k++) { const v = rec.tauOut[k][j]; if (Math.abs(v) > Math.abs(rigPk)) { rigPk = v; rigK = k; } }
    const comp = runCompliant(rec, j, { K: info.K, backlash: info.backlash, Jm: a.Jm, kScale: a.kScale, zetaG: a.zetaG });
    let cPk = 0, cK = 0;
    for (let k = 0; k < n; k++) if (Math.abs(comp.tg[k]) > Math.abs(cPk)) { cPk = comp.tg[k]; cK = k; }
    // 출력 베어링: 시간마다 P0, 최대 시점
    const ob = info.ob, P0 = new Float64Array(n);
    let pk = 0, pkK = 0;
    for (let k = 0; k < n; k++) { P0[k] = P0of(ob, rec.wrench[k][j]); if (P0[k] > pk) { pk = P0[k]; pkK = k; } }
    // 유연 모델에서 기어 토크가 커진 만큼 출력 베어링의 링크 측 하중도 같은 비율로 커진다고 보는 근사는 하지 않음 (반력은 강체값)
    const gRig = gearCheck(info, rigPk), gComp = gearCheck(info, cPk);
    const obS0 = ob.C0 / Math.max(pk, 1e-9);
    const parts = [
      { part: "기어 (강체 피크)", k: gRig.ratio },
      { part: "기어 (유연·백래시 피크)", k: gComp.ratio },
      { part: `${gComp.planetMin.stage}단 유성 베어링`, k: gComp.planetMin.s0 / info.target.s0 },
      { part: "출력 베어링", k: obS0 / info.target.s0 },
    ];
    const gov = parts.reduce((x, y) => (y.k < x.k ? y : x));
    return { j, name: rec.names[j], info, rigPk, rigT: rec.t[rigK], cPk, cT: rec.t[cK], daf: Math.abs(cPk) / Math.max(Math.abs(rigPk), 1e-9),
      comp, gRig, gComp, P0, obPk: pk, obT: rec.t[pkK], obW: rec.wrench[pkK][j], obS0, parts, gov, effort: rec.effort[j] };
  }

  return { reducerInfo, gearCheck, jointWrench, runRigid, runCompliant, evaluateJoint, P0of, ARCMIN };
});
