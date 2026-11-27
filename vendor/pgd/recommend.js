// 추천 제원 생성
// 입력: 최대 외경 × 높이, 목표 감속비와 허용 편차, 최소 1단 썬 PCD, 치직각 모듈 후보, 비틀림각, 치폭(빈칸 = 단별 자동 배분), 유성 개수 후보, 구성(1단 / 2단 링 공유 / 2단 링 분리)
// 1) 모듈마다 외경에 들어가는 링 잇수 범위를 잡고, 감속비 조건에 맞는 썬·유성·링 잇수 조합을 모두 찾는다
// 2) 간이 강도식(피팅·굽힘)으로 순위를 매겨 상위 후보만 남기고
// 3) 남은 후보를 전체 설계로 만들어 외형(layout)·강도(엔진)를 계산해 감속기 허용 토크 순으로 정렬한다.
// 기어 외 치수(핀·니들·출력 베어링·캐리어·하우징)는 예제 설계를 외경 비로 맞춘 초안이다.
(function (root) {
  'use strict';
  const clone = o => JSON.parse(JSON.stringify(o));
  const r05 = v => Math.round(v * 2) / 2, f05 = v => Math.floor(v * 2) / 2;
  const gcd = (a, b) => b ? gcd(b, a % b) : a;
  const frac = (n, d) => { const g = gcd(n, d); return [n / g, d / g]; };
  const OD0 = 68;                      // 예제 설계(RDP-RH065)의 외경 (layout 기준)
  const RAD = Math.PI / 180;

  const wallOf = OD => Math.max(2, r05(2 * OD / OD0));
  // 링 잇수 범위: 이뿌리원 + 림(약 3.5m씩)이 링 외경(= 외경 − 2·하우징 벽)에 들어가는 최대값부터 아래로
  function ringRange(spec, m) {
    const mt = m / Math.cos(spec.beta * RAD), ringOD = spec.OD - 2 * wallOf(spec.OD);
    const Rmax = Math.floor((ringOD - 9.5 * m) / mt);
    return { Rmax, Rmin: Math.max(36, Rmax - (spec.band ?? 24)) };
  }
  // 링 잇수 R 하나에 대한 단 조합 (썬·유성 12잇 이상, 등간격 조립, 인접 유성 이끝 간격 0.5m 이상)
  // 내접 인볼류트 간섭 사전 판정 (전위 0 근사, 엔진 판정식과 같은 꼴): P/R − 1 + tan αa2 / tan αw ≥ 0
  function internalOK(R, P, beta) {
    const at = Math.atan(Math.tan(20 * RAD) / Math.cos(beta * RAD)), mt = 1 / Math.cos(beta * RAD);
    const d2 = R * mt, da2 = d2 - 2, ca = d2 * Math.cos(at) / da2;
    return ca < 1 && P / R - 1 + Math.tan(Math.acos(ca)) / Math.tan(at) >= -0.01;
  }
  function stageOptions(m, R, ns, beta = 0, allowShift = false) {
    const out = [];
    for (const n of ns) for (let S = 12; S <= R - 24 + (allowShift ? 2 : 0); S++) {
      if ((S + R) % n) continue;
      const nominal = (R - S) / 2;
      const first = Math.max(12, Math.ceil(nominal - (allowShift ? 1 : 0)));
      const last = Math.floor(nominal + (allowShift ? 1 : 0));
      for (let P = first; P <= last; P++) {
        if (!allowShift && ((S + P) * Math.sin(Math.PI / n) <= P + 2.5 || !internalOK(R, P, beta))) continue;
        const [num, den] = frac(R + S, S);
        out.push({ m, R, s: S, p: P, n, num, den });
      }
    }
    return out;
  }

  // Solve the individual shifts for the sampled working shift sum and
  // ring coefficient. Shared stages must use the same physical ring.
  const ringGrid = [0, .1, -.1, .2, -.2, .3, -.3, .4, -.4, .5, -.5];
  const sumGrid = [0, ...Array.from({length:20}, (_, i) => [(i+1)/20, -(i+1)/20]).flat()];
  function geometryFailures(geo) {
    const failures = geo.checks.filter(c => !c.ok && !c.k.includes('ε')).map(c => c.k);
    return [...new Set([...geo.errors, ...failures])];
  }
  function profileAtRing(E, st, xr, reject) {
    for (const sum of sumGrid) {
      const xp = xr - E.ringShiftOffset(st, sum), xs = sum - xp;
      if (!Number.isFinite(xp) || Math.max(Math.abs(xs), Math.abs(xp)) > .5000000001) {
        if (reject) reject(['전위계수 ±0.5 범위']);
        continue;
      }
      const profile = { xs, xp, xr };
      const geo = E.geometry({ ...st, ...profile });
      const failures = geometryFailures(geo);
      if (!failures.length) return { profile, geo };
      if (reject) reject(failures);
    }
    return null;
  }
  function profileResolver(E, beta) {
    const cache = new Map();
    return (stage, xr) => {
      const key = [stage.s, stage.p, stage.R, stage.n, xr].join('/');
      if (!cache.has(key)) {
        const st = { s: stage.s, p: stage.p, r: stage.R, n: stage.n, m: 1, b: 6, alpha: 20, beta };
        cache.set(key, profileAtRing(E, st, xr)?.profile || null);
      }
      return cache.get(key);
    };
  }
  // Fixed single-stage tooth set. Return one passing profile per ring-grid point.
  function inspectTeeth(E, st, allowShift) {
    const reasons = new Map(), candidates = [];
    const reject = keys => keys.forEach(k => reasons.set(k, (reasons.get(k) || 0) + 1));
    const zero = E.geometry({ ...st, xs: 0, xp: 0, xr: 0 });
    const phase = (st.s + st.r) / st.n;
    if (!zero.meshes.length || !Number.isInteger(phase)) reject(zero.errors);
    else if (allowShift) {
      for (const xr of ringGrid) {
        const candidate = profileAtRing(E, st, xr, reject);
        if (candidate) candidates.push(candidate);
      }
    } else {
      const failures = geometryFailures(zero);
      if (failures.length) reject(failures);
      else candidates.push({ profile: { xs: 0, xp: 0, xr: 0 }, geo: zero });
    }
    return { ratio: 1 + st.r / st.s, phase, candidates, reasons: [...reasons], zero };
  }

  // ---- 간이 허용 토크 (그 단 캐리어 토크, N·m): 후보를 줄이는 데만 쓴다
  function proxy(o, b, beta) {
    const mt = o.m / Math.cos(beta * RAD), z1 = Math.min(o.s, o.p), u = Math.max(o.s, o.p) / z1, d1 = mt * z1, ds = mt * o.s;
    const FtH = 10.8 * b * d1 * u / (u + 1), FtF = 143 * b * o.m, kg = 1 + 0.08 * (o.n - 3);
    return o.n * Math.min(FtH, FtF) / kg * ds / 2 * (o.num / o.den) / 1000;
  }

  // ---- 후보 목록
  const ratioOf = stage => stage.num / stage.den;
  const candidateRatio = cand => cand.stages.reduce((ratio, stage) => ratio * ratioOf(stage), 1);

  function searchCandidates(spec, addCandidate, E = root.PGD) {
    const { ratio: target, ns, beta } = spec;
    const secondCounts = spec.ns2?.length ? spec.ns2 : ns;
    const counts = [...new Set([...ns, ...secondCounts])];
    const modes = spec.modes || { one: true, shared: true, sep: true };
    const tolerance = target * (spec.ratioTolPct ?? 0) / 100;
    const rounding = 16 * Number.EPSILON * target;
    const lo = target - tolerance - rounding, hi = target + tolerance + rounding;
    const minSun = spec.minSunPCD ?? 0, cosBeta = Math.cos(beta * RAD);
    const canUseFirst = stage => ns.includes(stage.n) && stage.s * stage.m / cosBeta + 1e-9 >= minSun;
    const allStages = [];
    const profileFor = spec.allowShift ? profileResolver(E, beta) : null;

    function add(type, stages) {
      if (!profileFor) {
        addCandidate({ type, stages });
        return;
      }
      if (type === 'shared') {
        for (const xr of ringGrid) {
          const first = profileFor(stages[0], xr);
          if (!first) continue;
          const second = profileFor(stages[1], xr);
          if (!second) continue;
          addCandidate({ type, stages: [{ ...stages[0], profile: first }, { ...stages[1], profile: second }] });
          return;
        }
        return;
      }
      const resolved = [];
      for (const stage of stages) {
        let profile;
        for (const xr of ringGrid) {
          profile = profileFor(stage, xr);
          if (profile) break;
        }
        if (!profile) return;
        resolved.push({ ...stage, profile });
      }
      addCandidate({ type, stages: resolved });
    }

    function pair(first, sorted, type) {
      const i1 = ratioOf(first), lower = lo / i1, upper = hi / i1;
      let left = 0, right = sorted.length;
      while (left < right) {
        const mid = Math.floor((left + right) / 2);
        if (ratioOf(sorted[mid]) < lower) left = mid + 1;
        else right = mid;
      }
      for (let i = left; i < sorted.length && ratioOf(sorted[i]) <= upper; i++) {
        const second = sorted[i];
        if (!secondCounts.includes(second.n)) continue;
        if (spec.hollow > 0 && second.s * second.m / cosBeta + 1e-9 < minSun) continue;   // 중공축이 2단 썬도 지나감
        if (type === 'sep' && (second.m < first.m || (second.m === first.m && second.R === first.R))) continue;
        add(type, [first, second]);
      }
    }

    for (const m of spec.mods) {
      const { Rmin, Rmax } = ringRange(spec, m);
      for (let R = Rmin; R <= Rmax; R++) {
        const stages = stageOptions(m, R, counts, beta, spec.allowShift);
        const firsts = stages.filter(canUseFirst);
        if (modes.one) for (const stage of firsts) {
          const ratio = ratioOf(stage);
          if (ratio >= lo && ratio <= hi) add('one', [stage]);
        }
        if (modes.shared) {
          const sorted = [...stages].sort((a, b) => ratioOf(a) - ratioOf(b));
          for (const stage of firsts) pair(stage, sorted, 'shared');
        }
        if (modes.sep) for (const stage of stages) allStages.push(stage);
      }
    }

    if (modes.sep) {
      const sorted = [...allStages].sort((a, b) => ratioOf(a) - ratioOf(b));
      for (const stage of allStages) if (canUseFirst(stage)) pair(stage, sorted, 'sep');
    }
  }

  function candidates(spec, E) {
    const list = [];
    searchCandidates(spec, cand => list.push(cand), E);
    return list;
  }

  // ---- 높이 자동 배분: 치폭 1 mm 늘면 높이도 1 mm 늘어난다고 보고, 2단은 토크 비(√i2)로 나눈다
  const f1Of = cand => 1 / (1 + Math.sqrt(cand.stages[1].num / cand.stages[1].den));
  function splitB(spec, cand, base) {
    if (spec.b > 0) return cand.stages.map(() => spec.b);
    const tot = Math.max(2, spec.H - base);
    if (cand.stages.length === 1) return [Math.max(2, f05(tot))];
    const f1 = f1Of(cand);
    return [Math.max(2, f05(tot * f1)), Math.max(2, f05(tot * (1 - f1)))];
  }

  // 출력 베어링: 카탈로그(bearings.js)가 있으면 외경에 들어가는 크로스 롤러 중 정정격하중 C0가 가장 큰 형번
  // (같으면 폭이 얇은 것). 폭은 길이 한계의 20 % 이하. 카탈로그가 없으면 외경 비 추정값.
  // 외부 하중(D.req.ext)이 있으면 비상 토크 배율에서 정적 안전율 s0 ≥ 목표를 만족하는 형번 중 가장 얇은 것(같으면 C0 큰 것),
  // 만족하는 형번이 없으면 C0 최대. 외경: 하우징 벽 + 1.5 mm 안쪽. 폭: 길이 한계의 30 % 이하.
  function pickOutBearing(spec, wall, req) {
    const list = root.PGDBearings?.crossRoller; if (!list?.length) return null;
    const Dmax = spec.OD - 2 * (wall + 1.5), Bmax = Math.max(6, .3 * spec.H);
    const fit = list.filter(b => b.D <= Dmax + 1e-9 && b.B <= Bmax + 1e-9 && b.C0 > 0);
    const ext = req?.ext, ke = req && req.Trated ? Math.abs((req.Temerg || req.Tpeak || req.Trated) / req.Trated) : 1;
    if (ext && [ext.Fr, ext.Fa, ext.M].every(v => Number.isFinite(v) && v >= 0) && (ext.Fr || ext.Fa || ext.M)) {
      const P0 = b => { const dp = b.dp ?? (b.d + b.D) / 2; return (ext.Fr + 2000 * ext.M / dp + .44 * ext.Fa) * ke; };
      // 동적: 정격 외부 하중(스펙트럼 최대 레벨 그대로, 보수)으로 L10 ≥ 목표 수명 @ 출력 회전수
      const nOut = (req.rpm || 0) / (spec.ratio || 1), P = b => { const dp = b.dp ?? (b.d + b.D) / 2, Fre = ext.Fr + 2000 * ext.M / dp; return Fa_ = ext.Fa / Math.max(Fre, 1e-9) <= 1.5 ? Fre + .45 * ext.Fa : .67 * (Fre + ext.Fa); };
      let Fa_ = 0;
      const life = b => !(nOut > 0 && req.lifeH) || (b.C / P(b)) ** (10 / 3) * 1e6 / (nOut * 60) >= req.lifeH;
      const ok = fit.filter(b => b.C0 / P0(b) >= (req.s0 || 1) && life(b));
      if (ok.length) return ok.sort((x, y) => x.B - y.B || y.C0 - x.C0)[0];
    }
    return fit.sort((x, y) => y.C0 - x.C0 || x.B - y.B)[0] || null;
  }
  function catalogBearing(T, b) {
    const dp = b.dp ?? (b.d + b.D) / 2, sec = (b.D - b.d) / 2, Dw = Math.max(.5, r05(Math.min(sec * .3, b.B * .7)));
    return { ...T, name: `${b.maker} ${b.model}`, type: 'roller', maker: b.maker, series: b.series, model: b.model, d: b.d, D: b.D, B: b.B, dp, dpEst: b.dp == null,
      C: b.C, C0: b.C0, mass: b.mass, r: b.r, rIn: b.rIn ?? b.r, rOut: b.rOut ?? b.r, rSrc: b.rSrc, status: b.status, source: b.source, holes: !!b.holes, Dw, Lw: +(Dw * .9).toFixed(2), DwEst: true, Z: 0, auto: '추천 제원: 외경 안 최대 C0' };
  }

  // 사이트가 설계를 불러올 때(app.js fillMissing)와 같은 기본값 보충 — 추천 결과 값이 적용·새로고침 후와 같도록
  function fillMissing(dst, src) { for (const k in src) { if (dst[k] === undefined || dst[k] === null && src[k] !== null && typeof src[k] === 'object') dst[k] = JSON.parse(JSON.stringify(src[k])); else if (Array.isArray(src[k]) && Array.isArray(dst[k])) { dst[k].forEach((v, i) => { if (v && typeof v === 'object' && src[k][i] && typeof src[k][i] === 'object' && !Array.isArray(v)) fillMissing(v, src[k][i]); }); } else if (src[k] && typeof src[k] === 'object' && dst[k] && typeof dst[k] === 'object') fillMissing(dst[k], src[k]); } return dst; }

  // ---- 후보 → 전체 설계
  function build(base, spec, cand, bs, E) {
    const T = (E.templateDesign || E.defaultDesign)(), D = clone(T);   // 비례 기준: RDP-RH065 템플릿
    // 사용자의 요구·재료·윤활·하중 기준은 현재 설계에서 가져온다
    for (const k of ['req', 'materials', 'lube', 'loads', 'design', 'reliability', 'perf', 'mc', 'anim']) if (base[k]) D[k] = clone(base[k]);
    D.req.ratio = spec.ratio;
    const k = spec.OD / OD0, kz = k, ns = cand.stages.length, beta = spec.beta;   // 축방향 부품(커버·판·보스)도 외경 비로
    // 축방향 부품 두께: 외경 비로 키운 값(a = 1)과 가공 가능한 최소 두께(a = 0) 사이. 길이 한계가 짧으면 baseHeight가 a를 줄인다.
    const a = cand.axial ?? 1, ax = (v, min) => Math.max(min, r05(min + (Math.max(min, v) - min) * a));
    const tag = cand.stages.map(s => `m${s.m} ${s.s}/${s.p}/${s.R}×${s.n}`).join(' + ');
    D.name = `추천 · i ${Number(candidateRatio(cand).toFixed(6))} · ${tag}`;
    D.config = { stages: ns, sharedRing: cand.type === 'shared', floatingSun: false };
    const wall = wallOf(spec.OD);
    const Dw = Math.max(1.5, r05(2.5 * k * (a + (1 - a) * .8)));
    const cat = pickOutBearing(spec, wall, D.req);
    D.outBearing = cat ? catalogBearing(T.outBearing, cat)
      : { ...T.outBearing, name: `크로스 롤러 Dw ${Dw} (외경 비 추정 · 카탈로그 확인)`, Dw, Lw: Dw, dp: spec.OD - 2 * (wall + 1.5) - 2.2 * Dw, C: Math.round(T.outBearing.C * k * k), C0: Math.round(T.outBearing.C0 * k * k) };
    D.geom = {};
    const sc = o => Object.fromEntries(Object.entries(o).map(([key, v]) => [key, typeof v === 'number' && !['n', 'p'].includes(key) ? r05(v * k) || v : v]));
    D.struct = { ...T.struct, c1s2Dim: sc(T.struct.c1s2Dim), s1Dim: sc(T.struct.s1Dim) };
    D.loads = { ...D.loads, extraFsh: [0, 0] };
    const ringOD = spec.OD - 2 * wall;   // 외곽 여유 (하우징 부품은 삭제, BOM-1 — 링 외경은 기존 비례 유지)
    D.needle = [];
    D.stages = cand.stages.map((c, i) => {
      const out = i === ns - 1, tpl = T.stages[out ? 1 : 0], m = c.m, b = bs[i], mt = m / Math.cos(beta * RAD);
      // 유성 보어: 이뿌리원의 약 55 % (예제 비율), 니들 Dw ≈ 보어의 0.21, 핀 = 보어 − 2Dw
      const dfP = mt * c.p + (2 * (c.profile?.xp || 0) - 2.5) * m, bore = Math.max(3, f05(0.55 * dfP)), nDw = Math.max(1, r05(0.21 * bore)), pin = Math.max(2, bore - 2 * nDw);
      const Z = Math.max(6, Math.floor(Math.PI * (pin + nDw) / (nDw * 1.05)));
      D.needle.push({ use: true, apply: true, Dw: nDw, Lw: null, Z, HRCpin: 60, HRCbore: 58 });
      const bscale = (Z * nDw * b) / (11 * 1.5 * tpl.b);
      const plate = ax((out ? 2.9 : 4) * kz, 2.5);
      return Object.assign(clone(tpl), {
        s: c.s, p: c.p, r: c.profile ? c.R : null, n: c.n, m, beta, b,
        ...(c.profile || { xs: 0, xp: 0, xr: 0 }), autoRingShift: !c.profile, sunBore: spec.hollow > 0 ? +spec.hollow.toFixed(1) : null, planetBore: bore, ringOD,   // 중공 = 썬 보어 (중공축은 범위 밖, BOM-1)
        tipChamfer: [0, 0, 0],
        bearing: { name: `풀니들 Ø${nDw} × ${Z} (추정)`, C: Math.round(1600 * bscale), C0: Math.round(1500 * bscale), count: 1 },
        pin: out ? { d: pin, type: 'double', material: 'SUJ2', wall: plate, allow: 1200, holeMode: a < 1 ? 'through' : 'blind', depth: null }   // 얇은 판: 막힌 홀 드릴 끝(118°)이 판을 다 먹으므로 관통
          : { d: pin, type: 'single', material: 'SUJ2', wall: plate, allow: 1200, holeMode: 'through', depth: null },
        carrier: out ? { width: null, groove: null, pocket: null, pocketShape: 'slot', outerD: null, innerD: null, bossD: null, bossT: ax(9.6 * kz, 2), material: 'S45C' }
          : { width: plate, outerD: null, innerD: null, bossT: 0, bossD: null, axialClearance: 0.3, material: 'S45C' }
      });
    });
    return E.defaultDesign ? fillMissing(D, E.defaultDesign()) : D;
  }

  function envelope(D, E, layout) {
    const ctx = E.prepare(D);
    if (ctx.errors?.length) return { errors: ctx.errors };
    try { const L = layout({ ctx }, D); return { OD: 2 * Math.max(L.housR, L.flangeR), H: L.zFlange[1] - L.zIn }; }
    catch (e) { return { errors: ['배치 계산 실패: ' + e.message] }; }
  }

  // 치폭 2 mm 기준 설계의 높이 − 치폭 합 = 치폭 외 고정 높이 (구성·모듈 조합별로 한 번)
  // 축방향 부품 두께 a(1 = 외경 비, 0 = 최소)는 길이 한계에서 치폭 몫(전체의 30 % 이상)이 남는 가장 큰 값으로 정한다.
  function baseHeight(base, spec, cand, E, layout, cache) {
    const key = cand.type + cand.stages.map(s => s.m).join(',');
    if (!cache.has(key)) {
      const bs = cand.stages.map(() => 2), faceMin = Math.max(4 * bs.length, .3 * spec.H);
      let pick = null;
      for (const a of [1, .75, .5, .25, 0]) {
        const env = envelope(build(base, spec, { ...cand, axial: a }, bs, E), E, layout);
        const h = env.errors ? spec.H * 0.6 : env.H - 2 * bs.length;
        pick = { h, a };
        if (h <= spec.H - faceMin) break;
      }
      cache.set(key, pick);
    }
    const p = cache.get(key); cand.axial = p.a; return p.h;
  }

  // 후보 하나 정밀 평가: 치폭 확정(자동이면 높이에 맞춰 두 번 보정) → 외형 확인 → 강도 계산 → 단별 허용 토크
  function evaluateOne(base, spec, cand, E, layout) {
    let bs = cand.b, D = build(base, spec, cand, bs, E), env = envelope(D, E, layout);
    if (!(spec.b > 0) && !env.errors) for (let it = 0; it < 2 && (env.H > spec.H + 0.05 || spec.H - env.H > 0.6); it++) {
      const d = spec.H - env.H, f1 = bs.length > 1 ? f1Of(cand) : 1;
      bs = bs.length > 1 ? [Math.max(2, f05(bs[0] + d * f1)), Math.max(2, f05(bs[1] + d * (1 - f1)))] : [Math.max(2, f05(bs[0] + d))];
      D = build(base, spec, cand, bs, E); env = envelope(D, E, layout);
      if (env.errors) break;
    }
    const row = { cand, D, b: bs, ratio: candidateRatio(cand), stages: cand.stages, type: cand.type, proxy: cand.proxy };
    if (env.errors) return { ...row, ok: false, why: env.errors[0] };
    Object.assign(row, { OD: env.OD, H: env.H });
    if (env.OD > spec.OD + 0.05) return { ...row, ok: false, why: `외경 Ø${env.OD.toFixed(1)} > Ø${spec.OD}` };
    if (env.H > spec.H + 0.05) return { ...row, ok: false, why: `높이 ${env.H.toFixed(1)} > ${spec.H} mm` };
    // 사이트와 같은 해석 경로: 기본값 보충 + 풀니들 계산값 적용 (축척 추정 C·C0 대신)
    if (root.PGDChecks) { root.PGDChecks.ensure(D); root.PGDChecks.applyNeedles(D); }
    const r = E.analyze(D);
    if (r.errors?.length) return { ...row, ok: false, why: r.errors[0] };
    for (const stage of r.ctx.stages) {
      const failures = geometryFailures(stage.geo);
      if (failures.length) return { ...row, ok: false, why: `${stage.i + 1}단 ${failures.join(' · ')}` };
    }
    // 단별 허용 토크: 그 단의 기어(치면·치근)와 유성 베어링 중 작은 값. 출력축 기준으로 구한 뒤 그 단 캐리어 토크도 함께
    const op = { rpm: D.req.rpm, cycles: E.cyclesFor(D.req.lifeH) };
    const after = i => r.ctx.stages.slice(i + 1).reduce((v, s) => v * s.geo.ratio, 1);
    const stageCap = r.ctx.stages.map((s, i) => { const out = Math.min(E.capacity(r.ctx, op, D.req, i + 1), r.bearingCap?.[i] ?? Infinity); return { out, own: out / after(i) }; });
    return { ...row, ok: true, cap: r.reducerCap, gov: r.reducerGov, stageCap, fails: r.verdict.filter(v => !v.ok).length, warns: r.warns.length };
  }

  // 허용 토크 내림차순 (0.1 N·m 동률이면 유성 총수 적은 순 → 단 수 적은 순)
  const order = (a, b) => (b.ok - a.ok) || (Math.round((b.cap || 0) * 10) - Math.round((a.cap || 0) * 10))
    || (a.stages.reduce((v, t) => v + t.n, 0) - b.stages.reduce((v, t) => v + t.n, 0)) || (a.stages.length - b.stages.length);

  // 간이 강도로 줄이기: 구성별 상위 keep개
  function shortlist(base, spec, E, layout) {
    const hcache = new Map(), keep = spec.keep || { one: 30, shared: 60, sep: 60 };
    const counts = { one: 0, shared: 0, sep: 0 }, bins = { one: [], shared: [], sep: [] };
    let total = 0;
    const trim = (bin, max) => bin.sort((a, b) => b.proxy - a.proxy).slice(0, max);
    searchCandidates(spec, c => {
      total++;
      counts[c.type]++;
      const limit = keep[c.type] ?? 0;
      if (limit <= 0) return;
      c.b = splitB(spec, c, baseHeight(base, spec, c, E, layout, hcache));
      const t = c.stages.map((s, i) => proxy(s, c.b[i], spec.beta));
      c.proxy = c.stages.length === 1 ? t[0] : Math.min(t[0] * c.stages[1].num / c.stages[1].den, t[1]);
      const bin = bins[c.type];
      bin.push(c);
      if (bin.length >= 2 * limit) bins[c.type] = trim(bin, limit);
    }, E);
    return { total, counts, list: Object.keys(bins).flatMap(type => trim(bins[type], keep[type] ?? 0)) };
  }

  // 비동기 실행 (화면 멈춤 방지): onProgress(done, total)
  async function run(base, spec, E, layout, onProgress) {
    const sl = shortlist(base, spec, E, layout), res = [];
    for (let i = 0; i < sl.list.length; i++) {
      res.push(evaluateOne(base, spec, sl.list[i], E, layout));
      if (onProgress && (i % 3 === 2 || i === sl.list.length - 1)) { onProgress(i + 1, sl.list.length); await new Promise(r => setTimeout(r, 0)); }
    }
    res.sort(order);
    return { res, total: sl.total, counts: sl.counts, checked: sl.list.length };
  }
  function runSync(base, spec, E, layout) {
    const sl = shortlist(base, spec, E, layout), res = sl.list.map(c => evaluateOne(base, spec, c, E, layout));
    res.sort(order);
    return { res, total: sl.total, counts: sl.counts, checked: sl.list.length };
  }

  const api = { pickOutBearing, catalogBearing, inspectTeeth, ringRange, stageOptions, candidates, proxy, build, envelope, evaluateOne, shortlist, run, runSync };
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  root.PGDRecommend = api;
})(typeof globalThis !== 'undefined' ? globalThis : this);
