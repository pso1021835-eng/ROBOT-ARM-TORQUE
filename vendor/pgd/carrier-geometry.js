/* Nominal carrier and planet-pin geometry. Millimetres, radians, stage-local +Z toward output. */
(function (root) {
  'use strict';
  const EPS = 1e-8;
  const positive = n => Number.isFinite(n) && n > 0;
  const overlap = ([a, b], [c, d]) => Math.max(0, Math.min(b, d) - Math.max(a, c));
  const midpoint = ([a, b]) => (a + b) / 2;
  const interval = (a, b) => a <= b ? [a, b] : [b, a];
  const area = points => Math.abs(points.reduce((sum, point, i) => {
    const next = points[(i + 1) % points.length];
    return sum + point.x * next.y - point.y * next.x;
  }, 0)) / 2;

  // The gap between neighbouring outward straight slots is bounded in polar
  // coordinates by the central bore, outside circle and two slot edges.
  function webSections(inner, outer, width, count) {
    const half = width / 2, pitch = 2 * Math.PI / count;
    const start = Math.asin(half / outer), end = pitch - start;
    if (!Number.isFinite(start) || end <= start + EPS) return [];
    const corner = inner > half ? Math.asin(half / inner) : Infinity;
    const values = Array.from({ length: 33 }, (_, i) => start + (end - start) * i / 32);
    for (const t of [corner, pitch / 2, pitch - corner]) if (t > start && t < end) values.push(t);
    values.sort((a, b) => a - b);
    return Array.from({ length: count }, (_, i) => {
      const vec = (r, t) => ({ x: r * Math.cos(t + i * pitch), y: r * Math.sin(t + i * pitch) });
      const inside = values.slice(1, -1).reverse().map(t => vec(Math.max(inner, half / Math.sin(Math.min(t, pitch - t))), t));
      return [...values.map(t => vec(outer, t)), ...inside];
    });
  }

  // Sector-finished pockets (one-piece double carrier): each planet pocket is a sector whose radial edges are tangent
  // to the pocket circle (Ø = pocket) on the pin PCD, so the web left between neighbouring pockets is an annular sector.
  function sectorSections(inner, outer, width, count, orbit) {
    const pitch = 2 * Math.PI / count, phi = width / 2 < orbit ? Math.asin(width / 2 / orbit) : Infinity, half = pitch / 2 - phi;
    if (!(half > EPS)) return [];
    return Array.from({ length: count }, (_, i) => {
      const c = pitch / 2 + i * pitch, arc = (r, a, b) => Array.from({ length: 25 }, (_, k) => { const t = a + (b - a) * k / 24; return { x: r * Math.cos(t), y: r * Math.sin(t) }; });
      return [...arc(outer, c - half, c + half), ...arc(inner, c + half, c - half)];
    });
  }
  // Section properties of one web, in the web's own frame (x radial through its middle, y tangential).
  // I = ∫y² dA governs bending under the tangential web force; ymax is the extreme fibre.
  function webProps(points, centre) {
    const q = points.map(p => ({ x: p.x * Math.cos(-centre) - p.y * Math.sin(-centre), y: p.x * Math.sin(-centre) + p.y * Math.cos(-centre) }));
    let A = 0, I = 0;
    q.forEach((p, i) => { const n = q[(i + 1) % q.length], cr = p.x * n.y - n.x * p.y; A += cr / 2; I += cr * (p.y * p.y + p.y * n.y + n.y * n.y) / 12; });
    return { A: Math.abs(A), I: Math.abs(I), ymax: Math.max(...q.map(p => Math.abs(p.y))), rmin: Math.min(...q.map(p => Math.hypot(p.x, p.y))), rmax: Math.max(...q.map(p => Math.hypot(p.x, p.y))) };
  }

  function holeBands(hole, z) {
    const cylinder = hole.cylinder, lo = Math.max(z[0], cylinder[0]), hi = Math.min(z[1], cylinder[1]);
    const out = hi > lo + EPS ? [{ z: [lo, hi], r0: hole.radius, r1: hole.radius }] : [];
    if (hole.tip === hole.end) return out;
    const taper = interval(hole.end, hole.tip), a = Math.max(z[0], taper[0]), b = Math.min(z[1], taper[1]);
    const radius = at => hole.radius * Math.max(0, 1 - Math.abs(at - hole.end) / Math.abs(hole.tip - hole.end));
    if (b > a + EPS) out.push({ z: [a, b], r0: radius(a), r1: radius(b) });
    return out;
  }

  function resolve(stage, geo, i, legacyOD) {
    const pin = stage.pin || {}, carrier = stage.carrier || {}, gear = geo.parts[1];
    const errors = [], warnings = [], double = pin.type !== 'single', direction = double && i === 0 ? -1 : 1;  // single support: the plate sits on the output side
    const b = gear.width, orbit = geo.meshes[0].center, count = stage.n;
    const diameter = pin.d;
    const gap = double ? (carrier.groove ?? b + .6) - b : carrier.axialClearance ?? 0;
    // Carrier thickness (axial, plates + planet seat, boss excluded) is the input. Double support: the plates are
    // symmetric about the centre of the planet seat, each (thickness − seat width) / 2. Single: thickness = the one plate.
    // Empty → plate 4 mm (legacy pin.wall if set).
    const plate0 = pin.wall ?? 4, seat = double ? b + gap : 0;
    const width = positive(carrier.width) ? carrier.width : seat + (double ? 2 : 1) * plate0;
    let wall = double ? (width - seat) / 2 : width, wallShort = false;
    const outerD = carrier.outerD ?? (positive(legacyOD) ? legacyOD : null) ?? 2 * Math.min(orbit + gear.da * .375, geo.parts[2].da / 2 - .6);
    const innerD = carrier.innerD ?? geo.parts[0].da + 1.2;
    const pocket = carrier.pocket ?? gear.da + 4;
    const bossT = carrier.bossT ?? 4;
    const bossD = carrier.bossD ?? Math.min(outerD, Math.max(innerD + .5, 2 * orbit - diameter - 2));
    const radius = diameter / 2, ri = innerD / 2, ro = outerD / 2, rb = bossD / 2;
    const holeMode = pin.holeMode ?? 'through';
    // too thin: report it, but keep building the shape with a 0.3 mm plate so the part can still be drawn from the entered dimensions
    if (!(wall > EPS)) { errors.push(double ? `캐리어 두께 ${width} mm가 유성 안착 축방향 폭 ${seat.toFixed(2)} mm보다 커야 합니다 (판 두께 = (캐리어 두께 − 안착 폭) / 2).` : '캐리어 두께를 확인하세요.'); wallShort = true; wall = .3; }
    if (![b, orbit, wall, diameter, outerD, innerD].every(positive) || outerD <= innerD || !Number.isInteger(count) || count < 2) {
      return { errors: ['캐리어 내·외경, 판 두께, 핀 지름 및 유성 배치를 확인하세요.'], warnings };
    }
    if (!Number.isFinite(gap) || gap < 0) errors.push(double ? '양단 중앙 홈 폭은 유성 치폭 이상이어야 합니다.' : '편단 축방향 여유는 0 mm 이상이어야 합니다.');
    if (!Number.isFinite(bossT) || bossT < 0 || (bossT > 0 && (!positive(bossD) || rb <= ri || rb > ro))) errors.push('돌출 보스 외경은 캐리어 내경보다 크고 최대외경 이하여야 합니다. 보스 두께는 0 mm 이상입니다.');
    if (orbit - radius <= ri || orbit + radius >= ro) errors.push('핀 홀이 캐리어 내경 또는 외경과 겹칩니다.');
    if (diameter >= 2 * orbit * Math.sin(Math.PI / count)) errors.push('인접 핀 홀이 서로 겹칩니다.');
    if (double && (!positive(pocket) || pocket < Math.max(gear.da, diameter))) errors.push('유성기어 안착부 폭은 유성 이끝원경과 핀 지름 이상이어야 합니다.');
    if (!['through', 'blind'].includes(holeMode)) errors.push('핀 조립홀의 관통/깊이 모드를 확인하세요.');
    const bossCoversPin = bossT > 0 && rb >= orbit + radius - EPS;
    if (bossT > 0 && rb > orbit - radius + EPS && !bossCoversPin) errors.push('보스 외경이 핀 홀을 부분적으로 가릅니다. 홀을 전부 덮거나 피하도록 보스 외경을 지정하세요.');
    if (double && outerD >= geo.parts[2].da) warnings.push('캐리어 최대외경이 링 이끝 내경 이상입니다. 축방향 교차 구간을 확인하세요.');
    if (geo.rims?.[1] && geo.rims[1].diameter <= diameter) warnings.push('유성 보어가 핀 지름 이하입니다. 니들 궤도면과 핀 조립을 확인하세요.');
    const left = double ? [-gap / 2 - wall, -gap / 2] : direction < 0 ? [-gap - wall, -gap] : null;
    const right = double ? [b + gap / 2, b + gap / 2 + wall] : direction > 0 ? [b + gap, b + gap + wall] : null;
    const plates = [left, right].filter(Boolean);
    const supportPlate = direction < 0 ? left : right;
    const plateEnd = supportPlate[direction < 0 ? 0 : 1];
    const boss = bossT > 0 ? interval(plateEnd, plateEnd + direction * bossT) : null;
    const entry = double ? (direction < 0 ? right[1] : left[0]) : supportPlate[direction < 0 ? 1 : 0];
    const fullDepth = (plateEnd - entry) * direction + (bossCoversPin ? bossT : 0);
    const coneHeight = holeMode === 'blind' ? radius / Math.tan(59 * Math.PI / 180) : 0;
    const depth = holeMode === 'blind' ? (pin.depth ?? fullDepth - coneHeight - .3) : fullDepth;
    if (!positive(depth) || (holeMode === 'blind' && depth + coneHeight >= fullDepth - EPS)) errors.push('막힌 홀의 원통 깊이와 118° 끝을 합한 길이가 가공 가능 깊이를 넘거나 관통합니다.');
    const end = entry + direction * depth, tip = end + direction * coneHeight;
    const hole = { entry, end, tip, radius, cylinder: interval(entry, end), direction, mode: holeMode, depth, autoDepth: fullDepth - coneHeight - .3 };
    // The cylindrical area supports the pin. The cone is drill clearance.
    const engage = plates.map(z => overlap(z, hole.cylinder));
    if (engage.some(v => v <= EPS)) errors.push('원통 홀 깊이가 지지판까지 닿지 않습니다.');
    const stock = double ? hole.cylinder : interval(direction < 0 ? b : 0, end);
    const loadInterval = [0, b];
    if (overlap(stock, loadInterval) < b - EPS) errors.push('핀이 유성기어 지지 구간 전체를 덮지 않습니다.');

    const pocketShape = 'slot';   // pockets are cut by a radially plunged end mill → parallel walls (the sector option was retired)
    const web = double ? { z: [left[1], right[0]], shape: pocketShape, sections: pocketShape === 'slot' ? webSections(ri, ro, pocket, count) : sectorSections(ri, ro, pocket, count, orbit) } : null;
    if (web?.sections.length) { Object.assign(web, webProps(web.sections[0], Math.PI / count)); web.angle = pocketShape === 'sector' ? 2 * (Math.PI / count - Math.asin(pocket / 2 / orbit)) : null; }
    if (web && !web.sections.length) errors.push('유성 안착 홈이 겹쳐 두 지지판을 연결할 재료가 남지 않습니다.');
    // The pocket is cut by an end mill plunged radially (tool axis = radial), so each corner where a pocket wall meets a
    // plate face keeps the tool radius r. Empty → the largest whole-mm tool that fits the seat width.
    let pocketR = null, pocketRAuto = false, fillets = [];
    if (web?.sections.length) {
      pocketRAuto = !(Number.isFinite(carrier.pocketR) && carrier.pocketR >= 0);
      pocketR = pocketRAuto ? Math.max(.5, Math.floor(Math.min(web.z[1] - web.z[0], pocket) + EPS)) / 2 : carrier.pocketR;
      const half = pocket / 2, seatW = web.z[1] - web.z[0];
      if (2 * pocketR > seatW + EPS) errors.push(`엔드밀 R${pocketR} (Ø${2 * pocketR})가 유성 안착 축방향 폭 ${seatW.toFixed(2)} mm보다 커서 포켓을 가공할 수 없습니다.`);
      else if (2 * pocketR > pocket + EPS) errors.push(`엔드밀 R${pocketR}가 유성 포켓 ${pocket} mm보다 큽니다.`);
      // planet envelope in the pocket's (tangential, axial) section: |t| ≤ da/2, gap/2 … gap/2 + b from each plate face
      const t = gear.da / 2, zg = gap / 2, c = half - pocketR;
      if (pocketR > zg + EPS && t > c + EPS && (t - c) ** 2 + (zg - pocketR) ** 2 > pocketR ** 2 + EPS) warnings.push(`포켓 모서리 R${pocketR}가 유성기어 이끝(Ø${gear.da.toFixed(2)})과 겹칠 수 있습니다. 포켓 폭 또는 안착 홈 폭을 늘리거나 작은 엔드밀을 쓰세요.`);
      if (pocketR > EPS) {
        const pitch = 2 * Math.PI / count, U = a => [Math.cos(a), Math.sin(a)];
        for (const pin of Array.from({ length: count }, (_, j) => j * pitch)) {
          const walls = pocketShape === 'slot'
            ? [1, -1].map(sg => { const e = U(pin), tn = [-e[1], e[0]], x0 = Math.max(Math.sqrt(Math.max(ri * ri - half * half, 0)), half / Math.tan(pitch / 2)), x1 = Math.sqrt(Math.max(ro * ro - half * half, 0));
              const P = x => [e[0] * x + tn[0] * sg * half, e[1] * x + tn[1] * sg * half]; return { A: P(x0), B: P(x1), n: [-tn[0] * sg, -tn[1] * sg] }; })
            : [1, -1].map(sg => { const ph = Math.asin(Math.min(1, half / orbit)), a = pin + sg * ph, e = U(a); return { A: [e[0] * ri, e[1] * ri], B: [e[0] * ro, e[1] * ro], n: [sg * Math.sin(a), -sg * Math.cos(a)] }; });
          for (const w of walls) if (Math.hypot(w.B[0] - w.A[0], w.B[1] - w.A[1]) > EPS) for (const [z, dir] of [[web.z[0], 1], [web.z[1], -1]]) fillets.push({ ...w, z, dir });
        }
      }
    }
    const pins = Array.from({ length: count }, (_, j) => {
      const angle = j * 2 * Math.PI / count;
      return { x: orbit * Math.cos(angle), y: orbit * Math.sin(angle), angle };
    });
    const slabs = plates.map(z => ({ inner: ri, outer: ro, z, drilling: holeBands(hole, z) }));
    if (boss) slabs.push({ inner: ri, outer: rb, z: boss, drilling: bossCoversPin ? holeBands(hole, boss) : [] });
    slabs.sort((x, y) => x.z[0] - y.z[0]);
    const materialVolume = slabs.reduce((sum, s) => sum + Math.PI * (s.outer ** 2 - s.inner ** 2) * (s.z[1] - s.z[0])
      - count * s.drilling.reduce((v, h) => v + Math.PI * (h.z[1] - h.z[0]) * (h.r0 ** 2 + h.r0 * h.r1 + h.r1 ** 2) / 3, 0), 0)
      + (web ? (web.z[1] - web.z[0]) * web.sections.reduce((v, points) => v + area(points), 0) : 0)
      + fillets.reduce((v, f) => v + (1 - Math.PI / 4) * pocketR ** 2 * Math.hypot(f.B[0] - f.A[0], f.B[1] - f.A[1]), 0);
    const firstSupport = midpoint([Math.max(left?.[0] ?? 0, stock[0]), left?.[1] ?? 0]);
    const lastSupport = midpoint([right?.[0] ?? b, Math.min(right?.[1] ?? b, stock[1])]);
    const span = double ? lastSupport - firstSupport : b + gap;
    return { errors, warnings, double, diameter, outerD, innerD, wall, wallShort, width, widthAuto: !positive(carrier.width), seat, gap, groove: double ? b + gap : null, pocket: double ? pocket : null, pocketR, pocketRAuto, fillets,
      bossD, bossT, boss, bossCoversPin, drivePCD: orbit * 2, orbit, plates, pins, web, slabs, materialVolume,
      hole, stock, pinLength: stock[1] - stock[0], engagement: engage, loadInterval, loadStart: double ? -firstSupport : gap,
      span, axial: [Math.min(stock[0], slabs[0].z[0]), Math.max(stock[1], slabs.at(-1).z[1])] };
  }

  const api = { resolve, webSections, sectorSections, webProps };
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  else root.PGDCarrier = api;
})(globalThis);
