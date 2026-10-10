// Browser port of the uavsim package (same model, same parameters).
// Python is the reference implementation; tests/ cross-check the numbers.

(function () {
  "use strict";

  const G = 9.81, RHO = 1.225;

  const VEHICLE = {
    mass: 1.20, arm: 0.20, inertia: 0.0125, rotorsPerSide: 2,
    propDiameter: 0.254, kThrust: 6.0e-6, omegaMaxNominal: 1000.0,
    motorTau: 0.04, cdaX: 0.10, cdaZ: 0.15, rotDamping: 0.010,
    figureOfMerit: 0.62, drivetrainEff: 0.80, avionicsPower: 5.0,
    collisionRadius: 0.30,
  };
  VEHICLE.diskArea = Math.PI * (VEHICLE.propDiameter / 2) ** 2;
  const sideThrustMax = (w = VEHICLE.omegaMaxNominal) => VEHICLE.rotorsPerSide * VEHICLE.kThrust * w * w;

  const PAYLOAD = { mass: 0.60, offset: 0.12 };
  const BATTERY = { cells: 4, capacityAh: 2.2, rInternal: 0.045, nominalVoltage: 14.8 };
  const LIPO_OCV = [[0, 3.27], [0.05, 3.50], [0.1, 3.68], [0.2, 3.74], [0.3, 3.77], [0.4, 3.80],
    [0.5, 3.84], [0.6, 3.87], [0.7, 3.92], [0.8, 3.98], [0.9, 4.06], [1, 4.20]];

  const WORLD_W = 18, WORLD_H = 11, PHYS_DT = 1 / 240;
  const CHECKPOINT_R = 0.75, ZONE_R = 0.76, HOVER_SPEED = 0.5, HOVER_TIME = 1.5;
  const CRASH_VZ = 2.5, CRASH_TILT = 35 * Math.PI / 180;

  const LEVELS = {
    1: { name: "Basic Navigation", start: [9, 0],
      checkpoints: [[4, 3], [13, 7.4], [8, 5], [15, 2.6], [3, 7], [10, 9], [14, 4], [6, 1.4], [11, 6], [8.4, 8]],
      gates: [], wind: { kind: "calm" }, windDelay: 0, soc: 1.0 },
    2: { name: "Gate Run in Crosswind", start: [1.6, 0],
      checkpoints: [[2.8, 5.6], [8, 5.6], [8, 9.4], [12.6, 6.6], [16.8, 4.6]],
      gates: [[5.6, 6.1, 3.8, 7.4], [10.4, 10.9, 4.8, 8.4], [14.6, 15.1, 2.8, 6.4]],
      wind: { kind: "steady", mean: [5, 0], sigma: 1.2, tau: 1.5 }, windDelay: 2, soc: 1.0 },
    3: { name: "Cargo Delivery, Low Battery", start: [1.6, 0], checkpoints: [],
      gates: [[6.2, 6.7, 3.6, 6.8], [11.8, 12.3, 1.8, 5.2]],
      pickup: [2.4, 4], dropzone: [16, 9],
      wind: { kind: "rotating", mean: [4, 1.5], sigma: 0.8, tau: 1.0 }, windDelay: null, soc: 0.18 },
  };

  const clamp = (v, lo, hi) => (v < lo ? lo : v > hi ? hi : v);
  const hypot = Math.hypot;

  function interp(t, x) {
    if (x <= t[0][0]) return t[0][1];
    for (let i = 1; i < t.length; i++) {
      if (x <= t[i][0]) {
        const [x0, y0] = t[i - 1], [x1, y1] = t[i];
        return y0 + (y1 - y0) * (x - x0) / (x1 - x0);
      }
    }
    return t[t.length - 1][1];
  }

  // seeded RNG so gusts repeat between runs
  function rng(seed) {
    let a = seed >>> 0;
    const u = () => {
      a = (a + 0x6D2B79F5) >>> 0;
      let t = a;
      t = Math.imul(t ^ (t >>> 15), t | 1);
      t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
    return () => Math.sqrt(-2 * Math.log(u() + 1e-12)) * Math.cos(2 * Math.PI * u());
  }

  class Battery {
    constructor(soc) { this.soc = soc; this.voltage = this.ocv(); this.current = 0; this.power = 0; this.energyWh = 0; }
    ocv() { return BATTERY.cells * interp(LIPO_OCV, Math.max(this.soc, 0)); }
    get empty() { return this.soc <= 0; }
    terminalVoltage(p) {
      const voc = this.ocv(), d = voc * voc - 4 * p * BATTERY.rInternal;
      return d <= 0 ? voc / 2 : 0.5 * (voc + Math.sqrt(d));
    }
    draw(p, dt) {
      if (this.empty) { this.voltage = this.current = this.power = 0; return; }
      const v = this.terminalVoltage(p), i = v > 0 ? p / v : 0;
      this.voltage = v; this.current = i; this.power = p;
      this.soc = Math.max(0, this.soc - i * dt / (3600 * BATTERY.capacityAh));
      this.energyWh += p * dt / 3600;
    }
  }

  class Wind {
    constructor(cfg, seed) {
      this.cfg = cfg; this.enabled = false; this.t = 0; this.vx = 0; this.vz = 0;
      this.gx = 0; this.gz = 0; this.nx = rng(seed); this.nz = rng(seed + 1);
    }
    step(dt) {
      this.t += dt;
      const c = this.cfg;
      if (!this.enabled || c.kind === "calm") { this.vx = this.vz = 0; return [0, 0]; }
      const a = Math.exp(-dt / c.tau), k = Math.sqrt(1 - a * a);
      this.gx = a * this.gx + c.sigma * k * this.nx();
      this.gz = a * this.gz + 0.4 * c.sigma * k * this.nz();
      let mx = c.mean[0], mz = c.mean[1];
      if (c.kind === "rotating") { mx *= Math.cos(0.30 * this.t); mz *= Math.sin(0.45 * this.t + 1.0); }
      this.vx = mx + this.gx; this.vz = mz + this.gz;
      return [this.vx, this.vz];
    }
  }

  class Quad {
    constructor(batt, x, z) {
      this.batt = batt;
      this.s = [x, z, 0, 0, 0, 0, 0, 0];  // x z vx vz theta q wl wr
      this.setPayload(null);
      this.onGround = z <= 0; this.impact = 0; this.accel = [0, 0];
    }
    setPayload(p) {
      this.payload = p;
      if (!p) { this.mass = VEHICLE.mass; this.inertia = VEHICLE.inertia; return; }
      this.mass = VEHICLE.mass + p.mass;
      this.inertia = VEHICLE.inertia + (VEHICLE.mass * p.mass / this.mass) * p.offset ** 2;
    }
    omegaMax() { return this.batt.empty ? 0 : VEHICLE.omegaMaxNominal * this.batt.voltage / BATTERY.nominalVoltage; }
    sideThrust(w) { return VEHICLE.rotorsPerSide * VEHICLE.kThrust * w * w; }
    rotorPower(t) {
      if (t <= 0) return 0;
      return t ** 1.5 / Math.sqrt(2 * RHO * VEHICLE.diskArea) / (VEHICLE.figureOfMerit * VEHICLE.drivetrainEff);
    }
    elecPower() {
      const n = VEHICLE.rotorsPerSide;
      return n * (this.rotorPower(this.sideThrust(this.s[6]) / n) + this.rotorPower(this.sideThrust(this.s[7]) / n)) + VEHICLE.avionicsPower;
    }
    deriv(s, cmd, wind) {
      const [, , vx, vz, th, q, wl, wr] = s;
      const tl = this.sideThrust(wl), tr = this.sideThrust(wr), T = tl + tr;
      const ux = vx - wind[0], uz = vz - wind[1];
      const dx = -0.5 * RHO * VEHICLE.cdaX * Math.abs(ux) * ux;
      const dz = -0.5 * RHO * VEHICLE.cdaZ * Math.abs(uz) * uz;
      return [vx, vz,
        (T * Math.sin(th) + dx) / this.mass,
        (T * Math.cos(th) + dz) / this.mass - G,
        q,
        (VEHICLE.arm * (tl - tr) - VEHICLE.rotDamping * q) / this.inertia,
        (cmd[0] - wl) / VEHICLE.motorTau,
        (cmd[1] - wr) / VEHICLE.motorTau];
    }
    step(dt, cl, cr, wind) {
      const wm = this.omegaMax();
      const cmd = [clamp(cl, 0, wm), clamp(cr, 0, wm)];
      const s = this.s, add = (a, k, h) => a.map((v, i) => v + h * k[i]);
      const k1 = this.deriv(s, cmd, wind);
      const k2 = this.deriv(add(s, k1, dt / 2), cmd, wind);
      const k3 = this.deriv(add(s, k2, dt / 2), cmd, wind);
      const k4 = this.deriv(add(s, k3, dt), cmd, wind);
      this.s = s.map((v, i) => v + dt / 6 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]));
      this.accel = [k1[2], k1[3]];
      this.batt.draw(this.elecPower(), dt);
      this.ground(dt);
    }
    ground(dt) {
      const s = this.s;
      this.impact = 0;
      if (s[1] <= 0) {
        if (!this.onGround) this.impact = -s[3];
        s[1] = 0; if (s[3] < 0) s[3] = 0;
        s[2] *= Math.max(0, 1 - 6 * dt); s[4] *= Math.max(0, 1 - 12 * dt); s[5] *= Math.max(0, 1 - 12 * dt);
        this.onGround = true;
      } else this.onGround = false;
    }
  }

  class PID {
    constructor(kp, ki = 0, kd = 0, lim = Infinity) { Object.assign(this, { kp, ki, kd, lim }); this.i = 0; }
    reset() { this.i = 0; }
    update(e, dt, d = 0, integrate = true) {
      if (integrate && this.ki) this.i = clamp(this.i + e * dt, -this.lim, this.lim);
      return this.kp * e + this.ki * this.i - this.kd * d;
    }
  }

  const MODES = ["ASSIST", "ANGLE", "RATE", "AUTO"];
  const MAX_TILT = 25 * Math.PI / 180, VXY = 2.5, VZ = 2.0;
  const DEG = Math.PI / 180;

  class FlightController {
    constructor(quad) {
      this.quad = quad; this.mode = "ASSIST"; this.hold = [null, null]; this.vsp = [0, 0];
      const I = VEHICLE.inertia, wn = 14, z = 0.8;
      this.wn = wn; this.zeta = z;
      this.att = new PID(I * wn * wn, 0.4, 2 * z * wn * I, 0.3);
      this.rate = new PID(I * 25);
      this.velX = new PID(2.8, 0.6, 0, 3); this.velZ = new PID(4.0, 1.5, 0, 3);
      this.throttle = 0; this.thetaCmd = 0; this.target = null; this.final = true; this.sat = false;
    }
    setMode(m) {
      if (m === this.mode) return;
      const q = this.quad;
      if (m !== "AUTO") this.throttle = clamp((q.sideThrust(q.s[6]) + q.sideThrust(q.s[7])) / (2 * sideThrustMax()), 0, 1);
      this.att.reset(); this.velX.reset(); this.velZ.reset();
      this.hold = [null, null]; this.vsp = [0, 0];
      this.mode = m;
    }
    hoverThrottle() { return this.quad.mass * G / (2 * sideThrustMax()); }
    update(dt, stick, thr) {
      const q = this.quad, s = q.s, tMax = sideThrustMax(q.omegaMax());
      let coll, thCmd;
      if (this.mode === "AUTO" && this.target) [coll, thCmd] = this.position(dt);
      else if (this.mode === "ASSIST") [coll, thCmd] = this.assist(dt, stick, thr);
      else {
        this.throttle = clamp(this.throttle + 0.35 * thr * dt, 0, 1);
        coll = this.throttle * 2 * sideThrustMax();
        thCmd = stick * 25 * DEG;
      }
      let torque;
      if (this.mode === "RATE") {
        torque = this.rate.update(stick * 200 * DEG - s[5], dt);
        this.thetaCmd = s[4];
      } else {
        this.thetaCmd = clamp(thCmd, -MAX_TILT, MAX_TILT);
        torque = this.att.update(this.thetaCmd - s[4], dt, s[5], !this.sat);
      }
      return this.mix(coll, torque, tMax);
    }
    assist(dt, stick, thr) {
      // stick sets a velocity; released, the drone brakes and holds position
      const q = this.quad, s = q.s, sp = [];
      const axes = [[stick, 3.0, s[0], s[2], 1.2], [thr, 2.0, s[1], s[3], 1.5]];
      axes.forEach(([cmd, vmax, pos, vel, kp], i) => {
        let target;
        if (cmd) { this.hold[i] = null; target = cmd * vmax; }
        else {
          if (this.hold[i] === null && Math.abs(vel) < 0.3) this.hold[i] = pos;
          target = this.hold[i] === null ? 0 : clamp(kp * (this.hold[i] - pos), -vmax, vmax);
        }
        // ramp the setpoint so key presses feel smooth instead of jerky
        this.vsp[i] += clamp(target - this.vsp[i], -4.0 * dt, 4.0 * dt);
        sp.push(this.vsp[i]);
      });
      if (q.onGround && sp[1] <= 0) {
        this.velX.reset(); this.velZ.reset(); this.hold = [null, null]; this.vsp = [0, 0];
        return [0, 0];
      }
      return this.velocity(dt, sp[0], sp[1]);
    }
    velocity(dt, vxs, vzs) {
      const q = this.quad, s = q.s;
      const ax = this.velX.update(vxs - s[2], dt, 0, !this.sat);
      const az = this.velZ.update(vzs - s[3], dt, 0, !this.sat);
      return [q.mass * (G + az) / Math.max(Math.cos(s[4]), 0.5), clamp(Math.atan2(ax, G), -MAX_TILT, MAX_TILT)];
    }
    position(dt) {
      const s = this.quad.s, [tx, tz] = this.target;
      let vxs, vzs;
      if (this.final) {
        vxs = clamp(1.2 * (tx - s[0]), -VXY, VXY);
        vzs = clamp(1.5 * (tz - s[1]), -VZ, VZ);
      } else {
        const dx = tx - s[0], dz = tz - s[1], d = hypot(dx, dz) || 1;
        vxs = VXY * dx / d; vzs = clamp(VXY * dz / d, -VZ, VZ);
      }
      return this.velocity(dt, vxs, vzs);
    }
    mix(coll, torque, tMax) {
      const k = VEHICLE.rotorsPerSide * VEHICLE.kThrust;
      const raw = torque / (2 * VEHICLE.arm);
      const hd = clamp(raw, -tMax / 2, tMax / 2);
      const lo = Math.abs(hd), hi = tMax - Math.abs(hd);
      this.sat = coll / 2 < lo || coll / 2 > hi || Math.abs(raw) > tMax / 2;
      const hc = clamp(coll / 2, lo, hi);
      return [Math.sqrt(Math.max(hc + hd, 0) / k), Math.sqrt(Math.max(hc - hd, 0) / k)];
    }
  }

  function gateWalls(gates) {
    const w = [];
    for (const [x0, x1, lo, hi] of gates) { w.push([x0, -1, x1, lo]); w.push([x0, hi, x1, WORLD_H + 1]); }
    return w;
  }
  function circleHitsRect(cx, cz, r, [x0, z0, x1, z1]) {
    const nx = clamp(cx, x0, x1), nz = clamp(cz, z0, z1);
    return (cx - nx) ** 2 + (cz - nz) ** 2 < r * r;
  }

  class Mission {
    constructor(level, mode = "ASSIST") {
      this.level = level; this.cfg = LEVELS[level];
      this.batt = new Battery(this.cfg.soc);
      this.quad = new Quad(this.batt, ...this.cfg.start);
      this.fc = new FlightController(this.quad); this.fc.setMode(mode);
      this.wind = new Wind(this.cfg.wind, 7 + level);
      this.walls = gateWalls(this.cfg.gates);
      Object.assign(this, { t: 0, cp: 0, cargo: false, delivered: false, hoverT: 0, crashed: false,
        reason: "", complete: false, path: [], pathGoal: null, events: [] });
    }
    objective() {
      if (this.complete) return null;
      if (this.level === 3) return this.cargo ? this.cfg.dropzone : this.cfg.pickup;
      return this.cp < this.cfg.checkpoints.length ? this.cfg.checkpoints[this.cp] : null;
    }
    plan(goal) {
      const q = this.quad.s, path = [];
      const gates = [...this.cfg.gates].sort((a, b) => goal[0] < q[0] ? b[0] - a[0] : a[0] - b[0]);
      for (const [x0, x1, lo, hi] of gates) {
        const xc = (x0 + x1) / 2;
        if (Math.min(q[0], goal[0]) < xc && xc < Math.max(q[0], goal[0])) {
          const zc = (lo + hi) / 2, d = goal[0] > q[0] ? 1 : -1;
          path.push([xc - d * 1.2, zc], [xc + d * 1.2, zc]);
        }
      }
      path.push(goal);
      return path;
    }
    autopilot() {
      const goal = this.objective(), s = this.quad.s;
      if (!goal) { this.fc.target = [s[0], s[1]]; this.fc.final = true; return; }
      if (goal !== this.pathGoal) { this.path = this.plan(goal); this.pathGoal = goal; }
      while (this.path.length > 1 && hypot(s[0] - this.path[0][0], s[1] - this.path[0][1]) < 0.6) this.path.shift();
      this.fc.target = this.path[0]; this.fc.final = this.path.length === 1;
    }
    step(frameDt, stick, thr) {
      if (this.crashed || this.complete) return;
      const n = Math.max(1, Math.round(frameDt / PHYS_DT));
      for (let i = 0; i < n && !this.crashed && !this.complete; i++) this.physics(PHYS_DT, stick, thr);
    }
    physics(dt, stick, thr) {
      this.t += dt;
      const d = this.cfg.windDelay;
      if (d !== null && this.t >= d && this.cfg.wind.kind !== "calm") this.wind.enabled = true;
      const wind = this.wind.step(dt);
      if (this.fc.mode === "AUTO") this.autopilot();
      const [wl, wr] = this.fc.update(dt, stick, thr);
      this.quad.step(dt, wl, wr, wind);
      this.bounds(); this.crashCheck();
      if (!this.crashed) this.objectives(dt);
    }
    bounds() {
      const s = this.quad.s, r = VEHICLE.collisionRadius;
      if (s[0] < r) { s[0] = r; s[2] = Math.max(s[2], 0); }
      else if (s[0] > WORLD_W - r) { s[0] = WORLD_W - r; s[2] = Math.min(s[2], 0); }
      if (s[1] > WORLD_H - r) { s[1] = WORLD_H - r; s[3] = -0.3 * Math.abs(s[3]); }
    }
    crashCheck() {
      const q = this.quad, s = q.s;
      if (this.walls.some(w => circleHitsRect(s[0], s[1], VEHICLE.collisionRadius, w))) this.crash("hit a wall");
      else if (q.impact > CRASH_VZ) this.crash(`hard landing (${q.impact.toFixed(1)} m/s)`);
      else if (q.onGround && Math.abs(s[4]) > CRASH_TILT && q.impact > 0) this.crash("landed tilted");
    }
    crash(r) { this.crashed = true; this.reason = (this.batt.empty ? "battery depleted, " : "") + r; }
    objectives(dt) {
      const s = this.quad.s;
      if (this.level < 3) {
        const [cx, cz] = this.cfg.checkpoints[this.cp];
        if (hypot(s[0] - cx, s[1] - cz) < CHECKPOINT_R) {
          this.cp++; this.events.push("checkpoint");
          if (this.cp >= this.cfg.checkpoints.length) this.complete = true;
        }
        return;
      }
      const z = this.cargo ? this.cfg.dropzone : this.cfg.pickup;
      if (hypot(s[0] - z[0], s[1] - z[1]) < ZONE_R && hypot(s[2], s[3]) < HOVER_SPEED) {
        this.hoverT += dt;
        if (this.hoverT >= HOVER_TIME) {
          this.hoverT = 0; this.events.push("checkpoint");
          if (!this.cargo) { this.cargo = true; this.quad.setPayload(PAYLOAD); this.wind.enabled = true; }
          else { this.cargo = false; this.delivered = true; this.quad.setPayload(null); this.complete = true; }
        }
      } else this.hoverT = Math.max(0, this.hoverT - 2 * dt);
    }
  }

  // Rendering. Colours follow the desktop game.
  const SCALE = 50, W = 900, H = 600, GROUND_Y = 550;
  const px = (x, z) => [x * SCALE, GROUND_Y - z * SCALE];

  function circle(ctx, x, y, r, fill, stroke, lw = 1) {
    ctx.beginPath(); ctx.arc(x, y, r, 0, 2 * Math.PI);
    if (fill) { ctx.fillStyle = fill; ctx.fill(); }
    if (stroke) { ctx.strokeStyle = stroke; ctx.lineWidth = lw; ctx.stroke(); }
  }
  function line(ctx, a, b, color, lw = 1, dash = null) {
    ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]);
    ctx.strokeStyle = color; ctx.lineWidth = lw; ctx.setLineDash(dash || []); ctx.stroke(); ctx.setLineDash([]);
  }
  function text(ctx, s, x, y, color, size = 13, align = "left", weight = "") {
    ctx.font = `${weight} ${size}px "IBM Plex Mono", ui-monospace, monospace`;
    ctx.fillStyle = color; ctx.textAlign = align; ctx.textBaseline = "middle"; ctx.fillText(s, x, y);
  }

  function render(ctx, m, now, overlay) {
    ctx.fillStyle = "#0f0f19"; ctx.fillRect(0, 0, W, H);
    ctx.lineWidth = 1;
    for (let i = 1; i <= WORLD_W; i++) line(ctx, [i * SCALE, 0], [i * SCALE, GROUND_Y], "#161626");
    for (let j = 1; j <= 11; j++) line(ctx, [0, GROUND_Y - j * SCALE], [W, GROUND_Y - j * SCALE], "#161626");
    ctx.fillStyle = "#19191b"; ctx.fillRect(0, GROUND_Y, W, H - GROUND_Y);
    line(ctx, [0, GROUND_Y], [W, GROUND_Y], "#505050", 2);
    for (const [x0, z0, x1, z1] of m.walls) {
      const [ax, ay] = px(x0, Math.max(z1, 0)), [bx, by] = px(x1, Math.max(z0, 0));
      ctx.fillStyle = "#3c1414"; ctx.fillRect(ax, ay, bx - ax, by - ay);
      ctx.strokeStyle = "#c83c3c"; ctx.lineWidth = 2; ctx.strokeRect(ax, ay, bx - ax, by - ay);
    }
    const pulse = a => Math.abs(Math.sin(now * 3)) * a;

    // autopilot path
    if (m.fc.mode === "AUTO" && m.path.length) {
      const pts = [px(m.quad.s[0], m.quad.s[1]), ...m.path.map(p => px(...p))];
      for (let i = 1; i < pts.length; i++) line(ctx, pts[i - 1], pts[i], "#5aa078", 1, [5, 5]);
      pts.slice(1).forEach(p => circle(ctx, p[0], p[1], 3, "#5aa078"));
    }

    // objectives
    if (m.level < 3) {
      if (m.cp < m.cfg.checkpoints.length) {
        const [cx, cy] = px(...m.cfg.checkpoints[m.cp]), r = CHECKPOINT_R * SCALE;
        circle(ctx, cx, cy, r, "#14143c", "#ffffff", 3);
        circle(ctx, cx, cy, r + pulse(10), null, "#c8c8ff", 1);
        text(ctx, String(m.cp + 1), cx, cy, "#ffffff", 14, "center");
      }
    } else {
      const r = ZONE_R * SCALE;
      if (m.delivered) {
        const [cx, cy] = px(...m.cfg.dropzone);
        circle(ctx, cx, cy, r, "#143c14", "#64ff64", 3);
      } else {
        const [cx, cy] = px(...(m.cargo ? m.cfg.dropzone : m.cfg.pickup));
        const col = m.cargo ? "#ffc832" : "#3cdc50";
        circle(ctx, cx, cy, r, m.cargo ? "#281e0a" : "#0a280a", col, 3);
        circle(ctx, cx, cy, r + pulse(8), null, col, 1);
        text(ctx, m.cargo ? "DELIVER" : "PICKUP 0.6 kg", cx, cy + r + 14, col, 13, "center");
        if (m.hoverT > 0) {
          ctx.beginPath(); ctx.arc(cx, cy, r + 10, -Math.PI / 2, -Math.PI / 2 + 2 * Math.PI * m.hoverT / HOVER_TIME);
          ctx.strokeStyle = "#ffff00"; ctx.lineWidth = 4; ctx.stroke();
        }
      }
    }

    // pointer to the next objective
    const goal = m.objective();
    if (goal && m.fc.mode !== "AUTO" && !m.crashed && !m.complete) {
      const dx = goal[0] - m.quad.s[0], dz = goal[1] - m.quad.s[1], d = hypot(dx, dz);
      if (d > 2) {
        const ux = dx / d, uy = -dz / d, [x, y] = px(m.quad.s[0], m.quad.s[1]);
        ctx.beginPath();
        ctx.moveTo(x + ux * 84, y + uy * 84);
        ctx.lineTo(x + ux * 70 - uy * 7, y + uy * 70 + ux * 7);
        ctx.lineTo(x + ux * 70 + uy * 7, y + uy * 70 - ux * 7);
        ctx.closePath(); ctx.fillStyle = "#ffdc50"; ctx.fill();
      }
    }
    drawDrone(ctx, m);
    drawHud(ctx, m);
    if (overlay) overlay(ctx);
  }

  function drawDrone(ctx, m) {
    const q = m.quad, s = q.s, [x, y] = px(s[0], s[1]);
    const ca = Math.cos(s[4]), sa = Math.sin(s[4]), ux = sa, uy = -ca;
    const tm = sideThrustMax(), tl = q.sideThrust(s[6]) / tm, tr = q.sideThrust(s[7]) / tm;
    const F = 38, B = 26, BO = 10;
    const fl = [x - F * ca, y - F * sa], fr = [x + F * ca, y + F * sa];
    const bl = [x - B * ca - BO * ux, y - B * sa - BO * uy], br = [x + B * ca - BO * ux, y + B * sa - BO * uy];
    for (const p of [bl, br]) {
      line(ctx, [x, y], p, "#5a5a5a", 2);
      circle(ctx, p[0], p[1], 7, `rgb(${40 + 140 * Math.min(tl, 1)},120,180)`, "#969696");
    }
    for (const [p, t] of [[fl, tl], [fr, tr]]) {
      line(ctx, [x, y], p, "#aaaaaa", 3);
      const n = t * 40;
      if (n > 2) line(ctx, p, [p[0] + ux * n, p[1] + uy * n], "#ff8c1e", 2);
    }
    for (const [p, t] of [[fl, tl], [fr, tr]]) circle(ctx, p[0], p[1], 10, `rgb(${60 + 195 * Math.min(t, 1)},180,255)`, "#dcdcdc");
    if (m.cargo) {
      const d = 0.12 * SCALE * 2, cx = x - ux * d, cy = y - uy * d;
      line(ctx, [x, y], [cx, cy], "#967832");
      ctx.fillStyle = "#c8a032"; ctx.fillRect(cx - 6, cy - 5, 12, 10);
      ctx.strokeStyle = "#ffc850"; ctx.lineWidth = 1; ctx.strokeRect(cx - 6, cy - 5, 12, 10);
    }
    circle(ctx, x, y, 10, m.cargo ? "#b47814" : "#2828b4", "#6464ff");
    circle(ctx, fr[0], fr[1], 4, "#ff3c3c");
  }

  function hbar(ctx, x, y, w, h, f, color) {
    ctx.fillStyle = "#1e1e1e"; ctx.fillRect(x, y, w, h);
    ctx.fillStyle = color; ctx.fillRect(x, y, w * clamp(f, 0, 1), h);
    ctx.strokeStyle = "#646464"; ctx.lineWidth = 1; ctx.strokeRect(x + 0.5, y + 0.5, w - 1, h - 1);
  }

  function drawHud(ctx, m) {
    const q = m.quad, s = q.s, b = m.batt, fc = m.fc;
    const rpm = w => (w * 60 / (2 * Math.PI)).toFixed(0).padStart(6);
    const f = (v, n = 6, d = 2) => v.toFixed(d).padStart(n);
    const grey = "#c8c8c8", blue = "#50c8ff";
    const modeCol = { AUTO: "#5adc8c", RATE: "#ff7878", ANGLE: "#b464ff", ASSIST: "#50c8ff" }[fc.mode];
    const rows = [
      [`Mode      ${fc.mode}`, modeCol],
      [`Altitude  ${f(s[1])} m`, grey],
      [`Speed     ${f(hypot(s[2], s[3]))} m/s`, grey],
      [`Tilt      ${f(s[4] / DEG, 6, 1)} deg`, grey],
      [`Motor L   ${rpm(s[6])} rpm`, blue],
      [`Motor R   ${rpm(s[7])} rpm`, blue],
      [`Power     ${f(b.power, 6, 0)} W`, "#ffb45a"],
      [`Time      ${f(m.t, 6, 1)} s`, "#ffdc50"],
    ];
    if (m.level === 3) rows.push([`Cargo     ${m.delivered ? "DELIVERED" : m.cargo ? "CARRYING" : "GO TO PICKUP"}`, m.cargo ? "#50ff78" : grey]);
    else rows.push([`Target    ${Math.min(m.cp + 1, m.cfg.checkpoints.length)} / ${m.cfg.checkpoints.length}`, "#ffffff"]);
    rows.forEach(([t, c], i) => text(ctx, t, 12, 18 + i * 19, c));

    const x0 = 520, y1 = GROUND_Y + 13, y2 = GROUND_Y + 33, lab = "#969696";
    const thr = (fc.mode === "AUTO" || fc.mode === "ASSIST") ? (q.sideThrust(s[6]) + q.sideThrust(s[7])) / (2 * sideThrustMax()) : fc.throttle;
    text(ctx, "THR", x0, y1, lab);
    hbar(ctx, x0 + 36, y1 - 5, 120, 10, thr, "#50c8ff");
    const hx = x0 + 36 + 120 * fc.hoverThrottle();
    line(ctx, [hx, y1 - 8], [hx, y1 + 8], "#ffdc50", 2);
    text(ctx, `${(thr * 100).toFixed(0).padStart(3)}%`, x0 + 164, y1, lab);
    const soc = Math.max(b.soc, 0), bc = soc > 0.3 ? "#3cdc50" : soc > 0.1 ? "#ffc832" : "#ff3c3c";
    text(ctx, "BAT", x0, y2, lab);
    hbar(ctx, x0 + 36, y2 - 5, 120, 10, soc, bc);
    text(ctx, `${(soc * 100).toFixed(0).padStart(3)}% ${b.voltage.toFixed(1)}V`, x0 + 164, y2, bc);

    const cx = W - 24, cy = GROUND_Y + 25;
    circle(ctx, cx, cy, 17, null, "#28283c");
    if (!m.wind.enabled) { text(ctx, "WIND OFF", cx - 130, y1, "#505064"); }
    else {
      const sp = hypot(m.wind.vx, m.wind.vz);
      if (sp > 0.05) {
        const e = [cx + m.wind.vx / sp * 14, cy - m.wind.vz / sp * 14];
        line(ctx, [cx, cy], e, "#64b4ff", 2); circle(ctx, e[0], e[1], 3, "#64b4ff");
      }
      text(ctx, `WIND ${sp.toFixed(1)}m/s`, cx - 130, y1, "#64b4ff");
    }
    text(ctx, "W/S up/down  A/D move  F mode  R reset", 10, GROUND_Y + 30, "#46465a", 12);

    if (m.crashed) {
      ctx.fillStyle = "rgba(0,0,0,0.45)"; ctx.fillRect(0, 0, W, H);
      text(ctx, `CRASHED: ${m.reason}`, W / 2, H / 2 - 14, "#ff3c3c", 21, "center", "bold");
      text(ctx, "Press R or tap Reset", W / 2, H / 2 + 18, "#dca0a0", 15, "center");
    } else if (m.complete) {
      ctx.fillStyle = "rgba(0,0,0,0.6)"; ctx.fillRect(0, 0, W, H);
      text(ctx, m.level < 3 ? `LEVEL ${m.level} COMPLETE` : "MISSION COMPLETE", W / 2, H / 2 - 40, "#50ff78", 22, "center", "bold");
      text(ctx, `Time ${m.t.toFixed(1)} s    Energy ${b.energyWh.toFixed(2)} Wh`, W / 2, H / 2, "#ffdc50", 16, "center");
      text(ctx, m.level < 3 ? "Press N or tap Next level" : "Press R to fly again", W / 2, H / 2 + 36, "#c8c8c8", 15, "center");
    }
  }

  window.UAVSim = { Mission, LEVELS, MODES, VEHICLE, PAYLOAD, BATTERY, render, sideThrustMax, G, W, H };
})();
