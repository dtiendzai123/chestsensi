// === Advanced Vector3 với tính năng mở rộng ===
class Vector3 {
  constructor(x = 0, y = 0, z = 0) {
    this.x = x;
    this.y = y;
    this.z = z;
  }

  add(v) { return new Vector3(this.x + v.x, this.y + v.y, this.z + v.z); }
  subtract(v) { return new Vector3(this.x - v.x, this.y - v.y, this.z - v.z); }
  multiplyScalar(s) { return new Vector3(this.x * s, this.y * s, this.z * s); }

  lengthSquared() { return this.x * this.x + this.y * this.y + this.z * this.z; }
  length() { return Math.sqrt(this.lengthSquared()); }

  normalize() {
    const len = this.length();
    return len > 0 ? this.multiplyScalar(1 / len) : new Vector3();
  }

  distanceToSquared(v) {
    const dx = this.x - v.x;
    const dy = this.y - v.y;
    const dz = this.z - v.z;
    return dx * dx + dy * dy + dz * dz;
  }

  distanceTo(v) { return Math.sqrt(this.distanceToSquared(v)); }

  lerp(v, t) {
    return new Vector3(
      this.x + (v.x - this.x) * t,
      this.y + (v.y - this.y) * t,
      this.z + (v.z - this.z) * t
    );
  }

  angleTo(v) {
    const dot = this.dot(v);
    const len1 = this.length();
    const len2 = v.length();
    return Math.acos(Math.max(-1, Math.min(1, dot / (len1 * len2))));
  }

  dot(v) { return this.x * v.x + this.y * v.y + this.z * v.z; }

  cross(v) {
    return new Vector3(
      this.y * v.z - this.z * v.y,
      this.z * v.x - this.x * v.z,
      this.x * v.y - this.y * v.x
    );
  }

  clone() { return new Vector3(this.x, this.y, this.z); }
  copy(v) { this.x = v.x; this.y = v.y; this.z = v.z; return this; }
}

// === Predictive Kalman Filter ===
class PredictiveKalmanFilter {
  constructor(R = 0.005, Q = 0.001, dt = 0.016) {
    this.R = R;
    this.Q = Q;
    this.dt = dt;
    this.x = [0, 0];
    this.P = [[1, 0], [0, 1]];
    this.initialized = false;
    this.F = [[1, dt], [0, 1]];
    this.H = [[1, 0]];
    this.Q_matrix = [[Q * dt * dt * dt / 3, Q * dt * dt / 2],
                     [Q * dt * dt / 2, Q * dt]];
  }

  predict() {
    const newX = this.F[0][0] * this.x[0] + this.F[0][1] * this.x[1];
    const newV = this.F[1][0] * this.x[0] + this.F[1][1] * this.x[1];
    this.x = [newX, newV];

    const P11 = this.P[0][0] + this.dt * this.P[0][1] + this.dt * this.P[1][0] + this.dt * this.dt * this.P[1][1] + this.Q_matrix[0][0];
    const P12 = this.P[0][1] + this.dt * this.P[1][1] + this.Q_matrix[0][1];
    const P21 = this.P[1][0] + this.dt * this.P[1][1] + this.Q_matrix[1][0];
    const P22 = this.P[1][1] + this.Q_matrix[1][1];

    this.P = [[P11, P12], [P21, P22]];
  }

  update(measurement) {
    if (!this.initialized) {
      this.x[0] = measurement;
      this.x[1] = 0;
      this.initialized = true;
      return this.x[0];
    }

    this.predict();
    const S = this.P[0][0] + this.R;
    const K1 = this.P[0][0] / S;
    const K2 = this.P[1][0] / S;
    const innovation = measurement - this.x[0];
    this.x[0] += K1 * innovation;
    this.x[1] += K2 * innovation;

    const P11 = this.P[0][0] * (1 - K1);
    const P12 = this.P[0][1] * (1 - K1);
    const P21 = this.P[1][0] - K2 * this.P[0][0];
    const P22 = this.P[1][1] - K2 * this.P[0][1];

    this.P = [[P11, P12], [P21, P22]];
    return this.x[0];
  }

  predictFuture(time) {
    return this.x[0] + this.x[1] * time;
  }

  getVelocity() {
    return this.x[1];
  }

  reset() {
    this.x = [0, 0];
    this.P = [[1, 0], [0, 1]];
    this.initialized = false;
  }
}

// === Multi-target tracking system ===
class TargetTracker {
  constructor() {
    this.targets = new Map();
    this.maxTargets = 999;
    this.targetTimeout = 2000;
  }

  updateTarget(id, position, priority = 1) {
    const now = Date.now();
    if (!this.targets.has(id)) {
      this.targets.set(id, {
        kalmanX: new PredictiveKalmanFilter(),
        kalmanY: new PredictiveKalmanFilter(),
        kalmanZ: new PredictiveKalmanFilter(),
        lastUpdate: now,
        priority,
        hitCount: 0,
        missCount: 0
      });
    }

    const target = this.targets.get(id);
    target.lastUpdate = now;
    target.priority = priority;

    target.filteredPos = new Vector3(
      target.kalmanX.update(position.x),
      target.kalmanY.update(position.y),
      target.kalmanZ.update(position.z)
    );

    return target;
  }

  predictTarget(id, time = 0.1) {
    const target = this.targets.get(id);
    if (!target) return null;

    return new Vector3(
      target.kalmanX.predictFuture(time),
      target.kalmanY.predictFuture(time),
      target.kalmanZ.predictFuture(time)
    );
  }

  getBestTarget() {
    let best = null;
    let bestScore = -1;

    for (const [id, target] of this.targets) {
      const age = Date.now() - target.lastUpdate;
      if (age > this.targetTimeout) {
        this.targets.delete(id);
        continue;
      }

      const accuracy = target.hitCount / Math.max(1, target.hitCount + target.missCount);
      const score = target.priority * accuracy * (1 - age / this.targetTimeout);

      if (score > bestScore) {
        bestScore = score;
        best = { id, target, score };
      }
    }

    return best;
  }

  recordHit(id) {
    const target = this.targets.get(id);
    if (target) target.hitCount++;
  }

  recordMiss(id) {
    const target = this.targets.get(id);
    if (target) target.missCount++;
  }

  cleanup() {
    const now = Date.now();
    for (const [id, target] of this.targets) {
      if (now - target.lastUpdate > this.targetTimeout) {
        this.targets.delete(id);
      }
    }
  }
}

// === Bone definitions ===
const BONE_CONFIGS = {
  head: {
    position: new Vector3(-0.0456970781, -0.004478302, -0.0200432576),
    hitbox: 1.0,
    priority: 100,
    damageMultiplier: 2.5,
    bindpose: {
      e00: -1.34559613e-13, e01: 8.881784e-14, e02: -1.0, e03: 0.487912,
      e10: -2.84512817e-6, e11: -1.0, e12: 8.881784e-14, e13: -2.842171e-14,
      e20: -1.0, e21: 2.84512817e-6, e22: -1.72951931e-13, e23: 0.0,
      e30: 0, e31: 0, e32: 0, e33: 1
    }
  },
  chest: {
    position: new Vector3(-0.143705, -0.0049735, 0.0),
    hitbox: 0.15,
    priority: 80,
    damageMultiplier: 1.0,
    bindpose: {
      e00: 0.0, e01: 0.0, e02: -1.0, e03: 0.511902,
      e10: -0.000000922564, e11: -1.0, e12: 0.0, e13: 0.0001275,
      e20: -1.0, e21: 0.000000922564, e22: 0.0, e23: 0.0,
      e30: 0, e31: 0, e32: 0, e33: 1
    }
  },
  stomach: {
    position: new Vector3(-0.143705, -0.0049735, -0.12),
    hitbox: 0.12,
    priority: 60,
    damageMultiplier: 0.8
  }
};

function multiplyMatrixVec(m, v) {
  return new Vector3(
    m.e00 * v.x + m.e01 * v.y + m.e02 * v.z + m.e03,
    m.e10 * v.x + m.e11 * v.y + m.e12 * v.z + m.e13,
    m.e20 * v.x + m.e21 * v.y + m.e22 * v.z + m.e23
  );
}

function getWorldPosition(boneConfig) {
  return boneConfig.bindpose ?
    multiplyMatrixVec(boneConfig.bindpose, boneConfig.position) :
    boneConfig.position.clone();
}

// === Intelligent targeting system ===
class IntelligentTargeting {
  constructor() {
    this.bonePositions = {};
    this.updateBonePositions();
  }

  updateBonePositions() {
    for (const [name, config] of Object.entries(BONE_CONFIGS)) {
      this.bonePositions[name] = getWorldPosition(config);
    }
  }

  findBestBone(currentAim, maxDistance = 0.3) {
    let bestBone = null;
    let bestScore = -1;

    for (const [name, config] of Object.entries(BONE_CONFIGS)) {
      const bonePos = this.bonePositions[name];
      const distance = currentAim.distanceTo(bonePos);

      if (distance <= maxDistance) {
        const distanceScore = (maxDistance - distance) / maxDistance;
        const priorityScore = config.priority / 100;
        const hitboxScore = config.hitbox / 0.15;
        const score = (priorityScore * 0.5 + distanceScore * 0.3 + hitboxScore * 0.2) * config.damageMultiplier;

        if (score > bestScore) {
          bestScore = score;
          bestBone = { name, position: bonePos, config, score };
        }
      }
    }

    return bestBone;
  }

  isInTargetRegion(aimPos, boneName) {
    const config = BONE_CONFIGS[boneName];
    if (!config) return false;

    const bonePos = this.bonePositions[boneName];
    return aimPos.distanceTo(bonePos) <= config.hitbox;
  }
}

// === Advanced Aimbot System ===
class AdvancedAimbotSystem {
  constructor(options = {}) {
    this.targeting = new IntelligentTargeting();
    this.tracker = new TargetTracker();

    this.config = {
      smoothing: options.smoothing || 0.25,
      prediction: options.prediction || 0.08,
      fovLimit: options.fovLimit || 360,
      maxDistance: options.maxDistance || 9999,
      aimAssist: options.aimAssist !== false,
      triggerBot: options.triggerBot || false,
      antiRecoil: options.antiRecoil || false,
      ...options
    };

    this.currentTarget = null;
    this.lastAim = new Vector3();
    this.recoilPattern = [];
    this.shotCount = 0;
  }

  isInFOV(aimPos, targetPos, fovDegrees) {
    const forward = new Vector3(0, 0, 1);
    const toTarget = targetPos.subtract(aimPos).normalize();
    const angle = Math.acos(forward.dot(toTarget)) * (180 / Math.PI);
    return angle <= fovDegrees / 2;
  }

  compensateRecoil(aimPos) {
    if (!this.config.antiRecoil || this.recoilPattern.length === 0) return aimPos;

    const recoilIndex = Math.min(this.shotCount, this.recoilPattern.length - 1);
    const recoil = this.recoilPattern[recoilIndex];
    return aimPos.subtract(recoil);
  }

  processAim(currentAim, targetId = "default") {
    this.tracker.updateTarget(targetId, currentAim);

    const bestBone = this.targeting.findBestBone(currentAim, this.config.maxDistance);
    if (!bestBone) return currentAim;

    if (!this.isInFOV(currentAim, bestBone.position, this.config.fovLimit)) return currentAim;

    let targetPos = bestBone.position;
    if (this.config.prediction > 0) {
      const predicted = this.tracker.predictTarget(targetId, this.config.prediction);
      if (predicted) targetPos = predicted;
    }

    const smoothedAim = this.config.aimAssist ?
      currentAim.lerp(targetPos, this.config.smoothing) :
      targetPos;

    const finalAim = this.compensateRecoil(smoothedAim);

    this.lastAim = finalAim;
    this.currentTarget = bestBone;

    return finalAim;
  }

  shouldTrigger(currentAim) {
    if (!this.config.triggerBot || !this.currentTarget) return false;

    const distance = currentAim.distanceTo(this.currentTarget.position);
    const triggerThreshold = this.currentTarget.config.hitbox * 0.8;
    return distance <= triggerThreshold;
  }

  recordShot() {
    this.shotCount++;
    if (this.currentTarget && this.currentTarget.name) {
      this.tracker.recordHit(this.currentTarget.name);
    }
  }

  resetShots() {
    this.shotCount = 0;
  }

  setRecoilPattern(pattern) {
    this.recoilPattern = pattern;
  }

  getCurrentTarget() {
    return this.currentTarget;
  }
}

// === High-performance execution loop ===
class AimbotExecutor {
  constructor(options = {}) {
    this.aimbot = new AdvancedAimbotSystem(options);
    this.isActive = false;
    this.stats = {
      fps: 0,
      frameCount: 0,
      lastTime: 0,
      avgProcessTime: 0
    };

    this.onAim = options.onAim || ((pos) =>
      console.log(`🎯 Aim: ${pos.x.toFixed(4)}, ${pos.y.toFixed(4)}, ${pos.z.toFixed(4)}`)
    );
    this.onTrigger = options.onTrigger || (() => console.log("🔫 TRIGGER!"));
    this.getCurrentAim = options.getCurrentAim || this.simulateAim;

    this.loop = this.loop.bind(this);
  }

  simulateAim() {
    const time = Date.now() * 0.001;
    return new Vector3(
      -0.14 + Math.sin(time) * 0.05,
      -0.005 + Math.cos(time * 1.2) * 0.03,
      0.01 + Math.sin(time * 0.7) * 0.02
    );
  }

  loop() {
    if (!this.isActive) return;

    const startTime = Date.now();
    const currentAim = this.getCurrentAim();
    const optimizedAim = this.aimbot.processAim(currentAim);

    this.onAim(optimizedAim);

    if (this.aimbot.shouldTrigger(currentAim)) {
      this.onTrigger();
      this.aimbot.recordShot();
    }

    const endTime = Date.now();
    this.updateStats(endTime - startTime);

    setTimeout(this.loop, 16); // ~60 FPS
  }

  updateStats(processTime) {
    this.stats.frameCount++;
    this.stats.avgProcessTime = (this.stats.avgProcessTime + processTime) / 2;

    const now = Date.now();
    if (now - this.stats.lastTime >= 1000) {
      this.stats.fps = this.stats.frameCount;
      this.stats.frameCount = 0;
      this.stats.lastTime = now;

      console.log(`📊 FPS: ${this.stats.fps}, Avg Process: ${this.stats.avgProcessTime.toFixed(2)}ms`);
    }
  }

  start() {
    if (this.isActive) return;
    console.log("🚀 Advanced Aimbot System ACTIVATED");
    this.isActive = true;
    this.stats.lastTime = Date.now();
    this.loop();
  }

  stop() {
    if (!this.isActive) return;
    console.log("⏹️ Aimbot System DEACTIVATED");
    this.isActive = false;
  }

  configure(newConfig) {
    Object.assign(this.aimbot.config, newConfig);
  }
}

// === Usage ===
const executor = new AimbotExecutor({
  smoothing: 0.35,
  prediction: 0.1,
  fovLimit: 360,
  maxDistance: 9999,
  aimAssist: true,
  triggerBot: true,
  antiRecoil: true,

  onAim: (pos) => {
    console.log(`🎯 AIM: X=${pos.x.toFixed(6)} Y=${pos.y.toFixed(6)} Z=${pos.z.toFixed(6)}`);
  },

  onTrigger: () => {
    console.log("🔫 AUTO TRIGGER ACTIVATED!");
  }
});

// Cài đặt recoil pattern (AK-47)
executor.aimbot.setRecoilPattern([
  new Vector3(0, 0, 0),
  new Vector3(0, 0.02, 0),
  new Vector3(-0.005, 0.035, 0),
  new Vector3(0.008, 0.045, 0),
  new Vector3(-0.012, 0.05, 0)
]);

// Bắt đầu hệ thống
executor.start();
