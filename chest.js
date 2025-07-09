// === Optimized Vector3 class ===
class Vector3 {
  constructor(x = 0, y = 0, z = 0) {
    this.x = x;
    this.y = y;
    this.z = z;
  }

  addMut(v) {
    this.x += v.x;
    this.y += v.y;
    this.z += v.z;
    return this;
  }

  subtractMut(v) {
    this.x -= v.x;
    this.y -= v.y;
    this.z -= v.z;
    return this;
  }

  multiplyScalarMut(s) {
    this.x *= s;
    this.y *= s;
    this.z *= s;
    return this;
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

  normalizeMut() {
    const len = this.length();
    if (len > 0) this.multiplyScalarMut(1 / len);
    return this;
  }

  set(x, y, z) {
    this.x = x;
    this.y = y;
    this.z = z;
    return this;
  }

  copy(v) {
    this.x = v.x;
    this.y = v.y;
    this.z = v.z;
    return this;
  }

  clone() {
    return new Vector3(this.x, this.y, this.z);
  }

  lerp(v, t) {
    const invT = 1 - t;
    return new Vector3(
      this.x * invT + v.x * t,
      this.y * invT + v.y * t,
      this.z * invT + v.z * t
    );
  }

  lerpMut(v, t) {
    const invT = 1 - t;
    this.x = this.x * invT + v.x * t;
    this.y = this.y * invT + v.y * t;
    this.z = this.z * invT + v.z * t;
    return this;
  }
}

// === Optimized Kalman Filter ===
class KalmanFilter {
  constructor(R = 0.01, Q = 0.0001) {
    this.R = R;
    this.Q = Q;
    this.A = 1;
    this.C = 1;
    this.cov = NaN;
    this.x = NaN;
    this.initialized = false;
  }

  filter(z) {
    if (!this.initialized) {
      this.x = z;
      this.cov = this.R;
      this.initialized = true;
      return this.x;
    }

    const predX = this.A * this.x;
    const predCov = this.A * this.cov * this.A + this.Q;

    const innovation = z - this.C * predX;
    const S = this.C * predCov * this.C + this.R;
    const K = predCov * this.C / S;

    this.x = predX + K * innovation;
    this.cov = predCov - K * this.C * predCov;

    return this.x;
  }

  reset() {
    this.cov = NaN;
    this.x = NaN;
    this.initialized = false;
  }
}

// === Matrix multiply for bindpose * position ===
function multiplyMatrixVec(m, v) {
  return new Vector3(
    m.e00 * v.x + m.e01 * v.y + m.e02 * v.z + m.e03,
    m.e10 * v.x + m.e11 * v.y + m.e12 * v.z + m.e13,
    m.e20 * v.x + m.e21 * v.y + m.e22 * v.z + m.e23
  );
}

// === Bone Data ===
const bone_Head = {
  position: new Vector3(-0.0456970781, -0.004478302, -0.0200432576),
  rotation: { x: 0.0258, y: -0.0861, z: -0.1402, w: 0.9860 },
  scale: new Vector3(1, 1, 1),
  bindpose: {
    e00: -1.34559613e-13, e01: 8.881784e-14, e02: -1.0, e03: 0.487912,
    e10: -2.84512817e-6, e11: -1.0, e12: 8.881784e-14, e13: -2.842171e-14,
    e20: -1.0, e21: 2.84512817e-6, e22: -1.72951931e-13, e23: 0.0,
    e30: 0, e31: 0, e32: 0, e33: 1
  }
};

const bone_Chest = {
  position: new Vector3(-0.143705, -0.0049735, 0.0),
  rotation: { x: 0.0, y: 0.0, z: -0.14392, w: 0.989589 },
  scale: new Vector3(1, 1, 1),
  bindpose: {
    e00: 0.0, e01: 0.0, e02: -1.0, e03: 0.511902,
    e10: -0.000000922564, e11: -1.0, e12: 0.0, e13: 0.0001275,
    e20: -1.0, e21: 0.000000922564, e22: 0.0, e23: 0.0,
    e30: 0, e31: 0, e32: 0, e33: 1
  }
};

// === Cache world positions
let cachedHeadWorldPos = null;
let cachedChestWorldPos = null;

function getWorldPosition(bone) {
  return multiplyMatrixVec(bone.bindpose, bone.position);
}

function initializeWorldPositions() {
  cachedHeadWorldPos = getWorldPosition(bone_Head);
  cachedChestWorldPos = getWorldPosition(bone_Chest);
}

// === Check if aiming at chest
function isAimInChestRegion(aimPos, chestPos, radiusSquared = 0.0225) {
  const dx = aimPos.x - chestPos.x;
  const dy = aimPos.y - chestPos.y;
  const dz = aimPos.z - chestPos.z;
  return (dx * dx + dy * dy + dz * dz) <= radiusSquared;
}

// === Aim Lock System ===
class AimLockSystem {
  constructor(options = {}) {
    this.kalmanX = new KalmanFilter(options.kalmanR || 0.01, options.kalmanQ || 0.0001);
    this.kalmanY = new KalmanFilter(options.kalmanR || 0.01, options.kalmanQ || 0.0001);
    this.kalmanZ = new KalmanFilter(options.kalmanR || 0.01, options.kalmanQ || 0.0001);

    this.smoothFactor = options.smoothFactor || 0.15;
    this.headSnapEnabled = options.headSnapEnabled !== false;
    this.chestRadius = options.chestRadius || 0.15;
    this.chestRadiusSquared = this.chestRadius * this.chestRadius;

    this.frameCount = 0;
    this.lastPerfCheck = Date.now();

    this.tempVec = new Vector3();
    this.lastSmoothedAim = new Vector3();
  }

  kalmanFilter(vec3) {
    return new Vector3(
      this.kalmanX.filter(vec3.x),
      this.kalmanY.filter(vec3.y),
      this.kalmanZ.filter(vec3.z)
    );
  }

  updateAim(currentAim) {
    this.frameCount++;
    this.tempVec.copy(currentAim);

    if (
      this.headSnapEnabled &&
      isAimInChestRegion(currentAim, cachedChestWorldPos, this.chestRadiusSquared)
    ) {
      this.tempVec.copy(cachedHeadWorldPos);
    }

    const filteredAim = this.kalmanFilter(this.tempVec);
    const smoothedAim = this.lastSmoothedAim.lerp(filteredAim, this.smoothFactor);
    this.lastSmoothedAim.copy(smoothedAim);

    return smoothedAim;
  }

  checkPerformance() {
    const now = Date.now();
    if (now - this.lastPerfCheck >= 1000) {
      const fps = this.frameCount;
      console.log(`🚀 AimLock FPS: ${fps}`);
      this.frameCount = 0;
      this.lastPerfCheck = now;
    }
  }

  reset() {
    this.kalmanX.reset();
    this.kalmanY.reset();
    this.kalmanZ.reset();
    this.lastSmoothedAim.set(0, 0, 0);
  }
}

// === Render Loop ===
class AimLoop {
  constructor(options = {}) {
    this.aimLock = new AimLockSystem(options);
    this.isRunning = false;
    this.frameId = null;
    this.targetFPS = options.targetFPS || 60;
    this.frameTime = 1000 / this.targetFPS;
    this.lastFrameTime = 0;
    this.deltaTime = 0;

    this.loop = this.loop.bind(this);
    this.getCurrentAim = options.getCurrentAim || this.getSimulatedAim;
    this.setAim = options.setAim || this.logAim;
  }

  getSimulatedAim() {
    const time = Date.now() * 0.001;
    return new Vector3(
      -0.14 + Math.sin(time) * 0.02,
      -0.005 + Math.cos(time * 1.5) * 0.01,
      0.01 + Math.sin(time * 0.8) * 0.005
    );
  }

  logAim(aimPos) {
    console.log(
      `🎯 Aim: X=${aimPos.x.toFixed(6)} Y=${aimPos.y.toFixed(6)} Z=${aimPos.z.toFixed(6)}`
    );
  }

  loop(currentTime) {
    if (!this.isRunning) return;

    this.deltaTime = currentTime - this.lastFrameTime;
    if (this.deltaTime >= this.frameTime) {
      const currentAim = this.getCurrentAim();
      const optimizedAim = this.aimLock.updateAim(currentAim);
      this.setAim(optimizedAim);
      this.aimLock.checkPerformance();
      this.lastFrameTime = currentTime;
    }

    this.frameId = requestAnimationFrame(this.loop);
  }

  start() {
    if (this.isRunning) return;
    console.log("🎮 Starting optimized aim loop...");
    initializeWorldPositions();
    this.isRunning = true;
    this.lastFrameTime = performance.now();
    this.frameId = requestAnimationFrame(this.loop);
  }

  stop() {
    if (!this.isRunning) return;
    console.log("⏹️ Stopping aim loop...");
    this.isRunning = false;
    if (this.frameId) {
      cancelAnimationFrame(this.frameId);
      this.frameId = null;
    }
  }

  reset() {
    this.aimLock.reset();
  }
}

// === Usage Example ===
const aimLoop = new AimLoop({
  targetFPS: 120,
  smoothFactor: 0.2,
  chestRadius: 0.15,
  kalmanR: 0.008,
  kalmanQ: 0.0001,
  headSnapEnabled: true
});

aimLoop.start();

setTimeout(() => {
  aimLoop.stop();
  console.log("🏁 Demo completed");
}, 10000);
