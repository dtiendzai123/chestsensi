// === Vector3 & Kalman ===
class Vector3 {
  constructor(x = 0, y = 0, z = 0) { this.x = x; this.y = y; this.z = z; }
  add(v) { return new Vector3(this.x + v.x, this.y + v.y, this.z + v.z); }
  subtract(v) { return new Vector3(this.x - v.x, this.y - v.y, this.z - v.z); }
  multiplyScalar(s) { return new Vector3(this.x * s, this.y * s, this.z * s); }
  length() { return Math.sqrt(this.x ** 2 + this.y ** 2 + this.z ** 2); }
  normalize() { const len = this.length(); return len > 0 ? this.multiplyScalar(1 / len) : new Vector3(); }
  clone() { return new Vector3(this.x, this.y, this.z); }
  static zero() { return new Vector3(0, 0, 0); }
}

class KalmanFilter {
  constructor(R = 0.01, Q = 0.0001) {
    this.R = R; this.Q = Q; this.A = 1; this.B = 0; this.C = 1;
    this.cov = NaN; this.x = NaN;
  }
  filter(z) {
    if (isNaN(this.x)) {
      this.x = (1 / this.C) * z;
      this.cov = (1 / this.C) * this.R * (1 / this.C);
    } else {
      const predX = this.A * this.x;
      const predCov = this.A * this.cov * this.A + this.Q;
      const K = predCov * this.C / (this.C * predCov * this.C + this.R);
      this.x = predX + K * (z - this.C * predX);
      this.cov = predCov - K * this.C * predCov;
    }
    return this.x;
  }
  reset() { this.cov = NaN; this.x = NaN; }
}

// === Bindpose helper ===
function quaternionToMatrix(q) {
  const { x, y, z, w } = q;
  return [
    1 - 2 * y * y - 2 * z * z, 2 * x * y - 2 * z * w, 2 * x * z + 2 * y * w, 0,
    2 * x * y + 2 * z * w, 1 - 2 * x * x - 2 * z * z, 2 * y * z - 2 * x * w, 0,
    2 * x * z - 2 * y * w, 2 * y * z + 2 * x * w, 1 - 2 * x * x - 2 * y * y, 0,
    0, 0, 0, 1
  ];
}

function multiplyMatrixVec(m, v) {
  return new Vector3(
    m[0] * v.x + m[1] * v.y + m[2] * v.z + m[3],
    m[4] * v.x + m[5] * v.y + m[6] * v.z + m[7],
    m[8] * v.x + m[9] * v.y + m[10] * v.z + m[11]
  );
}

// === AimLock Class ===
class AimLockFull {
  constructor() {
    this.velocity = Vector3.zero();
    this.prevPos = null;
    this.kalman = {
      x: new KalmanFilter(),
      y: new KalmanFilter(),
      z: new KalmanFilter()
    };
    this.lastUpdate = Date.now();
  }

  getWorldFromBone(position, rotation, scale, bindpose) {
    const mat = quaternionToMatrix(rotation);
    const modelMatrix = [
      mat[0] * scale.x, mat[1] * scale.y, mat[2] * scale.z, position.x,
      mat[4] * scale.x, mat[5] * scale.y, mat[6] * scale.z, position.y,
      mat[8] * scale.x, mat[9] * scale.y, mat[10] * scale.z, position.z,
      0, 0, 0, 1
    ];
    const bind = [
      bindpose.e00, bindpose.e01, bindpose.e02, bindpose.e03,
      bindpose.e10, bindpose.e11, bindpose.e12, bindpose.e13,
      bindpose.e20, bindpose.e21, bindpose.e22, bindpose.e23,
      bindpose.e30, bindpose.e31, bindpose.e32, bindpose.e33
    ];
    return multiplyMatrixVec(bind, new Vector3(modelMatrix[3], modelMatrix[7], modelMatrix[11]));
  }

  trackKalman(vec) {
    const now = Date.now();
    const dt = (now - this.lastUpdate) / 1000;
    if (dt > 0 && this.prevPos) {
      const delta = vec.subtract(this.prevPos).multiplyScalar(1 / dt);
      this.velocity = delta;
    }
    this.prevPos = vec.clone();
    this.lastUpdate = now;

    return new Vector3(
      this.kalman.x.filter(vec.x),
      this.kalman.y.filter(vec.y),
      this.kalman.z.filter(vec.z)
    );
  }

  assistFromChest(head, chest) {
    const dragZone = 0.35; // vùng ngắm nhạy quanh chest
    const diff = head.subtract(chest);
    return diff.length() < dragZone;
  }

  aimTo(boneHeadInfo, boneChestInfo) {
    const worldHead = this.getWorldFromBone(boneHeadInfo.position, boneHeadInfo.rotation, boneHeadInfo.scale, boneHeadInfo.bindpose);
    const worldChest = this.getWorldFromBone(boneChestInfo.position, boneChestInfo.rotation, boneChestInfo.scale, boneChestInfo.bindpose);

    const shouldAssist = this.assistFromChest(worldHead, worldChest);
    const target = shouldAssist ? worldHead : worldChest;
    const final = this.trackKalman(target);

    this.setAim(final);
  }

  setAim(vec3) {
    console.log("🎯 Auto Aim:", vec3.x.toFixed(6), vec3.y.toFixed(6), vec3.z.toFixed(6));
    // GameAPI.setCrosshair(vec3.x, vec3.y, vec3.z); // <-- nếu hook vào Free Fire API
  }

  runLoop(boneHeadInfo, boneChestInfo) {
    const loop = () => {
      this.aimTo(boneHeadInfo, boneChestInfo);
      setTimeout(loop, 16); // 60fps
    };
    loop();
  }
}

// === Bone data ===
const boneHeadInfo = {
  position: { x: -0.0456970781, y: -0.004478302, z: -0.0200432576 },
  rotation: { x: 0.0258174837, y: -0.08611039, z: -0.1402113, w: 0.9860321 },
  scale: { x: 0.99999994, y: 1.00000012, z: 1.0 },
  bindpose: {
    e00: -1.34559613e-13, e01: 8.881784e-14, e02: -1.0, e03: 0.487912,
    e10: -2.84512817e-6, e11: -1.0, e12: 8.881784e-14, e13: -2.842171e-14,
    e20: -1.0, e21: 2.84512817e-6, e22: -1.72951931e-13, e23: 0.0,
    e30: 0.0, e31: 0.0, e32: 0.0, e33: 1.0
  }
};

const boneChestInfo = {
  position: { x: -0.045, y: -0.08, z: -0.01 },
  rotation: { x: 0, y: 0, z: 0, w: 1 },
  scale: { x: 1, y: 1, z: 1 },
  bindpose: {
    e00: 1, e01: 0, e02: 0, e03: 0.4,
    e10: 0, e11: 1, e12: 0, e13: 0,
    e20: 0, e21: 0, e22: 1, e23: 0,
    e30: 0, e31: 0, e32: 0, e33: 1
  }
};

// === Run AimLock ===
const aimSystem = new AimLockFull();
aimSystem.runLoop(boneHeadInfo, boneChestInfo);
