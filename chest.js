// === Vector3 class ===
class Vector3 {
  constructor(x=0,y=0,z=0) { this.x=x; this.y=y; this.z=z; }
  add(v) { return new Vector3(this.x+v.x,this.y+v.y,this.z+v.z); }
  subtract(v) { return new Vector3(this.x-v.x,this.y-v.y,this.z-v.z); }
  multiplyScalar(s) { return new Vector3(this.x*s,this.y*s,this.z*s); }
  length() { return Math.sqrt(this.x**2+this.y**2+this.z**2); }
  normalize() {
    const len=this.length();
    return len>0 ? this.multiplyScalar(1/len) : new Vector3();
  }
  clone() { return new Vector3(this.x,this.y,this.z); }
}

// === Kalman Filter cho từng trục ===
class KalmanFilter {
  constructor(R=0.01, Q=0.0001) {
    this.R = R;  // Noise measurement covariance
    this.Q = Q;  // Process noise covariance
    this.A = 1;  // State transition coefficient
    this.C = 1;  // Measurement coefficient
    this.cov = NaN;
    this.x = NaN; // Estimated state
  }
  filter(z) {
    if (isNaN(this.x)) {
      this.x = z;
      this.cov = this.R;
    } else {
      // Predict
      const predX = this.A * this.x;
      const predCov = this.A * this.cov * this.A + this.Q;
      // Kalman Gain
      const K = predCov * this.C / (this.C * predCov * this.C + this.R);
      // Update
      this.x = predX + K * (z - this.C * predX);
      this.cov = predCov - K * this.C * predCov;
    }
    return this.x;
  }
  reset() {
    this.cov = NaN;
    this.x = NaN;
  }
}

// === Helper nhân matrix 4x4 với Vector3 (cộng 1 cho vị trí) ===
function multiplyMatrixVec(m, v) {
  return new Vector3(
    m.e00 * v.x + m.e01 * v.y + m.e02 * v.z + m.e03,
    m.e10 * v.x + m.e11 * v.y + m.e12 * v.z + m.e13,
    m.e20 * v.x + m.e21 * v.y + m.e22 * v.z + m.e23
  );
}

// === Bone info và bindpose (bone_Head và bone_Chest) ===
const bone_Head = {
  position: new Vector3(-0.0456970781, -0.004478302, -0.0200432576),
  rotation: { x: 0.0258174837, y: -0.08611039, z: -0.1402113, w: 0.9860321 },
  scale: new Vector3(0.99999994, 1.00000012, 1.0),
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
  scale: new Vector3(1,1,1),
  bindpose: {
    e00: 0.0, e01: 0.0, e02: -1.0, e03: 0.511902,
    e10: -0.000000922564, e11: -1.0, e12: 0.0, e13: 0.0001275,
    e20: -1.0, e21: 0.000000922564, e22: 0.0, e23: 0.0,
    e30: 0, e31: 0, e32: 0, e33: 1
  }
};

// === Chuyển vị trí bone từ local space sang world space với bindpose ===
function getWorldPosition(bone) {
  return multiplyMatrixVec(bone.bindpose, bone.position);
}

// === Kiểm tra aim nằm trong vùng chest (bán kính radius) ===
function isAimInChestRegion(aimPos, chestPos, radius=0.15) {
  return aimPos.subtract(chestPos).length() <= radius;
}

// === AimLock System tích hợp Kalman Filter ===
class AimLockSystem {
  constructor() {
    this.kalmanX = new KalmanFilter();
    this.kalmanY = new KalmanFilter();
    this.kalmanZ = new KalmanFilter();
  }

  kalmanFilter(vec3) {
    return new Vector3(
      this.kalmanX.filter(vec3.x),
      this.kalmanY.filter(vec3.y),
      this.kalmanZ.filter(vec3.z)
    );
  }

  updateAim(currentAim) {
    // Tính vị trí thế giới của chest và head
    const chestWorldPos = getWorldPosition(bone_Chest);
    const headWorldPos = getWorldPosition(bone_Head);

    // Nếu aim trong vùng chest thì auto kéo aim về head
    let desiredAim = currentAim.clone();
    if (isAimInChestRegion(currentAim, chestWorldPos)) {
      desiredAim = headWorldPos.clone();
    }

    // Lọc nhiễu bằng Kalman Filter
    const smoothAim = this.kalmanFilter(desiredAim);
    return smoothAim;
  }
}

// === Vòng lặp chạy aim liên tục (60fps) ===
function runAimLoop() {
  const aimLock = new AimLockSystem();

  function loop() {
    // Lấy tâm ngắm hiện tại (bạn lấy dữ liệu thực tế từ game hoặc API)
    // Ví dụ:
    const currentAim = new Vector3(-0.14, -0.005, 0.01); // Giả lập tâm ngắm đang ở vùng thân (chest)

    // Cập nhật aim đã lọc và drag
    const aimToSet = aimLock.updateAim(currentAim);

    // Gọi API set aim của game (thay thế đoạn này)
    console.log(`🎯 Aim set to: X=${aimToSet.x.toFixed(6)} Y=${aimToSet.y.toFixed(6)} Z=${aimToSet.z.toFixed(6)}`);

    // Lặp lại ~60fps
    setTimeout(loop, 16);
  }
  loop();
}

runAimLoop();
