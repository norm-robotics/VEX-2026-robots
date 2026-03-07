from config import * # lsp-only
from utils import * # lsp-only
from init import * # lsp-only

class SensorFusion:
    # √2 correction for X-drive: wheels are at 45° to chassis axes,
    # so encoder travel under-reports actual robot displacement by 1/√2.
    XDRIVE_SCALE = 1.41421  # √2

    def __init__(self, wheel_diameter):
        self.wheel_circ = wheel_diameter * 3.14159
        self.x = 0.0           # estimated field X
        self.y = 0.0           # estimated field Y
        self.heading = 0.0     # estimated heading (degrees, 0-360)
        self._prev_enc = [0.0, 0.0, 0.0, 0.0]  # FL, FR, RL, RR
        self._prev_vx = 0.0    # previous encoder-derived velocity (robot X)
        self._prev_vy = 0.0    # previous encoder-derived velocity (robot Y)
        self._prev_time = 0    # ms timestamp of last update
        self._collision_until = 50  # ms timestamp when collision cooldown expires
        self._heading_before_collision = 0.0
        self._collision_flag = False  # set True by callback, consumed by update()
        self._initialized = False

        # Register collision callback on the IMU
        imu.collision(self._on_collision)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def initialize(self):
        """Seed state from sensors.  Call once after calibration."""
        self.x = gps.x_position()
        self.y = gps.y_position()
        self.heading = imu.heading()
        self._prev_enc = self._read_encoders()
        self._prev_vx = 0.0
        self._prev_vy = 0.0
        self._prev_time = brain.timer.time()
        self._collision_until = 0
        self._heading_before_collision = self.heading
        self._initialized = True

    def update(self):
        """Call once per loop (~20 ms).  Returns (x, y, heading)."""
        if not self._initialized:
            self.initialize()
            return self.x, self.y, self.heading

        now = brain.timer.time()

        # ----- Collision detection -----
        # Callback sets _collision_flag; we consume it here
        if self._collision_flag:
            self._collision_flag = False
            self._collision_until = now + COLLISION_COOLDOWN_MS
            self._heading_before_collision = self.heading
            # Consume encoder deltas without using them (keep prev in sync)
            self._prev_enc = self._read_encoders()
            self._prev_vx = 0.0
            self._prev_vy = 0.0
            self._prev_time = now

        in_cooldown = now < self._collision_until

        # ----- Collision cooldown: only trust GPS -----
        if in_cooldown:
            # Consume encoder deltas to keep prev in sync, but don't use them
            enc = self._read_encoders()
            self._prev_enc = enc
            self._prev_vx = 0.0
            self._prev_vy = 0.0
            self._prev_time = now

            # Only update from GPS if quality is acceptable
            quality = gps.quality()
            if quality >= GPS_QUALITY_MIN:
                self.x = gps.x_position()
                self.y = gps.y_position()
                self.heading = gps.heading()
            # Otherwise hold previous position and heading – no other data is reliable

            return self.x, self.y, self.heading

        # ===== Normal (non-collision) path =====

        # ----- Heading (IMU) -----
        self.heading = imu.heading()

        enc = self._read_encoders()
        deltas = [e - p for e, p in zip(enc, self._prev_enc)]
        self._prev_enc = enc

        d_fl = self._enc_to_dist(deltas[0])
        d_fr = self._enc_to_dist(deltas[1])
        d_rl = self._enc_to_dist(deltas[2])
        d_rr = self._enc_to_dist(deltas[3])

        robot_dx = (d_fl - d_fr - d_rl + d_rr) / 4.0
        robot_dy = (d_fl + d_fr + d_rl + d_rr) / 4.0

        # X-drive √2 correction: wheels at 45° under-report by factor 1/√2
        robot_dx *= self.XDRIVE_SCALE
        robot_dy *= self.XDRIVE_SCALE

        # ----- Slippage detection via IMU accelerometer -----
        dt = (now - self._prev_time) / 1000.0  # seconds
        if dt < 0.001:
            dt = 0.02  # fallback

        # Encoder-derived velocity and acceleration (robot frame)
        enc_vx = robot_dx / dt
        enc_vy = robot_dy / dt
        enc_ax = (enc_vx - self._prev_vx) / dt
        enc_ay = (enc_vy - self._prev_vy) / dt
        self._prev_vx = enc_vx
        self._prev_vy = enc_vy
        self._prev_time = now

        # IMU measured acceleration (robot frame, in/s²)
        # VEX IMU returns G's; convert to in/s² (1 G ≈ 386.09 in/s²)
        imu_ax = imu.acceleration(XAXIS) * 386.09
        imu_ay = imu.acceleration(YAXIS) * 386.09

        # Mismatch magnitude between encoder-predicted and IMU-measured accel
        accel_err = math.sqrt((enc_ax - imu_ax) ** 2 + (enc_ay - imu_ay) ** 2)

        # Scale down encoder trust when slippage detected
        if accel_err > SLIP_ACCEL_THRESHOLD:
            slip_scale = SLIP_ACCEL_THRESHOLD / accel_err  # 0..1, shrinks as mismatch grows
            robot_dx *= slip_scale
            robot_dy *= slip_scale

        # Rotate robot frame → field frame  (inverse of field→robot)
        h_rad = self.heading * (3.14159 / 180.0)
        cos_h = math.cos(h_rad)
        sin_h = math.sin(h_rad)
        field_dx =  robot_dx * cos_h + robot_dy * sin_h
        field_dy = -robot_dx * sin_h + robot_dy * cos_h

        self.x += field_dx
        self.y += field_dy

        # ----- Correct: blend GPS when quality is acceptable -----
        quality = gps.quality()
        if quality >= GPS_QUALITY_MIN:
            # Alpha rises linearly from 0 → ALPHA_MAX over the quality range
            t = (quality - GPS_QUALITY_MIN) / max(100 - GPS_QUALITY_MIN, 1)
            t = min(t, 1.0)
            alpha = t * ALPHA_MAX

            gps_x = gps.x_position()
            gps_y = gps.y_position()

            self.x += alpha * (gps_x - self.x)
            self.y += alpha * (gps_y - self.y)

        return self.x, self.y, self.heading

    def get_position(self):
        """Return the latest fused (x, y)."""
        return self.x, self.y

    def get_heading(self):
        """Return the latest fused heading."""
        return self.heading

    def set_heading(self, heading):
        """Reset heading (e.g., when driver presses the reset button)."""
        imu.set_heading(heading)
        self.heading = heading

    def _read_encoders(self):
        """Read one motor per corner: [FL, FR, RL, RR] in degrees."""
        return [
            lBottFront.position(DEGREES),
            rBottFront.position(DEGREES),
            lBottBack.position(DEGREES),
            rBottBack.position(DEGREES),
        ]

    def _enc_to_dist(self, degrees):
        """Convert encoder degrees to linear wheel travel."""
        return (degrees / 360.0) * self.wheel_circ

    def _on_collision(self):
        """IMU collision callback – just sets a flag for update() to consume."""
        self._collision_flag = True

# Global instance – created after sensor calibration (init.py runs first)
fusion = SensorFusion(WHEEL_DIAMETER)
