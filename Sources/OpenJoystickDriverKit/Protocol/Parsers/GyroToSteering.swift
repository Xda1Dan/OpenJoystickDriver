import Foundation

/// Converts raw gyroscope angular velocity + accelerometer data into a
/// virtual left stick X axis value for steering.
///
/// Uses gyro integration for responsive tracking and an accelerometer-based
/// complementary filter to prevent drift over time.
///
/// The algorithm:
/// 1. Integrate gyro angular velocity over time to get current angle
/// 2. Use accelerometer to estimate tilt angle and correct gyro drift
/// 3. Apply deadzone to prevent drift when holding still
/// 4. Clamp to ±45° (full lock left/right)
/// 5. Scale to Int16 range for the virtual stick
public final class GyroToSteering: @unchecked Sendable {

  // MARK: - Configuration

  /// Maximum tilt angle in degrees for full lock (default: 45°)
  public var maxAngle: Double = 45.0

  /// Deadzone in degrees — angles within ±deadzone are treated as center
  public var deadzone: Double = 2.0

  /// Complementary filter coefficient (0.0–1.0).
  /// Higher = more trust in gyro (responsive but drifts).
  /// Lower = more trust in accel (stable but laggy).
  /// 0.98 is a good balance.
  public var gyroWeight: Double = 0.98

  /// Sensitivity multiplier applied to raw gyro before integration.
  /// Increase if steering feels too slow, decrease if too fast.
  public var sensitivity: Double = 1.0

  // MARK: - Internal state

  /// Current estimated roll angle in degrees
  private var angle: Double = 0.0

  /// Calibration bias (average gyro value when at rest)
  private var bias: Double = 0.0

  /// Whether calibration has been completed
  private var isCalibrated: Bool = false

  /// Samples collected during calibration phase
  private var calibrationSamples: Int = 0

  /// Sum of gyro values during calibration (for computing average)
  private var calibrationSum: Double = 0.0

  /// Number of samples needed for calibration (at ~800Hz, 500ms ≈ 400 samples)
  private let calibrationSampleCount: Int = 400

  /// Rate limit: minimum time between angle updates (prevents spikes)
  private var lastUpdateTime: TimeInterval = 0

  // MARK: - Public API

  public init() {}

  /// Process one frame of gyro + accel data and return the steering stick X value.
  ///
  /// - Parameters:
  ///   - gyroRaw: Raw 16-bit signed gyroscope value (angular velocity)
  ///   - accelY: Raw 16-bit signed accelerometer Y axis
  ///   - accelZ: Raw 16-bit signed accelerometer Z axis
  ///   - dt: Time delta in seconds since last frame
  /// - Returns: Int16 value (-32767...32767) for left stick X
  public func update(gyroRaw: Int16, accelY: Int16, accelZ: Int16, dt: Double) -> Int16 {
    // Convert raw gyro to degrees/second (BMI055 at ±2000°/s: 16.4 LSB per deg/s)
    let gyroDps = Double(gyroRaw) / 16.4 * sensitivity

    // Calibration phase: collect samples to determine bias
    if !isCalibrated {
      calibrationSum += gyroDps
      calibrationSamples += 1
      if calibrationSamples >= calibrationSampleCount {
        bias = calibrationSum / Double(calibrationSamples)
        isCalibrated = true
      }
      return 0  // no steering during calibration
    }

    // Remove calibration bias
    let correctedGyro = gyroDps - bias

    // Integrate angular velocity to get angle
    guard dt > 0 && dt < 0.1 else { return Int16(angle / maxAngle * 32_767) }
    angle += correctedGyro * dt

    // Accelerometer-based drift correction
    // When held as a steering wheel, accel Y and Z indicate tilt
    let accelYf = Double(accelY) / 8192.0  // normalize to ±1g
    let accelZf = Double(accelZ) / 8192.0
    let accelAngle = atan2(accelYf, accelZf) * (180.0 / .pi)

    // Complementary filter: blend gyro integration with accel estimate
    angle = gyroWeight * angle + (1.0 - gyroWeight) * accelAngle

    // Apply deadzone
    if abs(angle) < deadzone {
      // Slowly decay toward zero when within deadzone
      angle *= 0.9
      if abs(angle) < 0.1 { angle = 0 }
    }

    // Clamp to max angle
    angle = max(-maxAngle, min(maxAngle, angle))

    // Scale to Int16 range
    return Int16(angle / maxAngle * 32_767)
  }

  /// Reset the steering angle to center. Call when disabling gyro or re-centering.
  public func reset() {
    angle = 0.0
  }

  /// Force recalibration (place controller flat on surface, then call).
  public func recalibrate() {
    isCalibrated = false
    calibrationSamples = 0
    calibrationSum = 0.0
    angle = 0.0
  }
}
