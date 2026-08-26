import Foundation
import Testing

@testable import OpenJoystickDriverKit

/// Unit tests for the gyro-to-steering complementary filter.
///
/// Raw gyro values use the BMI055 ±2000°/s scale: 16.4 LSB per degree/second.
/// Tests configure `gyroWeight = 1.0` (pure gyro integration, accel blending
/// disabled) unless stated, so results are deterministic.
@Suite("Gyro To Steering Tests") struct GyroToSteeringTests {

  /// Feed one frame and return the steering value.
  @discardableResult
  private func step(
    _ g: GyroToSteering,
    gyroRaw: Int16,
    accelY: Int16 = 0,
    accelZ: Int16 = 8192,
    dt: Double
  ) -> Int16 {
    g.update(gyroRaw: gyroRaw, accelY: accelY, accelZ: accelZ, dt: dt)
  }

  @Test("Calibration phase returns zero steering regardless of input")
  func calibrationReturnsZero() {
    let g = GyroToSteering()
    // Hard-spin the gyro during the whole calibration window.
    for _ in 0..<400 {
      let v = g.update(gyroRaw: Int16.max, accelY: 0, accelZ: 8192, dt: 0.001)
      #expect(v == 0)
    }
  }

  @Test("First post-calibration frame starts producing steering output")
  func firstFrameAfterCalibrationProducesOutput() {
    let g = GyroToSteering()
    g.gyroWeight = 1.0
    g.deadzone = 0
    for _ in 0..<400 {
      _ = g.update(gyroRaw: 0, accelY: 0, accelZ: 8192, dt: 0.001)
    }
    // Frame 401 is the first integrated frame: 1640 LSB = 100 deg/s.
    let v = step(g, gyroRaw: 1640, dt: 0.005)
    #expect(v > 0)
  }

  @Test("recalibrate() re-enters the calibration phase")
  func recalibrateReentersCalibration() {
    let g = GyroToSteering()
    g.gyroWeight = 1.0
    for _ in 0..<450 {
      _ = g.update(gyroRaw: 1640, accelY: 0, accelZ: 8192, dt: 0.005)
    }
    g.recalibrate()
    for _ in 0..<400 {
      #expect(step(g, gyroRaw: 1640, dt: 0.005) == 0)
    }
  }

  @Test("Positive yaw rotation steers right (positive stick X)")
  func positiveYawSteersRight() {
    let g = GyroToSteering()
    g.gyroWeight = 1.0
    g.deadzone = 0
    for _ in 0..<400 { _ = g.update(gyroRaw: 0, accelY: 0, accelZ: 8192, dt: 0.001) }
    var last: Int16 = 0
    for _ in 0..<50 {
      last = step(g, gyroRaw: 1640, dt: 0.005)
    }
    #expect(last > 0)
  }

  @Test("Negative yaw rotation steers left (negative stick X)")
  func negativeYawSteersLeft() {
    let g = GyroToSteering()
    g.gyroWeight = 1.0
    g.deadzone = 0
    for _ in 0..<400 { _ = g.update(gyroRaw: 0, accelY: 0, accelZ: 8192, dt: 0.001) }
    var last: Int16 = 0
    for _ in 0..<50 {
      last = step(g, gyroRaw: -1640, dt: 0.005)
    }
    #expect(last < 0)
  }

  @Test("Sustained hard rotation saturates at full lock")
  func saturationAtMaxAngle() {
    let g = GyroToSteering()
    g.gyroWeight = 1.0
    g.deadzone = 0
    g.maxAngle = 45
    for _ in 0..<400 { _ = g.update(gyroRaw: 0, accelY: 0, accelZ: 8192, dt: 0.001) }
    var last: Int16 = 0
    for _ in 0..<200 {
      last = step(g, gyroRaw: 16_400, dt: 0.002)
    }
    #expect(last == 32_767)
    // Reverse rotation recovers from full lock toward the opposite side.
    for _ in 0..<600 {
      last = step(g, gyroRaw: -16_400, dt: 0.002)
    }
    #expect(last == -32_767)
  }

  @Test("Output never exceeds Int16 stick range across an input sweep")
  func outputStaysInStickRange() {
    let g = GyroToSteering()
    for raw in stride(from: Int16(-32_767), through: 32_767, by: 997) {
      for _ in 0..<3 {
        let v = g.update(gyroRaw: raw, accelY: 0, accelZ: 8192, dt: 0.001)
        #expect(v >= -32_767 && v <= 32_767)
      }
    }
  }

  @Test("Higher sensitivity reaches full lock faster")
  func sensitivityScalesRotationRate() {
    // Returns true if a sustained rotation at the given sensitivity saturates
    // the stick within `frames` post-calibration updates.
    func reachesFullLock(sensitivity: Double, frames: Int) -> Bool {
      let g = GyroToSteering()
      g.gyroWeight = 1.0
      g.deadzone = 0
      g.sensitivity = sensitivity
      for _ in 0..<400 {
        _ = g.update(gyroRaw: 0, accelY: 0, accelZ: 8192, dt: 0.002)
      }
      for _ in 0..<frames {
        let v = g.update(gyroRaw: 1_640, accelY: 0, accelZ: 8192, dt: 0.002)
        if v == 32_767 { return true }
      }
      return false
    }
    // 2x sensitivity: 0.4°/frame → saturates (~46°) well within 300 frames.
    #expect(reachesFullLock(sensitivity: 2.0, frames: 300))
    // 0.5x sensitivity: 0.1°/frame → only ~30° after 300 frames, no saturation.
    #expect(!reachesFullLock(sensitivity: 0.5, frames: 300))
  }

  @Test("Non-finite time deltas are ignored and hold current position")
  func invalidDtHoldsPosition() {
    let g = GyroToSteering()
    g.gyroWeight = 1.0
    g.deadzone = 0
    for _ in 0..<400 { _ = g.update(gyroRaw: 0, accelY: 0, accelZ: 8192, dt: 0.001) }
    // dt = 0 and dt >= 0.1 both bypass integration.
    for bad in [0.0, -0.5, 0.1, 2.5] {
      let v = step(g, gyroRaw: 16_400, dt: bad)
      #expect(v == 0)  // angle still centered; no rotation applied
    }
  }

  @Test("reset() recenters steering")
  func resetCentersSteering() {
    let g = GyroToSteering()
    g.gyroWeight = 1.0
    g.deadzone = 2.0
    for _ in 0..<400 { _ = g.update(gyroRaw: 0, accelY: 0, accelZ: 8192, dt: 0.001) }
    for _ in 0..<200 { _ = step(g, gyroRaw: 16_400, dt: 0.002) }
    g.reset()
    let v = step(g, gyroRaw: 0, dt: 0.002)
    #expect(v == 0)
  }
}