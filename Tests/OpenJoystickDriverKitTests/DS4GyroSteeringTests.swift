import Foundation
import Testing

@testable import OpenJoystickDriverKit

/// Integration tests: DS4Parser emits `.gyroSteeringChanged` events derived
/// from the gyro/accelerometer section of DS4 HID input reports.
@Suite("DS4 Gyro Steering Tests") struct DS4GyroSteeringTests {

  /// Build a DS4 USB-style input report (`0x01` report ID + payload).
  /// Payload layout matches `DS4Parser.ReportOffset`: sticks/buttons/triggers,
  /// BT timestamp at 9, gyro X/Y/Z at 12/14/16, accel X/Y/Z at 18/20/22.
  private func makeDS4Report(
    gyroX: Int16 = 0,
    gyroY: Int16 = 0,
    gyroZ: Int16 = 0,
    accelX: Int16 = 0,
    accelY: Int16 = 0,
    accelZ: Int16 = 8192,
    timestamp: UInt16 = 0,
    payloadLength: Int = 24
  ) -> Data {
    var payload = [UInt8](repeating: 0, count: 24)
    payload[0] = 128  // left stick center
    payload[1] = 128
    payload[2] = 128
    payload[3] = 128
    payload[9] = UInt8(timestamp & 0xFF)
    payload[10] = UInt8(timestamp >> 8)
    func put(_ value: Int16, at offset: Int) {
      let u = UInt16(bitPattern: value)
      payload[offset] = UInt8(u & 0xFF)
      payload[offset + 1] = UInt8(u >> 8)
    }
    put(gyroX, at: 12)
    put(gyroY, at: 14)
    put(gyroZ, at: 16)
    put(accelX, at: 18)
    put(accelY, at: 20)
    put(accelZ, at: 22)
    // Truncate after filling so callers can build short (<24-byte) payloads.
    return Data([0x01]) + Data(payload.prefix(payloadLength))
  }

  /// Feed `frames` reports with a sustained yaw rotation; timestamps advance
  /// so the parser computes a real dt (4000 units × 5.33μs ≈ 21.3ms/frame).
  private func steerValueAfterSustainedYaw(
    parser: DS4Parser,
    gyroY: Int16,
    frames: Int
  ) -> Int16? {
    var last: Int16?
    for i in 0..<frames {
      let report = makeDS4Report(
        gyroY: gyroY,
        timestamp: UInt16(truncatingIfNeeded: UInt64(i + 1) * 4_000)
      )
      guard let events = try? parser.parse(data: report) else { continue }
      for event in events {
        if case .gyroSteeringChanged(let value) = event {
          last = value
        }
      }
    }
    return last
  }

  @Test("Long reports emit a gyro steering event")
  func longReportEmitsGyroEvent() throws {
    let parser = DS4Parser()
    let events = try parser.parse(data: makeDS4Report())
    #expect(events.contains { if case .gyroSteeringChanged = $0 { return true }; return false })
  }

  @Test("Short reports (<24 bytes) emit no gyro steering events")
  func shortReportEmitsNoGyroEvents() throws {
    let parser = DS4Parser()
    let events = try parser.parse(data: makeDS4Report(payloadLength: 23))
    #expect(!events.contains { if case .gyroSteeringChanged = $0 { return true }; return false })
  }

  @Test("gyroSteeringEnabled = false suppresses gyro steering events")
  func disabledGyroSuppressesEvents() throws {
    let parser = DS4Parser()
    parser.gyroSteeringEnabled = false
    let events = try parser.parse(data: makeDS4Report(gyroY: 8_000))
    #expect(!events.contains { if case .gyroSteeringChanged = $0 { return true }; return false })
  }

  @Test("Sustained right twist steers positive through the parser")
  func sustainedRightTwistSteersPositive() {
    let parser = DS4Parser()
    let last = steerValueAfterSustainedYaw(parser: parser, gyroY: 1_640, frames: 500)
    #expect(last != nil && last! > 20_000)
  }

  @Test("Sustained left twist steers negative through the parser")
  func sustainedLeftTwistSteersNegative() {
    let parser = DS4Parser()
    let last = steerValueAfterSustainedYaw(parser: parser, gyroY: -1_640, frames: 500)
    #expect(last != nil && last! < -20_000)
  }

  @Test("Stationary controller after calibration stays near center")
  func stationaryStaysNearCenter() {
    let parser = DS4Parser()
    let last = steerValueAfterSustainedYaw(parser: parser, gyroY: 0, frames: 600)
    #expect(last != nil && abs(Int32(last!)) < 1_500)
  }

  @Test("steeringAxis selects which gyro axis drives steering")
  func steeringAxisSelection() {
    // Roll axis driven hard while yaw axis sits at 100 deg/s: roll-selected
    // parser must saturate past what yaw-only rotation produces.
    let rollParser = DS4Parser()
    rollParser.steeringAxis = .roll
    var lastRoll: Int16?
    for i in 0..<500 {
      let report = makeDS4Report(
        gyroY: 1_640,
        gyroZ: 16_400,
        timestamp: UInt16(truncatingIfNeeded: UInt64(i + 1) * 4_000)
      )
      guard let events = try? rollParser.parse(data: report) else { continue }
      for event in events {
        if case .gyroSteeringChanged(let value) = event { lastRoll = value }
      }
    }
    #expect(lastRoll != nil && lastRoll! == 32_767)
  }
}