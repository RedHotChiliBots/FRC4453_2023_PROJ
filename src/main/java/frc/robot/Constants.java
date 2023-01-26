// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

	public static final class CANidConstants {
		public static final int kPDP = 0;
		public static final int kCompressor = 0;

		public static final int kRightMasterMotor = 10;
		public static final int kRightFollowerMotor = 11;
		public static final int kLeftMasterMotor = 12;
		public static final int kLeftFollowerMotor = 13;

		public static final int kCraneTurretMotor = 20;
		public static final int kCraneTiltMotor = 21;
		public static final int kCraneArmMotor = 22;

		public static final int kIntakeLeftMotor = 30;
		public static final int kIntakeRightMotor = 31;

		public static final int kShooterLMotor = 30;
		public static final int kShooterRMotor = 31;
	}

	public static final class PneumaticChannelConstants {
		public static final int kGearShifterHi = 1;
		public static final int kGearShifterLo = 0;
		public static final int kIntakeArmOpen = 2;
		public static final int kIntakeArmClose = 3;
		public static final int kLeftPistonClose = 4;
		public static final int kLeftPistonOpen = 5;
		public static final int kRightPistonClose = 6;
		public static final int kRightPistonOpen = 7;
	}

	public static final class DIOChannelConstants {
		public static final int kClimberLeftLimit = 0;
		public static final int kClimberRightLimit = 1;
		public static final int kCollectorExiting = 2;
		public static final int kHopperEntering = 3;
		public static final int kHopperExiting = 4;
		public static final int kFeederEntering = 5;
		public static final int kFeederExiting = 6;
	}

	public static final class PWMChannelConstants {
		public static final int kShooterLeftServo = 0;
		public static final int kShooterRightServo = 1;
	}

	public static final class OIConstants {
		public static final int kDriverControllerPort = 0;
		public static final int kOperatorControllerPort = 1;

		public static final long kRumbleDelay = 1000; // milliseconds
	}

	public static final class AnalogIOConstants {
		public static final int kHiPressureChannel = 0;
		public static final int kLoPressureChannel = 1;

		public static final double kInputVoltage = 5.0;
	}

	public static final class ChassisConstants {
		// Constants for Drive PIDs
		public static final double kP = 5.5108E-07;	// from sysID
		public static final double kI = 0.0;
		public static final double kD = 0.0;
		public static final double kIz = 0.0;
		public static final double kFF = 0.000156;

		public static final double kA = 0.29701;	// from sysID
		public static final double kS = 0.16439;	// from SysID
		public static final double kV = 1.3349;		// from sysID

		public static final double kMinOutput = -1.0; // -0.5;
		public static final double kMaxOutput = 1.0; // 0.5;
		public static final double maxRPM = 5700;
		public static final double maxVel = 2000;
		public static final double minVel = 1000;
		public static final double allowedErr = 100.0;
		public static final double maxAcc = 1500;

		public static final double kMetersPerInch = 0.0254;
		public static final double kTrackWidth = 26.341 * kMetersPerInch; // meters
		public static final double kWheelCirc = (Math.PI * 8.0) * kMetersPerInch; // meters
		public static final int kEncoderResolution = 1; // not used, NEO's native units are rotations
		public static final double kGearBoxRatio = 10.71;
		public static final double kPosFactor = kWheelCirc / kGearBoxRatio; // Meters per Revolution
		public static final double kVelFactor = kWheelCirc / kGearBoxRatio / 60.0; // Meters per Second
		public static final double kCountsPerRevGearbox = kEncoderResolution * kGearBoxRatio;

		public static final double kPosFactorMPC = kWheelCirc / kCountsPerRevGearbox; // Meters per Revolution
		public static final double kPosFactorCPM = kCountsPerRevGearbox / kWheelCirc; // Meters per Revolution

		// Example value only - as above, this must be tuned for your drive!
		public static final double kPDriveVel = 0.5;

		public static final double kMaxSpeedMetersPerSecond = 1.5; // was 1.0
		public static final double kMaxAccelerationMetersPerSecondSquared = 2.0; // was 0.7

		public static final double kDistP = 0.15;
		public static final double kDistI = 0.0;
		public static final double kDistD = 0.0;

		public static final double kLevelP = 0.015;
		public static final double kLevelI = 0.0;
		public static final double kLevelD = 0.0075;

		public static final double kLevelSetPoint = 0.0;
		public static final double kLevelSetTolerance = 1.0;

		public static final double kRateP = 0.015;
		public static final double kRateI = 0.0;
		public static final double kRateD = 0.0075;

		public static final double kRateSetPoint = 3.0;		// rate, deg/sec
		public static final double kRateSetTolerance = 1.0;

		public static final double kDistanceTolerance = 0.025;	// meters

		public static final double kAngleRungAttached = 15.0;	// degrees

		// Reasonable baseline values for a RAMSETE follower in units of meters and
		// seconds
		public static final double kRamseteB = 2;
		public static final double kRamseteZeta = 0.7;
	}

	public static final class CraneConstants {
		public static final double kTurretP = 0.35;
		public static final double kTurretI = 0.0000;
		public static final double kTurretD = 0.0;
		public static final double kTurretSetPoint = 0;
		public static final double kTurretSetTolerance = 0.5;

		public static final double kTiltP = 0.35;
		public static final double kTiltI = 0.0000;
		public static final double kTiltD = 0.0;
		public static final double kTiltSetPoint = 0;
		public static final double kTiltSetTolerance = 0.5;

		public static final double kArmP = 0.35;
		public static final double kArmI = 0.0000;
		public static final double kArmD = 0.0;
		public static final double kArmSetPoint = 0;
		public static final double kArmTolerance = 0.5;

		public static final double kTurretPosFactor = 0; //kWheelCirc / kCountsPerRevGearbox; // Meters per Revolution
		public static final double kTiltPosFactor = 0; //kWheelCirc / kCountsPerRevGearbox; // Meters per Revolution
		public static final double kArmPosFactor = 0; //kWheelCirc / kCountsPerRevGearbox; // Meters per Revolution

		public static final double kIz = 0.0;
		public static final double kFF = 0.0;
		public static final double kMinOutput = -1.0;
		public static final double kMaxOutput = 1.0;

		public static final double kRopeDia = 0.125; // inches add to Circ calc
		public static final double kPulleyCirc = Math.PI * ((20.2 / 25.4) + kRopeDia); // inches
		public static final int kEncoderResolution = 1; // not used, NEO's native units are rotations
		public static final int kGearBoxRatio = 12;
		public static final double kCountsPerRevGearbox = kEncoderResolution * kGearBoxRatio;
		public static final double kPosFactorIPC = kPulleyCirc / kCountsPerRevGearbox; // inches per count
		public static final double kPosFactorCPI = kCountsPerRevGearbox / kPulleyCirc; // counts per inch
	}

	public static final class CollectorConstants {
		public static final double kP = 0.00008;
		public static final double kI = 0.0000004;
		public static final double kD = 0.0;
		public static final double kIz = 0.0;
		public static final double kFF = 0.0;
		public static final double kMinOutput = -0.5;
		public static final double kMaxOutput = 0.5;

		public static final double kStopRPMs = 0.0;
		public static final double kMinRPM = -5700.0;
		public static final double kMaxRPM = 5700.0; // 2800 rpm when prototype tested 1-18-22

		public static final double kCollectorRPMs = kMaxRPM * 1.0;

		public static final double kVelocityTolerance = 50.0; // rpms

		public static final long kArmDelay = 2000; // milliseconds
	}

	public static final class HopperConstants {
		public static final double kP = 0.00008;
		public static final double kI = 0.0000004;
		public static final double kD = 0.0;
		public static final double kIz = 0.0;
		public static final double kFF = 0.0;
		public static final double kMinOutput = -0.5;
		public static final double kMaxOutput = 0.5;

		public static final double kStopRPMs = 0.0;
		public static final double kMinRPM = 5000.0; // -4540.0;
		public static final double kMaxRPM = 5000.0; // 5676 free spin. 2800 rpm when prototype tested 1-18-22

		public static final double kHopperRPMs = kMaxRPM * 0.6;
		public static final double kHopperShootRPMS = kMaxRPM; // was 0.75

		public static final double kVelocityTolerance = 50.0; // rpms
	}

	public static class FeederConstants {
		public static final double kFeederP = 0.00008;
		public static final double kFeederI = 0.0000004;
		public static final double kFeederD = 0.0;
		public static final double kFeederIz = 0.0;
		public static final double kFeederFF = 0.0;
		public static final double kFeederMinOutput = -1.0;
		public static final double kFeederMaxOutput = 1.0;

		public static final double kMinFeederRPM = -4540.0;
		public static final double kMaxFeederRPM = 4540.8; // 2800 rpm when prototype tested 1-18-22

		public static final double kFeederRPMs = kMaxFeederRPM * 0.25;

		public static final double kFeederVelocityTolerance = 5.0; // rpms
	}

	public static final class ShooterConstants {
		public static final double kShootP = 0.00005; // was 0.00008
		public static final double kShootI = 0.0000005;
		public static final double kShootD = 0.0;
		public static final double kShootIz = 0.0;
		public static final double kShootFF = 0.0;
		public static final double kShootMinOutput = -1.0;
		public static final double kShootMaxOutput = 1.0;

		public static final double kStopRPMs = 0.0;
		public static final double kMinShootRPM = -4540.0; // 5676 free spin max
		public static final double kMaxShootRPM = 4540.8; // 2800 rpm when prototype tested 1-18-22

		public static final double kShooterRPMs = 1500; // was kMaxShootRPM * 0.35; // 0.6;
		public static final double kShooterSuckRPMS = kMaxShootRPM * 0.3;

		public static final double kShootVelocityTolerance = 50.0; // rpms

		public static final double kTimeShootAfterEmpty = 3.0;
	}
}
