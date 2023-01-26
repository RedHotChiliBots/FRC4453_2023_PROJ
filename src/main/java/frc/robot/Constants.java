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

		public static final int kClimbLeftMotor = 20;
		public static final int kClimbRightMotor = 21;

		public static final int kHopperMotor = 40;
		public static final int kCollectorMotor = 50;

		public static final int kShooterLMotor = 30;
		public static final int kShooterRMotor = 31;
	}

	public static final class PneumaticChannelConstants {
		public static final int kGearShifterHi = 1;
		public static final int kGearShifterLo = 0;
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

	public static final class ClimberConstants {
		public static final double kP = 0.35;
		public static final double kI = 0.0000;
		public static final double kD = 0.0;
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

		// all climber measurements are from floor to underside of hook and floor to top
		// of rung
		public static final double kFloor2Hook = 10.4 + 32.0; // inches from floor to top of outer climber tube
		public static final double kClearUnder = 3.0; // inches below top of rung to clear
		public static final double kEngageOver = 3.0; // inches avobe top of rung to engage

		public static final double kLowRung = 48.75; // inches above floor per rules
		public static final double kMidRung = 60.25 - 2.75; // inches aboave floor per rules
		public static final double kMaxExtend = -(60.25 - 2.75 - ClimberConstants.kFloor2Hook
				+ ClimberConstants.kEngageOver); // inches aboave floor per rules

		public static final double kClearLowRung = -(ClimberConstants.kLowRung - ClimberConstants.kFloor2Hook
				- ClimberConstants.kClearUnder);
		public static final double kEngageLowRung = -(ClimberConstants.kLowRung - ClimberConstants.kFloor2Hook
				+ ClimberConstants.kEngageOver); // inches
		public static final double kClearMidRung = -(ClimberConstants.kMidRung - ClimberConstants.kFloor2Hook
				- ClimberConstants.kClearUnder); // inches
		public static final double kEngageMidRung = kMaxExtend; // inches

		public static final double kEngageHighTrav = kMaxExtend - 0.5; // inches to position for
		public static final double kHookHighTrav = kMaxExtend + 3.0; // inches to position for latch
																		// high/traverse
		// rungs
		// public static final double kFullExtendPerpendicular = 66.0; // inches
		// public static final double kFullExtendSwivel = 72.92;// inches
		public static final double kPullUpLatch = -(0.0); // inches to latch climber
		public static final double kPullUpClear = -(3.0); // inches to unhook while latched
		public static final double kStow = 0.0;
		public static final double kOneRev = -ClimberConstants.kPulleyCirc;

		public static final double kDistanceTolerance = 0.125; // inches

		public static final long kLatchDelay = 250; // milliseconds
		public static final long kSwivelDelay = 750; // milliseconds
		// public static final double kInitDelay = 0.25; // seconds
		public static final double kInitSafety = 10.0; // seconds

		public static final double kMaxAmps = 10.0;
		public static final double kInitSpeed = 0.6;

		public static final double kMaxPitch = 2.0; // degrees

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
