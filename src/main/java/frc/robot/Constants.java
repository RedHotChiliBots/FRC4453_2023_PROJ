// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

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
	}

	public static final class PneumaticChannelConstants {
		public static final int kGearShifterHi = 0;
		public static final int kGearShifterLo = 1;
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
		public static final double kP = 5.5108E-07; // from sysID
		public static final double kI = 0.0; // from sysID
		public static final double kD = 0.0; // from sysID
		public static final double kIz = 0.0;
		public static final double kFF = 0.000156;

		public static final double kA = 0.29701; // from sysID
		public static final double kS = 0.16439; // from SysID
		public static final double kV = 1.3349; // from sysID

		public static final double kMinOutput = -1.0; // -0.5;
		public static final double kMaxOutput = 1.0; // 0.5;
		public static final double maxRPM = 5700;
		public static final double maxVel = 2000;
		public static final double minVel = 1000;
		public static final double allowedErr = 100.0;
		public static final double maxAcc = 1500;

		public static final double kTrackWidth = Units.inchesToMeters(26.341); // meters
		public static final double kWheelCirc = Units.inchesToMeters(Math.PI * 8.0); // meters
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

		public static final double kDistP = 0.015;
		public static final double kDistI = 0.0;
		public static final double kDistD = 0.0;

		public static final double kLevelP = 1.0;
		public static final double kLevelI = 0.0;
		public static final double kLevelD = 0.0;

		public static final double kLevelSetPoint = 0.0;
		public static final double kLevelSetTolerance = 1.0;

		public static final double kRateP = 0.00015;
		public static final double kRateI = 0.0;
		public static final double kRateD = 0.0;

		public static final double kRateSetPoint = 3.0; // rate, deg/sec
		public static final double kRateSetTolerance = 1.0;

		public static final double kDistanceTolerance = 0.025; // meters

		public static final double kAngleRungAttached = 15.0; // degrees

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

		public static final double kTurretIz = 0.0;
		public static final double kTurretFF = 0.0;
		public static final double kTurretMinOutput = -1.0;
		public static final double kTurretMaxOutput = 1.0;

		public static final double kTiltP = 0.35;
		public static final double kTiltI = 0.0000;
		public static final double kTiltD = 0.0;
		public static final double kTiltSetPoint = 0;
		public static final double kTiltSetTolerance = 0.5;

		public static final double kTiltIz = 0.0;
		public static final double kTiltFF = 0.0;
		public static final double kTiltMinOutput = -1.0;
		public static final double kTiltMaxOutput = 1.0;

		public static final double kArmP = 0.35;
		public static final double kArmI = 0.0000;
		public static final double kArmD = 0.0;
		public static final double kArmSetPoint = 0;
		public static final double kArmTolerance = 0.5;

		public static final double kArmIz = 0.0;
		public static final double kArmFF = 0.0;
		public static final double kArmtMinOutput = -1.0;
		public static final double kArmtMaxOutput = 1.0;

		public static final double kTurretSprocketTeeth = 170;
		public static final double kTurretMotorSprocketTeeth = 18;
		public static final double kTurretSprocketRatio = kTurretSprocketTeeth / kTurretMotorSprocketTeeth;
		public static final double kTurretGearBoxRatio = 1;
		public static final double kTurretRotationsPerDegree = (kTurretSprocketRatio * kTurretGearBoxRatio) / 360.0;
		public static final double kTurretDegreesPerRotation = 360.0 / (kTurretSprocketRatio * kTurretGearBoxRatio);

		public static final double kTurretPosFactor = kTurretDegreesPerRotation;

		public static final double kTiltSprocketTeeth = 60;
		public static final double kTiltMotorSprocketTeeth = 18;
		public static final double kTiltSprocketRatio = kTiltSprocketTeeth / kTiltMotorSprocketTeeth;
		public static final double kTiltGearBoxRatio = 20;
		public static final double kTiltRotationsPerDegree = (kTiltSprocketRatio * kTiltGearBoxRatio) / 360.0;
		public static final double kTiltDegreesPerRotation = 360.0 / (kTiltSprocketRatio * kTiltGearBoxRatio);

		public static final double kTiltPosFactor = kTiltDegreesPerRotation;

		public static final double kArmShaftDia = 0.75;
		public static final double kArmShaftCirc = kArmShaftDia * Math.PI;
		public static final double kArmGearBoxRatio = 4.0;
		public static final double kArmRotationsPerInch = kArmGearBoxRatio / kArmShaftCirc;
		public static final double kArmInchesPerRotation = kArmShaftCirc / kArmGearBoxRatio;

		public static final double kArmPosFactor = kArmInchesPerRotation;
	}
}
