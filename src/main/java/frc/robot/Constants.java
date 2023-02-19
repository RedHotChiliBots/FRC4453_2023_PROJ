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

	public enum E {
		NA,
		CONE,
		CUBE
	}

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

	public static final class PneumaticModuleConstants {
		public static final int kPCM0 = 0;
		public static final int kPCM1 = 1;
	}

	public static final class Pneumatic0ChannelConstants {
		public static final int kChassisShifterHi = 0;
		public static final int kChassisShifterLo = 1;
		public static final int kIntakeArmOpen = 2;
		public static final int kIntakeArmClose = 3;
		public static final int kClawLeftClose = 4;
		public static final int kClawLeftOpen = 5;
		public static final int kClawRightClose = 6;
		public static final int kClawRightOpen = 7;
	}

	public static final class Pneumatic1ChannelConstants {
		public static final int kRatchetLock = 0;
		public static final int kRatchetUnlock = 1;
		public static final int kIntakeBarEnabled = 2;
		public static final int kIntakeBarDisabled = 3;
	}

	public static final class DIOChannelConstants {
		public static final int kElementIn = 0;
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

	public static final class AnalogInConstants {
		public static final int kHiPressureChannel = 0;
		public static final int kLoPressureChannel = 1;

		public static final double kInputVoltage = 5.0;
	}

	public static final class RelayConstants {
		// None defined
	}

	public static final class ChassisConstants {
		// Constants for Drive PIDs
		public static final double kP = 0.0032218; // from sysID
		public static final double kI = 0.0; // from sysID
		public static final double kD = 0.00047213; // from sysID
		public static final double kIz = 0.0;
		public static final double kFF = 0.000156;

		public static final double kA = 0.69884; // from sysID
		public static final double kS = 0.087827; // from SysID
		public static final double kV = 1.0907; // from sysID

		public static final double kMinOutput = -1.0; // -0.5;
		public static final double kMaxOutput = 1.0; // 0.5;
		public static final double maxRPM = 5700;
		public static final double maxVel = 2000;
		public static final double minVel = 1000;
		public static final double allowedErr = 100.0;
		public static final double maxAcc = 1500;

		public static final double kTrackWidth = Units.inchesToMeters(26.341); // meters
		public static final double kWheelCirc = Units.inchesToMeters(Math.PI * 8.0); // meters
		public static final double kEncoderResolution = 1.0; // not used, NEO's native units are rotations
		public static final double kGearBoxRatio = 10.71;
		public static final double kPosFactor = kWheelCirc / (kGearBoxRatio * kEncoderResolution); // Meters / Rev
		public static final double kVelFactor = kWheelCirc / (kGearBoxRatio * kEncoderResolution) / 60.0; // Meters /
																											// Sec
		public static final double kCountsPerRevGearbox = kEncoderResolution * kGearBoxRatio;

		public static final double kPosFactorMPR = kWheelCirc / kCountsPerRevGearbox; // Meters / Rev
		public static final double kPosFactorRPM = kCountsPerRevGearbox / kWheelCirc; // Rev / Meter
		// Example value only - as above, this must be tuned for your drive!
		public static final double kPDriveVel = 0.5;

		public static final double kMaxSpeedMetersPerSecond = 1.5; // was 1.0
		public static final double kMaxAccelerationMetersPerSecondSquared = 2.0; // was 0.7

		public static final double kDistP = 0.015;
		public static final double kDistI = 0.0;
		public static final double kDistD = 0.0;

		public static final double kLevelP = 0.05;
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
		public static final int kTurretSlot = 0;
		public static final double kTurretP = 5e-5;
		public static final double kTurretI = 0.0; // 1e-6;
		public static final double kTurretD = 0.0;
		public static final double kTurretInitPos = 0.0; // degrees
		public static final double kTurretStowPos = 0.0; // degrees
		public static final double kTurretReceivePos = 0.0; // degrees
		public static final double kTurretReadyPos = 180.0; // degrees
		public static final double kTurretNodePos = 180.0; // degrees

		public static final double kTurretIz = 0.0;
		public static final double kTurretFF = 0.000156;
		public static final double kTurretAllowErr = 0.5;
		public static final double kTurretMinOutput = -1.0;
		public static final double kTurretMaxOutput = 1.0;
		public static final double kTurretMinVel = 0.0;

		public static final int kTiltSlot = 0;
		public static final double kTiltP = 5e-5;
		public static final double kTiltI = 1e-6;
		public static final double kTiltD = 0.0;
		public static final double kTiltInitPos = -83.2; // degrees
		public static final double kTiltClearChassisPos = -45.0; // degrees
		public static final double kTiltStowPos = -83.2; // degrees
		public static final double kTiltReceivePos = -70.0; // degrees
		public static final double kTiltReadyPos = 0.0; // degrees

		public static final double kTiltIz = 0.0;
		public static final double kTiltFF = 0.000156;
		public static final double kTiltAllowErr = 0.5;
		public static final double kTiltMinOutput = -1.0;
		public static final double kTiltMaxOutput = 1.0;
		public static final double kTiltMinVel = 0.0;

		public static final int kArmSlot = 0;
		public static final double kArmP = 5e-5;
		public static final double kArmI = 1e-6;
		public static final double kArmD = 0.0;
		public static final double kArmInitPos = 25.065; // inches
		public static final double kArmStowPos = 25.065; // inches
		public static final double kArmReceivePos = 30.0; // inches
		public static final double kArmReadyPos = 25.065; // inches

		public static final double kArmIz = 0.0;
		public static final double kArmFF = 0.000156;
		public static final double kArmAllowErr = 0.5;
		public static final double kArmMinOutput = -1.0;
		public static final double kArmMaxOutput = 1;
		public static final double kArmMinVel = 0.0;

		public static final double kTurretSprocketTeeth = 170;
		public static final double kTurretMotorSprocketTeeth = 24;
		public static final double kTurretSprocketRatio = kTurretSprocketTeeth / kTurretMotorSprocketTeeth;
		public static final double kTurretGearBoxRatio = 20;
		public static final double kTurretRotationsPerDegree = (kTurretSprocketRatio * kTurretGearBoxRatio) / 360.0;
		public static final double kTurretDegreesPerRotation = 360.0 / (kTurretSprocketRatio * kTurretGearBoxRatio);
		public static final double kTurretMaxVel = 360.0 / kTurretDegreesPerRotation * 60.0; // RPM
		public static final double kTurretMaxAccel = kTurretMaxVel * (2.0 / 3.0); // RPM^2

		public static final double kTiltSprocketTeeth = 60;
		public static final double kTiltMotorSprocketTeeth = 18;
		public static final double kTiltSprocketRatio = kTiltSprocketTeeth / kTiltMotorSprocketTeeth;
		public static final double kTiltGearBoxRatio = 100;
		public static final double kTiltRotationsPerDegree = (kTiltSprocketRatio * kTiltGearBoxRatio) / 360.0;
		public static final double kTiltDegreesPerRotation = 360.0 / (kTiltSprocketRatio * kTiltGearBoxRatio);
		public static final double kTiltMaxVel = 360.0 / kTiltDegreesPerRotation * 60.0; // RPM
		public static final double kTiltMaxAccel = kTiltMaxVel * (2.0 / 3.0); // RPM^2

		public static final double kArmShaftDia = 1.0;
		public static final double kArmShaftCirc = kArmShaftDia * Math.PI;
		public static final double kArmGearBoxRatio = 4;
		public static final double kArmRotationsPerInch = kArmGearBoxRatio / kArmShaftCirc;
		public static final double kArmInchesPerRotation = kArmShaftCirc / kArmGearBoxRatio;
		public static final double kArmMaxVel = 36.0 / kArmInchesPerRotation * 60.0; // RPM
		public static final double kArmMaxAccel = kArmMaxVel * (2.0 / 3.0); // RPM^2

		public static final double kCraneArmClear = 0.0; // inches
		public static final double kCraneArmEngage = 5.0; // inches
		public static final double kCraneTiltClear = -45.0; // degrees
		public static final double kCraneTurretGridSide = 180.0; // degrees
		public static final double kCraneTurretElemSide = 0.0; // degrees
		public static final double kCraneWait = 2.0; // seconds
	}

	public static final class VisionConstants {
		public static final String kCameraName = "OV5647";

		// Constants such as camera and target height stored. Change per robot and goal!
		public static final double kCameraHeight = Units.inchesToMeters(15.5);
		public static final double kTargetHeight = Units.inchesToMeters(18.75);

		// Angle between horizontal and the camera.
		public static final double kCameraPitch = Units.degreesToRadians(0);

		// How far from the target we want to be
		public static final double kTargetDist = Units.feetToMeters(3);

		// PID constants should be tuned per robot
		public static final double kDistP = 0.6;
		public static final double kDistI = 0.0;
		public static final double kDistD = 0.0;

		public static final double kTurnP = 0.03;
		public static final double kTurnI = 0.0;
		public static final double kTurnD = 0.0;
	}
}
