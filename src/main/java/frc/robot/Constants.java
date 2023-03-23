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
		CUBE,
		OTHER
	}

	public static final class CANidConstants {
		public static final int kPDP = 0;
		public static final int kCompressor = 0;

		public static final int kRightMasterMotor = 10;
		public static final int kRightFollower1Motor = 11;
		public static final int kRightFollower2Motor = 14;
		public static final int kLeftMasterMotor = 12;
		public static final int kLeftFollower1Motor = 13;
		public static final int kLeftFollower2Motor = 15;

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
		public static final int kClawRightOpen = 4;
		public static final int kClawRightClose = 5;
		public static final int kClawLeftOpen = 6;
		public static final int kClawLeftClose = 7;
	}

	public static final class Pneumatic1ChannelConstants {
		public static final int kRatchetLock = 0;
		public static final int kRatchetUnlock = 1;
		public static final int kIntakeBarEnabled = 2;
		public static final int kIntakeBarDisabled = 3;
	}

	public static final class DIOChannelConstants {
		public static final int kElementIn = 0;
		public static final int kTiltAngleSensor = 1;
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
		public static final int kDriveSlot = 0; // from sysID
		// Constants for Drive PIDs
		public static final double kP = 0.0032145; // 0.0032218; // from sysID
		public static final double kI = 0.0; // from sysID
		public static final double kD = 0.00035147; // 0.00047213; // from sysID
		public static final double kIz = 0.0;
		public static final double kFF = 0.0;

		public static final double kA = 0.37529; // from sysID
		public static final double kS = 0.021284; // from SysID
		public static final double kV = 0.51487; // from sysID

		// Example value only - as above, this must be tuned for your drive!
		public static final double kPDriveVel = 0.5;

		public static final double kTrackWidth = 1.9631; // Units.inchesToMeters(23.0); // meters

		public static final double kMaxSpeedMetersPerSecond = 4.0; // was 1.0
		public static final double kMaxAccelerationMetersPerSecondSquared = 3.0; // was 0.7

		// Reasonable baseline values for a RAMSETE follower in units of meters and
		// seconds
		public static final double kRamseteB = 2;
		public static final double kRamseteZeta = 0.7;

		// ============================================================================

		public static final double kMinOutput = -1.0; // -0.5;
		public static final double kMaxOutput = 1.0; // 0.5;
		public static final double kDriveMaxRPM = 5700;
		public static final double kDriveMaxVel = 11.0; // RPM
		public static final double kDriveMaxAccel = kDriveMaxVel * (2.0 / 3.0); // RPM^2
		public static final double kDriveMinVel = 1000;
		public static final double kDriveMaxAcc = 1500;
		public static final double kDriveAllowErr = 0.01;

		public static final double kInches2Meters = 39.3700787402;	// conversion constant
		public static final double kWheelDia = 7.639;	// diameter in inches
		public static final double kWheelCircInches = Math.PI * kWheelDia; // circ in inches
		public static final double kWheelCircMeters = kWheelCircInches / kInches2Meters; // circ in meters

		public static final double kEncoderResolution = 1.0; // not used, NEO's native units are rotations
		public static final double kTBGearBoxRatio = 10.71;
		public static final double kHIGearBoxRatio = 25.0 / 6.0;
		public static final double kLOGearBoxRatio = 15625 / 1734;

		public static final double kLevelP = 0.0775;
		public static final double kLevelI = 0.0;
		public static final double kLevelD = 0.0;

		public static final double kLevelSetPoint = 0.0;
		public static final double kLevelSetTolerance = 1.0;

		public static final double kAutonBalanceLevel = 0.0; // degrees

		public static final double kLevelMinOutput = -0.3; // -0.5;
		public static final double kLevelMaxOutput = 0.3; // 0.5;

		public static final double kRateP = 0.00015;
		public static final double kRateI = 0.0;
		public static final double kRateD = 0.0;

		public static final double kRateSetPoint = 3.0; // rate, deg/sec
		public static final double kRateSetTolerance = 1.0;

		public static final double kDistanceTolerance = .00625; // meters
		public static final int kDistSlot = 0;

		public static final double kDistP = 1.5;	//0.5; // 5e-5;
		public static final double kDistI = 0.0; // 1e-6;
		public static final double kDistD = 0.0; // 0.0;
		public static final double kDistIz = 0.0;
		public static final double kDistFF = 0.0; // 0.000156;

		// public static final double kDistA = 0.69884; // from sysID
		// public static final double kDistS = 0.087827; // from SysID
		// public static final double kDistV = 1.0907; // from sysID

		public static final double kDistMinOutput = -0.5; // -0.5;
		public static final double kDistMaxOutput = 0.5; // 0.5;
		// public static final double kDistMaxRPM = 5700;
		// public static final double kDistMaxVel = 2000;
		// public static final double kDistMinVel = 1000;
		// public static final double kDistMaxAcc = 1500;
		// public static final double kDistAllowErr = 0.125;
		// public static final double kDistSetTolerance = 0.5;

		public static final double kAutonBalanceDist = 2.35; // meters
		public static final double kAutonMobilityDist = 4.0; // meters
		public static final double kAutonMobility2BalanceDist = -1.75; // meters
		public static final double kAutonParkDist = Units.feetToMeters(4.0); // meters

		public static final double kAutonTurnDist = Units.inchesToMeters((Math.PI * kTrackWidth) / 2.0); // meters
		public static final double kAutonAbort = 10.0; // sec
	}

	public static final class CraneConstants {
		public static final int kTurretSlot = 0;
		public static final double kTurretP = 5e-5;
		public static final double kTurretI = 0.0; // 1e-6;
		public static final double kTurretD = 0.0;
		public static final double kTurretInitPos = 0.0; // degrees
		public static final double kTurretStowPos = 0.0; // degrees
		public static final double kTurretHoldPos = 0.0; // degrees
		public static final double kTurretReceivePos = 0.0; // degrees
		public static final double kTurretGripPos = 0.0; // degrees
		public static final double kTurretReadyPos = 180.0; // degrees
		public static final double kTurretNodePos = 180.0; // degrees
		public static final double kTurretSafe2TiltArm = -40.0; // degrees
		public static final double kTurretPositionTolerance = 1.0; // degrees

		public static final double kTurretIz = 0.0;
		public static final double kTurretFF = 0.000156;
		public static final double kTurretAllowErr = 0.5;
		public static final double kTurretMinOutput = -1.0;
		public static final double kTurretMaxOutput = 1.0;
		public static final double kTurretMinVel = 0.0;

		public static final int kTiltSlot = 0;
		public static final double kTiltP = 5e-5;
		public static final double kTiltI = 0.0; // 1e-6;
		public static final double kTiltD = 0.0;
		public static final double kTiltInitPos = -87; // degrees
		public static final double kTiltClearChassisPos = -45.0; // degrees
		public static final double kTiltStowPos = -87; // degrees
		public static final double kTiltGripPos = -82; // degrees
		public static final double kTiltReceivePos = -82; // degrees
		public static final double kTiltHoldPos = -82; // degrees
		public static final double kTiltReadyPos = 0.0; // degrees
		public static final double kTiltSafe2Rotate = -60; // degrees
		public static final double kTiltPositionTolerance = 1.0; // degrees

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
		public static final double kArmHoldPos = 25.065; // inches
		public static final double kArmReadyPos = 25.065; // inches
		public static final double kArmReceivePos = 33.0; // inches
		public static final double kArmSafe2Rotate = 25.0; // inches
		public static final double kArmGripCone = 35.5; // inches
		public static final double kArmGripCube = 33.0; // inches
		public static final double kArmPositionTolerance = 0.5; // inches

		public static final double kArmIz = 0.0;
		public static final double kArmFF = 0.000156;
		public static final double kArmAllowErr = 0.125;
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

		public static final double kTiltSprocketTeeth = 120;
		public static final double kTiltMotorSprocketTeeth = 18;
		public static final double kTiltSprocketRatio = kTiltSprocketTeeth / kTiltMotorSprocketTeeth;
		public static final double kTiltGearBoxRatio = 80;
		public static final double kTiltRotationsPerDegree = (kTiltSprocketRatio * kTiltGearBoxRatio) / 360.0;
		public static final double kTiltDegreesPerRotation = 360.0 / (kTiltSprocketRatio * kTiltGearBoxRatio);
		public static final double kTiltMaxVel = 540.0 / kTiltDegreesPerRotation * 60.0; // RPM
		public static final double kTiltMaxAccel = kTiltMaxVel * (2.0 / 3.0); // RPM^2

		public static final double kArmShaftDia = 1.0;
		public static final double kArmShaftCirc = kArmShaftDia * Math.PI;
		public static final double kArmGearBoxRatio = 12;
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

		public static final double kTurretInc = 15.0 * 0.020; // degrees / step
		public static final double kTurretMax = 225.0; // degrees
		public static final double kTurretMin = 0.0; // degrees
		public static final double kTiltInc = 7.5 * 0.020; // degrees / step
		public static final double kTiltMax = 30.0; // degrees
		public static final double kTiltMin = -87.0; // degrees
		public static final double kArmInc = -1.5 * 0.020; // inches / step
		public static final double kArmMax = 63.0; // inches
		public static final double kArmMin = 25.0; // inches

		public static final double kAutonTurretPos = -21.0;
		public static final double kAutonTiltPos = 18.0;
		public static final double kAutonArmPos = 55.0;
	}

	public static final class IntakeConstants {
		public static final double colorConf = 0.95;

		public static final double coneR = 0.34509;
		public static final double coneG = 0.51764;
		public static final double coneB = 0.13333;

		public static final double cubeR = 0.22745;
		public static final double cubeG = 0.39607;
		public static final double cubeB = 0.37254;
	}

	public static final class VisionConstants {
		public static final String kCameraName = "OV5647";

		// Constants such as camera and target height stored. Change per robot and goal!
		public static final double kCameraHeight = Units.inchesToMeters(19.0);
		public static final double kTargetHeight = Units.inchesToMeters(18.75);

		// Angle between horizontal and the camera.
		public static final double kCameraPitch = Units.degreesToRadians(0);

		// How far from the target we want to be
		public static final double kTargetDist = Units.feetToMeters(2);

		public static final double kRange2Rumble = Units.feetToMeters(20.0);

		// PID constants should be tuned per robot
		public static final double kDistP = 0.6;
		public static final double kDistI = 0.0;
		public static final double kDistD = 0.0;

		public static final double kTurnP = 0.03;
		public static final double kTurnI = 0.0;
		public static final double kTurnD = 0.0;
	}
}
