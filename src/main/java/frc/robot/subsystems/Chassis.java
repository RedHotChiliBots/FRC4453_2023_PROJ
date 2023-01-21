// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.AnalogIOConstants;
import frc.robot.Constants.CANidConstants;
import frc.robot.Constants.ChassisConstants;

public class Chassis extends SubsystemBase {

	// ==============================================================
	// Define the left side motors, master and follower
	private final CANSparkMax leftMaster = new CANSparkMax(
			CANidConstants.kLeftMasterMotor,
			MotorType.kBrushless);
	private final CANSparkMax leftFollower = new CANSparkMax(
			CANidConstants.kLeftFollowerMotor,
			MotorType.kBrushless);

	// Define the right side motors, master and follower
	private final CANSparkMax rightMaster = new CANSparkMax(
			CANidConstants.kRightMasterMotor,
			MotorType.kBrushless);
	private final CANSparkMax rightFollower = new CANSparkMax(
			CANidConstants.kRightFollowerMotor,
			MotorType.kBrushless);

	private final DifferentialDrive diffDrive = new DifferentialDrive(leftMaster, rightMaster);

	// ==============================================================
	// Define encoders and PID controllers
	private final RelativeEncoder leftEncoder = leftMaster.getEncoder();;
	private final RelativeEncoder rightEncoder = rightMaster.getEncoder();

	private final SparkMaxPIDController leftPIDController = leftMaster.getPIDController();
	private final SparkMaxPIDController rightPIDController = rightMaster.getPIDController();

	private final PIDController levelPIDController = new PIDController(ChassisConstants.kLevelP,
			ChassisConstants.kLevelI, ChassisConstants.kLevelD);

	private final PIDController ratePIDController = new PIDController(ChassisConstants.kRateP,
			ChassisConstants.kRateI, ChassisConstants.kRateD);

	// ==============================================================
	// Define autonomous support functions
	public final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(
			ChassisConstants.kTrackWidth);

	private DifferentialDriveOdometry odometry;

	// Create a voltage constraint to ensure we don't accelerate too fast
	// private DifferentialDriveVoltageConstraint autoVoltageConstraint;

	// Create config for trajectory
	private TrajectoryConfig config;
	private TrajectoryConfig configReversed;

	// An example trajectory to follow. All units in meters.
	public Trajectory fwdStraight;
	public Trajectory revStraight;

	// ==============================================================
	// Initialize NavX AHRS board
	// Alternatively: I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB
	private final AHRS ahrs = new AHRS(SPI.Port.kMXP);

	// ==============================================================
	// Identify PDP and PCM
	private final PowerDistribution pdp = new PowerDistribution();
	private final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

	// ==============================================================
	// Identify compressor hi and lo sensors
	private final AnalogInput hiPressureSensor = new AnalogInput(AnalogIOConstants.kHiPressureChannel);
	private final AnalogInput loPressureSensor = new AnalogInput(AnalogIOConstants.kLoPressureChannel);

	// ==============================================================
	// Define local variables
	// private double currPitch = 0.0;
	// private double lastPitch = 0.0;
	// private double maxPitch = Integer.MIN_VALUE;
	// private double minPitch = Integer.MAX_VALUE;
	// private boolean isPitchIncreasing = false;
	private double setPoint = 0.0;
	private double leftError = 0.0;
	private double rightError = 0.0;

	// ==============================================================
	// Define Shuffleboard data
	private final ShuffleboardTab pidTab = Shuffleboard.getTab("Chassis");
	private final GenericEntry sbLevelPID = pidTab.addPersistent("Level PID", 0).getEntry();

	private final ShuffleboardTab chassisTab = Shuffleboard.getTab("Chassis");
	private final GenericEntry sbLeftPos = chassisTab.addPersistent("ML Position", 0).getEntry();
	private final GenericEntry sbLeftVel = chassisTab.addPersistent("ML Velocity", 0).getEntry();
	private final GenericEntry sbRightPos = chassisTab.addPersistent("MR Position", 0).getEntry();
	private final GenericEntry sbRightVel = chassisTab.addPersistent("MR Velocity", 0).getEntry();
	private final GenericEntry sbLeftPow = chassisTab.addPersistent("ML Power", 0).getEntry();
	private final GenericEntry sbRightPow = chassisTab.addPersistent("MR Power", 0).getEntry();
	private final GenericEntry sbPitch = chassisTab.addPersistent("Pitch", 0).getEntry();
	private final GenericEntry sbAngle = chassisTab.addPersistent("Angle", 0).getEntry();
	private final GenericEntry sbHeading = chassisTab.addPersistent("Heading", 0).getEntry();

	private final GenericEntry sbX = chassisTab.addPersistent("Pose X", 0).getEntry();
	private final GenericEntry sbY = chassisTab.addPersistent("Pose Y", 0).getEntry();
	private final GenericEntry sbDeg = chassisTab.addPersistent("Pose Deg", 0).getEntry();

	private final GenericEntry sbSetPt = chassisTab.addPersistent("Setpoint", 0.0).getEntry();
	private final GenericEntry sbLeftErr = chassisTab.addPersistent("Left Error", 0.0).getEntry();
	private final GenericEntry sbRightErr = chassisTab.addPersistent("Right Error", 0.0).getEntry();
	private final GenericEntry sbAtTgt = chassisTab.addPersistent("At Target", false).getEntry();

	private final ShuffleboardTab pneumaticsTab = Shuffleboard.getTab("Pneumatics");
	private final GenericEntry sbHiPressure = pneumaticsTab.addPersistent("Hi Pressure", 0).getEntry();
	private final GenericEntry sbLoPressure = pneumaticsTab.addPersistent("Lo Pressure", 0).getEntry();

	public Chassis() {
		System.out.println("+++++ Chassis Constructor starting +++++");

		// ==============================================================
		// Configure PDP
		pdp.clearStickyFaults();

		// ==============================================================
		// Configure the left side motors, master and follower
		leftMaster.restoreFactoryDefaults();
		leftFollower.restoreFactoryDefaults();

		leftMaster.clearFaults();
		leftFollower.clearFaults();

		leftMaster.setIdleMode(IdleMode.kBrake);
		leftFollower.setIdleMode(IdleMode.kBrake);

		// Configure the right side motors, master and follower
		rightMaster.restoreFactoryDefaults();
		rightFollower.restoreFactoryDefaults();

		rightMaster.clearFaults();
		rightFollower.clearFaults();

		rightMaster.setIdleMode(IdleMode.kBrake);
		rightFollower.setIdleMode(IdleMode.kBrake);

		rightMaster.setInverted(true);
		rightFollower.setInverted(true);

		// Group the left and right motors
		leftFollower.follow(leftMaster);
		rightFollower.follow(rightMaster);

		// ==============================================================
		// Configure PID controllers
		leftPIDController.setP(ChassisConstants.kP);
		leftPIDController.setI(ChassisConstants.kI);
		leftPIDController.setD(ChassisConstants.kD);
		// leftPIDController.setIZone(ChassisConstants.kIz);
		// leftPIDController.setFF(ChassisConstants.kFF);
		// leftPIDController.setOutputRange(ChassisConstants.kMinOutput,
		// ChassisConstants.kMaxOutput);

		rightPIDController.setP(ChassisConstants.kP);
		rightPIDController.setI(ChassisConstants.kI);
		rightPIDController.setD(ChassisConstants.kD);
		// rightPIDController.setIZone(ChassisConstants.kIz);
		// rightPIDController.setFF(ChassisConstants.kFF);
		// rightPIDController.setOutputRange(ChassisConstants.kMinOutput,
		// ChassisConstants.kMaxOutput);

		levelPIDController.setSetpoint(ChassisConstants.kLevelSetPoint);
		levelPIDController.setTolerance(ChassisConstants.kLevelSetTolerance);

		ratePIDController.setSetpoint(ChassisConstants.kRateSetPoint);
		ratePIDController.setTolerance(ChassisConstants.kRateSetTolerance);

		// ==============================================================
		// Configure encoders
		leftEncoder.setPositionConversionFactor(ChassisConstants.kPosFactorMPC);
		rightEncoder.setPositionConversionFactor(ChassisConstants.kPosFactorMPC);

		leftEncoder.setVelocityConversionFactor(ChassisConstants.kVelFactor);
		rightEncoder.setVelocityConversionFactor(ChassisConstants.kVelFactor);

		// ==============================================================
		// Define autonomous support functions
		odometry = new DifferentialDriveOdometry(getAngle(), leftEncoder.getPosition(), rightEncoder.getPosition());

		// Create a voltage constraint to ensure we don't accelerate too fast
		// Create a voltage constraint to ensure we don't accelerate too fast
		var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
				new SimpleMotorFeedforward(
						ChassisConstants.kS,
						ChassisConstants.kV,
						ChassisConstants.kA),
				kinematics,
				10);

		// Create config for trajectory
		config = new TrajectoryConfig(ChassisConstants.kMaxSpeedMetersPerSecond,
				ChassisConstants.kMaxAccelerationMetersPerSecondSquared)
				// Add kinematics to ensure max speed is actually obeyed
				.setKinematics(kinematics)
				// Apply the voltage constraint
				.addConstraint(autoVoltageConstraint)
				.setReversed(false);

		configReversed = new TrajectoryConfig(ChassisConstants.kMaxSpeedMetersPerSecond,
				ChassisConstants.kMaxAccelerationMetersPerSecondSquared)
				// Add kinematics to ensure max speed is actually obeyed
				.setKinematics(kinematics)
				// Apply the voltage constraint
				.addConstraint(autoVoltageConstraint)
				.setReversed(true);

		fwdStraight = TrajectoryGenerator.generateTrajectory(
				// Start at the origin facing the +X direction
				new Pose2d(Units.inchesToMeters(0.0), Units.inchesToMeters(0.0), new Rotation2d(180)),
				List.of(),
				new Pose2d(Units.inchesToMeters(36.0), Units.inchesToMeters(0.0), new Rotation2d(0)),
				// Pass config
				config);

		revStraight = TrajectoryGenerator.generateTrajectory(
				// Start at the origin facing the +X direction
				new Pose2d(Units.inchesToMeters(36.0), Units.inchesToMeters(0.0), new Rotation2d(180)),
				List.of(),
				new Pose2d(Units.inchesToMeters(0.0), Units.inchesToMeters(0.0), new Rotation2d(0)),
				// Pass config
				configReversed);

		// ==============================================================
		// Add static variables to Shuffleboard
		chassisTab.addPersistent("ML Pos Factor", leftEncoder.getPositionConversionFactor());
		chassisTab.addPersistent("MR Pos Factor", rightEncoder.getPositionConversionFactor());
		chassisTab.addPersistent("ML Vel Factor", leftEncoder.getVelocityConversionFactor());
		chassisTab.addPersistent("MR Vel Factor", rightEncoder.getVelocityConversionFactor());

		pidTab.addPersistent("Level PID", levelPIDController);

		// ==============================================================
		// Initialize devices before starting
		resetFieldPosition(0.0, 0.0); // Reset the field and encoder positions to zero

		// Update field position - for autonomous
		// resetOdometry(RobotContainer.BlueRungSideCargoToHub.getInitialPose());

		System.out.println("----- Chassis Constructor finished -----");
	}

	@Override
	public void periodic() {
		sbLeftPos.setDouble(leftEncoder.getPosition());
		sbLeftVel.setDouble(leftEncoder.getVelocity());
		sbRightPos.setDouble(rightEncoder.getPosition());
		sbRightVel.setDouble(rightEncoder.getVelocity());
		sbLeftPow.setDouble(leftMaster.getAppliedOutput());
		sbRightPow.setDouble(rightMaster.getAppliedOutput());

		sbPitch.setDouble(getPitch());
		sbAngle.setDouble(getAngle().getDegrees());
		sbHeading.setDouble(getHeading());

		sbHiPressure.setDouble(getHiPressure());
		sbLoPressure.setDouble(getLoPressure());

		sbSetPt.setDouble(setPoint);
		sbLeftErr.setDouble(leftError);
		sbRightErr.setDouble(rightError);
		sbAtTgt.setBoolean(atTarget());

		// // Update field position - for autonomous
		// resetOdometry(BlueSideRung.getInitialPose());

		updateOdometry();

		// // Get the desired pose from the trajectory.
		// var desiredPose = trajectory.sample(timer.get());

		// // Get the reference chassis speeds from the Ramsete controller.
		// var refChassisSpeeds = m_ramseteController.calculate(drive.getPose(),
		// desiredPose);

		// // Set the linear and angular speeds.
		// drive.drive(refChassisSpeeds.vxMetersPerSecond,
		// refChassisSpeeds.omegaRadiansPerSecond);

		Pose2d pose = odometry.getPoseMeters();
		Translation2d trans = pose.getTranslation();
		double x = trans.getX();
		double y = trans.getY();
		Rotation2d rot = pose.getRotation();
		double deg = rot.getDegrees();

		sbX.setDouble(x);
		sbY.setDouble(y);
		sbDeg.setDouble(deg);

//		setIsPitchIncreasing();
	}

	public void levelChargingStation() {
		double pitch = ahrs.getPitch();
		double pidOut = levelPIDController.calculate(pitch);
		drive(pidOut, 0.0);
	}

	private double lastPitch;
	private double currPitch;
	private double ratePitch;

	public void rateChargingStation() {
		lastPitch = currPitch;
		ratePitch = (lastPitch - currPitch) / 0.020; // deg / sec
		currPitch = ahrs.getPitch();
		double pidOut = levelPIDController.calculate(ratePitch);
		drive(pidOut, 0.0);
	}
	
	/**
	 * Returns the current robot pitch reported by navX sensor.
	 * 
	 * @see com.kauailabs.navx.frc.AHRS.getPitch()
	 * @return The current pitch value in degrees (-180 to 180).
	 */
	public double getPitch() {
		// adjust for orientation of roborio - use roll
		// adjust for pitch on floor
		return ahrs.getRoll() + 2.3;
	}
	public double getPitchRate() {
		return ratePitch;
	}

	public DifferentialDriveOdometry getOdometry() {
		return odometry;
	}

	public Pose2d getPose() {
		return odometry.getPoseMeters();
	}

	public DifferentialDriveKinematics getKinematics() {
		return kinematics;
	}

	public DifferentialDriveWheelSpeeds getWheelSpeeds() {
		return new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(), rightEncoder.getVelocity());
	}

	public double getDuration(Trajectory t) {
		return t.getTotalTimeSeconds();
	}

	public void resetFieldPosition(double x, double y) {
		ahrs.zeroYaw();
		resetEncoders();
		// odometry.resetPosition(new Pose2d(x, y, getAngle()), getAngle());
	}

	// public void initMonitorPitch() {
	// 	maxPitch = 0.0;
	// 	minPitch = 0.0;
	// }

	// public void setIsPitchIncreasing() {
	// 	lastPitch = currPitch;
	// 	currPitch = getPitch();
	// 	if (currPitch > maxPitch)
	// 		maxPitch = currPitch;
	// 	if (currPitch < minPitch)
	// 		minPitch = currPitch;
	// 	isPitchIncreasing = (currPitch > lastPitch) ? true : false;
	// }

	// public double getMinPitch() {
	// 	return minPitch;
	// }

	// public double getMaxPitch() {
	// 	return maxPitch;
	// }

	// public boolean getIsPitchIncreasing() {
	// 	return isPitchIncreasing;
	// }

	public void driveTankVolts(double leftVolts, double rightVolts) {
		leftMaster.setVoltage(leftVolts);
		rightMaster.setVoltage(rightVolts);
		diffDrive.feed();
	}

	public void driveTank(double left, double right) {
		diffDrive.tankDrive(-left, -right);
	}

	public void driveArcade(double spd, double rot) {
		diffDrive.arcadeDrive(-spd, rot);
	}

	/**
	 * Returns the "fused" (9-axis) heading.
	 * 
	 * @see com.kauailabs.navx.frc.AHRS.getFusedHeading()
	 * @return Fused Heading in Degrees (range 0-360)
	 * 
	 */
	public double getHeading() {
		return ahrs.getFusedHeading();
	}

	public Rotation2d getAngle() {
		// Negating the angle because WPILib gyros are CW positive.
		return ahrs.getRotation2d();
	}

	/**
	 * Drives the robot with the given linear velocity and angular velocity.
	 *
	 * @param xSpeed Linear velocity in m/s.
	 * @param rot    Angular velocity in rad/s.
	 */
	// @SuppressWarnings("ParameterName")
	public void drive(double xSpeed, double xRot) {
		DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, -xRot));
		leftMaster.set(wheelSpeeds.leftMetersPerSecond);
		rightMaster.set(wheelSpeeds.rightMetersPerSecond);
	}

	/**
	 * Updates the field-relative position.
	 */

	public void updateOdometry() {
		odometry.update(getAngle(), leftEncoder.getPosition(), rightEncoder.getPosition());
	}

	public void resetOdometry(Pose2d pose) {
		resetEncoders();
		// odometry.resetPosition(new Pose2d(x, y, getAngle()), getAngle());
	}

	public void resetEncoders() {
		leftEncoder.setPosition(0.0);
		rightEncoder.setPosition(0.0);
	}

	public void driveTrajectory(double left, double right) {
		leftMaster.set(left);
		rightMaster.set(right);
	}

	public void drivePosition(double setPoint) {
		this.setPoint = setPoint;
		leftPIDController.setReference(setPoint, ControlType.kPosition);
		rightPIDController.setReference(setPoint, ControlType.kPosition);
	}

	public boolean atTarget() {
		leftError = Math.abs(setPoint - leftEncoder.getPosition());
		rightError = Math.abs(setPoint - rightEncoder.getPosition());
		return leftError <= ChassisConstants.kDistanceTolerance && rightError <= ChassisConstants.kDistanceTolerance;

	}

	// public void turn(double angle) {
	// driveWithHeading(0, angle);
	// }

	// public PIDController getRotPID() {
	// return m_rotPIDController;
	// }

	// public void driveWithHeading(double speed, double angle) {
	// getRotPID();
	// setSetpoint(angle);
	// PIDSpeed = speed;
	// }

	// public boolean angleOnTarget() {
	// return getPIDController().onTarget();
	// }

	public double getLoPressure() {
		return 250.0 * (loPressureSensor.getVoltage() / AnalogIOConstants.kInputVoltage) - 25.0;
	}

	public double getHiPressure() {
		return 250.0 * (hiPressureSensor.getVoltage() / AnalogIOConstants.kInputVoltage) - 25.0;
	}
}