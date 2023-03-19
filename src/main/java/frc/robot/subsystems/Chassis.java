// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import java.text.SimpleDateFormat;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AnalogInConstants;
import frc.robot.Constants.CANidConstants;
import frc.robot.Constants.ChassisConstants;
import frc.robot.Constants.Pneumatic0ChannelConstants;
import frc.robot.Constants.PneumaticModuleConstants;
import frc.robot.Library;

public class Chassis extends SubsystemBase {

	// ==============================================================
	// Define the left side motors, master and follower
	private final CANSparkMax leftMaster = new CANSparkMax(
			CANidConstants.kLeftMasterMotor,
			MotorType.kBrushless);
	private final CANSparkMax leftFollower1 = new CANSparkMax(
			CANidConstants.kLeftFollower1Motor,
			MotorType.kBrushless);
	private final CANSparkMax leftFollower2 = new CANSparkMax(
			CANidConstants.kLeftFollower2Motor,
			MotorType.kBrushless);

	// Define the right side motors, master and follower
	private final CANSparkMax rightMaster = new CANSparkMax(
			CANidConstants.kRightMasterMotor,
			MotorType.kBrushless);
	private final CANSparkMax rightFollower1 = new CANSparkMax(
			CANidConstants.kRightFollower1Motor,
			MotorType.kBrushless);
	private final CANSparkMax rightFollower2 = new CANSparkMax(
			CANidConstants.kRightFollower2Motor,
			MotorType.kBrushless);

	private final DifferentialDrive diffDrive = new DifferentialDrive(leftMaster, rightMaster);

	// ==============================================================
	// Define encoders and PID controllers
	private final SparkMaxPIDController leftPIDController = leftMaster.getPIDController();
	private final SparkMaxPIDController rightPIDController = rightMaster.getPIDController();

	public final RelativeEncoder leftEncoder = leftMaster.getEncoder();;
	public final RelativeEncoder rightEncoder = rightMaster.getEncoder();

	private final PIDController levelPIDController = new PIDController(ChassisConstants.kLevelP,
			ChassisConstants.kLevelI, ChassisConstants.kLevelD);

	private final PIDController ratePIDController = new PIDController(ChassisConstants.kRateP,
			ChassisConstants.kRateI, ChassisConstants.kRateD);

	private final PIDController distPIDController = new PIDController(ChassisConstants.kDistP,
			ChassisConstants.kDistI, ChassisConstants.kDistD);

	// ==============================================================
	// Define autonomous support functions
	private DifferentialDriveOdometry odometry;
	public final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(ChassisConstants.kTrackWidth);

	// ==============================================================
	// Initialize NavX AHRS board
	// Alternatively: I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB
	private final AHRS ahrs = new AHRS(SPI.Port.kMXP);

	// ==============================================================
	// Identify PDP and PCM
	private final PowerDistribution pdp = new PowerDistribution();
	private final PneumaticsControlModule pcm0 = new PneumaticsControlModule();
	private final PneumaticsControlModule pcm1 = new PneumaticsControlModule();
	private final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

	// ==============================================================
	// Identify compressor hi and lo sensors
	private final AnalogInput hiPressureSensor = new AnalogInput(AnalogInConstants.kHiPressureChannel);
	private final AnalogInput loPressureSensor = new AnalogInput(AnalogInConstants.kLoPressureChannel);

	// ==============================================================
	// Identify pneumatics for gear shifters
	private final DoubleSolenoid gearShifter = new DoubleSolenoid(
			PneumaticModuleConstants.kPCM0,
			PneumaticsModuleType.CTREPCM,
			Pneumatic0ChannelConstants.kChassisShifterHi,
			Pneumatic0ChannelConstants.kChassisShifterLo);

	// ==============================================================
	// Define field for pose visualization on Shuffleboard
	private Field2d field = new Field2d();

	// ==============================================================
	// Define local variables
	public enum DirState {
		FORWARD,
		REVERSE
	}

	public enum GearShifterState {
		NA,
		HI,
		LO
	}

	public enum DriveState {
		TANK,
		ARCADE,
		LARCADE
	}

	private double setPoint = 0.0;
	private double leftError = 0.0;
	private double rightError = 0.0;
	private DirState dirState = DirState.FORWARD;
	private DriveState driveState = DriveState.TANK;

	private GearShifterState shifterState;
	private double gearBoxRatio;
	private double posFactor; // Meters / Rev
	private double velFactor; // Meters / Sec

	public final Library lib = new Library();

	public SimpleDateFormat timeStamp = new SimpleDateFormat("yyyy.MM.dd.HH.mm.ss.SSS");

	// ==============================================================
	// Define Shuffleboard data - Chassis Tab
	private final ShuffleboardTab chassisTab = Shuffleboard.getTab("Chassis");

	private final GenericEntry sbSetPt = chassisTab.addPersistent("Setpoint", 0.0)
			.withWidget("Text View").withPosition(0, 0).withSize(2, 1).getEntry();
	private final GenericEntry sbLeftErr = chassisTab.addPersistent("Left Error", 0.0)
			.withWidget("Text View").withPosition(0, 1).withSize(2, 1).getEntry();
	private final GenericEntry sbLeftPos = chassisTab.addPersistent("ML Position", 0)
			.withWidget("Text View").withPosition(0, 2).withSize(2, 1).getEntry();
	private final GenericEntry sbLeftVel = chassisTab.addPersistent("ML Velocity", 0)
			.withWidget("Text View").withPosition(0, 3).withSize(2, 1).getEntry();
	private final GenericEntry sbLeftPow = chassisTab.addPersistent("ML Power", 0)
			.withWidget("Text View").withPosition(0, 4).withSize(2, 1).getEntry();

	private final GenericEntry sbAtTgt = chassisTab.addPersistent("At Target", false)
			.withWidget("Boolean Box").withPosition(2, 0).withSize(2, 1).getEntry();
	private final GenericEntry sbRightErr = chassisTab.addPersistent("Right Error", 0.0)
			.withWidget("Text View").withPosition(2, 1).withSize(2, 1).getEntry();
	private final GenericEntry sbRightPos = chassisTab.addPersistent("MR Position", 0)
			.withWidget("Text View").withPosition(2, 2).withSize(2, 1).getEntry();
	private final GenericEntry sbRightVel = chassisTab.addPersistent("MR Velocity", 0)
			.withWidget("Text View").withPosition(2, 3).withSize(2, 1).getEntry();
	private final GenericEntry sbRightPow = chassisTab.addPersistent("MR Power", 0)
			.withWidget("Text View").withPosition(2, 4).withSize(2, 1).getEntry();

	private final GenericEntry sbAngle = chassisTab.addPersistent("Angle", 0)
			.withWidget("Text View").withPosition(5, 0).withSize(2, 1).getEntry();
	private final GenericEntry sbAvgPitch = chassisTab.addPersistent("Avg Pitch", 0)
			.withWidget("Text View").withPosition(5, 1).withSize(2, 1).getEntry();
	private final GenericEntry sbX = chassisTab.addPersistent("Pose X", 0)
			.withWidget("Text View").withPosition(5, 2).withSize(2, 1).getEntry();
	private final GenericEntry sbHiPressure = chassisTab.addPersistent("Hi Pressure", 0)
			.withWidget("Text View").withPosition(5, 5).withSize(2, 1).getEntry();

	private final GenericEntry sbHeading = chassisTab.addPersistent("Heading", 0)
			.withWidget("Text View").withPosition(7, 0).withSize(2, 1).getEntry();
	private final GenericEntry sbAvgRate = chassisTab.addPersistent("Avg Rate", 0)
			.withWidget("Text View").withPosition(7, 1).withSize(2, 1).getEntry();
	private final GenericEntry sbY = chassisTab.addPersistent("Pose Y", 0)
			.withWidget("Text View").withPosition(7, 2).withSize(2, 1).getEntry();
	private final GenericEntry sbDeg = chassisTab.addPersistent("Pose Deg", 0)
			.withWidget("Text View").withPosition(6, 3).withSize(2, 1).getEntry();
	private final GenericEntry sbLoPressure = chassisTab.addPersistent("Lo Pressure", 0)
			.withWidget("Text View").withPosition(7, 5).withSize(2, 1).getEntry();

	private final GenericEntry sbLeftFrontTemp = chassisTab.addPersistent("LF temp", 0)
			.withWidget("Text View").withPosition(10, 0).withSize(2, 1).getEntry();
	private final GenericEntry sbRightFrontTemp = chassisTab.addPersistent("RF temp", 0)
			.withWidget("Text View").withPosition(10, 1).withSize(2, 1).getEntry();
	private final GenericEntry sbLeftBackTemp = chassisTab.addPersistent("LB temp", 0)
			.withWidget("Text View").withPosition(10, 2).withSize(2, 1).getEntry();
	private final GenericEntry sbRightBackTemp = chassisTab.addPersistent("RB temp", 0)
			.withWidget("Text View").withPosition(10, 3).withSize(2, 1).getEntry();
	private final GenericEntry sbLeftCenterTemp = chassisTab.addPersistent("LC temp", 0)
			.withWidget("Text View").withPosition(10, 4).withSize(2, 1).getEntry();
	private final GenericEntry sbRightCenterTemp = chassisTab.addPersistent("RC temp", 0)
			.withWidget("Text View").withPosition(10, 5).withSize(2, 1).getEntry();

	private final GenericEntry sbLeftFrontAmp = chassisTab.addPersistent("LF amp", 0)
			.withWidget("Text View").withPosition(12, 0).withSize(2, 1).getEntry();
	private final GenericEntry sbRightFrontAmp = chassisTab.addPersistent("RF amp", 0)
			.withWidget("Text View").withPosition(12, 1).withSize(2, 1).getEntry();
	private final GenericEntry sbLeftBackAmp = chassisTab.addPersistent("LB amp", 0)
			.withWidget("Text View").withPosition(12, 2).withSize(2, 1).getEntry();
	private final GenericEntry sbRightBackAmp = chassisTab.addPersistent("RB amp", 0)
			.withWidget("Text View").withPosition(12, 3).withSize(2, 1).getEntry();
	private final GenericEntry sbLeftCenterAmp = chassisTab.addPersistent("LC amp", 0)
			.withWidget("Text View").withPosition(12, 4).withSize(2, 1).getEntry();
	private final GenericEntry sbRightCenterAmp = chassisTab.addPersistent("RC amp", 0)
			.withWidget("Text View").withPosition(12, 5).withSize(2, 1).getEntry();

	private final GenericEntry sbMLPosFactor = chassisTab.addPersistent("ML Pos Factor", 0)
			.withWidget("Text View").withPosition(0, 6).withSize(2, 1).getEntry();
	private final GenericEntry sbMRPosFactor = chassisTab.addPersistent("MR Pos Factor", 0)
			.withWidget("Text View").withPosition(2, 6).withSize(2, 1).getEntry();
	private final GenericEntry sbMLVelFactor = chassisTab.addPersistent("ML Vel Factor", 0)
			.withWidget("Text View").withPosition(0, 7).withSize(2, 1).getEntry();
	private final GenericEntry sbMRVelFactor = chassisTab.addPersistent("MR Vel Factor", 0)
			.withWidget("Text View").withPosition(2, 7).withSize(2, 1).getEntry();

	// ==============================================================
	// Define Shuffleboard data - Competition Tab
	private final ShuffleboardTab compTab = Shuffleboard.getTab("Competition");

	private final GenericEntry sbDriveType = compTab.addPersistent("Drive Type", "")
			.withWidget("Text View").withPosition(9, 0).withSize(2, 1).getEntry();
	private final GenericEntry sbDir = compTab.addPersistent("Direction", "")
			.withWidget("Text View").withPosition(9, 1).withSize(2, 1).getEntry();
	private final GenericEntry sbShifter = compTab.addPersistent("Gear Shift", "")
			.withWidget("Text View").withPosition(9, 2).withSize(2, 1).getEntry();

	public Chassis() {
		System.out.println("+++++ Chassis Constructor starting +++++");

		// ==============================================================
		// Configure PDP and PCM
		pdp.clearStickyFaults();
		pcm0.clearAllStickyFaults();
		pcm1.clearAllStickyFaults();

		// ==============================================================
		// Configure motor current limits
		leftMaster.setSmartCurrentLimit(45, 30);
		leftFollower1.setSmartCurrentLimit(45, 30);
		leftFollower2.setSmartCurrentLimit(45, 30);
		rightMaster.setSmartCurrentLimit(45, 30);
		rightFollower1.setSmartCurrentLimit(45, 30);
		rightFollower2.setSmartCurrentLimit(45, 30);

		// ==============================================================
		// Restore motor factory defaults
		leftMaster.restoreFactoryDefaults();
		leftFollower1.restoreFactoryDefaults();
		leftFollower2.restoreFactoryDefaults();
		rightMaster.restoreFactoryDefaults();
		rightFollower1.restoreFactoryDefaults();
		rightFollower2.restoreFactoryDefaults();

		// ==============================================================
		// Clear motor faults
		leftMaster.clearFaults();
		leftFollower1.clearFaults();
		leftFollower2.clearFaults();
		rightMaster.clearFaults();
		rightFollower1.clearFaults();
		rightFollower2.clearFaults();

		// ==============================================================
		// Configure motor idle mode
		leftMaster.setIdleMode(IdleMode.kBrake);
		leftFollower1.setIdleMode(IdleMode.kBrake);
		leftFollower2.setIdleMode(IdleMode.kBrake);
		rightMaster.setIdleMode(IdleMode.kBrake);
		rightFollower1.setIdleMode(IdleMode.kBrake);
		rightFollower2.setIdleMode(IdleMode.kBrake);

		// ==============================================================
		// Configure motor inversion
		leftMaster.setInverted(true);
		leftFollower1.setInverted(true);
		leftFollower2.setInverted(true);

		// ==============================================================
		// Group the left and right motor followers
		leftFollower1.follow(leftMaster);
		leftFollower2.follow(leftMaster);
		rightFollower1.follow(rightMaster);
		rightFollower2.follow(rightMaster);

		// ==============================================================
		// Configure PID controllers
		leftPIDController.setP(ChassisConstants.kP);
		leftPIDController.setI(ChassisConstants.kI);
		leftPIDController.setD(ChassisConstants.kD);
		leftPIDController.setIZone(ChassisConstants.kIz);
		leftPIDController.setFF(ChassisConstants.kFF);
		leftPIDController.setOutputRange(ChassisConstants.kMinOutput,
				ChassisConstants.kMaxOutput);

		leftPIDController.setSmartMotionMaxVelocity(ChassisConstants.kDriveMaxVel, ChassisConstants.kDriveSlot);
		leftPIDController.setSmartMotionMinOutputVelocity(ChassisConstants.kDriveMinVel, ChassisConstants.kDriveSlot);
		leftPIDController.setSmartMotionMaxAccel(ChassisConstants.kDriveMaxAccel, ChassisConstants.kDriveSlot);
		leftPIDController.setSmartMotionAllowedClosedLoopError(
				ChassisConstants.kDriveAllowErr, ChassisConstants.kDriveSlot);

		rightPIDController.setP(ChassisConstants.kP);
		rightPIDController.setI(ChassisConstants.kI);
		rightPIDController.setD(ChassisConstants.kD);
		rightPIDController.setIZone(ChassisConstants.kIz);
		rightPIDController.setFF(ChassisConstants.kFF);
		rightPIDController.setOutputRange(ChassisConstants.kMinOutput,
				ChassisConstants.kMaxOutput);

		rightPIDController.setSmartMotionMaxVelocity(ChassisConstants.kDriveMaxVel, ChassisConstants.kDriveSlot);
		rightPIDController.setSmartMotionMinOutputVelocity(ChassisConstants.kDriveMinVel, ChassisConstants.kDriveSlot);
		rightPIDController.setSmartMotionMaxAccel(ChassisConstants.kDriveMaxAccel, ChassisConstants.kDriveSlot);
		rightPIDController.setSmartMotionAllowedClosedLoopError(
				ChassisConstants.kDriveAllowErr, ChassisConstants.kDriveSlot);

		// ==============================================================
		// Configure level by pitch PID Controller
		levelPIDController.setSetpoint(ChassisConstants.kLevelSetPoint);
		levelPIDController.setTolerance(ChassisConstants.kLevelSetTolerance);

		// ==============================================================
		// Configure level by rate PID Controller
		ratePIDController.setSetpoint(ChassisConstants.kRateSetPoint);
		ratePIDController.setTolerance(ChassisConstants.kRateSetTolerance);

		// ==============================================================
		// Configure distance PID Controller
		distPIDController.setTolerance(ChassisConstants.kDistanceTolerance);

		// ==============================================================
		// Define autonomous Kinematics & Odometry functions
		resetFieldPosition(0.0, 0.0); // Reset the field and encoder positions to zero
		odometry = new DifferentialDriveOdometry(getAngle(), leftEncoder.getPosition(), rightEncoder.getPosition());

		// ==============================================================
		// Initialze Chassis
		stopChassis();

		setGearShifter(GearShifterState.HI);
		setDriveState(DriveState.TANK);
		setDirState(DirState.FORWARD);

		lib.initLibrary();

		// ==============================================================
		// Define field on Smartdashboard with inital Auton pose
		SmartDashboard.putData("Field", field);

		System.out.println("----- Chassis Constructor finished -----");
	}

	@Override
	public void periodic() {

		// ==============================================================
		// Post data to Shuffleboard
		sbLeftPos.setDouble(leftEncoder.getPosition());
		sbLeftVel.setDouble(leftEncoder.getVelocity());
		sbRightPos.setDouble(rightEncoder.getPosition());
		sbRightVel.setDouble(rightEncoder.getVelocity());
		sbLeftPow.setDouble(leftMaster.getAppliedOutput());
		sbRightPow.setDouble(rightMaster.getAppliedOutput());

		sbAvgRate.setDouble(lib.getAvgRate());
		sbAvgPitch.setDouble(lib.getAvgPitch());
		sbAngle.setDouble(getAngle().getDegrees());
		sbHeading.setDouble(getHeading());

		sbLeftFrontTemp.setDouble(leftMaster.getMotorTemperature());
		sbRightFrontTemp.setDouble(rightMaster.getMotorTemperature());
		sbLeftBackTemp.setDouble(leftFollower1.getMotorTemperature());
		sbRightBackTemp.setDouble(rightFollower1.getMotorTemperature());
		sbLeftCenterTemp.setDouble(leftFollower2.getMotorTemperature());
		sbRightCenterTemp.setDouble(rightFollower2.getMotorTemperature());

		sbLeftFrontAmp.setDouble(leftMaster.getOutputCurrent());
		sbRightFrontAmp.setDouble(rightMaster.getOutputCurrent());
		sbLeftBackAmp.setDouble(leftFollower1.getOutputCurrent());
		sbRightBackAmp.setDouble(rightFollower1.getOutputCurrent());
		sbLeftCenterAmp.setDouble(leftFollower2.getOutputCurrent());
		sbRightCenterAmp.setDouble(rightFollower2.getOutputCurrent());

		sbHiPressure.setDouble(getHiPressure());
		sbLoPressure.setDouble(getLoPressure());

		sbSetPt.setDouble(setPoint);
		sbLeftErr.setDouble(leftError);
		sbRightErr.setDouble(rightError);
		sbAtTgt.setBoolean(atDistTarget());

		sbDir.setString(dirState.toString());
		sbDriveType.setString(driveState.toString());
		sbShifter.setString(getGearShifter().toString());

		sbMLPosFactor.setDouble(leftEncoder.getPositionConversionFactor());
		sbMRPosFactor.setDouble(rightEncoder.getPositionConversionFactor());
		sbMLVelFactor.setDouble(leftEncoder.getVelocityConversionFactor());
		sbMRVelFactor.setDouble(rightEncoder.getVelocityConversionFactor());

		// ==============================================================
		// Update field odometry
		updateOdometry();
		field.setRobotPose(odometry.getPoseMeters());

		// ==============================================================
		// Compute current pose
		Pose2d pose = odometry.getPoseMeters();
		Translation2d trans = pose.getTranslation();
		double x = trans.getX();
		double y = trans.getY();
		Rotation2d rot = pose.getRotation();
		double deg = rot.getDegrees();

		// ==============================================================
		// Post current/new pose to Shuffleboard
		sbX.setDouble(x);
		sbY.setDouble(y);
		sbDeg.setDouble(deg);

		// ==============================================================
		// Update pitch and rate calculations
		lib.updatePitch(getPitch());
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
	 * Returns the current robot pitch reported by navX sensor.
	 * 
	 * @see com.kauailabs.navx.frc.AHRS.getPitch()
	 * @return The current pitch value in degrees (-180 to 180).
	 */
	public double getPitch() {
		// adjust for orientation of roborio - use roll
		// adjust for pitch on floor
		return ahrs.getRoll() + 3.05;
	}

	public void driveTankVolts(double leftVolts, double rightVolts) {
		leftMaster.setVoltage(leftVolts);
		rightMaster.setVoltage(rightVolts);
		diffDrive.feed();
	}

	/**
	 * Drives the robot with the given linear velocity and angular velocity.
	 *
	 * @param xSpeed Linear velocity in m/s.
	 * @param rot    Angular velocity in rad/s.
	 */
	// @SuppressWarnings("ParameterName")
	public void drive(double xSpeed, double xRot) {
		DifferentialDriveWheelSpeeds wheelSpeeds = kinematics
				.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, -xRot));
		leftMaster.set(wheelSpeeds.leftMetersPerSecond);
		rightMaster.set(wheelSpeeds.rightMetersPerSecond);
	}

	/**
	 * Updates the field-relative position.
	 */
	public void updateOdometry() {
		odometry.update(getAngle(), leftEncoder.getPosition(), rightEncoder.getPosition());
	}

	public void resetPose(Pose2d pose) {
		resetEncoders();
		odometry.resetPosition(getAngle(), leftEncoder.getPosition(), rightEncoder.getPosition(), pose);
	}

	public void resetEncoders() {
		leftEncoder.setPosition(0.0);
		rightEncoder.setPosition(0.0);
	}

	public void driveTrajectory(double left, double right) {
		leftMaster.set(left);
		rightMaster.set(right);
	}

	public void setDistSetPoint(double setPoint) {
		this.setPoint = setPoint;
		distPIDController.setSetpoint(setPoint);
	}

	public double driveDistance() {
		double currPosition = (leftEncoder.getPosition() + leftEncoder.getPosition()) / 2.0;
		double pidOut = distPIDController.calculate(currPosition, setPoint);
		driveArcade(pidOut, 0.0);
		return pidOut;
	}

	public double driveTurn() {
		double currPosition = (leftEncoder.getPosition() + leftEncoder.getPosition()) / 2.0;
		double pidOut = distPIDController.calculate(currPosition);
		driveArcade(pidOut, 1.0);
		return pidOut;
	}

	public void driveDistPosition(double setPoint) {

		this.setPoint = setPoint;
		leftPIDController.setReference(setPoint, CANSparkMax.ControlType.kSmartMotion);
		rightPIDController.setReference(setPoint, CANSparkMax.ControlType.kSmartMotion);

		// DriverStation.reportWarning("SetPoint: " + this.setPoint, false);
		// DriverStation.reportWarning("Position: " + leftEncoder.getPosition() + " : "
		// + rightEncoder.getPosition(), false);

		System.out.println("From subsystem");
	}

	public void driveTurnPosition(double setPoint) {
		this.setPoint = setPoint;
		// leftPIDController.setReference(setPoint,
		// CANSparkMax.ControlType.kSmartMotion);
		// rightPIDController.setReference(-setPoint,
		// CANSparkMax.ControlType.kSmartMotion);

		leftPIDController.setReference(setPoint, CANSparkMax.ControlType.kPosition);
		rightPIDController.setReference(setPoint, CANSparkMax.ControlType.kPosition);
	}

	public boolean atDistTarget() {
		return distPIDController.atSetpoint();
	}

	public double levelChargingStation() {
		double currPitch = lib.getAvgPitch();
		double pidOut = levelPIDController.calculate(currPitch);
		driveArcade(-pidOut, 0.0);
		return pidOut;
	}

	public double rateChargingStation() {
		double currRate = lib.getAvgRate();
		double pidOut = levelPIDController.calculate(currRate);
		driveArcade(pidOut, 0.0);
		return currRate;
	}

	public void toggleDir() {
		DirState dir = getDirState();
		switch (dir) {
			case FORWARD:
				dir = DirState.REVERSE;
				break;
			case REVERSE:
				dir = DirState.FORWARD;
				break;
		}
		this.dirState = dir;
	}

	public void setDirState(DirState dir) {
		this.dirState = dir;
	}

	public DirState getDirState() {
		return dirState;
	}

	public void setDriveState(DriveState d) {
		this.driveState = d;
	}

	public DriveState getDriveState() {
		return driveState;
	}

	public DriveState toggleDriveState() {
		switch (driveState) {
			case TANK:
				driveState = DriveState.LARCADE;
				break;
			case ARCADE:
				driveState = DriveState.TANK;
				break;
			case LARCADE:
				driveState = DriveState.TANK;
				break;
		}
		return driveState;
	}

	public void driveSelected(double leftX, double leftY, double rightX, double rightY) {
		// DriverStation.reportWarning("Position: " + leftEncoder.getPosition() + " : "
		// + rightEncoder.getPosition(),false);

		switch (driveState) {
			case TANK:
				switch (dirState) {
					case FORWARD:
						diffDrive.tankDrive(-leftY, -rightY);
						break;
					case REVERSE:
						diffDrive.tankDrive(rightY, leftY);
						break;
				}
				break;

			case ARCADE:
				switch (dirState) {
					case FORWARD:
						diffDrive.arcadeDrive(-leftY, -leftX);
						break;
					case REVERSE:
						diffDrive.arcadeDrive(leftY, leftX);
						break;
				}
				break;

			case LARCADE:
				switch (dirState) {
					case FORWARD:
						diffDrive.arcadeDrive(-leftY, -rightX);
						break;
					case REVERSE:
						diffDrive.arcadeDrive(leftY, rightX);
						break;
				}
				break;
		}
	}

	public void driveTank(double left, double right) {
		// DriverStation.reportWarning("Position: " + leftEncoder.getPosition() + " : "
		// + rightEncoder.getPosition(),false);

		switch (dirState) {
			case FORWARD:
				diffDrive.tankDrive(left, right);
				break;
			case REVERSE:
				diffDrive.tankDrive(-right, -left);
				break;
		}
	}

	public void driveArcade(double spd, double rot) {
		// DriverStation.reportWarning("Position: " + leftEncoder.getPosition() + " : "
		// + rightEncoder.getPosition(),false);

		switch (dirState) {
			case FORWARD:
				diffDrive.arcadeDrive(spd, -rot);
				break;
			case REVERSE:
				diffDrive.arcadeDrive(-spd, -rot);
				break;
		}
	}

	public void setGearShifter(GearShifterState state) {
		shifterState = state;
		switch (state) {
			case HI:
				gearShifter.set(Value.kReverse);
				break;
			case LO:
				gearShifter.set(Value.kForward);
				break;
			default:
				DriverStation.reportWarning(String.format("Chassis: Illegal GearShifter State %s", state), false);
		}

		UpdateGearRatios(shifterState);
	}

	public GearShifterState getGearShifter() {
		return shifterState;
	}

	public void UpdateGearRatios(GearShifterState shifterState) {
		gearBoxRatio = (shifterState == Chassis.GearShifterState.HI ? ChassisConstants.kHIGearBoxRatio
				: ChassisConstants.kLOGearBoxRatio);
		posFactor = ChassisConstants.kWheelCirc / (gearBoxRatio * ChassisConstants.kEncoderResolution); // Meters // Rev
		velFactor = ChassisConstants.kWheelCirc / (gearBoxRatio * ChassisConstants.kEncoderResolution) / 60.0; // Meters / Sec

		// ==============================================================
		// Configure encoders
		leftEncoder.setPositionConversionFactor(posFactor * 0.5714);	// / 1.923)); /// 2.0)*1.2); //1.17842);
		rightEncoder.setPositionConversionFactor(posFactor * 0.5714);	// / 1.923)); /// 2.0)*1.2); //1.17842);

		leftEncoder.setVelocityConversionFactor(velFactor * 0.5714);	// / 1.923);
		rightEncoder.setVelocityConversionFactor(velFactor * 0.5714);	// / 1.923);
	}

	public void disableCompressor() {
		// compressor.disable();
	}

	public void enableCompressor() {
		// compressor.enableDigital();
	}

	private double leftMax;
	private double leftMin;
	private double rightMax;
	private double rightMin;

	public void getMotorData() {
		leftMax = leftPIDController.getOutputMax();
		leftMin = leftPIDController.getOutputMin();
		rightMax = rightPIDController.getOutputMax();
		rightMin = rightPIDController.getOutputMin();
	}

	public void setMotorData() {
		leftPIDController.setOutputRange(leftMin, leftMax);
		rightPIDController.setOutputRange(rightMin, rightMax);
	}

	public void setMotorData(double min, double max) {
		leftPIDController.setOutputRange(min, max);
		rightPIDController.setOutputRange(min, max);
	}

	public void stopChassis() {
		leftMaster.set(0.0);
		rightMaster.set(0.0);
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
		return 250.0 * (loPressureSensor.getVoltage() / AnalogInConstants.kInputVoltage) - 25.0;
	}

	public double getHiPressure() {
		return 250.0 * (hiPressureSensor.getVoltage() / AnalogInConstants.kInputVoltage) - 25.0;
	}
}