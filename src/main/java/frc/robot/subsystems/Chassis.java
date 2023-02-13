// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AnalogInConstants;
import frc.robot.Constants.CANidConstants;
import frc.robot.Constants.ChassisConstants;
import frc.robot.Constants.GearShifterState;
import frc.robot.Constants.PneumaticChannelConstants;

import frc.robot.Library;

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
	public final RelativeEncoder leftEncoder = leftMaster.getEncoder();;
	public final RelativeEncoder rightEncoder = rightMaster.getEncoder();

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

	// ==============================================================
	// Initialize NavX AHRS board
	// Alternatively: I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB
	private final AHRS ahrs = new AHRS(SPI.Port.kMXP);

	// ==============================================================
	// Identify PDP and PCM
	private final PowerDistribution pdp = new PowerDistribution();
	private final PneumaticsControlModule pcm = new PneumaticsControlModule();
	private final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

	// ==============================================================
	// Identify compressor hi and lo sensors
	private final AnalogInput hiPressureSensor = new AnalogInput(AnalogInConstants.kHiPressureChannel);
	private final AnalogInput loPressureSensor = new AnalogInput(AnalogInConstants.kLoPressureChannel);

	// ==============================================================
	// Identify pneumatics for gear shifters

	private final DoubleSolenoid gearShifter = new DoubleSolenoid(
			PneumaticsModuleType.CTREPCM,
			PneumaticChannelConstants.kChassisShifterHi,
			PneumaticChannelConstants.kChassisShifterLo);

	// ==============================================================
	// Define local variables
	private double currPitch = 0.0;
	private double lastPitch = 0.0;
	// private double[] lastPitch = new double[5];
	// private int indexPitch = 0;
	private double avgRate = 0.0;
	private double maxPitch = 0.0;
	private double minPitch = 0.0;
	public boolean isPitchIncreasing = false;
	public boolean isPitchDecreasing = false;
	private double setPoint = 0.0;
	private double leftError = 0.0;
	private double rightError = 0.0;

	public SimpleDateFormat timeStamp = new SimpleDateFormat("yyyy.MM.dd.HH.mm.ss.SSS");

	// ==============================================================
	// Define Shuffleboard data

	private final ShuffleboardTab pidTab = Shuffleboard.getTab("PID");
	// private final GenericEntry sbLevelPID = pidTab.add("Level PID",
	// false).getEntry();

	private final ShuffleboardTab chassisTab = Shuffleboard.getTab("Chassis");
	private final GenericEntry sbLeftPos = chassisTab.addPersistent("ML Position", 0).getEntry();
	// private final GenericEntry sbLeftVel = chassisTab.addPersistent("ML
	// Velocity", 0).getEntry();
	private final GenericEntry sbRightPos = chassisTab.addPersistent("MR Position", 0).getEntry();
	// private final GenericEntry sbRightVel = chassisTab.addPersistent("MR
	// Velocity", 0).getEntry();
	// private final GenericEntry sbLeftPow = chassisTab.addPersistent("ML Power",
	// 0).getEntry();
	// private final GenericEntry sbRightPow = chassisTab.addPersistent("MR Power",
	// 0).getEntry();
	private final GenericEntry sbAvgRate = chassisTab.addPersistent("Avg Rate", 0).getEntry();
	private final GenericEntry sbAvgPitch = chassisTab.addPersistent("Avg Pitch", 0).getEntry();
	private final GenericEntry sbAngle = chassisTab.addPersistent("Angle", 0).getEntry();
	private final GenericEntry sbHeading = chassisTab.addPersistent("Heading", 0).getEntry();

	// private final GenericEntry sbX = chassisTab.addPersistent("Pose X",
	// 0).getEntry();
	// private final GenericEntry sbY = chassisTab.addPersistent("Pose Y",
	// 0).getEntry();
	// private final GenericEntry sbDeg = chassisTab.addPersistent("Pose Deg",
	// 0).getEntry();

	// private final GenericEntry sbSetPt = chassisTab.addPersistent("Setpoint",
	// 0.0).getEntry();
	// private final GenericEntry sbLeftErr = chassisTab.addPersistent("Left Error",
	// 0.0).getEntry();
	// private final GenericEntry sbRightErr = chassisTab.addPersistent("Right
	// Error", 0.0).getEntry();
	private final GenericEntry sbAtTgt = chassisTab.addPersistent("At Target", false).getEntry();

	private final ShuffleboardTab pneumaticsTab = Shuffleboard.getTab("Pneumatics");
	private final GenericEntry sbHiPressure = pneumaticsTab.addPersistent("Hi Pressure", 0).getEntry();
	private final GenericEntry sbLoPressure = pneumaticsTab.addPersistent("Lo Pressure", 0).getEntry();

	public final Library lib = new Library();

	public Chassis() {
		System.out.println("+++++ Chassis Constructor starting +++++");

		// ==============================================================
		// Configure PDP and PCM
		pdp.clearStickyFaults();
		pcm.clearAllStickyFaults();

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
		leftEncoder.setPositionConversionFactor(ChassisConstants.kPosFactorMPR);
		rightEncoder.setPositionConversionFactor(ChassisConstants.kPosFactorMPR);

		leftEncoder.setVelocityConversionFactor(ChassisConstants.kVelFactor);
		rightEncoder.setVelocityConversionFactor(ChassisConstants.kVelFactor);

		// ==============================================================
		// Define autonomous Kinematics & Odometry functions
		odometry = new DifferentialDriveOdometry(getAngle(), leftEncoder.getPosition(), rightEncoder.getPosition());

		// ==============================================================
		// Add static variables to Shuffleboard
		chassisTab.addPersistent("ML Pos Factor", leftEncoder.getPositionConversionFactor());
		chassisTab.addPersistent("MR Pos Factor", rightEncoder.getPositionConversionFactor());
		chassisTab.addPersistent("ML Vel Factor", leftEncoder.getVelocityConversionFactor());
		chassisTab.addPersistent("MR Vel Factor", rightEncoder.getVelocityConversionFactor());

		// pidTab.add("Level PID", levelPIDController);

		// ==============================================================
		// Initialize devices before starting
		resetFieldPosition(0.0, 0.0); // Reset the field and encoder positions to zero

		// Update field position - for autonomous
		// resetOdometry(RobotContainer.BlueRungSideCargoToHub.getInitialPose());

		// for (int i = 0; i < 5; i++) {
		// lastPitch[i] = 0.0;
		// }

		stopChassis();
		
		setGearShifter(GearShifterState.HI);

		lib.initLibrary();

		System.out.println("----- Chassis Constructor finished -----");
	}

	@Override
	public void periodic() {
		sbLeftPos.setDouble(leftEncoder.getPosition());
		// sbLeftVel.setDouble(leftEncoder.getVelocity());
		sbRightPos.setDouble(rightEncoder.getPosition());
		// sbRightVel.setDouble(rightEncoder.getVelocity());
		// sbLeftPow.setDouble(leftMaster.getAppliedOutput());
		// sbRightPow.setDouble(rightMaster.getAppliedOutput());

		sbAvgRate.setDouble(lib.getAvgRate());
		sbAvgPitch.setDouble(lib.getAvgPitch());
		sbAngle.setDouble(getAngle().getDegrees());
		sbHeading.setDouble(getHeading());

		sbHiPressure.setDouble(getHiPressure());
		sbLoPressure.setDouble(getLoPressure());

		// sbSetPt.setDouble(setPoint);
		// sbLeftErr.setDouble(leftError);
		// sbRightErr.setDouble(rightError);
		sbAtTgt.setBoolean(atTarget());

		// // Update field position - for autonomous
		// resetOdometry(BlueSideRung.getInitialPose());

		resetEncoders();
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

		// sbX.setDouble(x);
		// sbY.setDouble(y);
		// sbDeg.setDouble(deg);

		avgRate = lib.updatePitch(getPitch());
	}

	public void disableCompressor() {
		compressor.disable();
	}

	public void enableCompressor() {
		compressor.enableDigital();
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

	public double levelChargingStation() {
		double currPitch = lib.getAvgPitch();
		double pidOut = levelPIDController.calculate(currPitch);
		driveArcade(pidOut, 0.0);
		return pidOut;
	}

	public double rateChargingStation() {
		double currRate = lib.getAvgRate();
		double pidOut = levelPIDController.calculate(currRate);
		driveArcade(pidOut, 0.0);
		return currRate;
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

	public void initMonitorPitch() {
		maxPitch = 0.0;
		minPitch = 0.0;
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

	// public double getAvgRate() {
	// return avgRate;
	// }

	// public double getMinPitch() {
	// return minPitch;
	// }

	// public double getMaxPitch() {
	// return maxPitch;
	// }

	public void driveTankVolts(double leftVolts, double rightVolts) {
		leftMaster.setVoltage(leftVolts);
		rightMaster.setVoltage(rightVolts);
		diffDrive.feed();
	}

	public void driveTank(double left, double right) {
		diffDrive.tankDrive(left, right);
	}

	public void driveArcade(double spd, double rot) {
		diffDrive.arcadeDrive(spd, -rot);
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

	public void setGearShifter(GearShifterState state) {
		switch (state) {
			case HI:
				gearShifter.set(Value.kForward);
				break;
			case LO:
				gearShifter.set(Value.kReverse);
				break;
			default:
				DriverStation.reportWarning(String.format("Chassis: Illegal GearShifter State %s", state), false);
		}
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