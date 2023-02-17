// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;
import java.util.concurrent.TimeUnit;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Crane;
import frc.robot.subsystems.CraneArm;
import frc.robot.subsystems.CraneTilt;
import frc.robot.subsystems.CraneTurret;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Intake.ArmState;
import frc.robot.subsystems.Intake.MotorState;
import frc.robot.subsystems.Claw.FingerState;
import frc.robot.subsystems.CraneTilt.RatchetState;
import frc.robot.commands.ChassisTankDrive;
import frc.robot.commands.ClawFinger;
import frc.robot.commands.CraneArm2Pos;
import frc.robot.commands.CraneTilt2Pos;
import frc.robot.commands.CraneTurret2Pos;
import frc.robot.commands.ChassisArcadeDrive;
import frc.robot.commands.ChassisSetGearShifter;
import frc.robot.commands.DoRumble;
import frc.robot.commands.IntakeArm;
import frc.robot.commands.IntakeMotor;
import frc.robot.commands.IntakeStow;
import frc.robot.commands.TiltRatchet;
import frc.robot.commands.AutonChargingStation;
import frc.robot.commands.AutonChgStnDrive;
import frc.robot.commands.AutonChgStnLevel;
import frc.robot.commands.AutonChgStnRate;
import frc.robot.commands.AutonGetGameElement;
import frc.robot.commands.AutonReturn;
import frc.robot.commands.AutonReturnToGrid;
import frc.robot.commands.AutonStraight;
import frc.robot.commands.AutonTrackAprilTag;
import frc.robot.commands.ChassisTeleopTrackAprilTag;
import frc.robot.Constants.ChassisConstants;
import frc.robot.Constants.GearShifterState;
import frc.robot.Constants.OIConstants;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	// =============================================================
	// Define Joysticks
	public static final XboxController driver = new XboxController(OIConstants.kDriverControllerPort);
	public static final XboxController operator = new XboxController(OIConstants.kOperatorControllerPort);

	// The robot's subsystems and commands are defined here...
	private static final Chassis chassis = new Chassis();
	private static final Claw claw = new Claw();
	private static final Intake intake = new Intake();
	private static final Crane crane = new Crane(operator);
	private static final CraneTurret craneTurret = new CraneTurret();
	private static final CraneTilt craneTilt = new CraneTilt();
	private static final CraneArm craneArm = new CraneArm();
	private static final Vision vision = new Vision();

	private final SlewRateLimiter speedLimiter = new SlewRateLimiter(3);
	private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);

	// Define a chooser for autonomous commands
	private final SendableChooser<Command> chooser = new SendableChooser<>();
	private final ShuffleboardTab cmdTab;

	// =============================================================
	// Define Commands here to avoid multiple instantiations
	// If commands use Shuffleboard and are instantiated multiple time, an error
	// is thrown on the second instantiation becuase the "title" already exists.
	private final ChassisTankDrive chassisTankDrive = new ChassisTankDrive(chassis,
			() -> getJoystick(driver.getLeftY()),
			() -> getJoystick(driver.getRightY()));
	private final ChassisArcadeDrive chassisArcadeDrive = new ChassisArcadeDrive(chassis,
			() -> getJoystick(driver.getLeftY()),
			() -> getJoystick(-driver.getLeftX()));

	private final DoRumble doRumble = new DoRumble(this);

	// =============================================================
	// Define Autonomous Commands here
	private final AutonChargingStation autonChargingStation = new AutonChargingStation(chassis);
	private final AutonChgStnDrive autonChgStnDrive = new AutonChgStnDrive(chassis);
	private final AutonChgStnRate autonChgStnRate = new AutonChgStnRate(chassis);
	private final AutonChgStnLevel autonChgStnLevel = new AutonChgStnLevel(chassis);
	private final AutonTrackAprilTag autonTrackAprilTag = new AutonTrackAprilTag(chassis, vision);

	private final ChassisTeleopTrackAprilTag teleopTrackAprilTag = new ChassisTeleopTrackAprilTag(chassis, vision);

	private final ChassisSetGearShifter chassisShiftHI = new ChassisSetGearShifter(chassis, GearShifterState.HI);
	private final ChassisSetGearShifter chassisShiftLO = new ChassisSetGearShifter(chassis, GearShifterState.LO);

	private final CraneArm2Pos craneArm2Pos = new CraneArm2Pos(craneArm);
	private final CraneTilt2Pos craneTilt2Pos = new CraneTilt2Pos(craneTilt);
	private final CraneTurret2Pos craneTurret2Pos = new CraneTurret2Pos(craneTurret);

	private final ClawFinger clawGrabCone = new ClawFinger(claw, FingerState.CONEGRAB);
	private final ClawFinger clawGrabCube = new ClawFinger(claw, FingerState.CUBEGRAB);
	private final ClawFinger clawRelease = new ClawFinger(claw, FingerState.RELEASE);

	private final IntakeStow intakeStow = new IntakeStow(intake);
	private final IntakeMotor intakeMotorIn = new IntakeMotor(intake, MotorState.IN);
	private final IntakeMotor IntakeMotorOut = new IntakeMotor(intake, MotorState.OUT);

	private final IntakeArm intakeOpen = new IntakeArm(intake, ArmState.OPEN);
	private final IntakeArm intakeClose = new IntakeArm(intake, ArmState.CLOSE);

	private final TiltRatchet ratchetLock = new TiltRatchet(craneTilt, RatchetState.LOCK);
	private final TiltRatchet ratchetUnlock = new TiltRatchet(craneTilt, RatchetState.UNLOCK);

	// =============================================================
	// Create a voltage constraint to ensure we don't accelerate too fast
	private DifferentialDriveVoltageConstraint autoVoltageConstraint;

	// Create configs for trajectory
	private TrajectoryConfig fwdConfig = null;
	private TrajectoryConfig revConfig = null;

	// =============================================================
	// Define Trajectory Commands here and add Trajectors below
	private Trajectory fwdStraight = null;
	private AutonStraight autonStraight = null;
	private Trajectory revStraight = null;
	private AutonReturn autonReturn = null;

	private Trajectory getGameElement = null;
	private AutonGetGameElement autonGetGameElement = null;

	private Trajectory returnToGrid = null;
	private AutonReturnToGrid autonReturnToGrid = null;

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		System.out.println("+++++ RobotContainer Constructor starting +++++");

		// ==============================================================================
		// Add Subsystems to Dashboard
		Shuffleboard.getTab("SubSystems").add("Chassis", chassis);
		Shuffleboard.getTab("SubSystems").add("Claw", claw);
		Shuffleboard.getTab("SubSystems").add("Crane", crane);
		Shuffleboard.getTab("SubSystems").add("Crane Turret", craneTurret);
		Shuffleboard.getTab("SubSystems").add("Crane Tilt", craneTilt);
		Shuffleboard.getTab("SubSystems").add("Crane Arm", craneArm);
		Shuffleboard.getTab("SubSystems").add("Intake", intake);
		Shuffleboard.getTab("SubSystems").add("Vision", vision);

		// =============================================================
		// Configure default commands for each subsystem
		chassis.setDefaultCommand(chassisArcadeDrive);
		craneTurret.setDefaultCommand(craneTurret2Pos);
		craneTilt.setDefaultCommand(craneTilt2Pos);
		craneArm.setDefaultCommand(craneArm2Pos);
		vision.setDefaultCommand(teleopTrackAprilTag);

		// =============================================================
		// Create a voltage constraint to ensure we don't accelerate too fast
		autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
				new SimpleMotorFeedforward(
						ChassisConstants.kS,
						ChassisConstants.kV,
						ChassisConstants.kA),
				chassis.getKinematics(),
				10);

		// Create config for trajectory
		fwdConfig = new TrajectoryConfig(ChassisConstants.kMaxSpeedMetersPerSecond,
				ChassisConstants.kMaxAccelerationMetersPerSecondSquared)
				// Add kinematics to ensure max speed is actually obeyed
				.setKinematics(chassis.getKinematics())
				// Apply the voltage constraint
				.addConstraint(autoVoltageConstraint)
				.setReversed(false);

		revConfig = new TrajectoryConfig(ChassisConstants.kMaxSpeedMetersPerSecond,
				ChassisConstants.kMaxAccelerationMetersPerSecondSquared)
				// Add kinematics to ensure max speed is actually obeyed
				.setKinematics(chassis.getKinematics())
				// Apply the voltage constraint
				.addConstraint(autoVoltageConstraint)
				.setReversed(true);

		fwdStraight = TrajectoryGenerator.generateTrajectory(
				// Start at the origin facing the +X direction
				new Pose2d(Units.inchesToMeters(0.0), Units.inchesToMeters(0.0), new Rotation2d(180)),
				List.of(),
				new Pose2d(Units.inchesToMeters(36.0), Units.inchesToMeters(0.0), new Rotation2d(0)),
				// Pass config
				fwdConfig);

		revStraight = TrajectoryGenerator.generateTrajectory(
				// Start at the origin facing the +X direction
				new Pose2d(Units.inchesToMeters(36.0), Units.inchesToMeters(0.0), new Rotation2d(180)),
				List.of(),
				new Pose2d(Units.inchesToMeters(0.0), Units.inchesToMeters(0.0), new Rotation2d(0)),
				// Pass config
				revConfig);

		try {
			Path GetGameElementPATH = Filesystem.getDeployDirectory().toPath().resolve("output/Straight.wpilib.json");
			getGameElement = TrajectoryUtil.fromPathweaverJson(GetGameElementPATH);

			Path ReturnToGridPATH = Filesystem.getDeployDirectory().toPath().resolve("output/Return.wpilib.json");
			returnToGrid = TrajectoryUtil.fromPathweaverJson(ReturnToGridPATH);

		} catch (IOException e) {
			DriverStation.reportError("Unable to open trajectory", e.getStackTrace());
		}

		autonStraight = new AutonStraight(chassis, fwdStraight);
		autonReturn = new AutonReturn(chassis, revStraight);
		autonGetGameElement = new AutonGetGameElement(chassis, getGameElement);
		autonReturnToGrid = new AutonReturnToGrid(chassis, returnToGrid);

		// ==============================================================================
		// Add commands to the autonomous command chooser
		chooser.setDefaultOption("Tank Drive", chassisTankDrive);
		chooser.addOption("Charging Station", autonChargingStation);
		chooser.addOption("Charging Station Drive", autonChgStnDrive);
		chooser.addOption("Charging Station Rate", autonChgStnRate);
		chooser.addOption("Charging Station Level", autonChgStnLevel);
		chooser.addOption("Get Game Element", autonGetGameElement);
		chooser.addOption("Return To Grid", autonReturnToGrid);
		chooser.addOption("Straight", autonStraight);
		chooser.addOption("Return", autonReturn);
		chooser.addOption("Track April Tag", autonTrackAprilTag);

		// =============================================================
		// Build chooser for autonomous commands

		// Put the chooser on the dashboard
		Shuffleboard.getTab("Competition").add("Auton Command", chooser);

		cmdTab = Shuffleboard.getTab("Commands");
		cmdTab.add("Gear Shift HI", chassisShiftHI);
		cmdTab.add("Gear Shift LO", chassisShiftLO);
		cmdTab.add("Claw Grab Cone", clawGrabCone);
		cmdTab.add("Claw Grab Cube", clawGrabCube);
		cmdTab.add("Claw Release", clawRelease);
		cmdTab.add("Tilt Ratchet Lock", ratchetLock);
		cmdTab.add("Tilt Ratchet Unlock", ratchetUnlock);
		cmdTab.add("Intake Stow", intakeStow);
		cmdTab.add("Intake Open", intakeOpen);
		cmdTab.add("Intake Close", intakeClose);
		cmdTab.add("Intake In", intakeMotorIn);
		cmdTab.add("Intake Out", IntakeMotorOut);

		configureButtonBindings();

		System.out.println("+++++ RobotContainer Constructor finished +++++");
	}

	// private final DriveTrajectory blueRungSideCargoToHubCommand = new
	// DriveTrajectory(chassis, BlueRungSideCargoToHub);

	//
	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by
	 * instantiating a {@link GenericHID} or one of its subclasses ({@link
	 * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
	 * it to a {@link
	 * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
	 */
	private void configureButtonBindings() {
		// new JoystickButton(driver, Button.kX.value).onTrue(autonChargingStation);
		// new JoystickButton(driver, Button.kY.value).onTrue(chassisArcadeDrive);
		// new JoystickButton(driver, Button.kA.value).onTrue(teleopTrackAprilTag);

		// new JoystickButton(driver, Button.kLeftBumper.value).onTrue(craneStartPIDs);
		new JoystickButton(driver, Button.kLeftBumper.value).onTrue(chassisShiftHI);
		new JoystickButton(driver, Button.kRightBumper.value).onTrue(chassisShiftLO);

		new JoystickButton(driver, Button.kY.value).onTrue(clawGrabCone);
		new JoystickButton(driver, Button.kX.value).onTrue(clawGrabCube);
		new JoystickButton(driver, Button.kB.value).onTrue(clawRelease);

		new JoystickButton(driver, Button.kStart.value).onTrue(ratchetLock);
		new JoystickButton(driver, Button.kBack.value).onTrue(ratchetUnlock);

		new JoystickButton(driver, Button.kA.value).onTrue(craneArm2Pos);
		new JoystickButton(operator, Button.kLeftBumper.value).onTrue(craneTilt2Pos);
		new JoystickButton(operator, Button.kRightBumper.value).onTrue(craneTurret2Pos);

		new JoystickButton(operator, Button.kY.value).onTrue(intakeStow);
		new JoystickButton(operator, Button.kX.value).whileTrue(intakeMotorIn);
		new JoystickButton(operator, Button.kA.value).whileTrue(IntakeMotorOut);
		new JoystickButton(operator, Button.kB.value).onTrue(intakeOpen);
		new JoystickButton(operator, Button.kStart.value).onTrue(intakeClose);
	}

	public double getDPad() {
		return operator.getPOV();
	}

	private final double MAXSPEED = 6.0; // meters per second or approx half rotation (PI) per sec

	public double getLimitedJoystick(double js, SlewRateLimiter limiter) {
		return limiter.calculate(js * MAXSPEED);
	}

	private static final double DEADZONE = 0.01;
	private static final double MAXACCEL = 0.001; // joystick units per 20ms
	private double lastValue = 0.0;

	public double getJoystick(double js) {
		double val = 0.0;
		if (js > 0) {
			val = Math.pow(Math.abs(js) < DEADZONE ? 0.0 : js, 2);
		} else {
			val = -1 * Math.pow(Math.abs(js) < DEADZONE ? 0.0 : js, 2);
		}

		// if (Math.abs(val - lastValue) > MAXACCEL) {
		// lastValue += MAXACCEL;
		// val = lastValue;
		// }

		return val;
	}

	public static void setDriverRumble(GenericHID.RumbleType t) {
		driver.setRumble(t, 1);
	}

	public static void resetDriverRumble(GenericHID.RumbleType t) {
		driver.setRumble(t, 0);
	}

	public static void setOperatorRumble(GenericHID.RumbleType t) {
		operator.setRumble(t, 1);
	}

	public static void resetOperatorRumble(GenericHID.RumbleType t) {
		operator.setRumble(t, 0);
	}

	public void doRumble(XboxController c, GenericHID.RumbleType t) {
		Thread thread = new Thread("Rumble") {
			@Override
			public void run() {

				c.setRumble(t, 1);

				try {
					TimeUnit.MILLISECONDS.sleep(OIConstants.kRumbleDelay);
				} catch (InterruptedException e) {
					DriverStation.reportError("Rumble sleep exception", true);
				}

				c.setRumble(t, 0);
			}
		};
		thread.start();
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		return chooser.getSelected();
	}
}
