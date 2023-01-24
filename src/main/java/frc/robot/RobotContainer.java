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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.subsystems.Chassis;
import frc.robot.commands.ChassisTankDrive;
import frc.robot.commands.ChassisArcadeDrive;
import frc.robot.commands.DoRumble;
import frc.robot.commands.AutonChargingStation;
import frc.robot.commands.AutonChgStnDrive;
import frc.robot.commands.AutonChgStnLevel;
import frc.robot.commands.AutonChgStnRate;
import frc.robot.commands.AutonGetGameElement;
import frc.robot.commands.AutonReturn;
import frc.robot.commands.AutonReturnToGrid;
import frc.robot.commands.AutonStraight;
import frc.robot.commands.PIDLevel;
import frc.robot.Constants.ChassisConstants;
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
	// The robot's subsystems and commands are defined here...
	private static final Chassis chassis = new Chassis();

	// =============================================================
	// Define Joysticks
	public final XboxController driver = new XboxController(OIConstants.kDriverControllerPort);
	public final XboxController operator = new XboxController(OIConstants.kOperatorControllerPort);

	private final SlewRateLimiter speedLimiter = new SlewRateLimiter(3);
	private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);

	// Define a chooser for autonomous commands
	private final SendableChooser<Command> chooser = new SendableChooser<>();

	// =============================================================
	// Define Commands here to avoid multiple instantiations
	// If commands use Shuffleboard and are instantiated multiple time, an error
	// is thrown on the second instantiation becuase the "title" already exists.
	private final ChassisTankDrive chassisTankDrive = new ChassisTankDrive(chassis,
			() -> getLimitedJoystick(driver.getLeftY(), speedLimiter),
			() -> getLimitedJoystick(driver.getRightY(), speedLimiter));
	private final ChassisArcadeDrive chassisArcadeDrive = new ChassisArcadeDrive(chassis,
			() -> getLimitedJoystick(driver.getLeftY(), speedLimiter),
			() -> getLimitedJoystick(-driver.getLeftX(), rotLimiter));

	private final DoRumble doRumble = new DoRumble(this);

	private final PIDLevel pidLevel = new PIDLevel(chassis);

	// =============================================================
	// Define Autonomous Commands here
	private final AutonChargingStation autonChargingStation = new AutonChargingStation(chassis);
	private final AutonChgStnDrive autonChgStnDrive = new AutonChgStnDrive(chassis);
	private final AutonChgStnRate autonChgStnRate = new AutonChgStnRate(chassis);
	private final AutonChgStnLevel autonChgStnLevel = new AutonChgStnLevel(chassis);

	// =============================================================
	// Define Trajectory Commands here and add Trajectors below
	private AutonStraight autonStraight = null;
	private AutonReturn autonReturn = null;

	private final String GetGameElementJSON = "paths/output/Straight.wpilib.json";
	private Trajectory getGameElement = null;
	private AutonGetGameElement autonGetGameElement = null;

	private final String ReturnToGridJSON = "paths/output/Return.wpilib.json";
	private Trajectory returnToGrid = null;
	private AutonReturnToGrid autonReturnToGrid = null;

	// =============================================================
	// Create a voltage constraint to ensure we don't accelerate too fast
	private DifferentialDriveVoltageConstraint autoVoltageConstraint;

	// Create configs for trajectory
	private TrajectoryConfig config;
	private TrajectoryConfig configReversed;

	// An example trajectory to follow. All units in meters.
	public Trajectory fwdStraight;
	public Trajectory revStraight;

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {

		// ==============================================================================
		// Add Subsystems to Dashboard
		SmartDashboard.putData("Chassis", chassis);

		// SmartDashboard.putData("Feeder", feeder);

		// =============================================================
		// Configure default commands for each subsystem
		chassis.setDefaultCommand(chassisArcadeDrive);

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
		config = new TrajectoryConfig(ChassisConstants.kMaxSpeedMetersPerSecond,
				ChassisConstants.kMaxAccelerationMetersPerSecondSquared)
				// Add kinematics to ensure max speed is actually obeyed
				.setKinematics(chassis.getKinematics())
				// Apply the voltage constraint
				.addConstraint(autoVoltageConstraint)
				.setReversed(false);

		configReversed = new TrajectoryConfig(ChassisConstants.kMaxSpeedMetersPerSecond,
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
				config);

		revStraight = TrajectoryGenerator.generateTrajectory(
				// Start at the origin facing the +X direction
				new Pose2d(Units.inchesToMeters(36.0), Units.inchesToMeters(0.0), new Rotation2d(180)),
				List.of(),
				new Pose2d(Units.inchesToMeters(0.0), Units.inchesToMeters(0.0), new Rotation2d(0)),
				// Pass config
				configReversed);

		try {
			Path GetGameElementPATH = Filesystem.getDeployDirectory().toPath().resolve(GetGameElementJSON);
			getGameElement = TrajectoryUtil.fromPathweaverJson(GetGameElementPATH);

			Path ReturnToGridPATH = Filesystem.getDeployDirectory().toPath().resolve(ReturnToGridJSON);
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

		// =============================================================
		// Build chooser for autonomous commands

		// Put the chooser on the dashboard
		SmartDashboard.putData(chooser);

		configureButtonBindings();
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
		new JoystickButton(driver, Button.kX.value).onTrue(autonChargingStation);
		new JoystickButton(driver, Button.kY.value).onTrue(chassisArcadeDrive);

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

	public void setDriverRumble(GenericHID.RumbleType t) {
		driver.setRumble(t, 1);
	}

	public void resetDriverRumble(GenericHID.RumbleType t) {
		driver.setRumble(t, 0);
	}

	public void setOperatorRumble(GenericHID.RumbleType t) {
		operator.setRumble(t, 1);
	}

	public void resetOperatorRumble(GenericHID.RumbleType t) {
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
