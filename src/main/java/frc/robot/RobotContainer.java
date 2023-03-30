// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.concurrent.TimeUnit;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

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
import frc.robot.subsystems.Chassis.GearShifterState;
import frc.robot.subsystems.CraneTilt.RatchetState;
import frc.robot.commands.ClawFinger;
import frc.robot.commands.CraneArm2Pos;
import frc.robot.commands.CraneReset;
import frc.robot.commands.CraneTilt2Pos;
import frc.robot.commands.Crane_ManualMove;
import frc.robot.commands.Crane_Move2Position;
import frc.robot.commands.ChassisDriveSelected;
import frc.robot.commands.ChassisSetGearShifter;
import frc.robot.commands.DoRumble;
import frc.robot.commands.IntakeArm;
import frc.robot.commands.IntakeMotor;
import frc.robot.commands.IntakeStow;
import frc.robot.commands.IntakeToggleElem;
import frc.robot.commands.VisionToggleRumble;
import frc.robot.commands.firstAttempt.Crane_Move2GripPos;
import frc.robot.commands.firstAttempt.Crane_Move2HoldPos;
import frc.robot.commands.firstAttempt.Crane_Move2NodePos;
import frc.robot.commands.firstAttempt.Crane_Move2ReadyPos;
import frc.robot.commands.firstAttempt.Crane_Move2ReceivePos;
import frc.robot.commands.firstAttempt.Crane_Move2StowPos;
import frc.robot.commands.firstAttempt.Crane_Move2SubStnPos;
import frc.robot.commands.firstAttempt.Crane_MoveTilt2Zero;
import frc.robot.commands.old.AutonChargingStation;
import frc.robot.commands.old.AutonChassisDriveTime;
import frc.robot.commands.AutonChgStnDrive;
import frc.robot.commands.old.AutonChgStnLevel;
import frc.robot.commands.old.AutonChgStnRate;
import frc.robot.commands.old.ChassisArcadeDrive;
import frc.robot.commands.old.ChassisLarcadeDrive;
import frc.robot.commands.old.ChassisTankDrive;
import frc.robot.commands.CraneTurret2Pos;
import frc.robot.commands.old.TiltRatchet;
import frc.robot.commands.TeleopMove2Elem;
import frc.robot.commands.TeleopMove2Node;
import frc.robot.commands.TeleopMove2Ready;
import frc.robot.commands.TeleopMove2SubStn;
import frc.robot.commands.TeleopScoreAtNode;
import frc.robot.commands.ChassisDriveDist;
import frc.robot.commands.AutonGripMove2Node;
import frc.robot.commands.AutonTrackAprilTag;
import frc.robot.commands.VisionTeleopTrackAprilTag;
import frc.robot.commands.ChassisToggleDir;
import frc.robot.commands.ChassisToggleDrive;
import frc.robot.Constants.OIConstants;
import frc.robot.GridCalcs.CRANESTATE;

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
	public static final Chassis chassis = new Chassis();
	public static final Crane crane = new Crane(operator);
	public static final Intake intake = new Intake(crane);
	public static final Claw claw = new Claw(crane, intake);
	public static final CraneTurret craneTurret = new CraneTurret(crane);
	public static final CraneTilt craneTilt = new CraneTilt(crane);
	public static final CraneArm craneArm = new CraneArm(crane);
	public static final Vision vision = new Vision();

	public static final Autos autos = new Autos(chassis);

	private final SlewRateLimiter speedLimiter = new SlewRateLimiter(3);
	private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);

	private final ShuffleboardTab cmdTab;
	private final ShuffleboardTab compTab;
	private final ShuffleboardTab subTab;

	// =============================================================
	// Define Commands here to avoid multiple instantiations
	// If commands use Shuffleboard and are instantiated multiple time, an error
	// is thrown on the second instantiation becuase the "title" already exists.

	private final ChassisDriveSelected chassisDriveSelected = new ChassisDriveSelected(chassis,
			() -> getJoystick(driver.getLeftX()),
			() -> getJoystick(driver.getLeftY()),
			() -> getJoystick(driver.getRightX()),
			() -> getJoystick(driver.getRightY()));
	private final ChassisTankDrive chassisTankDrive = new ChassisTankDrive(chassis,
			() -> getJoystick(driver.getLeftY()),
			() -> getJoystick(driver.getRightY()));
	private final ChassisArcadeDrive chassisArcadeDrive = new ChassisArcadeDrive(chassis,
			() -> getJoystick(driver.getLeftY()),
			() -> getJoystick(-driver.getLeftX()));
	private final ChassisLarcadeDrive chassisLarcadeDrive = new ChassisLarcadeDrive(chassis,
			() -> getJoystick(driver.getLeftY()),
			() -> getJoystick(-driver.getRightX()));

	private final Crane_ManualMove craneManualMove = new Crane_ManualMove(crane, craneTurret, craneTilt, craneArm,
			() -> getJoystick(operator.getRightX()),
			() -> getJoystick(operator.getRightY()),
			() -> getJoystick(operator.getLeftY()));

	private final DoRumble doRumble = new DoRumble(this);

	// =============================================================
	// Define Autonomous Commands here
	private final AutonChargingStation autonChargingStation = new AutonChargingStation(chassis);
	private final AutonChgStnDrive autonChgStnDrive = new AutonChgStnDrive(chassis);

	private final AutonChassisDriveTime autonChassisDriveTime10 = new AutonChassisDriveTime(chassis, -0.55, 10.0);
	private final AutonChassisDriveTime autonChassisDriveTime5 = new AutonChassisDriveTime(chassis, -0.55, 5.0);

	public final ChassisDriveDist autonChassisDriveDist = new ChassisDriveDist(chassis, 
			2.10773229598999, 5.0);

	private final AutonChgStnRate autonChgStnRate = new AutonChgStnRate(chassis);
	private final AutonChgStnLevel autonChgStnLevel = new AutonChgStnLevel(chassis);
	private final AutonTrackAprilTag autonTrackAprilTag = new AutonTrackAprilTag(chassis, vision, 2);

	private final AutonGripMove2Node autonGripScore = new AutonGripMove2Node(chassis,
			crane, craneTurret, craneTilt, craneArm, claw, intake);

	private final VisionTeleopTrackAprilTag teleopTrackAprilTag = new VisionTeleopTrackAprilTag(chassis, vision, 2);

	private final VisionToggleRumble visionToggleRumble = new VisionToggleRumble(vision);

	private final ChassisSetGearShifter chassisShiftHI = new ChassisSetGearShifter(chassis, GearShifterState.HI);
	private final ChassisSetGearShifter chassisShiftLO = new ChassisSetGearShifter(chassis, GearShifterState.LO);

	private final ChassisToggleDir chassisToggleDir = new ChassisToggleDir(chassis);
	private final ChassisToggleDrive chassisToggleDrive = new ChassisToggleDrive(chassis);

	private final CraneReset craneReset = new CraneReset(crane, craneTurret, craneTilt, craneArm);
	private final CraneArm2Pos craneArm2Pos = new CraneArm2Pos(craneArm);
	private final CraneTilt2Pos craneTilt2Pos = new CraneTilt2Pos(craneTilt);
	private final CraneTurret2Pos craneTurret2Pos = new CraneTurret2Pos(craneTurret);
	
	private final Crane_Move2Position crane_Move2NodePos = new Crane_Move2Position(crane, craneTurret, craneTilt,
			craneArm, CRANESTATE.NODE);
	private final Crane_Move2Position crane_Move2ReadyPos = new Crane_Move2Position(crane, craneTurret, craneTilt,
			craneArm, CRANESTATE.READY);
	private final Crane_Move2Position crane_Move2GripPos = new Crane_Move2Position(crane, craneTurret, craneTilt,
			craneArm, CRANESTATE.GRIP);
	private final Crane_Move2Position crane_Move2ReceivePos = new Crane_Move2Position(crane, craneTurret, craneTilt,
			craneArm, CRANESTATE.RECEIVE);
	private final Crane_Move2Position crane_Move2HoldPos = new Crane_Move2Position(crane, craneTurret, craneTilt,
			craneArm, CRANESTATE.HOLD);
	private final Crane_Move2Position crane_Move2StowPos = new Crane_Move2Position(crane, craneTurret, craneTilt,
			craneArm, CRANESTATE.STOW);
	private final Crane_Move2Position crane_Move2SubStnPos = new Crane_Move2Position(crane, craneTurret, craneTilt,
			craneArm, CRANESTATE.SUBSTATION);
	private final Crane_Move2Position crane_Move2LeftPos = new Crane_Move2Position(crane, craneTurret, craneTilt,
			craneArm, CRANESTATE.LEFT);
	private final Crane_Move2Position crane_Move2RightPos = new Crane_Move2Position(crane, craneTurret, craneTilt,
			craneArm, CRANESTATE.RIGHT);

	// private final Crane_MoveTilt2Zero crane_MoveTilt2Zero = new Crane_MoveTilt2Zero(crane, craneTurret, craneTilt,
	// 		craneArm);

	private final TeleopMove2Elem auton_Move2ElemPos = new TeleopMove2Elem(crane, craneTurret, craneTilt,
			craneArm, claw, intake);
	private final TeleopMove2Ready auton_Move2ReadyPos = new TeleopMove2Ready(crane, craneTurret, craneTilt,
			craneArm);
	private final TeleopMove2Node auton_Move2NodePos = new TeleopMove2Node(crane, craneTurret, craneTilt,
			craneArm);
	private final TeleopScoreAtNode auton_ScoreAtNodePos = new TeleopScoreAtNode(crane, craneTurret, craneTilt,
			craneArm, claw);
	private final TeleopMove2SubStn auton_Move2SubStnPos = new TeleopMove2SubStn(crane, craneTurret, craneTilt,
			craneArm, claw, intake);

	private final ClawFinger clawGrabCone = new ClawFinger(claw, FingerState.CONE);
	private final ClawFinger clawGrabCube = new ClawFinger(claw, FingerState.CUBE);
	private final ClawFinger clawRelease = new ClawFinger(claw, FingerState.RELEASE);
	private final ClawFinger clawGrip = new ClawFinger(claw, FingerState.GRIP);

	private final IntakeStow intakeStow = new IntakeStow(intake);
	private final IntakeMotor intakeMotorIn = new IntakeMotor(intake, MotorState.IN);
	private final IntakeMotor intakeMotorOut = new IntakeMotor(intake, MotorState.OUT);
	private final IntakeMotor intakeMotorStop = new IntakeMotor(intake, MotorState.STOP);
	private final IntakeToggleElem intakeToggleElem = new IntakeToggleElem(intake);

	private final IntakeArm intakeOpen = new IntakeArm(intake, ArmState.OPEN);
	private final IntakeArm intakeClose = new IntakeArm(intake, ArmState.CLOSE);

	private final TiltRatchet ratchetLock = new TiltRatchet(craneTilt, RatchetState.LOCK);
	private final TiltRatchet ratchetUnlock = new TiltRatchet(craneTilt, RatchetState.UNLOCK);

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		System.out.println("+++++ RobotContainer Constructor starting +++++");

		// =============================================================
		// Configure default commands for each subsystem
		chassis.setDefaultCommand(chassisDriveSelected);
		crane.setDefaultCommand(craneManualMove);
		craneTurret.setDefaultCommand(craneTurret2Pos);
		craneTilt.setDefaultCommand(craneTilt2Pos);
		craneArm.setDefaultCommand(craneArm2Pos);
		vision.setDefaultCommand(teleopTrackAprilTag);

		// =============================================================
		// Construct Autonomous commands
		autos.init();

		subTab = Shuffleboard.getTab("SubSystems");
		compTab = Shuffleboard.getTab("Competition");
		cmdTab = Shuffleboard.getTab("Commands");

		// ==============================================================================
		// Add Subsystems to Dashboard
		subTab.add("Chassis", chassis).withWidget("Basic Subsystem")
				.withPosition(1, 0).withSize(4, 2);
		subTab.add("Claw", claw).withWidget("Basic Subsystem")
				.withPosition(1, 2).withSize(4, 2);
		subTab.add("Intake", intake).withWidget("Basic Subsystem")
				.withPosition(1, 4).withSize(4, 2);
		subTab.add("Vision", vision).withWidget("Basic Subsystem")
				.withPosition(1, 6).withSize(4, 2);
		subTab.add("Crane", crane).withWidget("Basic Subsystem")
				.withPosition(6, 0).withSize(4, 2);
		subTab.add("Crane Turret", craneTurret).withWidget("Basic Subsystem")
				.withPosition(6, 2).withSize(4, 2);
		subTab.add("Crane Tilt", craneTilt).withWidget("Basic Subsystem")
				.withPosition(6, 4).withSize(4, 2);
		subTab.add("Crane Arm", craneArm).withWidget("Basic Subsystem")
				.withPosition(6, 6).withSize(4, 2);

		// ==============================================================================
		// Add Commands to Dashboard
		cmdTab.add("Gear Shift HI", chassisShiftHI).withWidget("Command")
				.withPosition(0, 0).withSize(2, 1);
		cmdTab.add("Gear Shift LO", chassisShiftLO).withWidget("Command")
				.withPosition(0, 1).withSize(2, 1);
		cmdTab.add("Direction", chassisToggleDir).withWidget("Command")
				.withPosition(0, 2).withSize(2, 1);

		cmdTab.add("Claw Grab Cone", clawGrabCone).withWidget("Command")
				.withPosition(3, 0).withSize(2, 1);
		cmdTab.add("Claw Grab Cube", clawGrabCube).withWidget("Command")
				.withPosition(3, 1).withSize(2, 1);
		cmdTab.add("Claw Grip", clawGrip).withWidget("Command")
				.withPosition(3, 2).withSize(2, 1);
		cmdTab.add("Claw Release", clawRelease).withWidget("Command")
				.withPosition(3, 3).withSize(2, 1);
		compTab.add("Claw Release", clawRelease).withWidget("Command")
				.withPosition(12, 0).withSize(2, 1);

		cmdTab.add("Intake In", intakeMotorIn).withWidget("Command")
				.withPosition(6, 0).withSize(2, 1);
		cmdTab.add("Intake Out", intakeMotorOut).withWidget("Command")
				.withPosition(6, 1).withSize(2, 1);
		cmdTab.add("Intake Stow", intakeStow).withWidget("Command")
				.withPosition(6, 2).withSize(2, 1);
		cmdTab.add("Intake Open", intakeOpen).withWidget("Command")
				.withPosition(6, 3).withSize(2, 1);
		cmdTab.add("Intake Close", intakeClose).withWidget("Command")
				.withPosition(6, 4).withSize(2, 1);
		cmdTab.add("Intake Stop", intakeMotorStop).withWidget("Command")
				.withPosition(6, 5).withSize(2, 1);

		cmdTab.add("Crane Reset", craneReset).withWidget("Command")
				.withPosition(9, 0).withSize(2, 1);
		cmdTab.add("Move Stow Pos", crane_Move2StowPos).withWidget("Command")
				.withPosition(9, 1).withSize(2, 1);
		cmdTab.add("Move Receive Pos", crane_Move2ReceivePos).withWidget("Command")
				.withPosition(9, 2).withSize(2, 1);
		cmdTab.add("Move Grip Pos", crane_Move2GripPos).withWidget("Command")
				.withPosition(9, 3).withSize(2, 1);
		cmdTab.add("Move Ready Pos", crane_Move2ReadyPos).withWidget("Command")
				.withPosition(9, 4).withSize(2, 1);
		cmdTab.add("Move Node Pos", crane_Move2NodePos).withWidget("Command")
				.withPosition(9, 5).withSize(2, 1);
		cmdTab.add("Move Hold Pos", crane_Move2HoldPos).withWidget("Command")
				.withPosition(9, 6).withSize(2, 1);
		cmdTab.add("Move SubStn Pos", crane_Move2SubStnPos).withWidget("Command")
				.withPosition(9, 7).withSize(2, 1);
		cmdTab.add("Move Left Pos", crane_Move2LeftPos).withWidget("Command")
				.withPosition(9, 8).withSize(2, 1);
		cmdTab.add("Move Right Pos", crane_Move2RightPos).withWidget("Command")
				.withPosition(9, 9).withSize(2, 1);

		// cmdTab.add("Move Tilt 2 Zero", crane_MoveTilt2Zero).withWidget("Command")
		// 		.withPosition(12, 0).withSize(2, 1);
		cmdTab.add("Get Elem", auton_Move2ElemPos).withWidget("Command")
				.withPosition(12, 1).withSize(2, 1);
		cmdTab.add("Move 2 Ready", auton_Move2ReadyPos).withWidget("Command")
				.withPosition(12, 2).withSize(2, 1);
		cmdTab.add("Move 2 Node", auton_Move2NodePos).withWidget("Command")
				.withPosition(12, 3).withSize(2, 1);
		cmdTab.add("Score At Node", auton_ScoreAtNodePos).withWidget("Command")
				.withPosition(12, 4).withSize(2, 1);

		cmdTab.add("Auton Grip Score", autonGripScore).withWidget("Command")
				.withPosition(2, 6).withSize(2, 1);

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

		// new JoystickButton(driver, Button.kX.value).onTrue(auton_Move2ElemPos);
		new JoystickButton(driver, Button.kY.value).onTrue(chassisToggleDrive);
		new JoystickButton(driver, Button.kA.value).onTrue(teleopTrackAprilTag);
		new JoystickButton(driver, Button.kB.value).onTrue(autonChargingStation);
		new JoystickButton(driver, Button.kX.value).toggleOnTrue(visionToggleRumble);

		new JoystickButton(driver, Button.kStart.value).onTrue(intakeOpen);
		new JoystickButton(driver, Button.kBack.value).onTrue(intakeClose);

		new JoystickButton(driver, Button.kLeftStick.value).onTrue(chassisShiftLO);
		new JoystickButton(driver, Button.kRightStick.value).onTrue(chassisShiftHI);
		new JoystickButton(driver, Button.kLeftBumper.value).onTrue(intakeMotorOut).onFalse(intakeMotorStop);
		new JoystickButton(driver, Button.kRightBumper.value).onTrue(intakeMotorIn).onFalse(intakeMotorStop);

		new JoystickButton(operator, Button.kStart.value).onTrue(intakeToggleElem);

		new JoystickButton(operator, Button.kX.value).onTrue(auton_Move2ElemPos);
		new JoystickButton(operator, Button.kY.value).onTrue(auton_Move2ReadyPos);
		new JoystickButton(operator, Button.kB.value).onTrue(auton_Move2NodePos);
		new JoystickButton(operator, Button.kA.value).onTrue(auton_ScoreAtNodePos);

		new JoystickButton(operator, Button.kLeftStick.value).onTrue(clawRelease);
		new JoystickButton(operator, Button.kRightStick.value).onTrue(crane_Move2StowPos);

		new JoystickButton(operator, Button.kBack.value).onTrue(auton_Move2SubStnPos);
		new JoystickButton(operator, Button.kLeftBumper.value).onTrue(clawGrip);
		new JoystickButton(operator, Button.kRightBumper.value).onTrue(crane_Move2HoldPos);
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
		// return Math.copySign(Math.pow(Math.abs(js) < DEADZONE ? 0.0 : js, 2), js);
		return Math.copySign(Math.abs(js) < DEADZONE ? 0.0 : js, js);

		// if (Math.abs(val - lastValue) > MAXACCEL) {
		// lastValue += MAXACCEL;
		// val = lastValue;
		// }

		// return val;
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
		return autos.chooser.getSelected();
		// autos.autonChassisDriveDist();
		//return new AutonChassisDriveDist(chassis, 2.25, 5);
	}
}
