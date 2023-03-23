package frc.robot;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.RamseteAutoBuilder;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
//import frc.robot.commands.Balance;
import frc.robot.Constants.ChassisConstants;
import frc.robot.commands.ChassisDriveDist;
import frc.robot.commands.ChassisLevel;
import frc.robot.commands.AutonBalance;
import frc.robot.commands.AutonChgStnDrive;
import frc.robot.commands.AutonInitialMove2Node;
import frc.robot.commands.AutonPlaceBalance;
import frc.robot.commands.AutonPlaceMobilitySStn;
import frc.robot.commands.AutonPlaceMobilityWall;
import frc.robot.subsystems.Chassis;

public class Autos {

    private Chassis chassis;

    // Define a chooser for autonomous commands
    public final SendableChooser<Command> chooser = new SendableChooser<>();

    // Put the chooser on the dashboard
    private final ShuffleboardTab compTab = Shuffleboard.getTab("Competition");

    /**
     * Events to be used in all Autos built with pathplanner
     */
    private final Map<String, Command> eventMap = new HashMap<>(Map.ofEntries(
            Map.entry("lime", new InstantCommand(
                    () -> NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3)))
    // Map.entry("stop", new InstantCommand(() -> chassis.drive(new
    // Translation2d(0,0), 0, true, true)))
    // Map.entry("balance", new Balance(RobotContainer.s_Swerve))

    /*
     * Map.entry("BackwardsCube", new InstantCommand(() ->
     * RobotContainer.arm.setState(Position.AUTOCUBE))),
     * Map.entry("Transit", new InstantCommand(() ->
     * RobotContainer.arm.setState(Position.TRANSIT))),
     * Map.entry("Cone", new InstantCommand(() ->
     * RobotContainer.arm.setState(Position.UPCONE))),
     * Map.entry("SetCube", new InstantCommand(() ->
     * RobotContainer.stationSelector.setType(Type.CUBE))),
     * Map.entry("HighPlace", new InstantCommand(() ->
     * RobotContainer.stationSelector.setType(Type.CONE))
     * .andThen(new InstantCommand(() ->
     * RobotContainer.stationSelector.setPos(Position.HIGHPLACE))
     * .andThen(Commands.run(() ->
     * RobotContainer.arm.setState(RobotContainer.stationSelector.getPos()))
     * .until(() -> RobotContainer.arm.getIntakeEncoder() >
     * RobotContainer.arm.intakePlacePos()+3)
     * .andThen(Commands.run(() -> RobotContainer.arm.setState(Position.TRANSIT))
     * .until(() -> Math.abs(RobotContainer.arm.getArmAngle()) < 5))))),
     * Map.entry("CubePlace", new InstantCommand(() ->
     * RobotContainer.stationSelector.setType(Type.CUBE))
     * .andThen(new InstantCommand(() ->
     * RobotContainer.stationSelector.setPos(Position.HIGHPLACE))
     * .andThen(Commands.run(() ->
     * RobotContainer.arm.setState(RobotContainer.stationSelector.getPos()))
     * .until(() -> RobotContainer.arm.getIntakeEncoder() >
     * RobotContainer.arm.intakePlacePos()+3)
     * .andThen(Commands.run(() -> RobotContainer.arm.setState(Position.TRANSIT))
     * .until(() -> Math.abs(RobotContainer.arm.getArmAngle()) < 5))))),
     * Map.entry("MidPlace", new InstantCommand(() ->
     * RobotContainer.stationSelector.setType(Type.CUBE))
     * .andThen(new InstantCommand(() ->
     * RobotContainer.stationSelector.setPos(Position.MIDPLACE))
     * .andThen(Commands.run(() ->
     * RobotContainer.arm.setState(RobotContainer.stationSelector.getPos()))
     * .until(() -> RobotContainer.arm.getIntakeEncoder() >
     * RobotContainer.arm.intakePlacePos()+3)
     * .andThen(Commands.run(() -> RobotContainer.arm.setState(Position.TRANSIT))
     * .until(() -> Math.abs(RobotContainer.arm.getArmAngle()) < 5)))))
     */
    ));

    private RamseteAutoBuilder autoBuilder;

    public Autos(Chassis chassis) {
        this.chassis = chassis;
    }

    public static PathPlannerTrajectory selectElement1 = PathPlanner.loadPath("SelectElement1",
            new PathConstraints(4, 3));
    public static PathPlannerTrajectory scoreElement1 = PathPlanner.loadPath("ScoreElement1",
            new PathConstraints(4, 3));

    public static PathPlannerTrajectory selectElement2 = PathPlanner.loadPath("SelectElement2",
            new PathConstraints(4, 3));
    public static PathPlannerTrajectory scoreElement2 = PathPlanner.loadPath("ScoreElement2",
            new PathConstraints(4, 3));

    // List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("Example
    // Path Group", new PathConstraints(4, 3));

    // public static List<PathPlannerTrajectory> scoreThreeElements = new
    // ArrayList<>(Arrays.asList(selectElement1, scoreElement1, selectElement2,
    // scoreElement2));

    public Command scoreTwoElements() {
        return autoBuilder.fullAuto(PathPlanner.loadPath("12_ScoreTwoElements",
                new PathConstraints(4, 3)));
    }

    public Command scoreThreeElements() {
        return autoBuilder.fullAuto(PathPlanner.loadPath("18_ScoreThreeElements",
                new PathConstraints(4, 3)));
    }

    public Command scoreMobilitySStn() {
        return autoBuilder.fullAuto(PathPlanner.loadPathGroup("9_Score-Mobility(SStn)",
                new PathConstraints(4, 3)));
    }

    public Command scoreMobilityWall() {
        return autoBuilder.fullAuto(PathPlanner.loadPathGroup("9_Score-Mobility(Wall)",
                new PathConstraints(4, 3)));
    }

    public Command scoreBalance() {
        return autoBuilder.fullAuto(PathPlanner.loadPathGroup("18_Score-Balance",
                new PathConstraints(4, 3)));
    }

    public Command scoreMobilityBalance() {
        return autoBuilder.fullAuto(PathPlanner.loadPathGroup("21_Score-Mobility-Balance",
                new PathConstraints(4, 3)));
    }

    public Command testZigZag() {
        return autoBuilder.fullAuto(PathPlanner.loadPathGroup("Test Zig-Zag",
                new PathConstraints(4, 3)));
    }

    public Command test2mForward() {
        return autoBuilder.fullAuto(PathPlanner.loadPathGroup("Test 2m Forward",
                new PathConstraints(4, 3)));
    }

    public Command test3mForward() {
        return autoBuilder.fullAuto(PathPlanner.loadPathGroup("Test 3m Forward",
                new PathConstraints(4, 3)));
    }

    public Command test35mForward() {
        return autoBuilder.fullAuto(PathPlanner.loadPathGroup("Test 3.5m Forward",
                new PathConstraints(4, 3)));
    }

    public Command test4mForward() {
        return autoBuilder.fullAuto(PathPlanner.loadPathGroup("Test 4m Forward",
                new PathConstraints(4, 3)));
    }

    public Command testReverse() {
        return autoBuilder.fullAuto(PathPlanner.loadPathGroup("Test Reverse",
                new PathConstraints(4, 3)));
    }

    public Command autonChassisDriveDist() {
            return new ChassisDriveDist(
                            chassis,
                            ChassisConstants.kAutonBalanceDist,
                            5.0);
    };

    public Command autonChassisLevel() {
            return new ChassisLevel(
                            chassis,
                            ChassisConstants.kAutonBalanceLevel,
                            10.0);
    };

    public Command autonBalance() {
            return new AutonBalance(
                            RobotContainer.chassis, RobotContainer.crane, 
                            RobotContainer.craneTurret, 
                            RobotContainer.craneTilt, 
                            RobotContainer.craneArm,
                            RobotContainer.claw, RobotContainer.intake);
                        //     chassis,
                        //     ChassisConstants.kAutonBalanceLevel,
                        //     10.0);
    };

    public Command autonChgStnDrive() {
            return new AutonChgStnDrive(chassis);
    }

    // public Command threePiece() {
    // return autoBuilder.fullAuto(PathPlanner.loadPathGroup("ThreePiece",
    // new PathConstraints(4, 3)));
    // }

    // public Command coneBalance() {
    // return autoBuilder.fullAuto(PathPlanner.loadPathGroup("ConePark",
    // new PathConstraints(4, 3)));
    // }

    // public Command twoPieceBalance() {
    // return autoBuilder.fullAuto(PathPlanner.loadPathGroup("TwoPieceBalance",
    // new PathConstraints(4, 3)));
    // }

    // public Command threePiecePlace() {
    // return autoBuilder.fullAuto(PathPlanner.loadPathGroup("ThreePieceMidPlace",
    // new PathConstraints(4, 4)));
    // }

    /**
     * Blank Autonomous to be used as default dashboard option
     * 
     * @return Autonomous command
     */
    public Command none() {
        return Commands.none();
    }

    public void init() {

        autoBuilder = new RamseteAutoBuilder(
                chassis::getPose, // Pose2d supplier
                chassis::resetPose, // Pose2d consumer, used to reset odometry at the beginning of auto
                new RamseteController(ChassisConstants.kRamseteB, ChassisConstants.kRamseteZeta),
                chassis.kinematics, // SwerveDriveKinematics
                new SimpleMotorFeedforward(ChassisConstants.kS,
                       ChassisConstants.kV),
                //        ChassisConstants.kA),
                chassis::getWheelSpeeds,
                new PIDConstants(ChassisConstants.kP,
                                        ChassisConstants.kI,
                                        ChassisConstants.kD),
                 // PID constants to correct for rotation
                // error (used to create the rotation
                // // controller)
                chassis::driveTankVolts, // Module states consumer used to output to the drive subsystem
                eventMap,
                true, // Should the path be automatically mirrored depending on alliance
                // color.
                // Optional, defaults to true
                chassis // The drive subsystem. Used to properly set the requirements of path following
                        // commands
        );

        // 2.10773229598999
/*        autonChassisDriveDist = new AutonChassisDriveDist(
                chassis,
                ChassisConstants.kAutonBalanceDist,
                5.0);
*/
        Command autonPlace = new AutonInitialMove2Node(
                        chassis, RobotContainer.crane, RobotContainer.craneTurret,
                        RobotContainer.craneTilt, RobotContainer.craneArm, RobotContainer.claw, RobotContainer.intake);

        Command autonPlaceMobilitySStn = new AutonPlaceMobilitySStn(
                        chassis, RobotContainer.crane, RobotContainer.craneTurret,
                        RobotContainer.craneTilt, RobotContainer.craneArm, RobotContainer.claw, RobotContainer.intake);

        Command autonPlaceMobilityWall = new AutonPlaceMobilityWall(
                        chassis, RobotContainer.crane, RobotContainer.craneTurret,
                        RobotContainer.craneTilt, RobotContainer.craneArm, RobotContainer.claw, RobotContainer.intake);

        Command autonPlaceBalance = new AutonPlaceBalance(
                        chassis, RobotContainer.crane, RobotContainer.craneTurret,
                        RobotContainer.craneTilt, RobotContainer.craneArm, RobotContainer.claw, RobotContainer.intake);

        compTab.add("Auton Command", chooser)
                .withWidget("ComboBox Chooser")
                .withPosition(0, 0).withSize(4, 1);

        // ==============================================================================
        // Add commands to the autonomous command chooser
        chooser.setDefaultOption("None", none());
        chooser.addOption("Drive Balance", autonBalance());
        chooser.addOption("Balance Dist", autonChassisDriveDist());
        chooser.addOption("Balance Pitch", autonChgStnDrive());
        chooser.addOption("Level", autonChassisLevel());
        chooser.addOption("Score", autonPlace);
        chooser.addOption("Score Mobility (SStn)", autonPlaceMobilitySStn);
        chooser.addOption("Score Mobility (Wall)", autonPlaceMobilityWall);
        chooser.addOption("Score Balance", autonPlaceBalance);
        chooser.addOption("Score Two Elements", scoreTwoElements());
        chooser.addOption("Score Balance", scoreBalance());
        chooser.addOption("Score Three Elements", scoreThreeElements());
        chooser.addOption("Score Mobility Balance", scoreMobilityBalance());
        chooser.addOption("Test Zig-Zag", testZigZag());
        chooser.addOption("Test 2m Forward", test2mForward());
        chooser.addOption("Test 3m Forward", test3mForward());
        chooser.addOption("Test 3.5m Forward", test35mForward());
        chooser.addOption("Test 4m Forward", test4mForward());
        // chooser.addOption("Test Reverse", testReverse());
    }


}
