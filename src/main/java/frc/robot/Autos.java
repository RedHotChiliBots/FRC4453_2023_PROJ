package frc.robot;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
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
import frc.robot.subsystems.Chassis;

public final class Autos {

    private static Chassis chassis;

    // Define a chooser for autonomous commands
    final SendableChooser<Command> chooser = new SendableChooser<>();

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

    private final RamseteAutoBuilder autoBuilder = new RamseteAutoBuilder(
            chassis::getPose, // Pose2d supplier
            chassis::resetPose, // Pose2d consumer, used to reset odometry at the beginning of auto
            new RamseteController(ChassisConstants.kRamseteB, ChassisConstants.kRamseteZeta),
            ChassisConstants.kDriveKinematics, // SwerveDriveKinematics
            new SimpleMotorFeedforward(ChassisConstants.kS, ChassisConstants.kV),
            chassis::getWheelSpeeds,
            new PIDConstants(ChassisConstants.kP,
                    ChassisConstants.kI,
                    ChassisConstants.kD), // PID constants to correct for rotation
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

    public Autos(Chassis chassis) {
        this.chassis = chassis;
    }

    public Command threePiece() {
        return autoBuilder.fullAuto(PathPlanner.loadPathGroup("ThreePiece",
                new PathConstraints(4, 3)));
    }

    public Command coneBalance() {
        return autoBuilder.fullAuto(PathPlanner.loadPathGroup("ConePark",
                new PathConstraints(4, 3)));
    }

    public Command twoPieceBalance() {
        return autoBuilder.fullAuto(PathPlanner.loadPathGroup("TwoPieceBalance",
                new PathConstraints(4, 3)));
    }

    public Command threePiecePlace() {
        return autoBuilder.fullAuto(PathPlanner.loadPathGroup("ThreePieceMidPlace",
                new PathConstraints(4, 4)));
    }

    /**
     * Blank Autonomous to be used as default dashboard option
     * 
     * @return Autonomous command
     */
    public Command none() {
        return Commands.none();
    }


    public void init() {

        compTab.add("Auton Command", chooser).withWidget("ComboBox Chooser").withPosition(0, 0).withSize(4, 1);

        // ==============================================================================
        // Add commands to the autonomous command chooser
        chooser.setDefaultOption("None", none());
    }
}
