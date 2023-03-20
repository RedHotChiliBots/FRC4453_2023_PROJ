// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Crane;
import frc.robot.subsystems.CraneArm;
import frc.robot.subsystems.CraneTilt;
import frc.robot.subsystems.CraneTurret;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonPlaceBalance extends SequentialCommandGroup {
  /** Creates a new AutonPlaceMobilitySStn. */
  public AutonPlaceBalance(Chassis chassis, Crane crane, CraneTurret craneTurret, CraneTilt craneTilt, CraneArm craneArm,
      Claw claw, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new AutonInitialMove2Node(chassis, crane, craneTurret, craneTilt, craneArm, claw, intake),
//          AutonChassisDriveDist(chassis, ),
            RobotContainer.autos.autonChassisDriveDist
//          RobotContainer.autos.scoreMobilityBalance()
    );
  }
}
