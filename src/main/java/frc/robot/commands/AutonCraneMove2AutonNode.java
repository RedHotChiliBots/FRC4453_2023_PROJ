// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Crane;
import frc.robot.subsystems.CraneArm;
import frc.robot.subsystems.CraneTilt;
import frc.robot.subsystems.CraneTurret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonCraneMove2AutonNode extends ParallelCommandGroup {
  /** Creates a new AutonCranePos. */
  public AutonCraneMove2AutonNode(Chassis chassis, Crane crane, CraneTurret craneTurret, CraneTilt craneTilt,
      CraneArm craneArm, Claw claw) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    // Assume start with
    // Chassis "U" facing Grid
    // Cone in Claw
    // Intake Closed; Motor Stop
    // Crane in Stow
    addCommands(
        // Move Crane to Scoring position, release Cone, and return to Receive position
        new Crane_Move2AutonNodePos(crane, craneTurret, craneTilt, craneArm),
        new AutonCraneScoreAtNode(crane, craneTurret, craneTilt, craneArm, claw),
        // Drive over Charging Station
        new AutonChassisDriveDist(chassis),
        // Turn 180 degrees
        new AutonChassisDriveTurn(chassis),
        // Drive up to Charging Station and Balance
        new AutonChassisDrive2ChgStn(chassis));
  }
}
