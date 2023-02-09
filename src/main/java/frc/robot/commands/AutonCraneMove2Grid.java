// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CraneConstants;
import frc.robot.subsystems.Crane;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonCraneMove2Grid extends SequentialCommandGroup {
  /** Creates a new AutonCranePos. */
  public AutonCraneMove2Grid(Crane crane) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    // Assume start with Element in Claw and in Chassis "U"
    addCommands(
      // Lift Claw out of "U"
      new CraneArm2Pos(crane, CraneConstants.kCraneArmClear),
      // Tilt Arm further out of "U
      new CraneTilt2Pos(crane, CraneConstants.kCraneTiltClear),
      // Rotate Turret 180 deg
      new CraneTurret2Pos(crane, CraneConstants.kCraneTurretGridSide)
    );
  }
}
