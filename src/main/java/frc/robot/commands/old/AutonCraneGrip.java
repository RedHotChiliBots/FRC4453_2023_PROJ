// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.old;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ClawFinger;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Crane;
import frc.robot.subsystems.CraneArm;
import frc.robot.subsystems.CraneTilt;
import frc.robot.subsystems.CraneTurret;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Claw.FingerState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonCraneGrip extends SequentialCommandGroup {
  /** Creates a new AutonCranePos. */
  public AutonCraneGrip(Crane crane,
      CraneTurret craneTurret, CraneTilt craneTilt, CraneArm craneArm, Claw claw, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    // Assume start with Element in Claw and in Chassis "U"
    addCommands(
        // Lift Claw out of "U"
        new ClawFinger(claw, FingerState.GRIP),
        new WaitCommand(1.0));
  }
}
