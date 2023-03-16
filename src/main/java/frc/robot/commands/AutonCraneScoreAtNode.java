// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Crane;
import frc.robot.subsystems.CraneArm;
import frc.robot.subsystems.CraneTilt;
import frc.robot.subsystems.CraneTurret;
import frc.robot.subsystems.Claw.FingerState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonCraneScoreAtNode extends SequentialCommandGroup {
  /** Creates a new AutonCranePos. */
  public AutonCraneScoreAtNode(Crane crane, CraneTurret craneTurret, CraneTilt craneTilt, CraneArm craneArm,
      Claw claw) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    // Assume start with Element in Claw and in Chassis "U"
    addCommands(
        // new Crane_Move2ElemClear(craneTilt, craneArm);
        // new Crane_Move2NodePos(crane, craneTurret, craneTilt, craneArm),
        new Crane_PlaceElement(crane, craneTurret, craneTilt, craneArm),
        new ClawFinger(claw, FingerState.RELEASE),
        new WaitCommand(0.2),
        new Crane_Move2ReceivePos(crane, craneTurret, craneTilt, craneArm));
  }
}
