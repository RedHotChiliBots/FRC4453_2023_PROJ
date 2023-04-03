// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.GridCalcs.CRANESTATE;
import frc.robot.subsystems.Crane;
import frc.robot.subsystems.CraneArm;
import frc.robot.subsystems.CraneTilt;
import frc.robot.subsystems.CraneTurret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TeleopMove2Node extends ParallelCommandGroup {
  /** Creates a new AutonCranePos. */
  public TeleopMove2Node(Crane crane, CraneTurret craneTurret, CraneTilt craneTilt, CraneArm craneArm) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    // Assume start with Element in Claw and in Chassis "U"
    addCommands(
        // In Parallel, Set Turret and Tilt position
        // Also in same Parallel, Run Sequetial Wait then Set Arm position
//        new Crane_Move2ReadyPos(crane, craneTurret, craneTilt, craneArm),
        new Crane_Move2Position(crane, craneTurret, craneTilt, craneArm, CRANESTATE.NODE));
  }
}
