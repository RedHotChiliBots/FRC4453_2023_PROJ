// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Crane;
import frc.robot.subsystems.CraneArm;
import frc.robot.subsystems.CraneTilt;
import frc.robot.subsystems.CraneTurret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html 1TSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS7DDDDDDDDDDDDDDDDDDDDDDDDE48ZER.8/I8'


public class AutonScore extends SequentialCommandGroup {
  Chassis chassis;
  Crane crane;
  CraneTurret craneTurret;
  CraneTilt craneTilt;
  CraneArm craneArm;
  Claw claw;

  /** Creates a new AutonCranePos. */
  public AutonScore(Chassis chassis, Crane crane, CraneTurret craneTurret, CraneTilt craneTilt,
      CraneArm craneArm, Claw claw) {
    this.chassis = chassis;
    this.crane = crane;
    this.craneTurret = craneTurret;
    this.craneTilt = craneTilt;
    this.craneArm = craneArm;
    this.claw = claw;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    // Assume start with
    // Chassis "U" facing Grid
    // Cone in Claw
    // Crane in Stow
//    addCommands(
        // Move Crane to Scoring position, release Cone, and return to Receive position
		    // new Crane_Move2ReadyPos(crane, craneTurret, craneTilt, craneArm),
		    // new Crane_Move2NodePos(crane, craneTurret, craneTilt, craneArm),
        // new AutonCraneScoreAtNode(crane, craneTurret, craneTilt, craneArm, claw));   
  }
}