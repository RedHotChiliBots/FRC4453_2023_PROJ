// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.GridCalcs.CRANESTATE;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Crane;
import frc.robot.subsystems.CraneArm;
import frc.robot.subsystems.CraneTilt;
import frc.robot.subsystems.CraneTurret;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Claw.FingerState;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html 1TSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS7DDDDDDDDDDDDDDDDDDDDDDDDE48ZER.8/I8'

public class AutonInitialMove2Node extends SequentialCommandGroup {
  Chassis chassis;
  Crane crane;
  CraneTurret craneTurret;
  CraneTilt craneTilt;
  CraneArm craneArm;
  Claw claw;
  Intake intake;

  /** Creates a new AutonCranePos. */
  public AutonInitialMove2Node(Chassis chassis, Crane crane, CraneTurret craneTurret, CraneTilt craneTilt,
      CraneArm craneArm, Claw claw, Intake intake) {
    this.chassis = chassis;
    this.crane = crane;
    this.craneTurret = craneTurret;
    this.craneTilt = craneTilt;
    this.craneArm = craneArm;
    this.claw = claw;
    this.intake = intake;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    // Assume start with
    // Chassis "U" facing Grid
    // Cone in Claw
    // Crane in Stow
    addCommands(
        // Move Crane to Scoring position, release Cone, and return to Receive position
        // new Crane_Move2ReceivePos(crane, craneTurret, craneTilt, craneArm),
//        new IntakeArm(intake, ArmState.OPEN),
//        new Crane_Auton2NodePos(crane, craneTurret, craneTilt, craneArm),
        new Crane_Move2Position(crane, craneTurret, craneTilt, craneArm, CRANESTATE.NODE),

        new Crane_PlaceElement(crane, craneTurret, craneTilt, craneArm),
        new ClawFinger(claw, FingerState.RELEASE),
        new WaitCommand(0.1),
  //      new Crane_Move2ReceivePos(crane, craneTurret, craneTilt, craneArm));
        new Crane_Move2Position(crane, craneTurret, craneTilt, craneArm, CRANESTATE.RECEIVE));
  }
}