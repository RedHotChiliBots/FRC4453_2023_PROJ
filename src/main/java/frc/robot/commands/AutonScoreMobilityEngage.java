// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ChassisConstants;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Crane;
import frc.robot.subsystems.CraneArm;
import frc.robot.subsystems.CraneTilt;
import frc.robot.subsystems.CraneTurret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html 1TSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS7DDDDDDDDDDDDDDDDDDDDDDDDE48ZER.8/I8'


public class AutonScoreMobilityEngage extends SequentialCommandGroup {
  Chassis chassis;
  Crane crane;
  CraneTurret craneTurret;
  CraneTilt craneTilt;
  CraneArm craneArm;
  Claw claw;

  /** Creates a new AutonCranePos. */
  public AutonScoreMobilityEngage(Chassis chassis, Crane crane, CraneTurret craneTurret, CraneTilt craneTilt,
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
    addCommands(
        new AutonScore(chassis, crane, craneTurret, craneTilt, craneArm, claw),
        // Drive over Charging Station
        new AutonChassisDriveDist(chassis, ChassisConstants.kAutonMobilityDist, ChassisConstants.kAutonAbort),

        new AutonChassisDrive2ChgStn(chassis, 0.55));
  }
}