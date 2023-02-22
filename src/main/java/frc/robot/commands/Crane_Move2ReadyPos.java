// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.CraneConstants;
import frc.robot.GridCalcs.CRANESTATE;
import frc.robot.subsystems.Crane;
import frc.robot.subsystems.CraneArm;
import frc.robot.subsystems.CraneTilt;
import frc.robot.subsystems.CraneTurret;

public class Crane_Move2ReadyPos extends CommandBase {
  Crane crane;
  CraneTurret craneTurret;
  CraneTilt craneTilt;
  CraneArm craneArm;
  int state = 0;
  boolean finish = false;

  /** Creates a new CraneMove2Pos. */
  public Crane_Move2ReadyPos(Crane crane, CraneTurret craneTurret, CraneTilt craneTilt, CraneArm craneArm) {
    this.crane = crane;
    this.craneTurret = craneTurret;
    this.craneTilt = craneTilt;
    this.craneArm = craneArm;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    state = 0;
    finish = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (state) {
      case 0:
        if (crane.getState() == CRANESTATE.READY) {
          finish = true;
        } else {
          crane.setState(CRANESTATE.MOVING);
          state++;
        }
        break;

      case 1:
        craneTilt.setTiltSetPoint(CraneConstants.kTiltReadyPos);
        craneArm.setArmSetPoint(CraneConstants.kArmReadyPos);
        state++;
        break;

      case 2:
        if ((craneTilt.getTiltPosition() == CraneConstants.kTiltReadyPos) && (craneArm.getArmPosition() == CraneConstants.kArmReadyPos)) {
          craneTurret.setTurretSetPoint(180.0);
          state++;
        }
        break;

      case 3:
        if (craneTurret.getTurretPosition() == CraneConstants.kTurretReadyPos) {
          crane.setState(CRANESTATE.READY);
          finish = true;
        }
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
