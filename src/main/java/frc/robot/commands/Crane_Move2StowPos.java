// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.CraneConstants;
import frc.robot.subsystems.Crane;
import frc.robot.subsystems.CraneArm;
import frc.robot.subsystems.CraneTilt;
import frc.robot.subsystems.CraneTurret;
import frc.robot.subsystems.Crane.CraneState;

public class Crane_Move2StowPos extends CommandBase {
  Crane crane;
  CraneTurret craneTurret;
  CraneTilt craneTilt;
  CraneArm craneArm;
  int state = 0;
  boolean finish = false;

  /** Creates a new CraneMove2Pos. */
  public Crane_Move2StowPos(Crane crane, CraneTurret craneTurret, CraneTilt craneTilt, CraneArm craneArm) {
    this.crane = crane;
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
        if (crane.getState() == CraneState.STOW) {
          finish = true;
        } else {
          craneTilt.setTiltSetPoint(CraneConstants.kTiltRotatePos);
          craneArm.setArmSetPoint(CraneConstants.kArmStowPos);
          crane.setState(CraneState.MOVING);
          state++;
        }
        break;

      case 1:
        if ((craneTilt.getTiltPosition() == CraneConstants.kTiltRotatePos)
            && (craneArm.getArmPosition() == CraneConstants.kArmStowPos)) {
          craneTurret.setTurretSetPoint(CraneConstants.kTurretStowPos);
          state++;
        }
        break;

      case 2:
        if (craneTurret.getTurretPosition() == CraneConstants.kTurretStowPos) {
          craneTilt.setTiltSetPoint(CraneConstants.kTiltStowPos);
          state++;
        }
        break;

      case 3:
        if (craneTilt.getTiltPosition() == CraneConstants.kTiltStowPos) {
          crane.setState(CraneState.STOW);
          finish = true;
        }
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
