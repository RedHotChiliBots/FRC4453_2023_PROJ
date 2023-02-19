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

public class Crane_Move2NodePos extends CommandBase {
  Crane crane;
  CraneTurret craneTurret;
  CraneTilt craneTilt;
  CraneArm craneArm;
  int state = 0;
  boolean finish = false;

  /** Creates a new CraneMove2Pos. */
  public Crane_Move2NodePos(Crane crane, CraneTurret craneTurret, CraneTilt craneTilt, CraneArm craneArm) {
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
      // If already at Node, then finish, else Retract Arm and check Tilt will clear Chassis
      case 0:
        if (crane.getState() == CraneState.NODE &&
            (craneTurret.getTurretPosition() == crane.getGridX() + CraneConstants.kTurretNodePos) &&
            craneTilt.getTiltPosition() == crane.getGridZ() &&
            craneArm.getArmPosition() == crane.getGridY()) {
          finish = true;
        } else {
          craneArm.setArmSetPoint(CraneConstants.kArmInitPos);
            
          if (craneTilt.getTiltPosition() < CraneConstants.kTiltClearChassisPos) {
            craneTilt.setTiltSetPoint(CraneConstants.kTiltClearChassisPos);
            crane.setState(CraneState.MOVING);
          }
          state++;
        }
        break;

      // If Tilt is below ClearChassis, raise to ClearChassis
      case 1:
        if (craneTilt.getTiltPosition() >= CraneConstants.kTiltClearChassisPos) {
          crane.setState(CraneState.CLEAR2MOVE);
          state++;
        }
        break;

      // If Crane is in Clear2Move, then move Turret and Tilt to Node's position
      case 2:
        if (crane.getState() == CraneState.CLEAR2MOVE) {
          finish = true;
        } else {
          craneTurret.setTurretSetPoint(crane.getGridX() + CraneConstants.kTurretNodePos);
          craneTilt.setTiltSetPoint(crane.getGridZ());
          crane.setState(CraneState.MOVING);
          state++;
        }
        break;

      // If Crane Turret and Tilt are positioned for Node, them move Arm
      case 3:
        if ((craneTurret.getTurretPosition() == crane.getGridX() + CraneConstants.kTurretNodePos) &&
            craneTilt.getTiltPosition() == crane.getGridZ()) {
          craneArm.setArmSetPoint(crane.getGridY());
          state++;
        }
        break;

      // If Crane Arm is at Node position, then finished
      case 4:
        if (craneArm.getArmPosition() == crane.getGridY()) {
          crane.setState(CraneState.NODE);
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
