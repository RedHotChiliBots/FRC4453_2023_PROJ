// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.CraneConstants;
import frc.robot.Constants.E;
import frc.robot.GridCalcs.CRANESTATE;
import frc.robot.subsystems.Crane;
import frc.robot.subsystems.CraneArm;
import frc.robot.subsystems.CraneTilt;
import frc.robot.subsystems.CraneTurret;

public class Crane_Move2GripPos extends CommandBase {
  Crane crane;
  CraneTurret craneTurret;
  CraneTilt craneTilt;
  CraneArm craneArm;
  int state = 0;
  boolean finish = false;

  /** Creates a new CraneMove2Pos. */
  public Crane_Move2GripPos(Crane crane, CraneTurret craneTurret, CraneTilt craneTilt, CraneArm craneArm) {
    this.crane = crane;
    this.craneTurret = craneTurret;
    this.craneTilt = craneTilt;
    this.craneArm = craneArm;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(crane);
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
      // If already at Node, then finish, else Rotate only when Tilt and Arm are clear
      case 0:
        // If already at Ready, do nothing
        if (crane.getState() == CRANESTATE.RECEIVE) {
          DriverStation.reportWarning("Already in Receive position", false);
          if (crane.getElem() == E.CONE) {
            craneArm.setArmSetPoint(CraneConstants.kArmGripCone);

          } else if (crane.getElem() == E.CONE) {
            craneArm.setArmSetPoint(CraneConstants.kArmGripCube);
          }
          state++;
          crane.setState(CRANESTATE.MOVING);
          
        } else {
          DriverStation.reportError("Arm must be in Receive position", false);
          finish = true;
        }

       // If Turret and Tilt are in Node pos, move Arm to Ready pos
      case 1:
        if (craneTurret.atTurrentSetPoint() &&
            craneTilt.atTiltSetPoint() &&
            craneArm.atArmSetPoint()) {
          crane.setState(CRANESTATE.GRIP);
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
