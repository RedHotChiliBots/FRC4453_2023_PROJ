// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
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
  CRANESTATE origState;
  CRANESTATE tgtState;

  /** Creates a new CraneMove2Pos. */
  public Crane_Move2ReadyPos(Crane crane, CraneTurret craneTurret, CraneTilt craneTilt, CraneArm craneArm) {
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
    origState = crane.getState();
    tgtState = CRANESTATE.SUBSTATION;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (state) {
      // If already at Node, then finish, else Rotate only when Tilt and Arm are clear
      case 0:
        // If already at Ready, do nothing
        if (crane.getState() == CRANESTATE.READY) {
          DriverStation.reportWarning("Already in Ready position", false);
          finish = true;

        } else if (crane.getState() == CRANESTATE.NODE) {
          craneArm.setSetPoint(CraneConstants.kArmReadyPos);
          DriverStation.reportWarning("In Node position, moving to Ready", false);
          state++;
          crane.setState(CRANESTATE.MOVING);

          // If Rotating from Elem side to Grid side, Arm = Safe Rptate, Tilt = Safe
          // Rotate
        } else if (Math.abs(craneTurret.getPosition() - CraneConstants.kCraneTurretGridSide) > 90.0) {
          craneTilt.setSetPoint(CraneConstants.kTiltSafe2Rotate);
          craneArm.setSetPoint(CraneConstants.kArmSafe2Rotate);
          DriverStation.reportWarning("Preparing Arm for Safe Move", false);
          state++;
          crane.setState(CRANESTATE.MOVING);
        }
        break;

      // If Tilt and Arm are in Safe positions, Rotate Turret to just outside Nodes
      case 1:
        if (craneTilt.atSetPoint() && craneArm.atSetPoint()) {
          craneTurret.setSetPoint(CraneConstants.kTurretReadyPos);
          craneTilt.setSetPoint(CraneConstants.kTiltReadyPos);
          state++;
        }
        break;

      // If Tilt and Arm are in Safe positions, Rotate Turret to just outside Nodes
      case 2:
        if (craneTurret.atSetPoint() && craneTilt.atSetPoint()) {
          craneArm.setSetPoint(CraneConstants.kArmReadyPos);
          state++;
        }
        break;

      // If Turret and Tilt are in Node pos, move Arm to Ready pos
      case 3:
        if (craneTurret.atSetPoint() &&
            craneTilt.atSetPoint() &&
            craneArm.atSetPoint()) {
          crane.setState(CRANESTATE.READY);
          finish = true;
        }
        break;
    }

    System.out.printf("From: %s, To: %s, Curr: %s.  State %d. Turret %s:%s, Tilt %s:%s, Arm %s:%s\n",
        origState, tgtState, crane.getState(), state,
        craneTurret.atSetPoint() ? "SP" : String.format("%7.3", craneTurret.getPosition()),
          String.format("%7.3", craneTurret.getSetPoint()),
        craneTilt.atSetPoint() ? "SP" : String.format("%6.3", craneTilt.getPosition()),
          String.format("%6.3", craneTilt.getSetPoint()),
        craneArm.atSetPoint() ? "SP" : String.format("%6.3", craneArm.getPosition()),
          String.format("%6.3", craneArm.getSetPoint()));
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
