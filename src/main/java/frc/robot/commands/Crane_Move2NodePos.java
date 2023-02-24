// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Library;
import frc.robot.Constants.CraneConstants;
import frc.robot.GridCalcs.CRANESTATE;
import frc.robot.subsystems.Crane;
import frc.robot.subsystems.CraneArm;
import frc.robot.subsystems.CraneTilt;
import frc.robot.subsystems.CraneTurret;

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
      // If already at Node, then finish, else Rotate only when Tilt and Arm are clear
      case 0:
        // If already at Node, do nothing
        if (crane.getState() == CRANESTATE.NODE &&
            (Library.approx(craneTurret.getTurretPosition(), crane.getGridX(), 1.0)) &&
            Library.approx(craneTilt.getTiltPosition(), crane.getGridZ(), 1.0) &&
            Library.approx(craneArm.getArmPosition(), crane.getGridY(), 0.5)) {
            DriverStation.reportWarning("Already at Node", false);
          finish = true;
          // If Rotating from Elem side to Grid side, Arm = STOW, Tilt = Safe Rotate
        } else if (Math.abs(craneTurret.getTurretPosition() - crane.getGridX()) > 90.0) {
          craneTilt.setTiltSetPoint(CraneConstants.kTiltSafe2Rotate);
          craneArm.setArmSetPoint(CraneConstants.kArmSafe2Rotate);
          state = 1;
          crane.setState(CRANESTATE.MOVING);
          DriverStation.reportWarning("Preparing Arm for Safe Move", false);

          // If Rotating within Grid or at Ready pos, Rotate = Node pos, Tilt = Node pos
        } else if (crane.getState() == CRANESTATE.NODE || crane.getState() == CRANESTATE.READY) {
          craneTurret.setTurretSetPoint(crane.getGridX());
          craneTilt.setTiltSetPoint(crane.getGridZ());
          state = 3;
          crane.setState(CRANESTATE.MOVING);
          DriverStation.reportWarning("Moving to new Node", false);
        }
        DriverStation.reportWarning("Current Pos: " + craneTurret.getTurretPosition() + "   Target Pos: "+ crane
            .getGridX(), false);
        DriverStation.reportWarning("Finish State " + state, false);
        finish = true;
        break;

      // If Tilt and Arm are in Safe positions, Rotate Turret to just outside Nodes
      case 1:
        if (Library.approx(craneTilt.getTiltPosition(), CraneConstants.kTiltSafe2Rotate, 1.0) &&
            Library.approx(craneArm.getArmPosition(), CraneConstants.kArmSafe2Rotate, 0.5)) {
          craneTurret.setTurretSetPoint(CraneConstants.kTurretSafe2TiltArm);
          state++;
        }
        break;

      // If Turret is in safe position, move Turret and Tilt to Node pos
      case 2:
        if (Library.approx(craneTurret.getTurretPosition(), CraneConstants.kTurretSafe2TiltArm, 1.0)) {
          craneTurret.setTurretSetPoint(crane.getGridX());
          craneTilt.setTiltSetPoint(crane.getGridZ());
          state++;
        }
        break;

      // If Turret and Tilt are in Node pos, move Arm to Node pos
      case 3:
        if (Library.approx(craneTurret.getTurretPosition(), crane.getGridX(), 1.0) &&
            Library.approx(craneTilt.getTiltPosition(), crane.getGridZ(), 1.0)) {
          craneArm.setArmSetPoint(crane.getGridY());
          state++;
        }
        break;

      // If Crane is in Node position, then finished
      case 4:
        if (Library.approx(craneArm.getArmPosition(), crane.getGridY(), 0.5)) {
          crane.setState(CRANESTATE.NODE);
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
