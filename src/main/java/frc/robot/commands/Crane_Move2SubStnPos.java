// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.GridCalcs;
import frc.robot.Constants.CraneConstants;
import frc.robot.GridCalcs.C;
import frc.robot.GridCalcs.CRANESTATE;
import frc.robot.subsystems.Crane;
import frc.robot.subsystems.CraneArm;
import frc.robot.subsystems.CraneTilt;
import frc.robot.subsystems.CraneTurret;

public class Crane_Move2SubStnPos extends CommandBase {
  Crane crane;
  CraneTurret craneTurret;
  CraneTilt craneTilt;
  CraneArm craneArm;
  int state = 0;
  boolean finish = false;
  GridCalcs grid = new GridCalcs();

  /** Creates a new CraneMove2Pos. */
  public Crane_Move2SubStnPos(Crane crane, CraneTurret craneTurret, CraneTilt craneTilt, CraneArm craneArm) {
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
        // If already at Node, do nothing
        if (crane.getState() == CRANESTATE.SUBSTATION &&
            craneTurret.atTurretNextPoint() &&
            craneTilt.atTiltNextPoint() &&
            craneArm.atArmNextPoint()) {
          DriverStation.reportWarning("Already at SubStation", false);
          finish = true;

          // If Rotating within Grid or at Ready pos, Turret = Node pos, Tilt = Node pos
        } else if (crane.getState() == CRANESTATE.STOW || crane.getState() == CRANESTATE.HOLD) {
          craneTurret.setTurretSetPoint(grid.getSubStationPos(C.TURRET));
          craneTilt.setTiltSetPoint(grid.getSubStationPos(C.TILT));
          craneArm.setArmSetPoint(grid.getSubStationPos(C.ARM));
          state++;
          crane.setState(CRANESTATE.MOVING);
          DriverStation.reportWarning("Moving to new Node", false);
        }

        DriverStation.reportWarning("Current Pos: " + craneTurret.getTurretPosition() + "   Target Pos: " + crane
            .getGridX(), false);
        DriverStation.reportWarning("Next State " + state, false);
        break;

      // If Crane is in Node position, then finished
      case 1:
        if (craneTurret.atTurretSetPoint() &&
            craneTilt.atTiltSetPoint() &&
            craneArm.atArmSetPoint()) {
          crane.setState(CRANESTATE.SUBSTATION);
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
