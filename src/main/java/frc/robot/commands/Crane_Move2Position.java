// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.GridCalcs.CRANESTATE;
import frc.robot.GridCalcs.CRANEAXIS;
import frc.robot.subsystems.Crane;
import frc.robot.subsystems.CraneArm;
import frc.robot.subsystems.CraneTilt;
import frc.robot.subsystems.CraneTurret;

public class Crane_Move2Position extends CommandBase {
  Crane crane;
  CraneTurret craneTurret;
  CraneTilt craneTilt;
  CraneArm craneArm;
  int state = 0;
  boolean finish = false;
  CRANESTATE origState;
  CRANESTATE tgtState;

  /** Creates a new CraneMove2Pos. */
  public Crane_Move2Position(Crane crane, CraneTurret craneTurret, CraneTilt craneTilt, CraneArm craneArm,
      CRANESTATE tgtState) {
    this.crane = crane;
    this.craneTurret = craneTurret;
    this.craneTilt = craneTilt;
    this.craneArm = craneArm;
    this.tgtState = tgtState;

    // Use addRequirements() here to declare subsystem dependencies.
//    addRequirements(crane);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    state = 0;
    finish = false;
    origState = crane.getState();
    DataLogManager.log(String.format("Crane_Move2Position From: %s, To: %s\n", origState, tgtState));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (origState == CRANESTATE.MOVING) {
      DataLogManager.log("MOVING:  Already running command.  Aborting.");
      finish = true;

    } else {
      // If node selected does not accept element, print error and finish
      if (Double.isNaN(crane.getGridX())) {
        DataLogManager.log("Illegal Node for Element choosen");
        finish = true;

      } else {

        DataLogManager.log(
            String.format("From: %s, To: %s, Curr: %s.  State %d. Turret %s:%s, Tilt %s:%s, Arm %s:%s\n",
                origState, tgtState, crane.getState(), state,
                craneTurret.atSetPoint() ? "SP" : String.format("%7.3f", craneTurret.getPosition()),
                String.format("%7.3f", craneTurret.getSetPoint()),
                craneTilt.atSetPoint() ? "SP" : String.format("%6.3f", craneTilt.getPosition()),
                String.format("%6.3f", craneTilt.getSetPoint()),
                craneArm.atSetPoint() ? "SP" : String.format("%6.3f", craneArm.getPosition()),
                String.format("%6.3f", craneArm.getSetPoint())));

        switch (state) {
          // If already at Node, then finish, else Rotate only when Tilt and Arm are clear
          case 0:
            // If already at Position, do nothing
            if (origState == tgtState &&
                craneTurret.atNextPoint(tgtState) &&
                craneTilt.atNextPoint(tgtState) &&
                craneArm.atNextPoint(tgtState)) {
              DataLogManager.log("Already at Position");
              finish = true;

            } else if (tgtState == CRANESTATE.GRIP &&
                origState != CRANESTATE.RECEIVE) {
              DataLogManager.log("Must be in Receive before moving to Grip");
              finish = true;

              // If Turret is not moving, move Tilt and Arm together
            } else if (craneTurret.getPosition() == crane.grid.getCranePos(tgtState, CRANEAXIS.TURRET)) {
              craneTilt.setSetPoint(crane.grid.getCranePos(tgtState, CRANEAXIS.TILT));
              craneArm.setSetPoint(crane.grid.getCranePos(tgtState, CRANEAXIS.ARM));
              DataLogManager.log("Turret not moving, move Tilt and Arm tegether");
              state = 3;
              crane.setState(CRANESTATE.MOVING);

              // If Turret move is less than 45 degrees (node to node), move Turret, Tilt, Arm
              // together
            } else if (Math
                .abs(craneTurret.getPosition() - crane.grid.getCranePos(tgtState, CRANEAXIS.TURRET)) < 45.0) {
              craneTurret.setSetPoint(crane.grid.getCranePos(tgtState, CRANEAXIS.TURRET));
              craneTilt.setSetPoint(crane.grid.getCranePos(tgtState, CRANEAXIS.TILT));
              craneArm.setSetPoint(crane.grid.getCranePos(tgtState, CRANEAXIS.ARM));
              DataLogManager.log("Turret moves < 45deg, move Turret, Tilt, ARm together");
              state = 3;
              crane.setState(CRANESTATE.MOVING);

              // If Turret move is greater than 60 degrees (front to back or side), move Tilt
              // and Arm to safe positions
              // Rotate
            } else if (Math
                .abs(craneTurret.getPosition() - crane.grid.getCranePos(tgtState, CRANEAXIS.TURRET)) > 60.0) {
              craneTilt.setSetPoint(crane.grid.getCranePos(CRANESTATE.CLEAR2MOVE, CRANEAXIS.TILT));
              craneArm.setSetPoint(crane.grid.getCranePos(CRANESTATE.CLEAR2MOVE, CRANEAXIS.ARM));
              DataLogManager.log("Turret moves > 60deg, move Tilt and Arm to Safe positions");
              state = 1;
              crane.setState(CRANESTATE.MOVING);
            }
            break;

          // If Tilt and Arm are in Safe positions, Rotate Turret to just outside Nodes
          case 1:
            if (craneTilt.atSetPoint() && craneArm.atSetPoint()) {
              craneTurret.setSetPoint(crane.grid.getCranePos(tgtState, CRANEAXIS.TURRET));
              craneTilt.setSetPoint(crane.grid.getCranePos(tgtState, CRANEAXIS.TILT));
              state++;
            }
            break;

          // If Turret and Tilt are in Node pos, move Arm to Node pos
          case 2:
            if (craneTurret.atSetPoint() && craneTilt.atSetPoint()) {
              craneArm.setSetPoint(crane.grid.getCranePos(tgtState, CRANEAXIS.ARM));
              state++;
            }
            break;

          // If Crane is in Node position, then finished
          case 3:
            if (craneTurret.atSetPoint() &&
                craneTilt.atSetPoint() &&
                craneArm.atSetPoint()) {
              crane.setState(tgtState);
              finish = true;
            }
            break;
        }
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    DataLogManager.log(
        String.format("From: %s, To: %s, Curr: %s.  State %d. Turret %s:%s, Tilt %s:%s, Arm %s:%s\n",
            origState, tgtState, crane.getState(), state,
            craneTurret.atSetPoint() ? "SP" : String.format("%7.3f", craneTurret.getPosition()),
            String.format("%7.3f", craneTurret.getSetPoint()),
            craneTilt.atSetPoint() ? "SP" : String.format("%6.3f", craneTilt.getPosition()),
            String.format("%6.3f", craneTilt.getSetPoint()),
            craneArm.atSetPoint() ? "SP" : String.format("%6.3f", craneArm.getPosition()),
            String.format("%6.3f", craneArm.getSetPoint())));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
