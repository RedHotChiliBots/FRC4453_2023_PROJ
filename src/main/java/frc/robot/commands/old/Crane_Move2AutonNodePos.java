// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.old;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.CraneConstants;
import frc.robot.GridCalcs.CRANESTATE;
import frc.robot.subsystems.Crane;
import frc.robot.subsystems.CraneArm;
import frc.robot.subsystems.CraneTilt;
import frc.robot.subsystems.CraneTurret;

public class Crane_Move2AutonNodePos extends CommandBase {
  Crane crane;
  CraneTurret craneTurret;
  CraneTilt craneTilt;
  CraneArm craneArm;
  int state = 0;
  boolean finish = false;
  CRANESTATE origState;
  CRANESTATE tgtState;

  /** Creates a new CraneMove2Pos. */
  public Crane_Move2AutonNodePos(Crane crane, CraneTurret craneTurret, CraneTilt craneTilt, CraneArm craneArm) {
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
    tgtState = CRANESTATE.NODE;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    switch (state) {
      // If already at Node, then finish, else Rotate only when Tilt and Arm are clear
      case 0:
        // If already at Node, do nothing
        if (crane.getState() == CRANESTATE.STOW) {
          DriverStation.reportWarning("At Stow", false);
          state++;
        }
        break;

      // Move Tilt and Turret to Auton Scoring Postion
      case 1:
        craneTilt.setSetPoint(CraneConstants.kAutonTiltPos);
        craneTurret.setSetPoint(CraneConstants.kAutonTurretPos);
        state++;
        break;

      // If Turret and Tilt are in Node pos, move Arm Auton Scoring Position
      case 2:
        if (craneTurret.atSetPoint() && craneTilt.atSetPoint()) {
          craneArm.setSetPoint(CraneConstants.kAutonArmPos);
          state++;
        }
        break;

      // If Crane is at Auton Node Position, then finished
      case 3:
        if (craneTurret.atSetPoint() &&
            craneTilt.atSetPoint() &&
            craneArm.atSetPoint()) {
          crane.setState(CRANESTATE.NODE);
          DriverStation.reportWarning("Crane_Move2AutonNodePos finish in Auton", false);

          finish = true;
        }
        break;
    }
    // strLog.append(
    //     String.format("From: %s, To: %s, Curr: %s.  State %d. Turret %s:%s, Tilt %s:%s, Arm %s:%s\n",
    //         origState, tgtState, crane.getState(), state,
    //         craneTurret.atSetPoint() ? "SP" : String.format("%7.3f", craneTurret.getPosition()),
    //         String.format("%7.3f", craneTurret.getSetPoint()),
    //         craneTilt.atSetPoint() ? "SP" : String.format("%6.3f", craneTilt.getPosition()),
    //         String.format("%6.3f", craneTilt.getSetPoint()),
    //         craneArm.atSetPoint() ? "SP" : String.format("%6.3f", craneArm.getPosition()),
    //         String.format("%6.3f", craneArm.getSetPoint())));
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
