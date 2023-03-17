// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.old;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CraneTurret;

public class CraneTurret2Pos extends CommandBase {
  private CraneTurret craneTurret;

  /** Creates a new CraneNodeChooser. */
  public CraneTurret2Pos(CraneTurret craneTurret) {
    this.craneTurret = craneTurret;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(craneTurret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    craneTurret.setSetPoint(craneTurret.getSetPoint());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
