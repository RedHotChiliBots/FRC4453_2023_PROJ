// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Crane;

public class CraneTurret2Pos extends CommandBase {
  private Crane crane;
  private double pos;

  /** Creates a new CraneNodeChooser. */
  public CraneTurret2Pos(Crane crane, Double pos) {
    this.crane = crane;
    this.pos = pos;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pos = crane.getGridX();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    crane.setTurretSetPoint(pos);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return crane.atTurrentSetPoint();
  }
}
