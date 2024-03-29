// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

public class ChassisDriveSelected extends CommandBase {
  /** Creates a new ChassisTankDrive. */

  private Chassis chassis;
  private DoubleSupplier leftX;
  private DoubleSupplier leftY;
  private DoubleSupplier rightX;
  private DoubleSupplier rightY;

  public ChassisDriveSelected(Chassis chassis, DoubleSupplier leftX, DoubleSupplier leftY, DoubleSupplier rightX, DoubleSupplier rightY) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.chassis = chassis;
    this.leftX = leftX;
    this.leftY = leftY;
    this.rightX = rightX;
    this.rightY = rightY;
    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    chassis.driveSelected(leftX.getAsDouble(), leftY.getAsDouble(), rightX.getAsDouble(), rightY.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
