// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

public class ChassisDriveDist extends CommandBase {
  /** Creates a new ChassisTankDrive. */

  private Chassis chassis;
  private double dist;
  private double time;
  private Timer timer = new Timer();

  public ChassisDriveDist(Chassis chassis, double dist, double time) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.chassis = chassis;
    this.dist = dist;
    this.time = time;

    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    chassis.resetEncoders();
    chassis.setDistSetPoint(dist);
    timer.start();
    timer.reset();
    DriverStation.reportWarning("AutonChassisDriveDist finish Initialize", false);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    chassis.driveDistance();
    System.out.println("From command");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriverStation.reportWarning("AutonChassisDriveDist End", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return chassis.atDistTarget() || timer.hasElapsed(time);
  }
}
