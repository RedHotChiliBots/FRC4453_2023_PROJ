// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ChassisConstants;
import frc.robot.subsystems.Chassis;

public class AutonChassisDriveTime extends CommandBase {
  /** Creates a new ChassisTankDrive. */

  private Chassis chassis;
  private double spd;
  private double time;
  private Timer timer = new Timer();

  public AutonChassisDriveTime(Chassis chassis, double spd, double time) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.chassis = chassis;
    this.spd = spd;
    this.time = time;

    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    DriverStation.reportWarning("AutonChassisDriveTime finish Initialize", false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("From command");
    chassis.driveTank(spd, spd);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriverStation.reportWarning("AutonChassisDriveTime End", false);
    chassis.driveTank(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(time);
  }
}
