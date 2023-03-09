// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ChassisConstants;
import frc.robot.subsystems.Chassis;

public class AutonChassisDriveDist extends CommandBase {
  /** Creates a new ChassisTankDrive. */

  private Chassis chassis;
  private Timer timer = new Timer();

  public AutonChassisDriveDist(Chassis chassis) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.chassis = chassis;

    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    chassis.resetEncoders();
    chassis.setDistSetPoint(ChassisConstants.kAutonStraightDist);
    timer.reset();
    timer.start();
    DriverStation.reportWarning("AutonChassisDriveDist finish Initialize", false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("From command");
    chassis.driveDistPosition(ChassisConstants.kAutonStraightDist);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriverStation.reportWarning("AutonChassisDriveDist End", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return chassis.atTarget() || timer.hasElapsed(ChassisConstants.kAutonAbort);
  }
}