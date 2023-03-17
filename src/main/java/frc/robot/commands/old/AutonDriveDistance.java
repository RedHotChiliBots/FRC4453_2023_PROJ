// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.old;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ChassisConstants;
import frc.robot.subsystems.Chassis;

public class AutonDriveDistance extends CommandBase {
  Chassis chassis = null;
  double motorSpd = 0.0;

  /** Creates a new LevelChargingStation. */
  public AutonDriveDistance(Chassis chassis) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(chassis);
    this.chassis = chassis;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    motorSpd = chassis.driveDistance();

    chassis.getMotorData();
    chassis.setMotorData(0.25, 0.25);

    chassis.resetEncoders();
    chassis.setDistSetPoint(ChassisConstants.kAutonMobilityDist);

    motorSpd = chassis.driveDistance();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    motorSpd = chassis.driveDistance();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.setMotorData();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
