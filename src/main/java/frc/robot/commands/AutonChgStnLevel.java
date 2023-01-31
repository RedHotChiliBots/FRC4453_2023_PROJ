// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

public class AutonChgStnLevel extends CommandBase {
  Chassis chassis = null;
  double pitch = 0.0;
  double counter = 0.0;

  /** Creates a new LevelChargingStation. */
  public AutonChgStnLevel(Chassis chassis) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(chassis);
    this.chassis = chassis;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    chassis.getMotorData();
    chassis.setMotorData(0.2, 02);

    pitch = chassis.levelChargingStation();
    String timeStamp = chassis.timeStamp.format(System.currentTimeMillis());
    System.out.println(timeStamp + "   Start Level: Pitch: " + pitch);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pitch = chassis.levelChargingStation();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.setMotorData();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if ((counter++ % 50) == 0.0) {
      String timeStamp = chassis.timeStamp.format(System.currentTimeMillis());
      System.out.println(timeStamp + "   Level: [" + counter + "]  Pitch: " + pitch);
    }
    return false;
  }
}
