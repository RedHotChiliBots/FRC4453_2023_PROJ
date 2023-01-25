// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

public class AutonChgStnRate extends CommandBase {
  Chassis chassis = null;
  double rate = 0.0;
  double currPitch;

  /** Creates a new LevelChargingStation. */
  public AutonChgStnRate(Chassis chassis) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(chassis);
    this.chassis = chassis;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currPitch = chassis.getPitch();
    rate = chassis.rateChargingStation();
    String timeStamp = chassis.timeStamp.format(System.currentTimeMillis());
    System.out.println(timeStamp + "   Start Rate: Pitch: " + currPitch + "   Rate: " + rate);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rate = chassis.rateChargingStation();
  }

  // Called once the command ends or is interrupted.
  public void end(boolean interrupted) {
    String timeStamp = chassis.timeStamp.format(System.currentTimeMillis());
    System.out.println(timeStamp + "   End Rate: Pitch: " + currPitch + "   Rate: " + rate);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    currPitch = chassis.getPitch();
    return Math.abs(currPitch) < 1.0;
  }
}
