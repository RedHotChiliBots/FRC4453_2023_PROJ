// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

public class AutonChgStnLevel extends CommandBase {
  Chassis chassis = null;
  double motorSpd = 0.0;
  double counter = 0.0;
  int printCount = 0;

  /** Creates a new LevelChargingStation. */
  public AutonChgStnLevel(Chassis chassis) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(chassis);
    this.chassis = chassis;
  }

  private void printStat(String method, boolean force) {
    if (printCount == 0) {
      System.out.println("    Counter   TimeStamp   Avg Pitch   Avg Rate   Speed");
    }
    if (printCount++ % 10 == 0 || force) {
      String timeStamp = chassis.timeStamp.format(System.currentTimeMillis());
      System.out.printf("%s  %03d   %s   %9.3f   %9.3f   %9.3f\n", method, printCount, timeStamp,
          chassis.lib.getAvgPitch(), chassis.lib.getAvgRate(), motorSpd);
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    printCount = 0;
    motorSpd = chassis.levelChargingStation();
    printStat("INIT", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    motorSpd = chassis.levelChargingStation();
    printStat("EXEC", false);
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
