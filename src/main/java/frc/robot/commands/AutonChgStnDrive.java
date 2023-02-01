// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

public class AutonChgStnDrive extends CommandBase {
  private Chassis chassis;
  private double avgPitch;
  private double currPos;
  private double motorSpd;
  private Timer timer;
  private int printCount = 0;
  private boolean oneTime = false;

  /** Creates a new AutonDrivePitch. */
  public AutonChgStnDrive(Chassis chassis) {
    this.chassis = chassis;
    addRequirements(chassis);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  private void printStat(String method, boolean force) {
    if (printCount == 0) {
      System.out.println("    Counter   TimeStamp   Avg Pitch   Avg Rate   Speed   Position   Tip Switch");
    }
    if (printCount++ % 10 == 0 || force) {
      String timeStamp = chassis.timeStamp.format(System.currentTimeMillis());
      System.out.printf("%s  %03d   %s   %10.4f   %10.4f   %10.4f   %10.4f   %s\n", method, printCount, timeStamp,
      chassis.lib.getAvgPitch(), chassis.lib.getAvgRate(), motorSpd, currPos, chassis.lib.getTipSwitch()?"True":"False");
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    motorSpd = 0.35;
    oneTime = false;
    printCount = 0;
    timer = new Timer();
    timer.reset();
    timer.start();
    avgPitch = Math.abs(chassis.lib.getAvgPitch());
    currPos = chassis.leftEncoder.getPosition();

    printStat("Init", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    avgPitch = Math.abs(chassis.lib.getAvgPitch());
    currPos = chassis.leftEncoder.getPosition();
    if (timer.hasElapsed(3) && chassis.lib.getTipSwitch()) {
      if (!oneTime) {
        printStat("TIP", true);
        oneTime = true;
      }
      motorSpd = 0.25;
      if (avgPitch < 8.0) {
        motorSpd = 0.05;
      } else if (avgPitch < 12.0) {
        motorSpd = 0.1;
      }
    }
    chassis.driveArcade(-motorSpd, 0.0);

    printStat("Exec", false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    avgPitch = chassis.lib.getAvgPitch();
    currPos = chassis.leftEncoder.getPosition();

    printStat("End", true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    avgPitch = Math.abs(chassis.lib.getAvgPitch());
    return (timer.hasElapsed(3) && avgPitch < 1.0);
  }
}
