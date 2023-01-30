// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

public class AutonChgStnDrive extends CommandBase {
  Chassis chassis;
  double avgPitch;
  double currPos;
  double motorSpd;
  Timer timer;
  int counter = 0;

  /** Creates a new AutonDrivePitch. */
  public AutonChgStnDrive(Chassis chassis) {
    this.chassis = chassis;
    addRequirements(chassis);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    motorSpd = 0.35;
    timer = new Timer();
    timer.reset();
    timer.start();
    avgPitch = chassis.lib.getAvgPitch();
    currPos = chassis.leftEncoder.getPosition();

    String timeStamp = chassis.timeStamp.format(System.currentTimeMillis());
    System.out.println(timeStamp + "   Start Drive: Pitch: " + avgPitch + "   Start Pos:  " + currPos);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    avgPitch = Math.abs(chassis.lib.getAvgPitch());
    currPos = chassis.leftEncoder.getPosition();
    if (timer.hasElapsed(3) && chassis.lib.getTipSwitch()) {
      if (avgPitch < 10.0) {
        motorSpd = 0.15;
      } else if (avgPitch < 5.0) {
        motorSpd = 0.075;
      }
    }
    chassis.driveArcade(-motorSpd, 0.0);

    if ((counter++ % 10) == 0.0) {
      String timeStamp = chassis.timeStamp.format(System.currentTimeMillis());
      System.out.println(timeStamp + "   Drive: [" + counter + "] Avg Pitch: " + chassis.lib.getAvgPitch() + "   Curr Pos: " + currPos);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    avgPitch = chassis.lib.getAvgPitch();
    currPos = chassis.leftEncoder.getPosition();

    String timeStamp = chassis.timeStamp.format(System.currentTimeMillis());
    System.out.println(timeStamp + "   End Drive: Pitch: " + avgPitch + "   End Pos: " + currPos);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (timer.hasElapsed(3) && avgPitch < 1.0);
  }
}
