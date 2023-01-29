// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

public class AutonChgStnDrive extends CommandBase {
  Chassis chassis;
  double lastPitch;
  double currPitch;
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
    currPitch = chassis.getPitch();
    currPos = chassis.leftEncoder.getPosition();

    String timeStamp = chassis.timeStamp.format(System.currentTimeMillis());
    System.out.println(timeStamp + "   Start Drive: Pitch: " + currPitch + "   Start Pos:  " + currPos);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    lastPitch = currPitch;
    currPitch = chassis.getPitch();
    currPos = chassis.leftEncoder.getPosition();
    if (timer.hasElapsed(3) && (chassis.getMaxPitch() - Math.abs(currPitch)) > 3.0) {
      motorSpd = 0.15;
    }
    chassis.driveArcade(-motorSpd, 0.0);

    if ((counter++ % 10) == 0.0) {
      String timeStamp = chassis.timeStamp.format(System.currentTimeMillis());
      System.out.println(timeStamp + "   Drive: [" + counter + "] Pitch: " + currPitch + "   Curr Pos: " + currPos);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    currPos = chassis.leftEncoder.getPosition();
    currPitch = chassis.getPitch();

    String timeStamp = chassis.timeStamp.format(System.currentTimeMillis());
    System.out.println(timeStamp + "   End Drive: Pitch: " + currPitch + "   End Pos: " + currPos);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (timer.hasElapsed(1) && Math.abs(currPitch) < 1.0);
  }
}
