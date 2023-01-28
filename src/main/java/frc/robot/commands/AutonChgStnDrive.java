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
  Timer timer;
  double diffPitch;
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
    timer = new Timer();
    timer.reset();
    timer.start();
    currPitch = chassis.getPitch();
    String timeStamp = chassis.timeStamp.format(System.currentTimeMillis());
    System.out.println(timeStamp + "   Start Drive: Pitch: " + currPitch);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    chassis.driveArcade(-0.35, 0.0);
    lastPitch = currPitch;
    currPitch = chassis.getPitch();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    String timeStamp = chassis.timeStamp.format(System.currentTimeMillis());
    System.out.println(timeStamp + "   End Drive: Pitch: " + currPitch + "   Diff: " + diffPitch);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    diffPitch = Math.abs(currPitch - lastPitch);
    if ((counter++ % 10) == 0.0) {
      String timeStamp = chassis.timeStamp.format(System.currentTimeMillis());
      System.out.println(timeStamp + "   Drive: [" + counter + "] Pitch: " + currPitch + "   Diff: " + diffPitch);
    }
    return  (timer.hasElapsed( 1) && Math.abs(diffPitch) > 1.0);
  }
}
