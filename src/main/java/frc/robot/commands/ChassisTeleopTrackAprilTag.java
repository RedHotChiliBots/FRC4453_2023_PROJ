// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Vision;

public class ChassisTeleopTrackAprilTag extends CommandBase {
  Chassis chassis;
  Vision vision;

  /** Creates a new AutonTrackAprilTag. */
  public ChassisTeleopTrackAprilTag(Chassis chassis, Vision vision) {
    this.chassis = chassis;
    this.vision = vision;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (vision.hasTargets()) {
      if (vision.getRange() < VisionConstants.kRange2Rumble) {
        RobotContainer.setDriverRumble(RumbleType.kLeftRumble);

        if (RobotContainer.driver.getAButton()) {
          RobotContainer.resetDriverRumble(RumbleType.kLeftRumble);

          double[] spd = vision.trackAprilTag();

          // Use our forward/turn speeds to control the drivetrain
          chassis.driveArcade(spd[0], spd[1]);
        }
      }
    } else {
      RobotContainer.resetDriverRumble(RumbleType.kLeftRumble);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (RobotContainer.driver.getAButton() && vision.atDistTarget());
  }
}
