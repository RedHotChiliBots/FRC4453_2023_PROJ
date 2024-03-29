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

public class VisionTeleopTrackAprilTag extends CommandBase {
  Chassis chassis;
  Vision vision;
  int targetId;

  /** Creates a new AutonTrackAprilTag. */
  public VisionTeleopTrackAprilTag(Chassis chassis, Vision vision, int targetId) {
    this.chassis = chassis;
    this.vision = vision;
    this.targetId = targetId;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    chassis.disableCompressor();
    vision.setTargetId(targetId);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (vision.hasTargets()) {
      if (vision.getRange() < VisionConstants.kRange2Rumble) {
        if (!vision.getCancelRumble()) {
          RobotContainer.setDriverRumble(RumbleType.kLeftRumble);
        }

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
    chassis.enableCompressor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (RobotContainer.driver.getAButton() && vision.atDistTarget());
  }
}
