// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.ChassisConstants;
import frc.robot.subsystems.Chassis;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PIDLevel extends PIDCommand {
  private Chassis chassis;
//  public final double max = 2.0;

  /** Creates a new PIDLevel. */
  public PIDLevel(Chassis chassis) {
    super(
        // The controller that the command will use
        new PIDController(
            0.015, 
            0.0, 
            0.0075),
        // This should return the measurement
        () -> chassis.getPitch(),
        // This should return the setpoint (can also be a constant)
        () -> ChassisConstants.kLevelSetPoint,
        // This uses the output
        output -> {
          chassis.drive(-(output > 0.5 ? 0.5 : output < -0.5 ? -0.5 : output), 0.0);
        });
    // // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(chassis);
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
