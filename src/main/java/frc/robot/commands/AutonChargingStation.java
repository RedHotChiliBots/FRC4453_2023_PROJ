// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Chassis;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonChargingStation extends SequentialCommandGroup {
  /** Creates a new AutonLevelChargingStation. */
  Chassis chassis;
  
  public AutonChargingStation(Chassis chassis) {
    this.chassis = chassis;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutonChgStnDrive(chassis), // first - drive up ramp
      //new AutonChgStnRate(chassis), // second - control rate of pitch change
      new AutonChgStnStop(chassis) // second - control rate of pitch change
//      new AutonChgStnLevel(chassis)  // third - level 
    );
  }
}
