// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticChannelConstants;

public class Claw extends SubsystemBase {
  private final DoubleSolenoid leftPiston = new DoubleSolenoid(
      PneumaticsModuleType.CTREPCM,
      PneumaticChannelConstants.kLeftPistonOpen,
      PneumaticChannelConstants.kLeftPistonClose);
  private final DoubleSolenoid rightPiston = new DoubleSolenoid(
      PneumaticsModuleType.CTREPCM,
      PneumaticChannelConstants.kRightPistonOpen,
      PneumaticChannelConstants.kRightPistonClose);

  /** Creates a new Claw. */
  public Claw() {
    closePiston();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void openLeftPiston() {
    leftPiston.set(Value.kReverse);
  }

  public void openRightPiston() {
    rightPiston.set(Value.kReverse);
  }

  public void closePiston() {
    leftPiston.set(Value.kReverse);
    rightPiston.set(Value.kReverse);
  }
}
