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
    System.out.println("+++++ Claw Constructor starting +++++");
    
    openPiston();

    System.out.println("+++++ Claw Constructor finishing +++++");
  }

  @Override
  public void periodic() {

  }

  public void closeLeftPiston() {
    leftPiston.set(Value.kForward);
  }

  public void closeRightPiston() {
    rightPiston.set(Value.kForward);
  }

  public void openPiston() {
    leftPiston.set(Value.kReverse);
    rightPiston.set(Value.kReverse);
  }
}
