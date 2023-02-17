// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Pneumatic0ChannelConstants;
import frc.robot.Constants.PneumaticModuleConstants;

public class Claw extends SubsystemBase {
  private final DoubleSolenoid leftPiston = new DoubleSolenoid(
      PneumaticModuleConstants.kPCM0,
      PneumaticsModuleType.CTREPCM,
      Pneumatic0ChannelConstants.kClawLeftOpen,
      Pneumatic0ChannelConstants.kClawLeftClose);

  private final DoubleSolenoid rightPiston = new DoubleSolenoid(
      PneumaticModuleConstants.kPCM0,
      PneumaticsModuleType.CTREPCM,
      Pneumatic0ChannelConstants.kClawRightOpen,
      Pneumatic0ChannelConstants.kClawRightClose);

  public enum FingerState {
    RELEASE,
    CONEGRAB,
    CUBEGRAB
  }
      
  /** Creates a new Claw. */
  public Claw() {
    System.out.println("+++++ Claw Constructor starting +++++");

    setFinger(FingerState.CONEGRAB);

    System.out.println("+++++ Claw Constructor finishing +++++");
  }

  @Override
  public void periodic() {

  }

  public void setFinger(FingerState state) {
    switch (state) {
      case RELEASE:
        leftPiston.set(Value.kReverse);
        rightPiston.set(Value.kReverse);
        break;
      case CUBEGRAB:
        leftPiston.set(Value.kForward);
        rightPiston.set(Value.kReverse);
        break;
      case CONEGRAB:
        rightPiston.set(Value.kForward);
        leftPiston.set(Value.kForward);
        break;
      default:
    }
  }
}
