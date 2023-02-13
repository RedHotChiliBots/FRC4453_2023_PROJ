// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CylinderState;
import frc.robot.Constants.PneumaticChannelConstants;

public class Claw extends SubsystemBase {
  private final DoubleSolenoid leftPiston = new DoubleSolenoid(
      PneumaticsModuleType.CTREPCM,
      PneumaticChannelConstants.kClawLeftOpen,
      PneumaticChannelConstants.kClawLeftClose);
  private final DoubleSolenoid rightPiston = new DoubleSolenoid(
      PneumaticsModuleType.CTREPCM,
      PneumaticChannelConstants.kClawRightOpen,
      PneumaticChannelConstants.kClawRightClose);

  /** Creates a new Claw. */
  public Claw() {
    System.out.println("+++++ Claw Constructor starting +++++");

    setClaw(CylinderState.CONECLOSE);

    System.out.println("+++++ Claw Constructor finishing +++++");
  }

  @Override
  public void periodic() {

  }

  public void setClaw(CylinderState state) {
    switch (state) {
      case OPEN:
        leftPiston.set(Value.kReverse);
        rightPiston.set(Value.kReverse);
        break;
      case CUBECLOSE:
        leftPiston.set(Value.kForward);
        rightPiston.set(Value.kReverse);
        break;
      case CONECLOSE:
        rightPiston.set(Value.kForward);
        leftPiston.set(Value.kForward);
        break;
      default:
    }
  }
}
