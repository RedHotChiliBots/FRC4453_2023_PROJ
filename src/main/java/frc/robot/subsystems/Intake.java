//Create the intake part / Moters that can spin wheels in and out
//moving arms for the intake system
//left and right moter

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANidConstants;
import frc.robot.Constants.CylinderState;
import frc.robot.Constants.Pneumatic0ChannelConstants;
import frc.robot.Constants.Pneumatic1ChannelConstants;
import frc.robot.Constants.PneumaticModuleConstants;

public class Intake extends SubsystemBase {
  private final CANSparkMax leftMotor = new CANSparkMax(
      CANidConstants.kIntakeLeftMotor,
      MotorType.kBrushless);

  private final CANSparkMax rightMotor = new CANSparkMax(
      CANidConstants.kIntakeRightMotor,
      MotorType.kBrushless);

  private final DoubleSolenoid intakeArm = new DoubleSolenoid(
      PneumaticModuleConstants.kPCM0,
      PneumaticsModuleType.CTREPCM,
      Pneumatic0ChannelConstants.kIntakeArmOpen,
      Pneumatic0ChannelConstants.kIntakeArmClose);

  private final DoubleSolenoid intakeBar = new DoubleSolenoid(
      PneumaticModuleConstants.kPCM1,
      PneumaticsModuleType.CTREPCM,
      Pneumatic1ChannelConstants.kIntakeBarEnabled,
      Pneumatic1ChannelConstants.kIntakeBarDisabled);

  /** Creates a new Intake. */
  public Intake() {
    System.out.println("+++++ Intake Constructor starting +++++");

    leftMotor.restoreFactoryDefaults();
    rightMotor.restoreFactoryDefaults();
    leftMotor.clearFaults();
    rightMotor.clearFaults();
    leftMotor.setIdleMode(IdleMode.kBrake);
    rightMotor.setIdleMode(IdleMode.kBrake);
    stopMoters();
    setIntake(CylinderState.CLOSE);

    System.out.println("+++++ Intake Constructor finishing +++++");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public void stopMoters() {
    leftMotor.set(0.0);
    rightMotor.set(0.0);
  }

  public void revMoters() {
    leftMotor.set(-0.28);
    rightMotor.set(0.28);
  }

  public void fwdMoters() {
    leftMotor.set(0.28);
    rightMotor.set(-0.28);
  }

  public void setIntake(CylinderState state) {
    switch (state) {
      case OPEN:
        intakeArm.set(Value.kForward);
        break;
      case CLOSE:
        intakeArm.set(Value.kReverse);
        break;
      default:
    }
  }

  public void setBar(CylinderState state) {
    switch (state) {
      case OPEN:
        intakeBar.set(Value.kForward);
        break;
      case CLOSE:
        intakeBar.set(Value.kReverse);
        break;
      default:
    }
  }
}
