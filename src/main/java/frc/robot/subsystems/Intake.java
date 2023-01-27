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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANidConstants;
import frc.robot.Constants.PneumaticChannelConstants;

public class Intake extends SubsystemBase {
  private final CANSparkMax leftMotor = new CANSparkMax(
    CANidConstants.kIntakeLeftMotor,
    MotorType.kBrushless);

    private final CANSparkMax rightMotor = new CANSparkMax(
      CANidConstants.kIntakeLeftMotor,
      MotorType.kBrushless);

      private final DoubleSolenoid intakeArms = new DoubleSolenoid(
			PneumaticsModuleType.CTREPCM,
			PneumaticChannelConstants.kIntakeArmOpen,
			PneumaticChannelConstants.kIntakeArmClose);


  /** Creates a new Intake. */
  public Intake() {

    leftMotor.restoreFactoryDefaults();
    rightMotor.restoreFactoryDefaults();
    leftMotor.clearFaults();
    rightMotor.clearFaults();
    leftMotor.setIdleMode(IdleMode.kBrake);
    rightMotor.setIdleMode(IdleMode.kBrake);
  stopLeftMoter();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
  public void stopLeftMoter(){
leftMotor.set(0.0);

  }
  public void stopRightMotor(){
rightMotor.set(0.0);

  }
}
