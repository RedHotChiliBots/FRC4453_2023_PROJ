// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.CANidConstants;
import frc.robot.Constants.CraneConstants;


public class Crane extends SubsystemBase {
  // ==============================================================
  // Define the left side motors, master and follower
  private final CANSparkMax turretMotor = new CANSparkMax(
      CANidConstants.kCraneTurretMotor,
      MotorType.kBrushless);

  private final CANSparkMax tiltMotor = new CANSparkMax(
      CANidConstants.kCraneTiltMotor,
      MotorType.kBrushless);

  private final CANSparkMax armMotor = new CANSparkMax(
      CANidConstants.kCraneArmMotor,
      MotorType.kBrushless);

  private final RelativeEncoder turretEncoder = turretMotor.getEncoder();
  private final RelativeEncoder tiltEncoder = tiltMotor.getEncoder();
  private final RelativeEncoder armEncoder = armMotor.getEncoder();

  private final SparkMaxPIDController turretPID = turretMotor.getPIDController();
  private final SparkMaxPIDController tiltPID = tiltMotor.getPIDController();
  private final SparkMaxPIDController armPID = armMotor.getPIDController();

  /** Creates a new Crane. */
  public Crane() {
    turretMotor.restoreFactoryDefaults();
    tiltMotor.restoreFactoryDefaults();
    armMotor.restoreFactoryDefaults();
 
    turretMotor.clearFaults();
    tiltMotor.clearFaults();
    armMotor.clearFaults();

    turretMotor.setIdleMode(IdleMode.kBrake);
    tiltMotor.setIdleMode(IdleMode.kBrake);
    armMotor.setIdleMode(IdleMode.kBrake);

    // ==============================================================
    // Configure PID controllers
    turretPID.setP(CraneConstants.kTurretP);
    turretPID.setI(CraneConstants.kTurretI);
    turretPID.setD(CraneConstants.kTurretD);

    tiltPID.setP(CraneConstants.kTiltP);
    tiltPID.setI(CraneConstants.kTiltI);
    tiltPID.setD(CraneConstants.kTiltD);

    armPID.setP(CraneConstants.kArmP);
    armPID.setI(CraneConstants.kArmI);
    armPID.setD(CraneConstants.kArmD);

    // turretPID.setSetSetpoint(CraneConstants.kTurretSetPoint);
    // turretPID.setTolerance(CraneConstants.kTurretSetTolerance);

    // tiltPID.setSetpoint(CraneConstants.kTiltSetPoint);
    // tiltPID.setTolerance(CraneConstants.kTiltSetTolerance);

    // armPID.setSetpoint(CraneConstants.kArmSetPoint);
    // armPID.setTolerance(CraneConstants.kArmSetTolerance);

    // ==============================================================
    // Configure encoders
    turretEncoder.setPositionConversionFactor(CraneConstants.kTurretPosFactor);
    tiltEncoder.setPositionConversionFactor(CraneConstants.kTiltPosFactor);
    armEncoder.setPositionConversionFactor(CraneConstants.kArmPosFactor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
