// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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

  private double turretSetPoint = CraneConstants.kTurretInitPos;
  private double tiltSetPoint = CraneConstants.kTiltInitPos;
  private double armSetPoint = CraneConstants.kArmInitPos;
  
  // ==============================================================
  // Define Shuffleboard data
  private final ShuffleboardTab craneTab = Shuffleboard.getTab("Crane");
  private final GenericEntry sbTurretSP = craneTab.addPersistent("Turret SetPoint", 0).getEntry();
  private final GenericEntry sbTiltSP = craneTab.addPersistent("Tilt SetPoint", 0).getEntry();
  private final GenericEntry sbArmSP = craneTab.addPersistent("Arm SetPoint", 0).getEntry();
  private final GenericEntry sbTurretPos = craneTab.addPersistent("Turret Pos", 0).getEntry();
  private final GenericEntry sbTiltPos = craneTab.addPersistent("Tilt Pos", 0).getEntry();
  private final GenericEntry sbArmPos = craneTab.addPersistent("Arm Pos", 0).getEntry();
  private final GenericEntry sbTurretVel = craneTab.addPersistent("Turret Vel", 0).getEntry();
  private final GenericEntry sbTiltVel = craneTab.addPersistent("Tilt Vel", 0).getEntry();
  private final GenericEntry sbArmVel = craneTab.addPersistent("Arm Vel", 0).getEntry();

  /** Creates a new Crane. */
  public Crane() {
    System.out.println("+++++ Crane Constructor starting +++++");

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
    turretPID.setOutputRange(CraneConstants.kTurretMinOutput, CraneConstants.kTurretMaxOutput);

    tiltPID.setP(CraneConstants.kTiltP);
    tiltPID.setI(CraneConstants.kTiltI);
    tiltPID.setD(CraneConstants.kTiltD);
    tiltPID.setOutputRange(CraneConstants.kTiltMinOutput, CraneConstants.kTiltMaxOutput);

    armPID.setP(CraneConstants.kArmP);
    armPID.setI(CraneConstants.kArmI);
    armPID.setD(CraneConstants.kArmD);
    armPID.setOutputRange(CraneConstants.kArmMinOutput, CraneConstants.kArmMaxOutput);

    // ==============================================================
    // Configure encoders
    turretEncoder.setPositionConversionFactor(CraneConstants.kTurretPosFactor);
    tiltEncoder.setPositionConversionFactor(CraneConstants.kTiltPosFactor);
    armEncoder.setPositionConversionFactor(CraneConstants.kArmPosFactor);

    initTurretPos();
    setTurretPosition(turretSetPoint);
    initTiltPos();
    setTiltPosition(tiltSetPoint);
    initArmPos();
    setArmPosition(armSetPoint);

    System.out.println("+++++ Crane Constructor finished +++++");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    sbTurretSP.setDouble(getTurretSetPoint());
    sbTiltSP.setDouble(getTiltSetPoint());
    sbArmSP.setDouble(getArmSetPoint());
    sbTurretPos.setDouble(getTurretPosition());
    sbTiltPos.setDouble(getTiltPosition());
    sbArmPos.setDouble(getArmPosition());
    sbTurretVel.setDouble(turretEncoder.getVelocity());
    sbTiltVel.setDouble(tiltEncoder.getVelocity());
    sbArmVel.setDouble(armEncoder.getVelocity());
  }

  public void initTurretPos() {
    turretEncoder.setPosition(CraneConstants.kTurretInitPos);
  }

  public void initTiltPos() {
    tiltEncoder.setPosition(CraneConstants.kTiltInitPos);
  }

  public void initArmPos() {
    armEncoder.setPosition(CraneConstants.kArmInitPos);
  }

  public void setTurretPosition(double setPoint) {
    this.turretSetPoint = setPoint;
    turretPID.setReference(setPoint, CANSparkMax.ControlType.kPosition);
  }

  public void setTiltPosition(double setPoint) {
    this.tiltSetPoint = setPoint;
    tiltPID.setReference(setPoint, CANSparkMax.ControlType.kPosition);
  }

  public void setArmPosition(double setPoint) {
    this.armSetPoint = setPoint;
    armPID.setReference(setPoint, CANSparkMax.ControlType.kPosition);
  }

  public double getTurretPosition() {
    return turretEncoder.getPosition();
  }

  public double getTiltPosition() {
    return tiltEncoder.getPosition();
  }

  public double getArmPosition() {
    return armEncoder.getPosition();
  }

  public double getTurretSetPoint() {
    return turretSetPoint;
  }

  public double getTiltSetPoint() {
    return tiltSetPoint;
  }

  public double getArmSetPoint() {
    return armSetPoint;
  }

  public void stopTurret() {
    turretMotor.set(0.0);
  }

  public void stopTilt() {
    tiltMotor.set(0.0);
  }

  public void stopArm() {
    armMotor.set(0.0);
  }  

  public void setTurretSpeed(double spd) {
    turretMotor.set(spd);
  }

  public void setTiltSpeed(double spd) {
    tiltMotor.set(spd);
  }

  public void setArmSpeed(double spd) {
    armMotor.set(spd);
  }
}
