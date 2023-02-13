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

public class CraneTurret extends SubsystemBase {
  // ==============================================================
  // Define the left side motors, master and follower
  private final CANSparkMax turretMotor = new CANSparkMax(
      CANidConstants.kCraneTurretMotor,
      MotorType.kBrushless);

  private final RelativeEncoder turretEncoder = turretMotor.getEncoder();

  private final SparkMaxPIDController turretPID = turretMotor.getPIDController();

  private double turretSetPoint = CraneConstants.kTurretInitPos;

  // ==============================================================
  // Define Shuffleboard data
  private final ShuffleboardTab craneTab = Shuffleboard.getTab("Crane");
  private final GenericEntry sbTurretSP = craneTab.addPersistent("Turret SetPoint", 0).getEntry();
  private final GenericEntry sbTurretPos = craneTab.addPersistent("Turret Pos", 0).getEntry();
  private final GenericEntry sbTurretVel = craneTab.addPersistent("Turret Vel", 0).getEntry();
  private final GenericEntry sbTurretFactor = craneTab.addPersistent("Turret Factor (dpr)", 0).getEntry();

  /** Creates a new Crane. */
  public CraneTurret() {
    System.out.println("+++++ CraneTurret Constructor starting +++++");

    turretMotor.restoreFactoryDefaults();

    turretMotor.clearFaults();

    turretMotor.setIdleMode(IdleMode.kBrake);

    // ==============================================================
    // Configure PID controllers
    turretPID.setP(CraneConstants.kTurretP);
    turretPID.setI(CraneConstants.kTurretI);
    turretPID.setD(CraneConstants.kTurretD);
    turretPID.setIZone(CraneConstants.kTurretIz);
    turretPID.setFF(CraneConstants.kTurretFF);
    turretPID.setOutputRange(CraneConstants.kTurretMinOutput, CraneConstants.kTurretMaxOutput);

    turretPID.setSmartMotionMaxVelocity(CraneConstants.kTurretMaxVel, CraneConstants.kTurretSlot);
    turretPID.setSmartMotionMinOutputVelocity(CraneConstants.kTurretMinVel, CraneConstants.kTurretSlot);
    turretPID.setSmartMotionMaxAccel(CraneConstants.kTurretMaxAccel, CraneConstants.kTurretSlot);
    turretPID.setSmartMotionAllowedClosedLoopError(CraneConstants.kTurretAllowErr, CraneConstants.kTurretSlot);

    // ==============================================================
    // Configure encoders
    turretEncoder.setPositionConversionFactor(CraneConstants.kTurretDegreesPerRotation);
  
    // ==============================================================
    // Initialize Axis Positions and Set Points   
    initTurretPos();
    setTurretSetPoint(turretSetPoint);

    // ==============================================================
    // Configure ShuffleBoard data
    sbTurretFactor.setDouble(CraneConstants.kTurretDegreesPerRotation);

    sbTurretSP.setDouble(getTurretSetPoint());

    System.out.println("+++++ CraneTurret Constructor finished +++++");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (turretSetPoint != sbTurretSP.getDouble(0.0)) {
      turretSetPoint = sbTurretSP.getDouble(0.0);
      setTurretSetPoint(turretSetPoint);
    }
    sbTurretPos.setDouble(getTurretPosition());
    sbTurretVel.setDouble(turretEncoder.getVelocity());
  }

  public double getTurretSBPos() {
    return sbTurretSP.getDouble(0.0);
  }

  // public double getGridX() {
  //   return grid.getX();
  // }

  // public double getGridY() {
  //   return grid.getY();
  // }

  // public double getGridZ() {
  //   return grid.getZ();
  // }

  public void initTurretPos() {
    turretEncoder.setPosition(CraneConstants.kTurretInitPos);
  }

  public void setTurretSetPoint(double setPoint) {
    this.turretSetPoint = setPoint;
    turretPID.setReference(setPoint, CANSparkMax.ControlType.kSmartMotion);
  }

  // TODO
  public boolean atTurrentSetPoint() {
    return false;
  }

  public double getTurretPosition() {
    return turretEncoder.getPosition();
  }

  public double getTurretSetPoint() {
    return turretSetPoint;
  }

  public void stopTurret() {
    turretMotor.set(0.0);
  }

  public void setTurretSpeed(double spd) {
    turretMotor.set(spd);
  }
}