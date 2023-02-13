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

public class CraneTilt extends SubsystemBase {
  // ==============================================================
  // Define the left side motors, master and follower

  private final CANSparkMax tiltMotor = new CANSparkMax(
      CANidConstants.kCraneTiltMotor,
      MotorType.kBrushless);

  private final RelativeEncoder tiltEncoder = tiltMotor.getEncoder();

  private final SparkMaxPIDController tiltPID = tiltMotor.getPIDController();

  private double tiltSetPoint = CraneConstants.kTiltInitPos;

  private Crane crane;

  // ==============================================================
  // Define Shuffleboard data
  private final ShuffleboardTab craneTab = Shuffleboard.getTab("Crane");
  private final GenericEntry sbTiltSP = craneTab.addPersistent("Tilt SetPoint", 0).getEntry();
  private final GenericEntry sbTiltPos = craneTab.addPersistent("Tilt Pos", 0).getEntry();
  private final GenericEntry sbTiltVel = craneTab.addPersistent("Tilt Vel", 0).getEntry();
  private final GenericEntry sbTiltFactor = craneTab.addPersistent("Tilt Factor (dpr)", 0).getEntry();

  /** Creates a new Crane. */
  public CraneTilt(Crane crane) {
    System.out.println("+++++ CraneTilt Constructor starting +++++");

    this.crane = crane;

    tiltMotor.restoreFactoryDefaults();

    tiltMotor.clearFaults();

    tiltMotor.setIdleMode(IdleMode.kBrake);

    // ==============================================================
    // Configure PID controllers

    tiltPID.setP(CraneConstants.kTiltP);
    tiltPID.setI(CraneConstants.kTiltI);
    tiltPID.setD(CraneConstants.kTiltD);
    tiltPID.setIZone(CraneConstants.kTiltIz);
    tiltPID.setFF(CraneConstants.kTiltFF);
    tiltPID.setOutputRange(CraneConstants.kTiltMinOutput, CraneConstants.kTiltMaxOutput);

    tiltPID.setSmartMotionMaxVelocity(CraneConstants.kTiltMaxVel, CraneConstants.kTiltSlot);
    tiltPID.setSmartMotionMinOutputVelocity(CraneConstants.kTiltMinVel, CraneConstants.kTiltSlot);
    tiltPID.setSmartMotionMaxAccel(CraneConstants.kTiltMaxAccel, CraneConstants.kTiltSlot);
    tiltPID.setSmartMotionAllowedClosedLoopError(CraneConstants.kTiltAllowErr, CraneConstants.kTiltSlot);

    // ==============================================================
    // Configure encoders
    tiltEncoder.setPositionConversionFactor(CraneConstants.kTiltDegreesPerRotation);

    // ==============================================================
    // Initialize Axis Positions and Set Points
    initTiltPos();
    setTiltSetPoint(tiltSetPoint);

    // ==============================================================
    // Configure ShuffleBoard data
    sbTiltFactor.setDouble(CraneConstants.kTiltDegreesPerRotation);
    sbTiltSP.setDouble(getTiltSetPoint());

    System.out.println("+++++ CraneTilt Constructor finished +++++");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
 
    if (tiltSetPoint != sbTiltSP.getDouble(0.0)) {
      tiltSetPoint = sbTiltSP.getDouble(0.0);
      setTiltSetPoint(tiltSetPoint);
    }
    sbTiltPos.setDouble(getTiltPosition());
    sbTiltVel.setDouble(tiltEncoder.getVelocity());
  }

  public double getTiltSBPos() {
    return sbTiltSP.getDouble(0.0);
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

  public void initTiltPos() {
    tiltEncoder.setPosition(CraneConstants.kTiltInitPos);
  }

  public void setTiltSetPoint(double setPoint) {
    this.tiltSetPoint = setPoint;
    tiltPID.setReference(setPoint, CANSparkMax.ControlType.kSmartMotion);
  }

  // TODO
  public boolean atTiltSetPoint() {
    return false;
  }

  public double getTiltPosition() {
    return tiltEncoder.getPosition();
  }
  public double getTiltSetPoint() {
    return tiltSetPoint;
  }

  public void stopTilt() {
    tiltMotor.set(0.0);
  }
 
  public void setTiltSpeed(double spd) {
    tiltMotor.set(spd);
  }
}
