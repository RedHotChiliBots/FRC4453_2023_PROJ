// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
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
import frc.robot.Constants.Pneumatic1ChannelConstants;
import frc.robot.Constants.PneumaticModuleConstants;

public class CraneTilt extends SubsystemBase {
  // ==============================================================
  // Define the left side motors, master and follower

  private final CANSparkMax tiltMotor = new CANSparkMax(
      CANidConstants.kCraneTiltMotor,
      MotorType.kBrushless);

  private final RelativeEncoder tiltEncoder = tiltMotor.getEncoder();

  private final SparkMaxPIDController tiltPID = tiltMotor.getPIDController();

  private double tiltSetPoint = CraneConstants.kTiltInitPos;

  private final DoubleSolenoid ratchet = new DoubleSolenoid(
      PneumaticModuleConstants.kPCM1,
      PneumaticsModuleType.CTREPCM,
      Pneumatic1ChannelConstants.kRatchetLock,
      Pneumatic1ChannelConstants.kRatchetUnlock);

  // ==============================================================
  // Define Shuffleboard data
  private final ShuffleboardTab craneTab = Shuffleboard.getTab("Crane");
  private final GenericEntry sbTiltSP = craneTab.addPersistent("Tilt SetPoint", 0)
      .withWidget("Text View")
      .withPosition(0, 2).withSize(1, 1).getEntry();
  private final GenericEntry sbTiltPos = craneTab.addPersistent("Tilt Pos", 0)
      .withWidget("Text View")
      .withPosition(1, 2).withSize(1, 1).getEntry();
  private final GenericEntry sbTiltVel = craneTab.addPersistent("Tilt Vel", 0)
      .withWidget("Text View")
      .withPosition(2, 2).withSize(1, 1).getEntry();
  private final GenericEntry sbTiltFactor = craneTab.addPersistent("Tilt Factor (dpr)", 0)
      .withWidget("Text View")
      .withPosition(3, 2).withSize(1, 1).getEntry();

  public enum RatchetState {
    LOCK,
    UNLOCK
  }

  /** Creates a new Crane. */
  public CraneTilt() {
    System.out.println("+++++ CraneTilt Constructor starting +++++");

    tiltMotor.restoreFactoryDefaults();

    tiltMotor.clearFaults();

    tiltMotor.setIdleMode(IdleMode.kBrake);
    tiltMotor.setInverted(true);

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
    // Initialize Axis Positions, Set Points, and Cylinder
    reset();

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
    sbTiltPos.setDouble(tiltEncoder.getPosition());
    sbTiltVel.setDouble(tiltEncoder.getVelocity());
  }

  public double getTiltSBPos() {
    return sbTiltSP.getDouble(0.0);
  }

  // public double getGridX() {
  // return grid.getX();
  // }

  // public double getGridY() {
  // return grid.getY();
  // }

  // public double getGridZ() {
  // return grid.getZ();
  // }

  public void reset() {
    initTiltPos();
    setTiltSetPoint(tiltSetPoint);
    setRatchet(RatchetState.UNLOCK);
  }

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

  public void setRatchet(RatchetState state) {
    switch (state) {
      case LOCK:
        ratchet.set(Value.kForward);
        break;
      case UNLOCK:
        ratchet.set(Value.kReverse);
        break;
      default:
    }
  }
}
