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
import frc.robot.GridCalcs.CRANEAXIS;
import frc.robot.GridCalcs.CRANESTATE;

public class CraneTurret extends SubsystemBase {
  // ==============================================================
  // Define the left side motors, master and follower
  private final CANSparkMax turretMotor = new CANSparkMax(
      CANidConstants.kCraneTurretMotor,
      MotorType.kBrushless);

  private final RelativeEncoder turretEncoder = turretMotor.getEncoder();

  private final SparkMaxPIDController turretPID = turretMotor.getPIDController();

  private double setPoint = CraneConstants.kTurretFwdPos;

  // ==============================================================
  // Define Shuffleboard data
  private final ShuffleboardTab craneTab = Shuffleboard.getTab("Crane");
  private final GenericEntry sbTurretSP = craneTab.addPersistent("Turret SetPoint", 0)
      .withWidget("Text View")
      .withPosition(0, 1).withSize(2, 1).getEntry();
  private final GenericEntry sbTurretPos = craneTab.addPersistent("Turret Pos", 0)
      .withWidget("Text View")
      .withPosition(2, 1).withSize(2, 1).getEntry();
  private final GenericEntry sbTurretVel = craneTab.addPersistent("Turret Vel", 0)
      .withWidget("Text View")
      .withPosition(4, 1).withSize(2, 1).getEntry();
  private final GenericEntry sbTurretFactor = craneTab.addPersistent("Turret Factor (dpr)", 0)
      .withWidget("Text View")
      .withPosition(7, 1).withSize(2, 1).getEntry();

  Crane crane;

  /** Creates a new Crane. */
  public CraneTurret(Crane crane) {
    System.out.println("+++++ CraneTurret Constructor starting +++++");
    this.crane = crane;

    turretMotor.restoreFactoryDefaults();

    turretMotor.clearFaults();

    turretMotor.setIdleMode(IdleMode.kBrake);
    turretMotor.setInverted(true);

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
    // Initialize Axis Positions and Set Points at PowerUp/Reboot
    // This requires that the Turret is On Hard Stop before PowerUp/Reboot
    reset();

    // ==============================================================
    // Configure ShuffleBoard data
    sbTurretFactor.setDouble(CraneConstants.kTurretDegreesPerRotation);

    sbTurretSP.setDouble(getSetPoint());

    System.out.println("+++++ CraneTurret Constructor finished +++++");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // if (setPoint != sbTurretSP.getDouble(0.0)) {
    // setPoint = sbTurretSP.getDouble(0.0);
    // setSetPoint(setPoint);
    // }

    sbTurretSP.setDouble(setPoint);
    sbTurretPos.setDouble(turretEncoder.getPosition());
    sbTurretVel.setDouble(turretEncoder.getVelocity());
  }

  public double getTurretSBPos() {
    return sbTurretSP.getDouble(0.0);
  }

  public void reset() {
    initPos();
    setSetPoint(setPoint);
  }

  public void initPos() {
    turretEncoder.setPosition(CraneConstants.kTurretFwdPos);
  }

  public void setSetPoint(double setPoint) {
    this.setPoint = setPoint;
    turretPID.setReference(setPoint, CANSparkMax.ControlType.kSmartMotion);
  }

  public boolean atSetPoint() {
    return Math.abs(setPoint - getPosition()) < CraneConstants.kTurretPositionTolerance;
  }

  public boolean atNextPoint(CRANESTATE tgtState) {
    return Math.abs(
        crane.grid.getCranePos(tgtState, CRANEAXIS.TURRET) - getPosition()) < CraneConstants.kTurretPositionTolerance;
  }

  public double getPosition() {
    return turretEncoder.getPosition();
  }

  public double getSetPoint() {
    return setPoint;
  }

  public void stop() {
    turretMotor.set(0.0);
  }

  public void setSpeed(double spd) {
    turretMotor.set(spd);
  }
}
