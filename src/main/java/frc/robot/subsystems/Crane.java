// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.GridCalcs;
import frc.robot.GridCalcs.H;
import frc.robot.GridCalcs.V;
import frc.robot.GridCalcs.E;
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
  
  private GridCalcs grid = new GridCalcs();
  private XboxController operator;
  private int dpadValue;

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
  private final GenericEntry sbTurretFactor = craneTab.addPersistent("Turret Factor (dpr)", 0).getEntry();
  private final GenericEntry sbTiltFactor = craneTab.addPersistent("Tilt Factor (dpr)", 0).getEntry();
  private final GenericEntry sbArmFactor = craneTab.addPersistent("Arm Factor (ipr)", 0).getEntry();

  /** Creates a new Crane. */
  public Crane( XboxController operator) {
    System.out.println("+++++ Crane Constructor starting +++++");

    this.operator = operator;
    grid.horz.set(H.CENTER);
    grid.vert.set(V.MID);
    grid.setElem(E.CONE);

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
    turretPID.setIZone(CraneConstants.kTurretIz);
    turretPID.setFF(CraneConstants.kTurretFF);
    turretPID.setOutputRange(CraneConstants.kTurretMinOutput, CraneConstants.kTurretMaxOutput);

    turretPID.setSmartMotionMaxVelocity(CraneConstants.kTurretMaxVel, CraneConstants.kTurretSlot);
    turretPID.setSmartMotionMinOutputVelocity(CraneConstants.kTurretMinVel, CraneConstants.kTurretSlot);
    turretPID.setSmartMotionMaxAccel(CraneConstants.kTurretMaxAccel, CraneConstants.kTurretSlot);
    turretPID.setSmartMotionAllowedClosedLoopError(CraneConstants.kTurretAllowErr, CraneConstants.kTurretSlot);

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

    armPID.setP(CraneConstants.kArmP);
    armPID.setI(CraneConstants.kArmI);
    armPID.setD(CraneConstants.kArmD);
    armPID.setIZone(CraneConstants.kArmIz);
    armPID.setFF(CraneConstants.kArmFF);
    armPID.setOutputRange(CraneConstants.kArmMinOutput, CraneConstants.kArmMaxOutput);

    armPID.setSmartMotionMaxVelocity(CraneConstants.kArmMaxVel, CraneConstants.kArmSlot);
    armPID.setSmartMotionMinOutputVelocity(CraneConstants.kArmMinVel, CraneConstants.kArmSlot);
    armPID.setSmartMotionMaxAccel(CraneConstants.kArmMaxAccel, CraneConstants.kTiltSlot);
    armPID.setSmartMotionAllowedClosedLoopError(CraneConstants.kArmAllowErr, CraneConstants.kArmSlot);

    // ==============================================================
    // Configure encoders
    turretEncoder.setPositionConversionFactor(CraneConstants.kTurretPosFactor);
    tiltEncoder.setPositionConversionFactor(CraneConstants.kTiltPosFactor);
    armEncoder.setPositionConversionFactor(CraneConstants.kArmPosFactor);

    initTurretPos();
    setTurretSetPoint(turretSetPoint);
    initTiltPos();
    setTiltSetPoint(tiltSetPoint);
    initArmPos();
    setArmSetPoint(armSetPoint);

    sbTurretFactor.setDouble(CraneConstants.kTurretPosFactor);
    sbTiltFactor.setDouble(CraneConstants.kTiltPosFactor);
    sbArmFactor.setDouble(CraneConstants.kArmPosFactor);

    System.out.println("+++++ Crane Constructor finished +++++");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    readDPad();

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

  public double getArmSBPos() {
    return sbArmSP.getDouble(0.0);
  }

  public double getTiltSBPos() {
    return sbTiltSP.getDouble(0.0);
  }

  public double getTurretSBPos() {
    return sbTurretSP.getDouble(0.0);
  }

  public double getGridX() {
    return grid.getX();
  }

  public double getGridY() {
    return grid.getY();
  }

  public double getGridZ() {
    return grid.getZ();
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

  public void setTurretSetPoint(double setPoint) {
    this.turretSetPoint = setPoint;
    turretPID.setReference(setPoint, CANSparkMax.ControlType.kSmartMotion);
  }

  public void setTiltSetPoint(double setPoint) {
    this.tiltSetPoint = setPoint;
    tiltPID.setReference(setPoint, CANSparkMax.ControlType.kSmartMotion);
  }

  public void setArmSetPoint(double setPoint) {
    this.armSetPoint = setPoint;
    armPID.setReference(setPoint, CANSparkMax.ControlType.kSmartMotion);
  }

  // TODO
  public boolean atTurrentSetPoint() {
    return false;
  }

  // TODO
  public boolean atTiltSetPoint() {
    return false;
  }

  // TODO
  public boolean atArmSetPoint() {
    return false;
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

  public void readDPad() {
    dpadValue = operator.getPOV();
    if (dpadValue != -1) {
      switch (dpadValue) {
        case 0:
          grid.vert.prev();
          break;
        case 45:
          grid.vert.prev();
          grid.horz.next();
          break;
        case 90:
          grid.horz.next();
          break;
        case 135:
          grid.horz.next();
          grid.vert.next();
          break;
        case 180:
          grid.vert.next();
          break;
        case 225:
          grid.horz.prev();
          grid.vert.next();
          break;
        case 270:
          grid.horz.prev();
          break;
        case 315:
          grid.horz.prev();
          grid.vert.prev();
          break;
        default:
      }
    }
  }
}
