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

public class CraneArm extends SubsystemBase {
  // ==============================================================
  // Define the left side motors, master and follower
  private final CANSparkMax armMotor = new CANSparkMax(
      CANidConstants.kCraneArmMotor,
      MotorType.kBrushless);

  private final RelativeEncoder armEncoder = armMotor.getEncoder();

  private final SparkMaxPIDController armPID = armMotor.getPIDController();

  private double setPoint = CraneConstants.kArmInitPos;

  // ==============================================================
  // Define Shuffleboard data
  private final ShuffleboardTab craneTab = Shuffleboard.getTab("Crane");
  private final GenericEntry sbArmSP = craneTab.addPersistent("Arm SetPoint", 0)
      .withWidget("Text View")
      .withPosition(0, 3).withSize(2, 1).getEntry();
  private final GenericEntry sbArmPos = craneTab.addPersistent("Arm Pos", 0)
      .withWidget("Text View")
      .withPosition(2, 3).withSize(2, 1).getEntry();
  private final GenericEntry sbArmVel = craneTab.addPersistent("Arm Vel", 0)
      .withWidget("Text View")
      .withPosition(4, 3).withSize(2, 1).getEntry();
  private final GenericEntry sbArmFactor = craneTab.addPersistent("Arm Factor (ipr)", 0)
      .withWidget("Text View")
      .withPosition(7, 3).withSize(2, 1).getEntry();

  Crane crane;

  /** Creates a new Crane. */
  public CraneArm(Crane crane) {
    System.out.println("+++++ CraneArm Constructor starting +++++");

    this.crane = crane;

    armMotor.restoreFactoryDefaults();

    armMotor.clearFaults();

    armMotor.setIdleMode(IdleMode.kBrake);
    armMotor.setInverted(true);

    // ==============================================================
    // Configure PID controllers
    armPID.setP(CraneConstants.kArmP);
    armPID.setI(CraneConstants.kArmI);
    armPID.setD(CraneConstants.kArmD);
    armPID.setIZone(CraneConstants.kArmIz);
    armPID.setFF(CraneConstants.kArmFF);
    armPID.setOutputRange(CraneConstants.kArmMinOutput, CraneConstants.kArmMaxOutput);

    armPID.setSmartMotionMaxVelocity(CraneConstants.kArmMaxVel, CraneConstants.kArmSlot);
    armPID.setSmartMotionMinOutputVelocity(CraneConstants.kArmMinVel, CraneConstants.kArmSlot);
    armPID.setSmartMotionMaxAccel(CraneConstants.kArmMaxAccel, CraneConstants.kArmSlot);
    armPID.setSmartMotionAllowedClosedLoopError(CraneConstants.kArmAllowErr, CraneConstants.kArmSlot);

    // ==============================================================
    // Configure encoders
    armEncoder.setPositionConversionFactor(CraneConstants.kArmInchesPerRotation);

    // ==============================================================
    // Initialize Axis Positions and Set Points at PowerUp/Reboot
    // This requires that the Arm is Retracted before PowerUp/Reboot
    reset();

    // ==============================================================
    // Configure ShuffleBoard data
    sbArmFactor.setDouble(CraneConstants.kArmInchesPerRotation);

    sbArmSP.setDouble(getSetPoint());

    System.out.println("+++++ CraneArm Constructor finished +++++");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // if (setPoint != sbArmSP.getDouble(0.0)) {
    // setPoint = sbArmSP.getDouble(0.0);
    // setSetPoint(setPoint);
    // }

    sbArmSP.setDouble(setPoint);
    sbArmPos.setDouble(getPosition());
    sbArmVel.setDouble(armEncoder.getVelocity());
  }

  public double getArmSBPos() {
    return sbArmSP.getDouble(0.0);
  }

  public void reset() {
    initPos();
    setSetPoint(setPoint);
  }

  public void initPos() {
    armEncoder.setPosition(CraneConstants.kArmInitPos);
  }

  public void setSetPoint(double setPoint) {
    // The Arm moves in two sections. Command 1" will move it 2". Need divide the
    // delta by 2.
    double sp = CraneConstants.kArmInitPos + ((setPoint - CraneConstants.kArmInitPos) / 2.0);
    this.setPoint = setPoint;
    armPID.setReference(sp, CANSparkMax.ControlType.kSmartMotion);
  }

  public double getPosition() {
    return CraneConstants.kArmInitPos + ((armEncoder.getPosition() - CraneConstants.kArmInitPos) * 2.0);
  }

  public boolean atSetPoint() {
    return Math.abs(setPoint - getPosition()) < CraneConstants.kArmPositionTolerance;
  }

  public boolean atNextPoint(CRANESTATE tgtState) {
    return Math.abs(
        crane.grid.getCranePos(tgtState, CRANEAXIS.ARM) - getPosition()) < CraneConstants.kArmPositionTolerance;
  }

  public double getSetPoint() {
    return setPoint;
  }

  public void stop() {
    armMotor.set(0.0);
  }

  public void setSpeed(double spd) {
    armMotor.set(spd);
  }
}
