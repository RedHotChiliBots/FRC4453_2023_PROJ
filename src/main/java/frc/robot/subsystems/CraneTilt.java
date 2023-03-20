// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
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
import frc.robot.Constants.DIOChannelConstants;
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

  private double setPoint = CraneConstants.kTiltInitPos;

  private final DoubleSolenoid ratchet = new DoubleSolenoid(
      PneumaticModuleConstants.kPCM1,
      PneumaticsModuleType.CTREPCM,
      Pneumatic1ChannelConstants.kRatchetLock,
      Pneumatic1ChannelConstants.kRatchetUnlock);

  private final DigitalInput tiltSensor = new DigitalInput(DIOChannelConstants.kTiltAngleSensor);

  private final EventLoop evtLoop = new EventLoop();

  private int oneTime = 0;

  // ==============================================================
  // Define Shuffleboard data
  private final ShuffleboardTab craneTab = Shuffleboard.getTab("Crane");
  private final GenericEntry sbTiltSP = craneTab.addPersistent("Tilt SetPoint", 0)
      .withWidget("Text View")
      .withPosition(0, 2).withSize(2, 1).getEntry();
  private final GenericEntry sbTiltPos = craneTab.addPersistent("Tilt Pos", 0)
      .withWidget("Text View")
      .withPosition(2, 2).withSize(2, 1).getEntry();
  private final GenericEntry sbTiltVel = craneTab.addPersistent("Tilt Vel", 0)
      .withWidget("Text View")
      .withPosition(4, 2).withSize(2, 1).getEntry();
  private final GenericEntry sbTiltFactor = craneTab.addPersistent("Tilt Factor (dpr)", 0)
      .withWidget("Text View")
      .withPosition(7, 2).withSize(2, 1).getEntry();

  private final GenericEntry sbTiltSensor = craneTab.addPersistent("Tilt Sensor", false)
      .withWidget("Boolean Box")
      .withPosition(1, 5).withSize(2, 1).getEntry();

  Crane crane;

  public enum RatchetState {
    LOCK,
    UNLOCK
  }

  /** Creates a new Crane. */
  public CraneTilt(Crane crane) {
    System.out.println("+++++ CraneTilt Constructor starting +++++");

    this.crane = crane;

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

    BooleanEvent tiltAtSensor = new BooleanEvent(evtLoop, () -> getTiltSensor());

    tiltAtSensor.rising().ifHigh(() -> initPos(-75.0, 1));

    // ==============================================================
    // Configure encoders
    tiltEncoder.setPositionConversionFactor(CraneConstants.kTiltDegreesPerRotation);

    // ==============================================================
    // Initialize Axis Positions, Set Points, and Cylinder at PowerUp/Reboot
    // This requires that the Tilt is On Hard Stop before PowerUp/Reboot
    reset();

    // ==============================================================
    // Configure ShuffleBoard data
    sbTiltFactor.setDouble(CraneConstants.kTiltDegreesPerRotation);
    sbTiltSP.setDouble(getSetPoint());

    System.out.println("+++++ CraneTilt Constructor finished +++++");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // if (tiltSetPoint != sbTiltSP.getDouble(0.0)) {
    // tiltSetPoint = sbTiltSP.getDouble(0.0);
    // setTiltSetPoint(tiltSetPoint);
    // }
    sbTiltSP.setDouble(setPoint);
    sbTiltPos.setDouble(tiltEncoder.getPosition());
    sbTiltVel.setDouble(tiltEncoder.getVelocity());

    sbTiltSensor.setBoolean(getTiltSensor());
  }

  private boolean getTiltSensor() {
    return !tiltSensor.get();
  }

  public double getTiltSBPos() {
    return sbTiltSP.getDouble(0.0);
  }

  public void reset() {
    initPos(CraneConstants.kTiltInitPos);
    setSetPoint(setPoint);
    setRatchet(RatchetState.UNLOCK);
  }

  public void initPos(double pos) {
    System.out.print("Pos: " + pos);
    if (this.oneTime == 0) {
      System.out.print("  Updated");
      tiltEncoder.setPosition(pos);
    }
    System.out.println(" ");
  }

  public void initPos(double pos, int oneTime) {
    System.out.print("Pos: " + pos + "  oneTime: " + oneTime);
    if (this.oneTime == 0) {
      System.out.print("  Updated");
      tiltEncoder.setPosition(pos);
    }
    this.oneTime++;
    System.out.println("this.oneTime: " + this.oneTime);
  }

  public void setSetPoint(double setPoint) {
    this.setPoint = setPoint;
    tiltPID.setReference(setPoint, CANSparkMax.ControlType.kSmartMotion);
  }

  public boolean atSetPoint() {
    return Math.abs(setPoint - getPosition()) < CraneConstants.kTiltPositionTolerance;
  }

  public boolean atNextPoint() {
    return Math.abs(crane.getGridZ() - getPosition()) < CraneConstants.kTiltPositionTolerance;
  }

  public double getPosition() {
    return tiltEncoder.getPosition();
  }

  public double getSetPoint() {
    return setPoint;
  }

  public void stop() {
    tiltMotor.set(0.0);
  }

  public void setSpeed(double spd) {
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
