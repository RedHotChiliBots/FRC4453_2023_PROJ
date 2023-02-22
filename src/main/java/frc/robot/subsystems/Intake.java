//Create the intake part / Moters that can spin wheels in and out
//moving arms for the intake system
//left and right moter

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorSensorV3.RawColor;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANidConstants;
import frc.robot.Constants.DIOChannelConstants;
import frc.robot.Constants.E;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.Pneumatic0ChannelConstants;
import frc.robot.Constants.Pneumatic1ChannelConstants;
import frc.robot.Constants.PneumaticModuleConstants;

public class Intake extends SubsystemBase {
  private final CANSparkMax leftMotor = new CANSparkMax(
      CANidConstants.kIntakeLeftMotor,
      MotorType.kBrushless);

  private final CANSparkMax rightMotor = new CANSparkMax(
      CANidConstants.kIntakeRightMotor,
      MotorType.kBrushless);

  private final DoubleSolenoid intakeArm = new DoubleSolenoid(
      PneumaticModuleConstants.kPCM0,
      PneumaticsModuleType.CTREPCM,
      Pneumatic0ChannelConstants.kIntakeArmOpen,
      Pneumatic0ChannelConstants.kIntakeArmClose);

  private final DoubleSolenoid intakeBar = new DoubleSolenoid(
      PneumaticModuleConstants.kPCM1,
      PneumaticsModuleType.CTREPCM,
      Pneumatic1ChannelConstants.kIntakeBarEnabled,
      Pneumatic1ChannelConstants.kIntakeBarDisabled);

  private final DigitalInput elemIn = new DigitalInput(DIOChannelConstants.kElementIn);

  private final I2C.Port i2cPort = I2C.Port.kMXP;
  private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch colorMatcher = new ColorMatch();
  private final Color colorCone = new Color(IntakeConstants.coneR, IntakeConstants.coneG, IntakeConstants.coneB);
  private final Color colorCube = new Color(IntakeConstants.cubeR, IntakeConstants.cubeG, IntakeConstants.cubeB);

  private Color detectedColor;
  private ColorMatchResult match;
  private RawColor rawColor;
  private Crane crane;
  private E element;

  public enum MotorState {
    STOP,
    IN,
    OUT
  }

  public enum ArmState {
    OPEN,
    CLOSE
  }

  public enum BarState {
    DEPLOY,
    STOW
  }

  private final ShuffleboardTab intakeTab = Shuffleboard.getTab("Intake");
  private final GenericEntry sbElemInside = intakeTab.add("Element Inside", false)
      .withWidget("Boolean Box").withPosition(0, 0).withSize(1, 1).getEntry();
  private final GenericEntry sbElemRed = intakeTab.add("Red", 0)
      .withWidget("Text View").withPosition(0, 1).withSize(1, 1).getEntry();
  private final GenericEntry sbElemGreen = intakeTab.add("Green", 0)
      .withWidget("Text View").withPosition(0, 2).withSize(1, 1).getEntry();
  private final GenericEntry sbElemBlue = intakeTab.add("Blue", 0)
      .withWidget("Text View").withPosition(0, 3).withSize(1, 1).getEntry();
  private final GenericEntry sbElemIR = intakeTab.add("IR", 0)
      .withWidget("Text View").withPosition(0, 4).withSize(1, 1).getEntry();
  private final GenericEntry sbElemConf = intakeTab.add("Confidence", 0)
      .withWidget("Text View").withPosition(1, 0).withSize(1, 1).getEntry();
  private final GenericEntry sbElem = intakeTab.addPersistent("Element", "")
      .withWidget("Text View").withPosition(1, 1).withSize(1, 1).getEntry();
  private final GenericEntry sbElemCone = intakeTab.addPersistent("Cone", true)
      .withWidget("Boolean Box").withPosition(1, 2).withSize(1, 1).getEntry();
  private final GenericEntry sbElemCube = intakeTab.addPersistent("Cube", false)
      .withWidget("Boolean Box").withPosition(1, 3).withSize(1, 1).getEntry();

  /** Creates a new Intake. */
  public Intake(Crane crane) {
    System.out.println("+++++ Intake Constructor starting +++++");
    this.crane = crane;

    leftMotor.restoreFactoryDefaults();
    rightMotor.restoreFactoryDefaults();
    leftMotor.clearFaults();
    rightMotor.clearFaults();
    leftMotor.setIdleMode(IdleMode.kBrake);
    rightMotor.setIdleMode(IdleMode.kBrake);

    setMotor(MotorState.STOP);
    setArm(ArmState.CLOSE);
    setBar(BarState.STOW);

    colorMatcher.setConfidenceThreshold(IntakeConstants.colorConf);
    colorMatcher.addColorMatch(colorCone);
    colorMatcher.addColorMatch(colorCube);

    System.out.println("+++++ Intake Constructor finishing +++++");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    detectedColor = colorSensor.getColor();
    match = colorMatcher.matchColor(detectedColor);

    rawColor = colorSensor.getRawColor();

    if (match == null) {
      element = E.NA;
    } else if (match.color == colorCone) {
      element = E.CONE;
    } else if (match.color == colorCube) {
      element = E.CUBE;
    } else {
      element = E.OTHER;
    }

    crane.setElem(element);

    sbElemRed.setDouble(colorSensor.getRed());
    sbElemGreen.setDouble(colorSensor.getGreen());
    sbElemBlue.setDouble(colorSensor.getBlue());
    sbElemIR.setDouble(rawColor.ir);
//    sbElemConf.setDouble(match.confidence);
    sbElemInside.setBoolean(isElementIn());

    // sbElem.setString(crane.getElem().toString());

    // if (crane.getElem() == E.CONE) {
    //   sbElemCone.setBoolean(true);
    // } else {
    //   sbElemCone.setBoolean(false);
    // }

    // if (crane.getElem() == E.CUBE) {
    //   sbElemCube.setBoolean(true);
    // } else {
    //   sbElemCube.setBoolean(false);
    // }
  }

  public boolean isElementIn() {
    return !elemIn.get();
  }

  public E getElement() {
    return element;
  }

  public void toggleElem() {
    E elem = crane.getElem();
    switch (elem) {
      case CONE:
        crane.setElem(E.CUBE);
      break;

      case CUBE:
        crane.setElem(E.CONE);
      break;
    }
  }

  public void setMotor(MotorState state) {
    switch (state) {
      case STOP:
        leftMotor.set(0.0);
        rightMotor.set(0.0);
        break;
      case IN:
        leftMotor.set(0.25);
        rightMotor.set(-0.25);
        break;
      case OUT:
        leftMotor.set(-0.25);
        rightMotor.set(0.25);
        break;
      default:
    }
  }

  public void setArm(ArmState state) {
    switch (state) {
      case OPEN:
        intakeArm.set(Value.kForward);
        break;
      case CLOSE:
        intakeArm.set(Value.kReverse);
        break;
      default:
    }
  }

  public void setBar(BarState state) {
    switch (state) {
      case STOW:
        intakeBar.set(Value.kForward);
        break;
      case DEPLOY:
        intakeBar.set(Value.kReverse);
        break;
      default:
    }
  }
}
