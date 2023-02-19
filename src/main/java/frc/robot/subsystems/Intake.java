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
  private final Color colorCone = Color.kYellow;
  private final Color colorCube = Color.kPurple;

  private Color detectedColor;
  private ColorMatchResult colorMatch;
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

  private final ShuffleboardTab compTab = Shuffleboard.getTab("Competition");
  private final GenericEntry sbElemIn = compTab.add("Element In", false)
      .withWidget("Boolean Box").withPosition(4, 1).withSize(1, 1).getEntry();

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

    colorMatcher.addColorMatch(colorCone);
    colorMatcher.addColorMatch(colorCube);

    System.out.println("+++++ Intake Constructor finishing +++++");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    detectedColor = colorSensor.getColor();
    colorMatch = colorMatcher.matchClosestColor(detectedColor);
    if (colorMatch.color == colorCone) {
      element = E.CONE;
    } else if (colorMatch.color == colorCube) {
      element = E.CUBE;
    } else {
      element = E.NA;
    }

//    sbElemIn.setBoolean(isElementIn());

    crane.setElem(element);
  }

  public boolean isElementIn() {
    return elemIn.get();
  }

  public E getElement() {
    return element;
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
