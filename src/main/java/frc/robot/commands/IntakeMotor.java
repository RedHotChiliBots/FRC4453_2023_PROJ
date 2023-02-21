// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.MotorState;

public class IntakeMotor extends CommandBase {
  Intake intake;
  MotorState state;
  Timer timer = new Timer();
  boolean oneTime = true;
  boolean finish = true;

  /** Creates a new GrabCube. */
  public IntakeMotor(Intake intake, MotorState state) {
    this.intake = intake;
    this.state = state;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    oneTime = true;
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (state == MotorState.IN) {
      finish = false;
    }
    System.out.println(
        "oneTime: " + oneTime + "   MotorState: " + state + "    isElementIn: " + intake.isElementIn());

    if (oneTime) {
      System.out.println("State 1");

      if (state != MotorState.IN ||
          (state == MotorState.IN && !intake.isElementIn())) {
            System.out.println("State 2");

        intake.setMotor(state);

      } else if (oneTime && state == MotorState.IN && intake.isElementIn()) {
        System.out.println("State 3");

        timer.reset();
        oneTime = false;
      }

    } else if (state == MotorState.IN && timer.hasElapsed(0.5)) {
      System.out.println("State 4");

      intake.setMotor(MotorState.STOP);
      finish = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
