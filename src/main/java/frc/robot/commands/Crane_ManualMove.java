// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Library;
import frc.robot.Constants.CraneConstants;
import frc.robot.subsystems.Crane;
import frc.robot.subsystems.CraneArm;
import frc.robot.subsystems.CraneTilt;
import frc.robot.subsystems.CraneTurret;

public class Crane_ManualMove extends CommandBase {
  Crane crane;
  CraneTurret craneTurret;
  CraneTilt craneTilt;
  CraneArm craneArm;
  DoubleSupplier turretAxis;
  DoubleSupplier tiltAxis;
  DoubleSupplier armAxis;

  /** Creates a new CraneMove2Pos. */
  public Crane_ManualMove(Crane crane, CraneTurret craneTurret, CraneTilt craneTilt, CraneArm craneArm,
      DoubleSupplier turretAxis, DoubleSupplier tiltAxis, DoubleSupplier armAxis) {
    this.crane = crane;
    this.craneTurret = craneTurret;
    this.craneTilt = craneTilt;
    this.craneArm = craneArm;
    this.turretAxis = turretAxis;
    this.tiltAxis = tiltAxis;
    this.armAxis = armAxis;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(crane);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    craneTurret.setTurretSetPoint(Library.clamp(
        craneTurret.getTurretSetPoint() +
            (Math.abs(turretAxis.getAsDouble()) < 0.1 ? 0.0 : turretAxis.getAsDouble() * CraneConstants.kTurretInc),
        CraneConstants.kTurretMax,
        CraneConstants.kTurretMin));
    craneTilt.setTiltSetPoint(Library.clamp(
        craneTilt.getTiltSetPoint() +
            (Math.abs(tiltAxis.getAsDouble()) < 0.1 ? 0.0 : tiltAxis.getAsDouble() * CraneConstants.kTiltInc),
        CraneConstants.kTiltMax,
        CraneConstants.kTiltMin));
    craneArm.setArmSetPoint(Library.clamp(
        craneArm.getArmSetPoint() +
            (Math.abs(armAxis.getAsDouble()) < 0.1 ? 0.0 : armAxis.getAsDouble() * CraneConstants.kArmInc),
        CraneConstants.kArmMax,
        CraneConstants.kArmMin));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
