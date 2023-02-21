// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Intake.MotorState;

/** Add your docs here. */
public class Intake {
   boolean elemIn;
   MotorState state;

   public void setIsElemIn(boolean in) {
      this.elemIn = in;
   }

   public boolean isElementIn() {
      return elemIn;
   }

   public MotorState getMotor() {
      return state;
   }

   public void setMotor(MotorState state) {
      this.state = state;
   }
}
