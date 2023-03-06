// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Timer;

import frc.robot.subsystems.Intake.MotorState;

/** Add your docs here. */
public class Execute {


   public Intake intake;
//   public MotorState state;
   public Timer timer = new Timer();
   public boolean oneTime = true;
   public boolean finish = true;

   public Execute(Intake intake) {
      this.intake = intake;
   }
   public void execute(MotorState state) {
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

         } else if (state == MotorState.IN && intake.isElementIn()) {
            System.out.println("State 3");

            // Timer timer = timer.reset();
            oneTime = false;
         }

      } else if (state == MotorState.IN) { // && timer.hasElapsed(0.5)) {
         System.out.println("State 4");

         intake.setMotor(MotorState.STOP);
         finish = true;
      }
   }
}
