package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Timer;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;

import frc.robot.subsystems.Intake.MotorState;

import org.junit.jupiter.api.MethodOrderer.OrderAnnotation;


@TestMethodOrder(OrderAnnotation.class)
public class IntakeMotorTest {

   public Intake intake = new Intake();
   public Execute execute = new Execute(intake);

   @Test
   public void TestIntakeMotor() {

      execute.intake.setMotor(MotorState.STOP);
      assertEquals(MotorState.STOP, execute.intake.getMotor());
      assertEquals(true, execute.oneTime);
      assertEquals(true, execute.finish);
      assertEquals(false, execute.intake.isElementIn());

      execute.intake.setIsElemIn(false);
      execute.execute(MotorState.OUT);
      assertEquals(MotorState.OUT, execute.intake.getMotor());
      assertEquals(true, execute.oneTime);
      assertEquals(true, execute.finish);
      assertEquals(false, execute.intake.isElementIn());

      execute.intake.setIsElemIn(false);
      execute.execute(MotorState.IN);
      assertEquals(MotorState.IN, execute.intake.getMotor());
      assertEquals(true, execute.oneTime);
      assertEquals(false, execute.finish);
      assertEquals(false, execute.intake.isElementIn());

      execute.intake.setIsElemIn(true);
      execute.execute(MotorState.IN);
      assertEquals(MotorState.IN, execute.intake.getMotor());
      assertEquals(false, execute.oneTime);
      assertEquals(false, execute.finish);
      assertEquals(true, execute.intake.isElementIn());

      execute.intake.setIsElemIn(true);
      execute.execute(MotorState.IN);
      assertEquals(MotorState.STOP, execute.intake.getMotor());
      assertEquals(false, execute.oneTime);
      assertEquals(true, execute.finish);
      assertEquals(true, execute.intake.isElementIn());

      // execute.intake.setIsElemIn(true);
      // execute.execute(MotorState.IN);
      // assertEquals(MotorState.IN, execute.intake.getMotor());
      // assertEquals(false, execute.oneTime);
      // assertEquals(false, execute.finish);
      // assertEquals(true, execute.intake.isElementIn());

   }
}
