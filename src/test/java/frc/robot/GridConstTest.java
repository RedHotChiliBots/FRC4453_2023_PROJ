package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;
import org.junit.jupiter.api.MethodOrderer.OrderAnnotation;

import frc.robot.GridCalcs.H;
import frc.robot.GridCalcs.V;
import frc.robot.Constants.E;
import frc.robot.GridCalcs.C;

@TestMethodOrder(OrderAnnotation.class)
public class GridConstTest {

   GridCalcs grid = new GridCalcs();
   double DELTA = 0.0001;

   @Test
   public void TestkRobotArm() {
      assertEquals(0.0, grid.getRobotArm(C.TURRET), DELTA);
      assertEquals(-8.75, grid.getRobotArm(C.ARM), DELTA);
      assertEquals(39.3125, grid.getRobotArm(C.TILT), DELTA);
   }

   @Test
   public void TestkZG() {
      assertEquals(12.0, grid.getZG(E.CONE), DELTA);
      assertEquals(6.0, grid.getZG(E.CUBE), DELTA);
   }

   @Test
   public void TestXNode() {
      assertEquals(-21.25, grid.getTurretNode(H.LEFT), DELTA);
      assertEquals(0.0, grid.getTurretNode(H.CENTER), DELTA);
      assertEquals(21.25, grid.getTurretNode(H.RIGHT), DELTA);
   }

   @Test
   public void TestYNode() {
      assertEquals(39.75, grid.getArmNode(V.TOP), DELTA);
      assertEquals(22.75, grid.getArmNode(V.MIDDLE), DELTA);
      assertEquals(8.0, grid.getArmNode(V.BOTTOM), DELTA);
   }

   @Test
   public void TestZNode() {
      assertEquals(46.0, grid.getTiltNode(V.TOP, E.CONE), DELTA);
      assertEquals(35.5, grid.getTiltNode(V.TOP, E.CUBE), DELTA);
      assertEquals(34.0, grid.getTiltNode(V.MIDDLE, E.CONE), DELTA);
      assertEquals(23.5, grid.getTiltNode(V.MIDDLE, E.CUBE), DELTA);
      assertEquals(5.0, grid.getTiltNode(V.BOTTOM, E.CONE), DELTA);
      assertEquals(5.0, grid.getTiltNode(V.BOTTOM, E.CUBE), DELTA);
   }
}
