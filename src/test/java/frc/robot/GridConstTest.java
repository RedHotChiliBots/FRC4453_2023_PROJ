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
      assertEquals(0.0, grid.getRobotArm(C.X), DELTA);
      assertEquals(-8.75, grid.getRobotArm(C.Y), DELTA);
      assertEquals(39.3125, grid.getRobotArm(C.Z), DELTA);
   }

   @Test
   public void TestkZG() {
      assertEquals(12.0, grid.getZG(E.CONE), DELTA);
      assertEquals(6.0, grid.getZG(E.CUBE), DELTA);
   }

   @Test
   public void TestXNode() {
      assertEquals(-21.25, grid.getXNode(H.LEFT), DELTA);
      assertEquals(0.0, grid.getXNode(H.CENTER), DELTA);
      assertEquals(21.25, grid.getXNode(H.RIGHT), DELTA);
   }

   @Test
   public void TestYNode() {
      assertEquals(39.75, grid.getYNode(V.TOP), DELTA);
      assertEquals(22.75, grid.getYNode(V.MID), DELTA);
      assertEquals(8.0, grid.getYNode(V.BOT), DELTA);
   }

   @Test
   public void TestZNode() {
      assertEquals(46.0, grid.getZNode(V.TOP, E.CONE), DELTA);
      assertEquals(35.5, grid.getZNode(V.TOP, E.CUBE), DELTA);
      assertEquals(34.0, grid.getZNode(V.MID, E.CONE), DELTA);
      assertEquals(23.5, grid.getZNode(V.MID, E.CUBE), DELTA);
      assertEquals(5.0, grid.getZNode(V.BOT, E.CONE), DELTA);
      assertEquals(5.0, grid.getZNode(V.BOT, E.CUBE), DELTA);
   }
}
