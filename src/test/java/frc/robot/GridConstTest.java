package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;
import org.junit.jupiter.api.MethodOrderer.OrderAnnotation;

import frc.robot.GridCalcs.E;
import frc.robot.GridCalcs.H;
import frc.robot.GridCalcs.V;
import frc.robot.GridCalcs.C;

@TestMethodOrder(OrderAnnotation.class)
public class GridConstTest {

   GridCalcs grid = new GridCalcs();

   @Test
   public void TestkRobotArm() {
      assertEquals(0.0, grid.getRobotArm(C.X));
      assertEquals(-8.75, grid.getRobotArm(C.Y));
      assertEquals(39.0, grid.getRobotArm(C.Z));
   }

   @Test
   public void TestkZG() {
      assertEquals(12.0, grid.getZG(E.CONE));
      assertEquals(6.0, grid.getZG(E.CUBE));
   }

   @Test
   public void TestXNode() {
      assertEquals(-21.25, grid.getXNode(H.LEFT));
      assertEquals(0.0, grid.getXNode(H.CENTER));
      assertEquals(21.25, grid.getXNode(H.RIGHT));
   }

   @Test
   public void TestYNode() {
      assertEquals(39.75, grid.getYNode(V.TOP));
      assertEquals(22.75, grid.getYNode(V.MID));
      assertEquals(8.0, grid.getYNode(V.BOT));
   }

   @Test
   public void TestZNode() {
      assertEquals(46.0, grid.getZNode(V.TOP, E.CONE));
      assertEquals(35.5, grid.getZNode(V.TOP, E.CUBE));
      assertEquals(34.0, grid.getZNode(V.MID, E.CONE));
      assertEquals(23.5, grid.getZNode(V.MID, E.CUBE));
      assertEquals(5.0, grid.getZNode(V.BOT, E.CONE));
      assertEquals(5.0, grid.getZNode(V.BOT, E.CUBE));
   }
}
