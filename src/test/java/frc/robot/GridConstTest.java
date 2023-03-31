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
      assertEquals(0.0, grid.getRobotRevPos(C.TURRET), DELTA);
      assertEquals(-8.75, grid.getRobotRevPos(C.ARM), DELTA);
      assertEquals(39.3125, grid.getRobotRevPos(C.TILT), DELTA);
   }

   @Test
   public void TestkZG() {
      assertEquals(12.0, grid.getZG(E.CONE), DELTA);
      assertEquals(12.0, grid.getZG(E.CUBE), DELTA);
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
      assertEquals(22.75, grid.getYNode(V.MIDDLE), DELTA);
      assertEquals(8.0, grid.getYNode(V.BOTTOM), DELTA);
   }

   @Test
   public void TestZNode() {
      assertEquals(46.0, grid.getZNode(V.TOP, E.CONE), DELTA);
      assertEquals(35.5, grid.getZNode(V.TOP, E.CUBE), DELTA);
      assertEquals(34.0, grid.getZNode(V.MIDDLE, E.CONE), DELTA);
      assertEquals(23.5, grid.getZNode(V.MIDDLE, E.CUBE), DELTA);
      assertEquals(5.0, grid.getZNode(V.BOTTOM, E.CONE), DELTA);
      assertEquals(5.0, grid.getZNode(V.BOTTOM, E.CUBE), DELTA);
   }

   @Test
   public void TestSubstation() {
      assertEquals(37.375, grid.kSubStation.get(C.TILT), DELTA);
      assertEquals(12.5, grid.kSubStation.get(C.ARM), DELTA);
      assertEquals(0.0, grid.kSubStation.get(C.TURRET), DELTA);
      
      grid.setElem(E.CONE);
      assertEquals(5.2623, grid.getSubStationPos(C.TILT), DELTA);
      assertEquals(33.3907, grid.getSubStationPos(C.ARM), DELTA);
      assertEquals(0.0, grid.getSubStationPos(C.TURRET), DELTA);
      grid.setElem(E.CUBE);
      assertEquals(4.4069, grid.getSubStationPos(C.TILT), DELTA);
      assertEquals(33.3485, grid.getSubStationPos(C.ARM), DELTA);
      assertEquals(0.0, grid.getSubStationPos(C.TURRET), DELTA);
   }
}
