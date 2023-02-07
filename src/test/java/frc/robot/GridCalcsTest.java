package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;
import org.junit.jupiter.api.MethodOrderer.OrderAnnotation;

import frc.robot.GridCalcs.E;
import frc.robot.GridCalcs.H;
import frc.robot.GridCalcs.V;

@TestMethodOrder(OrderAnnotation.class)
public class GridCalcsTest {

   GridCalcs grid = new GridCalcs();
   double DELTA = 0.0001;

   @Test
   public void TestTop() {
      grid.vert.set(V.TOP);
      grid.horz.set(H.LEFT);
      grid.setElem(E.CONE);
      assertEquals(-22.2369, grid.getX(), DELTA);
      assertEquals(56.1518, grid.getY(), DELTA);
      assertEquals(19.4389, grid.getZ(), DELTA);

      grid.horz.set(H.RIGHT);
      assertEquals(22.2369, grid.getX(), DELTA);
      assertEquals(56.1518, grid.getY(), DELTA);
      assertEquals(19.4389, grid.getZ(), DELTA);

      grid.setElem(E.CUBE);
      grid.horz.set(H.CENTER);
      assertEquals(0.0, grid.getX(), DELTA);
      assertEquals(48.5493, grid.getY(), DELTA);
      assertEquals(2.5824, grid.getZ(), DELTA);
   }

   @Test
   public void TestMid() {
      grid.setElem(E.CONE);
      grid.vert.set(V.MID);
      grid.horz.set(H.LEFT);
      assertEquals(-33.4206, grid.getX(), DELTA);
      assertEquals(38.5815, grid.getY(), DELTA);
      assertEquals(9.9817, grid.getZ(), DELTA);

      grid.horz.set(H.RIGHT);
      assertEquals(33.4206, grid.getX(), DELTA);
      assertEquals(38.5815, grid.getY(), DELTA);
      assertEquals(9.9817, grid.getZ(), DELTA);

      grid.setElem(E.CUBE);
      grid.horz.set(H.CENTER);
      assertEquals(0.0, grid.getX(), DELTA);
      assertEquals(32.9929, grid.getY(), DELTA);
      assertEquals(-17.3022, grid.getZ(), DELTA);
   }

   @Test
   public void TestBot() {
      grid.setElem(E.CONE);
      grid.vert.set(V.BOT);
      grid.horz.set(H.LEFT);
      assertEquals(-37.2946, grid.getX(), DELTA);
      assertEquals(35.0709, grid.getY(), DELTA);
      assertEquals(-39.5097, grid.getZ(), DELTA);

      grid.horz.set(H.CENTER);
      assertEquals(0.0, grid.getX(), DELTA);
      assertEquals(27.9000, grid.getY(), DELTA);
      assertEquals(-53.1044, grid.getZ(), DELTA);

      grid.horz.set(H.RIGHT);
      assertEquals(37.2946, grid.getX(), DELTA);
      assertEquals(35.0709, grid.getY(), DELTA);
      assertEquals(-39.5097, grid.getZ(), DELTA);

      grid.setElem(E.CUBE);
      grid.horz.set(H.LEFT);
      assertEquals(-32.8612, grid.getX(), DELTA);
      assertEquals(39.1627, grid.getY(), DELTA);
      assertEquals(-46.2981, grid.getZ(), DELTA);

      grid.horz.set(H.CENTER);
      assertEquals(0.0, grid.getX(), DELTA);
      assertEquals(32.8962, grid.getY(), DELTA);
      assertEquals(-59.3909, grid.getZ(), DELTA);

      grid.horz.set(H.RIGHT);
      assertEquals(32.8612, grid.getX(), DELTA);
      assertEquals(39.1627, grid.getY(), DELTA);
      assertEquals(-46.2981, grid.getZ(), DELTA);
   }
}
