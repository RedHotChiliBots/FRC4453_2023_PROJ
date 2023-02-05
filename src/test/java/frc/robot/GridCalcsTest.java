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
      grid.setElem(E.CONE);
      grid.setVert(V.TOP);
      grid.setHorz(H.LEFT);
      assertEquals(-22.1933, grid.getX(), DELTA);
      assertEquals(56.2566, grid.getY(), DELTA);
      assertEquals(19.7391, grid.getZ(), DELTA);

      grid.setHorz(H.RIGHT);
      assertEquals(22.1933, grid.getX(), DELTA);
      assertEquals(56.2566, grid.getY(), DELTA);
      assertEquals(19.7391, grid.getZ(), DELTA);

      grid.setElem(E.CUBE);
      grid.setHorz(H.CENTER);
      assertEquals(0.0, grid.getX(), DELTA);
      assertEquals(48.5643, grid.getY(), DELTA);
      assertEquals(2.9507, grid.getZ(), DELTA);
   }

   @Test
   public void TestMid() {
      grid.setElem(E.CONE);
      grid.setVert(V.MID);
      grid.setHorz(H.LEFT);
      assertEquals(-33.3664, grid.getX(), DELTA);
      assertEquals(38.6369, grid.getY(), DELTA);
      assertEquals(10.4381, grid.getZ(), DELTA);

      grid.setHorz(H.RIGHT);
      assertEquals(33.3664, grid.getX(), DELTA);
      assertEquals(38.6369, grid.getY(), DELTA);
      assertEquals(10.4381, grid.getZ(), DELTA);

      grid.setElem(E.CUBE);
      grid.setHorz(H.CENTER);
      assertEquals(0.0, grid.getX(), DELTA);
      assertEquals(32.9013, grid.getY(), DELTA);
      assertEquals(-16.7826, grid.getZ(), DELTA);
   }

   @Test
   public void TestBot() {
      grid.setElem(E.CONE);
      grid.setVert(V.BOT);
      grid.setHorz(H.LEFT);
      assertEquals(-37.5428, grid.getX(), DELTA);
      assertEquals(34.8729, grid.getY(), DELTA);
      assertEquals(-39.1136, grid.getZ(), DELTA);

      grid.setHorz(H.CENTER);
      assertEquals(0.0, grid.getX(), DELTA);
      assertEquals(27.6507, grid.getY(), DELTA);
      assertEquals(-52.7156, grid.getZ(), DELTA);

      grid.setHorz(H.RIGHT);
      assertEquals(37.5428, grid.getX(), DELTA);
      assertEquals(34.8729, grid.getY(), DELTA);
      assertEquals(-39.1136, grid.getZ(), DELTA);

      grid.setElem(E.CUBE);
      grid.setHorz(H.LEFT);
      assertEquals(-33.0757, grid.getX(), DELTA);
      assertEquals(38.9374, grid.getY(), DELTA);
      assertEquals(-45.9803, grid.getZ(), DELTA);

      grid.setHorz(H.CENTER);
      assertEquals(0.0, grid.getX(), DELTA);
      assertEquals(32.6276, grid.getY(), DELTA);
      assertEquals(-59.1115, grid.getZ(), DELTA);

      grid.setHorz(H.RIGHT);
      assertEquals(33.0757, grid.getX(), DELTA);
      assertEquals(38.9374, grid.getY(), DELTA);
      assertEquals(-45.9803, grid.getZ(), DELTA);
   }
}
