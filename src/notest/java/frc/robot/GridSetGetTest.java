package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;
import org.junit.jupiter.api.MethodOrderer.OrderAnnotation;

import frc.robot.Constants.E;
import frc.robot.GridCalcs.H;
import frc.robot.GridCalcs.V;

@TestMethodOrder(OrderAnnotation.class)
public class GridSetGetTest {

   GridCalcs grid = new GridCalcs();

   @Test
   public void testSettersGetters() throws Exception {
      grid.vert.set(V.MIDDLE);
      assertEquals(V.MIDDLE, grid.vert.get());
      grid.horz.set(H.LEFT);
      assertEquals(H.LEFT, grid.horz.get());
      grid.setElem(E.CONE);
      assertEquals(E.CONE, grid.getElem());
      grid.horz.set(H.CENTER);
      assertEquals(H.CENTER, grid.horz.get());
      grid.vert.set(V.TOP);
      assertEquals(V.TOP, grid.vert.get());
      grid.horz.set(H.RIGHT);
      assertEquals(H.RIGHT, grid.horz.get());
      grid.vert.set(V.BOTTOM);
      assertEquals(V.BOTTOM, grid.vert.get());
   }
}
