package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;
import org.junit.jupiter.api.MethodOrderer.OrderAnnotation;

import frc.robot.GridCalcs.E;
import frc.robot.GridCalcs.H;
import frc.robot.GridCalcs.V;

@TestMethodOrder(OrderAnnotation.class)
public class GridSetGetTest {

   GridCalcs grid = new GridCalcs();

   @Test
   public void testSettersGetters() throws Exception {
      grid.vert.set(V.MID);
      assertEquals(V.MID, grid.vert.get());
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
      grid.vert.set(V.BOT);
      assertEquals(V.BOT, grid.vert.get());
   }
}
