package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;
import org.junit.jupiter.api.MethodOrderer.OrderAnnotation;

import frc.robot.GridCalcs.H;
import frc.robot.GridCalcs.V;

@TestMethodOrder(OrderAnnotation.class)
public class GridPrevNextTest {

   GridCalcs grid = new GridCalcs();

   @Test
   public void TestHorz() {
      grid.horz.set(H.LEFT);
      assertEquals(H.CENTER, grid.horz.next());
      assertEquals(H.RIGHT, grid.horz.next());
      assertEquals(H.RIGHT, grid.horz.next());
      assertEquals(H.CENTER, grid.horz.prev());
      assertEquals(H.LEFT, grid.horz.prev());
      assertEquals(H.LEFT, grid.horz.prev());
   }

   @Test
   public void TestVert() {
      grid.vert.set(V.TOP);
      assertEquals(V.MID, grid.vert.next());
      assertEquals(V.BOT, grid.vert.next());
      assertEquals(V.BOT, grid.vert.next());
      assertEquals(V.MID, grid.vert.prev());
      assertEquals(V.TOP, grid.vert.prev());
      assertEquals(V.TOP, grid.vert.prev());
   }
}
