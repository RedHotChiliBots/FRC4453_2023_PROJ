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
      grid.setElem(E.CONE);
      assertEquals(E.CONE, grid.getElem());
      grid.setHorz(H.CENTER);
      assertEquals(H.CENTER, grid.getHorz());
      grid.setVert(V.TOP);
      assertEquals(V.TOP, grid.getVert());
   }
}
