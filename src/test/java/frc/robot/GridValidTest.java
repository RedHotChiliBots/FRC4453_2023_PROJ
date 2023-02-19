package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;
import org.junit.jupiter.api.MethodOrderer.OrderAnnotation;

import frc.robot.Constants.E;
import frc.robot.GridCalcs.H;
import frc.robot.GridCalcs.V;

@TestMethodOrder(OrderAnnotation.class)
public class GridValidTest {

   GridCalcs grid = new GridCalcs();

   @Test
   public void TestTop() {
      grid.vert.set(V.TOP);
      grid.horz.set(H.LEFT);
      grid.setElem(E.CONE);
      assertEquals(true, grid.isNodeValid(grid.vert.get(), grid.horz.get(), grid.getElem()));
      grid.setElem(E.CUBE);
      assertEquals(false, grid.isNodeValid(grid.vert.get(), grid.horz.get(), grid.getElem()));
      grid.horz.set(H.CENTER);
      grid.setElem(E.CONE);
      assertEquals(false, grid.isNodeValid(grid.vert.get(), grid.horz.get(), grid.getElem()));
      grid.setElem(E.CUBE);
      assertEquals(true, grid.isNodeValid(grid.vert.get(), grid.horz.get(), grid.getElem()));
      grid.horz.set(H.RIGHT);
      grid.setElem(E.CONE);
      assertEquals(true, grid.isNodeValid(grid.vert.get(), grid.horz.get(), grid.getElem()));
      grid.setElem(E.CUBE);
      assertEquals(false, grid.isNodeValid(grid.vert.get(), grid.horz.get(), grid.getElem()));

      grid.vert.set(V.MID);
      grid.horz.set(H.LEFT);
      grid.setElem(E.CONE);
      assertEquals(true, grid.isNodeValid(grid.vert.get(), grid.horz.get(), grid.getElem()));
      grid.setElem(E.CUBE);
      assertEquals(false, grid.isNodeValid(grid.vert.get(), grid.horz.get(), grid.getElem()));
      grid.horz.set(H.CENTER);
      grid.setElem(E.CONE);
      assertEquals(false, grid.isNodeValid(grid.vert.get(), grid.horz.get(), grid.getElem()));
      grid.setElem(E.CUBE);
      assertEquals(true, grid.isNodeValid(grid.vert.get(), grid.horz.get(), grid.getElem()));
      grid.horz.set(H.RIGHT);
      grid.setElem(E.CONE);
      assertEquals(true, grid.isNodeValid(grid.vert.get(), grid.horz.get(), grid.getElem()));
      grid.setElem(E.CUBE);
      assertEquals(false, grid.isNodeValid(grid.vert.get(), grid.horz.get(), grid.getElem()));

      grid.vert.set(V.BOT);
      grid.horz.set(H.LEFT);
      grid.setElem(E.CONE);
      assertEquals(true, grid.isNodeValid(grid.vert.get(), grid.horz.get(), grid.getElem()));
      grid.setElem(E.CUBE);
      assertEquals(true, grid.isNodeValid(grid.vert.get(), grid.horz.get(), grid.getElem()));
      grid.horz.set(H.CENTER);
      grid.setElem(E.CONE);
      assertEquals(true, grid.isNodeValid(grid.vert.get(), grid.horz.get(), grid.getElem()));
      grid.setElem(E.CUBE);
      assertEquals(true, grid.isNodeValid(grid.vert.get(), grid.horz.get(), grid.getElem()));
      grid.horz.set(H.RIGHT);
      grid.setElem(E.CONE);
      assertEquals(true, grid.isNodeValid(grid.vert.get(), grid.horz.get(), grid.getElem()));
      grid.setElem(E.CUBE);
      assertEquals(true, grid.isNodeValid(grid.vert.get(), grid.horz.get(), grid.getElem()));
   }
}
