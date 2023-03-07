package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;
import org.junit.jupiter.api.MethodOrderer.OrderAnnotation;

import frc.robot.Constants.E;
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
      assertEquals(-21.3291, grid.getRevTurret(), DELTA);
      assertEquals(53.3989, grid.getRevArm(), DELTA);
      assertEquals(24.9964, grid.getRevTilt(), DELTA);

      grid.horz.set(H.RIGHT);
      assertEquals(21.3291, grid.getRevTurret(), DELTA);
      assertEquals(53.3989, grid.getRevArm(), DELTA);
      assertEquals(24.9964, grid.getRevTilt(), DELTA);

      grid.setElem(E.CUBE);
      grid.horz.set(H.CENTER);
      assertEquals(0.0, grid.getRevTurret(), DELTA);
      assertEquals(44.9562, grid.getRevArm(), DELTA);
      assertEquals(9.5820, grid.getRevTilt(), DELTA);

      grid.horz.set(H.LEFT);
      grid.setElem(E.CUBE);
      assertEquals(Float.NaN, grid.getRevTurret(), DELTA);
      assertEquals(Float.NaN, grid.getRevArm(), DELTA);
      assertEquals(Float.NaN, grid.getRevTilt(), DELTA);

      grid.horz.set(H.RIGHT);
      assertEquals(Float.NaN, grid.getRevTurret(), DELTA);
      assertEquals(Float.NaN, grid.getRevArm(), DELTA);
      assertEquals(Float.NaN, grid.getRevTilt(), DELTA);

      grid.setElem(E.CONE);
      grid.horz.set(H.CENTER);
      assertEquals(Float.NaN, grid.getRevTurret(), DELTA);
      assertEquals(Float.NaN, grid.getRevArm(), DELTA);
      assertEquals(Float.NaN, grid.getRevTilt(), DELTA);
   }

   @Test
   public void TestMid() {
      grid.setElem(E.CONE);
      grid.vert.set(V.MIDDLE);
      grid.horz.set(H.LEFT);
      assertEquals(-32.0363, grid.getRevTurret(), DELTA);
      assertEquals(36.6146, grid.getRevArm(), DELTA);
      assertEquals(18.4643, grid.getRevTilt(), DELTA);

      grid.horz.set(H.RIGHT);
      assertEquals(32.0363, grid.getRevTurret(), DELTA);
      assertEquals(36.6146, grid.getRevArm(), DELTA);
      assertEquals(18.4643, grid.getRevTilt(), DELTA);

      grid.setElem(E.CUBE);
      grid.horz.set(H.CENTER);
      assertEquals(0.0, grid.getRevTurret(), DELTA);
      assertEquals(29.0011, grid.getRevArm(), DELTA);
      assertEquals(-6.9010, grid.getRevTilt(), DELTA);

      grid.horz.set(H.LEFT);
      grid.setElem(E.CUBE);
      assertEquals(Float.NaN, grid.getRevTurret(), DELTA);
      assertEquals(Float.NaN, grid.getRevArm(), DELTA);
      assertEquals(Float.NaN, grid.getRevTilt(), DELTA);

      grid.horz.set(H.RIGHT);
      assertEquals(Float.NaN, grid.getRevTurret(), DELTA);
      assertEquals(Float.NaN, grid.getRevArm(), DELTA);
      assertEquals(Float.NaN, grid.getRevTilt(), DELTA);

      grid.setElem(E.CONE);
      grid.horz.set(H.CENTER);
      assertEquals(Float.NaN, grid.getRevTurret(), DELTA);
      assertEquals(Float.NaN, grid.getRevArm(), DELTA);
      assertEquals(Float.NaN, grid.getRevTilt(), DELTA);
   }

   @Test
   public void TestBot() {
      grid.setElem(E.CONE);
      grid.vert.set(V.BOTTOM);
      grid.horz.set(H.LEFT);
      assertEquals(-42.2666, grid.getRevTurret(), DELTA);
      assertEquals(28.8775, grid.getRevArm(), DELTA);
      assertEquals(-31.0847, grid.getRevTilt(), DELTA);

      grid.horz.set(H.CENTER);
      assertEquals(0.0, grid.getRevTurret(), DELTA);
      assertEquals(21.3700, grid.getRevArm(), DELTA);
      assertEquals(-44.2418, grid.getRevTilt(), DELTA);

      grid.horz.set(H.RIGHT);
      assertEquals(42.2666, grid.getRevTurret(), DELTA);
      assertEquals(28.8775, grid.getRevArm(), DELTA);
      assertEquals(-31.0847, grid.getRevTilt(), DELTA);

      grid.setElem(E.CUBE);
      grid.horz.set(H.LEFT);
      assertEquals(-37.2946, grid.getRevTurret(), DELTA);
      assertEquals(32.0548, grid.getRevArm(), DELTA);
      assertEquals(-39.5097, grid.getRevTilt(), DELTA);

      grid.horz.set(H.CENTER);
      assertEquals(0.0, grid.getRevTurret(), DELTA);
      assertEquals(25.5006, grid.getRevArm(), DELTA);
      assertEquals(-53.1044, grid.getRevTilt(), DELTA);

      grid.horz.set(H.RIGHT);
      assertEquals(37.2946, grid.getRevTurret(), DELTA);
      assertEquals(32.0548, grid.getRevArm(), DELTA);
      assertEquals(-39.5097, grid.getRevTilt(), DELTA);
   }
}
