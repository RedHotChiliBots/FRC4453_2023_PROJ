package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;
import org.junit.jupiter.api.MethodOrderer.OrderAnnotation;


@TestMethodOrder(OrderAnnotation.class)
public class DebounceTest {

   Library lib = new Library();

   @Test
   public void TestDebounce() {
      assertEquals(true, lib.deBounce(5));
      assertEquals(false, lib.deBounce(5));
      assertEquals(false, lib.deBounce(5));
      assertEquals(false, lib.deBounce(5));
      assertEquals(false, lib.deBounce(5));
      assertEquals(true, lib.deBounce(5));
      assertEquals(false, lib.deBounce(5));
      assertEquals(false, lib.deBounce(5));
      assertEquals(false, lib.deBounce(5));
      assertEquals(false, lib.deBounce(5));
      assertEquals(true, lib.deBounce(5));
   }
}
