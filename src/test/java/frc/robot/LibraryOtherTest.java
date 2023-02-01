package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Order;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;
import org.junit.jupiter.api.MethodOrderer.OrderAnnotation;

@TestMethodOrder(OrderAnnotation.class)
public class LibraryOtherTest {

	Library lib = new Library();
	double DELTA = 0.0001;

	public void init() {
		lib.initLibrary();
	}

	@Test
	@Order(1)
	public void testInit1() throws Exception {
		lib.initLibrary();
		double rate = 0.0;
		rate = lib.updatePitch(0.01);

		assertEquals(0.125, rate, DELTA);
	}

	@Test
	@Order(2)
	public void testIsTipSwitch1() throws Exception {

		assertFalse(lib.isTipSwitch());
	}

	@Test
	@Order(3)
	public void testInit2() throws Exception {
		double rate = 0.0;

		rate = lib.updatePitch(0.1);

		assertEquals(1.25, rate, DELTA);
	}

	@Test
	@Order(4)
	public void testIsTipSwitch2() throws Exception {
		assertTrue(lib.isTipSwitch());
	}

}
