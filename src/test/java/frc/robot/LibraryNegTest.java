package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Order;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;
import org.junit.jupiter.api.MethodOrderer.OrderAnnotation;

@TestMethodOrder(OrderAnnotation.class)
public class LibraryNegTest {

	Library lib = new Library();
	double DELTA = 0.0001;

	@Before
	public void init() {
		lib.initLibrary();
	}

	@Test
	@Order(1)
	public void updatePitch() throws Exception {
		lib.initLibrary();
		double rate = 0.0;
		for (int i = 0; i < 5; i++) {
			rate = lib.updatePitch(-i);
		}
		assertEquals(-50.0, rate, DELTA);
	}

	@Test
	@Order(2)
	public void testGetMinPitch() throws Exception {
		assertEquals(-4.0, lib.getMinPitch(), DELTA);
	}

	@Test
	@Order(3)
	public void testGetMaxPitch() throws Exception {
		assertEquals(0.0, lib.getMaxPitch(), DELTA);
	}

	@Test
	@Order(4)
	public void testIsPitchDecreasing() throws Exception {
		assertTrue(lib.isPitchDecreasing());
	}

	@Test
	@Order(5)
	public void testIsPitchIncreasing() throws Exception {
		assertFalse(lib.isPitchIncreasing());
	}
}
