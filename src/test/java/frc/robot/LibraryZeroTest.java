package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;

import org.junit.jupiter.api.Order;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;
import org.junit.jupiter.api.MethodOrderer.OrderAnnotation;

@TestMethodOrder(OrderAnnotation.class)
public class LibraryZeroTest {

	Library lib = new Library();
	double DELTA = 0.0001;

	public void init() {
		lib.initLibrary();
	}

	@Test
	@Order(1)
	public void testUpdatePitch() throws Exception {
		lib.initLibrary();
		double rate = 0.0;
		for (int i = 0; i < 5; i++) {
			rate = lib.updatePitch(0);
		}
		assertEquals(0.0, rate, DELTA);
	}

	@Test
	@Order(2)
	public void testGetMinPitch() throws Exception {
		assertEquals(0.0, lib.getMinPitch(), DELTA);
	}

	@Test
	@Order(3)
	public void testGetMaxPitch() throws Exception {
		assertEquals(0.0, lib.getMaxPitch(), DELTA);
	}

	@Test
	@Order(4)
	public void testIsPitchDecreasing() throws Exception {
		assertFalse(lib.isPitchDecreasing());
	}

	@Test
	@Order(5)
	public void testIsPitchIncreasing() throws Exception {
		assertFalse(lib.isPitchDecreasing());
	}

	@Test
	@Order(6)
	public void tesAvgPitch() throws Exception {
		assertEquals(0.0, lib.getAvgPitch(), DELTA);
	}

	@Test
	@Order(7)
	public void testAvgRate() throws Exception {
		assertEquals(0.0, lib.getAvgRate(), DELTA);
	}
}
