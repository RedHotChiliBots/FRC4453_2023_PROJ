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
			rate = lib.updatePitch(0);
		}
		//ystem.out.println("Rate: " + rate);
		assertEquals(0.0, rate, DELTA);
	}

	@Test
	@Order(2)
	public void testGetMinPitch() throws Exception {
		//System.out.println("Min: " + lib.getMinPitch());
		assertEquals(0.0, lib.getMinPitch(), DELTA);
	}

	@Test
	@Order(3)
	public void testGetMaxPitch() throws Exception {
		//System.out.println("Max: " + lib.getMaxPitch());
		assertEquals(0.0, lib.getMaxPitch(), DELTA);
	}

	@Test
	@Order(4)
	public void testIsPitchDecreasing() throws Exception {
		//System.out.println("Decresing: " + lib.isPitchDecreasing());
		assertFalse(lib.isPitchDecreasing());
	}

	@Test
	@Order(5)
	public void testIsPitchIncreasing() throws Exception {
		//System.out.println("Increasing: " + lib.isPitchIncreasing());
		assertFalse(lib.isPitchDecreasing());
	}

}
