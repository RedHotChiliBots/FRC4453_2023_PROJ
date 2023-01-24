package frc.robot;

import org.junit.jupiter.api.Order;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;
import org.junit.jupiter.api.MethodOrderer.OrderAnnotation;

@TestMethodOrder(OrderAnnotation.class)
public class LibraryTest {

	Library lib = new Library();

	@Test
	@Order(1)
	public void testIsPitchIncreasing() throws Exception {
		double rate = 0.0;
		for (int i = 0; i < 5; i++) {
			rate = lib.updatePitch((double)i);
			System.out.println("Loop: " + i + "  Rate: " + rate);
		}
		System.out.println("indexPitch: " + lib.indexPitch);
	}

	@Test
	@Order(2)
	public void testGetMinPitch() throws Exception {
		System.out.println("Min: " + lib.getMinPitch());
	}

	@Test
	@Order(3)
	public void testGetMaxPitch() throws Exception {
		System.out.println("Max: " + lib.getMaxPitch());
	}

	@Test
	@Order(4)
	public void testIsPitchDecreasing() throws Exception {
		System.out.println("Decresing: " + lib.isPitchDecreasing());
	}

	@Test
	@Order(5)
	public void testUpdatePitch() throws Exception {
		System.out.println("Increasing: " + lib.isPitchIncreasing());
	}

}
