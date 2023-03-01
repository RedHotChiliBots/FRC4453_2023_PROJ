package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class LibraryClampTest {

	Library lib = new Library();
	double DELTA = 0.0001;

	@Test
	public void testClampPos() throws Exception {
		assertEquals(50.0, Library.clamp(50.0, 100.0, 0.0), DELTA);
	}

	@Test
	public void testClampNeg() throws Exception {
		assertEquals(-50.0, Library.clamp(-50.0, 0.0, -100.0), DELTA);
	}

	@Test
	public void testClampMaxPos() throws Exception {
		assertEquals(100.0, Library.clamp(150.0, 100.0, 0.0), DELTA);
	}

	@Test
	public void testClampMaxNeg() throws Exception {
		assertEquals(-100.0, Library.clamp(-150.0, 0.0, -100.0), DELTA);
	}

	@Test
	public void testClampMinPos() throws Exception {
		assertEquals(0.0, Library.clamp(-50.0, 100.0, 0.0), DELTA);
	}

	@Test
	public void testClampMinNeg() throws Exception {
		assertEquals(-100.0, Library.clamp(-150.0, 0.0, -100.0), DELTA);
	}
}
