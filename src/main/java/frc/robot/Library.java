package frc.robot;

public class Library {

   private static final int rollingPitchSize = 5;
   private static double[] lastPitch = new double[rollingPitchSize];

   private static double ratePitch = 0.0;
   private static double maxPitch = 0.0;
   private static double minPitch = 0.0;
   private static boolean isPitchIncreasing = false;
   private static boolean isPitchDecreasing = false;

   public void Library() {
      initLibrary();
   }

   public void initLibrary() {
      for (int i = 0; i < rollingPitchSize; i++) {
         lastPitch[i] = 0.0;
      }
      ratePitch = 0.0;
      maxPitch = 0.0;
      minPitch = 0.0;
      isPitchIncreasing = false;
      isPitchDecreasing = false;
   }

   public boolean isPitchIncreasing() {
      return isPitchIncreasing;
   }

   public boolean isPitchDecreasing() {
      return isPitchDecreasing;
   }

   public double getMaxPitch() {
      return maxPitch;
   }

   public double getMinPitch() {
      return minPitch;
   }

   public double getRatePitch() {
      return ratePitch;
   }

   public double updatePitch(double pitch) {
      // collect rolling pitch list
      for (int i = 0; i < rollingPitchSize - 1; i++) {
         lastPitch[i] = lastPitch[i + 1];
      }
      lastPitch[rollingPitchSize - 1] = pitch;

      // calc pitch max min
      if (pitch > maxPitch)
         maxPitch = pitch;
      if (pitch < minPitch)
         minPitch = pitch;

      // calc pitch rate
      double sumRate = 0;
      for (int i = 1; i < rollingPitchSize; i++) {
         sumRate += ((lastPitch[i] - lastPitch[i - 1]) / 0.020);
      }
      ratePitch = sumRate / 4;

      // calc slope direction
      isPitchIncreasing = ratePitch > 0.0 ? true : false;
      isPitchDecreasing = ratePitch < 0.0 ? true : false;

      return ratePitch;
   }
}