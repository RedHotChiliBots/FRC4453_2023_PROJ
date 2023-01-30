package frc.robot;

public class Library {

   private static final int rollingPitchSize = 5;
   private static double[] lastPitch = new double[rollingPitchSize];

   private static double lastRate = 0.0;
   private static double avgRate = 0.0;
   private static double lastAvgRate = 0.0;
   private static double maxPitch = 0.0;
   private static double minPitch = 0.0;
   private static double avgPitch = 0.0;
   private static boolean isPitchIncreasing = false;
   private static boolean isPitchDecreasing = false;
   private static boolean dirSwitch = false;
   private static boolean tipSwitch = false;

   public void Library() {
      initLibrary();
   }

   public void initLibrary() {
      for (int i = 0; i < rollingPitchSize; i++) {
         lastPitch[i] = 0.0;
      }
      lastRate = 0.0;
      avgRate = 0.0;
      lastAvgRate = 0.0;
      maxPitch = 0.0;
      minPitch = 0.0;
      avgPitch = 0.0;
      isPitchIncreasing = false;
      isPitchDecreasing = false;
      dirSwitch = false;
      tipSwitch = false;
   }

   public boolean getTipSwitch() {
      return tipSwitch;
   }

   public boolean getDirSwitch() {
      return dirSwitch;
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

   public double getAvgPitch() {
      return avgRate;
   }

   public double getAvgRate() {
      return avgRate;
   }

   public double updatePitch(double pitch) {
      // collect rolling pitch list
      avgPitch = 0.0;
      for (int i = 0; i < rollingPitchSize - 1; i++) {
         lastPitch[i] = lastPitch[i + 1];
         avgPitch += lastPitch[i]; 
      }
      lastPitch[rollingPitchSize - 1] = pitch;
      avgPitch = (avgPitch + pitch) / 5;
      
      // calc pitch max min
      if (pitch > maxPitch)
         maxPitch = pitch;
      if (pitch < minPitch)
         minPitch = pitch;

      // calc pitch rate
      lastAvgRate = avgRate;
      double avgRate = 0;
      for (int i = 1; i < rollingPitchSize; i++) {
         avgRate += ((lastPitch[i] - lastPitch[i - 1]) / 0.020);
      }
      avgRate = avgRate / 4;

      tipSwitch = false;
      if (Math.abs(lastAvgRate - avgRate) > 1.0) {
         tipSwitch = true;
      }

      // calc slope direction
      lastRate = avgRate;
      isPitchIncreasing = avgRate > 0.0 ? true : false;
      isPitchDecreasing = avgRate < 0.0 ? true : false;

      if ((lastRate < 0 && avgRate > 0) || (avgRate < 0 && lastRate > 0)) {
          dirSwitch = true;
      }

      return avgRate;
   }
}