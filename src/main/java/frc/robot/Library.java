package frc.robot;

public class Library {

   private static final int rollingPitchSize = 5;
   private static double[] lastPitch = new double[rollingPitchSize];

   private static double lastRate = 0.0;
   private static double avgRate = 0.0;
   private static double deltaRate = 0.0;
   private static double lastAvgRate = 0.0;
   private static double maxPitch = 0.0;
   private static double minPitch = 0.0;
   private static double avgPitch = 0.0;
   private static boolean isPitchIncreasing = false;
   private static boolean isPitchDecreasing = false;
   private static boolean dirSwitch = false;
   private static boolean tipSwitch = false;

    public void initLibrary() {
      for (int i = 0; i < rollingPitchSize; i++) {
         lastPitch[i] = 0.0;
      }
      lastRate = 0.0;
      avgRate = 0.0;
      deltaRate = 0.0;
      lastAvgRate = 0.0;
      maxPitch = 0.0;
      minPitch = 0.0;
      avgPitch = 0.0;
      isPitchIncreasing = false;
      isPitchDecreasing = false;
      dirSwitch = false;
      tipSwitch = false;
   }

   public boolean isTipSwitch() {
      return tipSwitch;
   }

   public boolean isDirSwitch() {
      return dirSwitch;
   }

   public boolean isPitchIncreasing() {
      return isPitchIncreasing;
   }

   public boolean isPitchDecreasing() {
      return isPitchDecreasing;
   }

   public double getMinPitch() {
      return minPitch;
   }

   public double getMaxPitch() {
      return maxPitch;
   }

   public double getAvgPitch() {
      return avgPitch;
   }

   public double getAvgRate() {
      return avgRate;
   }

   public double getDeltaRate() {
      return deltaRate;
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
      avgRate = 0.0;
      for (int i = 1; i < rollingPitchSize; i++) {
         avgRate += lastPitch[i] - lastPitch[i - 1];
      }
      avgRate = avgRate / 0.020 / 4;
     
      deltaRate = Math.abs(lastAvgRate - avgRate);
//      System.out.println("avgRate " + avgRate + "    avgPitch " + avgPitch);
      if (avgRate < -0.5 && (avgPitch > 16.0)) {
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