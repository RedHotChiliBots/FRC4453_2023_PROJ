package frc.robot;

public class Library {

   private static double currPitch = 0.0;
   private static double lastPitch = 0.0;
   // private static double[] lastPitch = new double[5];
   // public int indexPitch = 0;
   private static double ratePitch = 0.0;
   private static double maxPitch = 0.0;
   private static double minPitch = 0.0;
   private static boolean isPitchIncreasing = false;
   private static boolean isPitchDecreasing = false;

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

   public double updatePitch(double pitch) {
      // collect pitch list
      lastPitch = currPitch;
      // lastPitch[indexPitch++] = currPitch;
      // if (indexPitch >= 5)
      // indexPitch = 0;
      currPitch = pitch;

      // calc pitch max min
      if (currPitch > maxPitch)
         maxPitch = currPitch;
      if (currPitch < minPitch)
         minPitch = currPitch;

      // calc pitch rate
      ratePitch = (lastPitch - currPitch) / 0.020;

      isPitchIncreasing = ratePitch < 0.0 ? true : false;
      isPitchDecreasing = ratePitch > 0.0 ? true : false;

      return ratePitch;
   }

   // public double updatePitch(double newPitch) {
   //    // collect pitch list
   //    lastPitch[indexPitch++] = newPitch;
   //    if (indexPitch >= 5)
   //       indexPitch = 0;
   //    currPitch = newPitch;
   //    System.out.println("newPitch: " + newPitch);

   //    // calc pitch max min
   //    if (currPitch > maxPitch)
   //       maxPitch = currPitch;
   //    System.out.println("maxPitch: " + maxPitch);
   //    if (currPitch < minPitch)
   //       minPitch = currPitch;
   //    System.out.println("minPitch: " + minPitch);

   //    // calc pitch rate
   //    double sumRate = 0.0;
   //    int j = indexPitch;
   //    int k = indexPitch - 1;
   //    if (k < 0)
   //       k = 4;
   //    for (int i = 0; i < 5; i++) {
   //       System.out.print("[i]: " + lastPitch[i] + "   ");
   //       if (j >= 5)
   //          j = 0;
   //       if (k >= 5)
   //          k = 0;

   //       sumRate += (lastPitch[j++] - lastPitch[k++]) / 0.020;
   //    }
   //    System.out.println("");
   //    System.out.println("sumRate: " + sumRate);

   //    ratePitch = sumRate / lastPitch.length;
   //    System.out.println("ratePitch: " + ratePitch);

   //    isPitchIncreasing = ratePitch < 0.0 ? true : false;
   //    isPitchDecreasing = ratePitch > 0.0 ? true : false;

   //    return ratePitch;
   // }
}