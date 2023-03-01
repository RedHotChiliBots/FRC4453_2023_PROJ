package frc.robot.tools;

import java.io.FileOutputStream;
import java.io.IOException;

import frc.robot.GridCalcs;
import frc.robot.Constants.E;
import frc.robot.GridCalcs.C;
import frc.robot.GridCalcs.H;
import frc.robot.GridCalcs.V;

public class PrintCraneConstants {

   static FileOutputStream fout = null;

   public static void printStr(String s) {
      if (fout == null) {
         try {
            fout = new FileOutputStream("constants.txt");
         } catch (IOException e) {
            System.out.println("An error occurred.");
            e.printStackTrace();
         }
      }
      try {
         fout.write(s.getBytes());
      } catch (IOException e1) {
         System.out.println("An error occurred.");
         e1.printStackTrace();
      }
   }

   public static void main(String[] args) {

      GridCalcs grid = new GridCalcs();

      printStr("Robot Arm\n");
      for (C c : C.values()) {
         printStr(String.format("%8s%10.4f", c, grid.getRobotArm(c)));
      }
      printStr("\n");

      printStr("Element Height above Score\n");
      for (E e : E.values()) {
         printStr(String.format("%8s%10.4f", e, grid.getZG(e)));
      }
      printStr("\n");

      printStr("Offset from Center (x)\n");
      for (H h : H.values()) {
         printStr(String.format("%8s%10.4f", h, grid.getXNode(h)));
      }
      printStr("\n");

      printStr("Depth of Grid (y)\n");
      for (V v : V.values()) {
         printStr(String.format("%10s%10.4f", v, grid.getYNode(v)));
      }
      printStr("\n");

      for (E e : E.values()) {
         if (e == E.NA || e == E.OTHER)
            continue;
         printStr(String.format("%-8sHeight from Floor (z)\n", e));
         for (V v : V.values()) {
            printStr(String.format("%10s%10.4f", v, grid.getZNode(v, e)));
         }
         printStr("\n");
      }

      try {
         fout.close();
      } catch (IOException e2) {
         System.out.println("An error occurred.");
         e2.printStackTrace();
      }
   }
}