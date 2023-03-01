package frc.robot.tools;

import java.io.FileOutputStream;
import java.io.IOException;

import frc.robot.GridCalcs;
import frc.robot.Constants.E;
import frc.robot.GridCalcs.H;
import frc.robot.GridCalcs.V;

public class PrintCranePositions {

   static FileOutputStream fout = null;

   public static void printStr(String s) {
      if (fout == null) {
         try {
            fout = new FileOutputStream("positions.txt");
         } catch (IOException e) {
            printStr("An error occurred.");
            e.printStackTrace();
         }
      }
      try {
         fout.write(s.getBytes());
      } catch (IOException e1) {
         printStr("An error occurred.");
         e1.printStackTrace();
      }
   }

   public static void main(String[] args) {

      GridCalcs grid = new GridCalcs();

      for (E e : E.values()) {
         if (e == E.NA || e == E.OTHER)
            continue;

         printStr(String.format("\n======= %s =======\n", e));
         grid.setElem(e);

         for (V v : V.values()) {
            grid.vert.set(v);
            printStr(String.format("\n%8s\n", v));

            for (H h : H.values()) {
               grid.horz.set(h);
               printStr(String.format("%8s\t", h));

               printStr(String.format("Turret: %8.4f   Tilt: %8.4f   Arm: %8.4f\n", grid.getFwdTurret(), grid.getFwdTilt(),
                     grid.getFwdArm()));
            }
         }
      }

      try {
         fout.close();
      } catch (IOException e2) {
         printStr("An error occurred.");
         e2.printStackTrace();
      }
   }
}