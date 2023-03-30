package frc.robot.tools;

import java.io.FileOutputStream;
import java.io.IOException;

import frc.robot.Constants.E;
import frc.robot.GridCalcs;
import frc.robot.GridCalcs.H;
import frc.robot.GridCalcs.V;
import frc.robot.GridCalcs.C;
import frc.robot.GridCalcs.CRANEAXIS;
import frc.robot.GridCalcs.CRANESTATE;

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

      printStr(String.format("Grid\n"));

      for (E e : E.values()) {
         if (e == E.NA || e == E.OTHER)
            continue;

         grid.setElem(e);
         printStr(String.format("\n======= %s =======\n", grid.getElem()));
 
         for (CRANESTATE p : CRANESTATE.values()) {
            printStr(String.format("\n======= %s =======\n", p));
            switch (p) {
               case MOVING:
                  break;

               case STOW:
               case RECEIVE:
               case HOLD:
               case GRIP:
               case READY:
               case SUBSTATION:
               case CLEAR2MOVE:
                  printStr(String.format("Turret: %8.4f   Tilt: %8.4f   Arm: %8.4f\n",
                        grid.getCranePos(p, CRANEAXIS.TURRET), // grid.getRevTurret(),
                        grid.getCranePos(p, CRANEAXIS.TILT), // grid.getRevTilt(),
                        grid.getCranePos(p, CRANEAXIS.ARM))); // grid.getRevArm()));
                  break;

               case NODE:
               case LEFT:
               case RIGHT:
                  for (V v : V.values()) {
                     grid.vert.set(v);
                     printStr(String.format("\n%8s\n", grid.vert.get()));

                     for (H h : H.values()) {
                        grid.horz.set(h);
                        printStr(String.format("%8s\t", grid.horz.get()));

                        printStr(String.format("Turret: %8.4f   Tilt: %8.4f   Arm: %8.4f\n",
                              grid.getCranePos(p, CRANEAXIS.TURRET), // grid.getRevTurret(),
                              grid.getCranePos(p, CRANEAXIS.TILT), // grid.getRevTilt(),
                              grid.getCranePos(p, CRANEAXIS.ARM))); // grid.getRevArm()));
                     } // grid.getCranePos(tgtState, CRANEAXIS.TILT)
                  }
                  break;
            }
         }
      }

      printStr(String.format("\nSubstation\n"));

      for (E e : E.values()) {
         if (e == E.NA || e == E.OTHER)
            continue;

         grid.setElem(e);
         printStr(String.format("\n======= %s =======\n", e));

         for (C c : C.values()) {
            printStr(String.format("%8s %8.4f\t", c, grid.getSubStationPos(c)));
         }
         printStr(String.format("\n"));
      }

      try {
         fout.close();
      } catch (IOException e2) {
         printStr("An error occurred.");
         e2.printStackTrace();
      }
   }
}