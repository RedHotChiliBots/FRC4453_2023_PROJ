// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.EnumMap;
import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.GridCalcs;
import frc.robot.Library;
import frc.robot.Constants.CraneConstants;
import frc.robot.Constants.E;
import frc.robot.GridCalcs.CRANESTATE;
import frc.robot.GridCalcs.H;
import frc.robot.GridCalcs.NODEPOS;
import frc.robot.GridCalcs.V;

public class Crane extends SubsystemBase {

  private CRANESTATE craneState = CRANESTATE.STOW;
  private DriverStation.Alliance dsAlliance;
  private int dsLocation;
  private int dpadValue;

  private final XboxController operator;
  public final GridCalcs grid = new GridCalcs();
  private final Library lib = new Library();

  // ==============================================================
  // Define Shuffleboard data
  // private final ShuffleboardTab craneTab = Shuffleboard.getTab("Crane");
  // private final GenericEntry sbCrane = craneTab.add("Crane", "")
  // .withWidget("Network Table Tree")
  // .withPosition(5, 1).withSize(2, 3).getEntry();

  private final ShuffleboardTab craneTab = Shuffleboard.getTab("Crane");
  private final GenericEntry sbDebug = craneTab.addPersistent("Debug", false)
      .withWidget("Text View").withPosition(10, 7).withSize(2, 1).getEntry();

  private final ShuffleboardTab compTab = Shuffleboard.getTab("Competition");
  private final GenericEntry sbAlliancePos = compTab.addPersistent("Alliance-Pos", "")
      .withWidget("Text View").withPosition(6, 0).withSize(2, 1).getEntry();
  private final GenericEntry sbState = compTab.addPersistent("Crane State", "")
      .withWidget("Text View").withPosition(6, 1).withSize(2, 1).getEntry();

  private final GenericEntry sbElem = compTab.addPersistent("Element", "")
      .withWidget("Text View").withPosition(0, 9).withSize(2, 1).getEntry();
  private final GenericEntry sbVert = compTab.addPersistent("Vert Pos", "")
      .withWidget("Text View").withPosition(2, 9).withSize(2, 1).getEntry();
  private final GenericEntry sbHorz = compTab.addPersistent("Horz Pos", "")
      .withWidget("Text View").withPosition(4, 9).withSize(2, 1).getEntry();

  private final EnumMap<NODEPOS, GenericEntry> sbSidePos = new EnumMap<>(Map.of(
      NODEPOS.FRONT, compTab.addPersistent("Front", false)
          .withWidget("Boolean Box").withPosition(2, 2).withSize(2, 1).getEntry(),
      NODEPOS.LEFT, compTab.addPersistent("Left", false)
          .withWidget("Boolean Box").withPosition(1, 3).withSize(2, 1).getEntry(),
      NODEPOS.RIGHT, compTab.addPersistent("Right", false)
          .withWidget("Boolean Box").withPosition(3, 3).withSize(2, 1).getEntry(),
      NODEPOS.BACK, compTab.addPersistent("Back", false)
          .withWidget("Boolean Box").withPosition(2, 4).withSize(2, 1).getEntry()));

  public final EnumMap<V, Map<H, GenericEntry>> sbGridPos = new EnumMap<>(Map.of(
      V.TOP, 
      Map.of(H.LEFT, compTab.addPersistent("Top Left", false)
          .withWidget("Boolean Box").withPosition(0, 6).withSize(2, 1).getEntry(),
          H.CENTER, compTab.addPersistent("Top Center", false)
              .withWidget("Boolean Box").withPosition(2, 6).withSize(2, 1).getEntry(),
          H.RIGHT, compTab.addPersistent("Top Right", false)
              .withWidget("Boolean Box").withPosition(4, 6).withSize(2, 1).getEntry()),
      V.MIDDLE,
      Map.of(H.LEFT, compTab.addPersistent("Mid Left", false)
          .withWidget("Boolean Box").withPosition(0, 7).withSize(2, 1).getEntry(),
          H.CENTER, compTab.addPersistent("Mid Center", false)
              .withWidget("Boolean Box").withPosition(2, 7).withSize(2, 1).getEntry(),
          H.RIGHT, compTab.addPersistent("Mid Right", false)
              .withWidget("Boolean Box").withPosition(4, 7).withSize(2, 1).getEntry()),
      V.BOTTOM,
      Map.of(H.LEFT, compTab.addPersistent("Bot Left", false)
          .withWidget("Boolean Box").withPosition(0, 8).withSize(2, 1).getEntry(),
          H.CENTER, compTab.addPersistent("Bot Center", false)
              .withWidget("Boolean Box").withPosition(2, 8).withSize(2, 1).getEntry(),
          H.RIGHT, compTab.addPersistent("Bot Right", false)
              .withWidget("Boolean Box").withPosition(4, 8).withSize(2, 1).getEntry())));

  private final EnumMap<E, GenericEntry> sbElemType = new EnumMap<>(Map.of(
      E.CONE, compTab.addPersistent("Cone", false)
          .withWidget("Boolean Box").withPosition(1, 5).withSize(2, 1).getEntry(),
      E.CUBE, compTab.addPersistent("Cube", false)
          .withWidget("Boolean Box").withPosition(3, 5).withSize(2, 1).getEntry()));

  /** Creates a new Crane. */
  public Crane(XboxController operator) {
    System.out.println("+++++ Crane Constructor startig +++++");

    this.operator = operator;
    grid.vert.set(V.TOP);
    grid.horz.set(H.LEFT);
    grid.setElem(E.CONE);
    craneState = CRANESTATE.STOW;
    sbGridPos.get(grid.vert.get()).get(grid.horz.get()).setBoolean(true);
    sbSidePos.get(grid.getNodePos()).setBoolean(true);
    sbElemType.get(E.CONE).setBoolean(true);

    dsAlliance = DriverStation.getAlliance();
    dsLocation = DriverStation.getLocation();

    System.out.println("+++++ Crane Constructor finished +++++");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    sbState.setString(getState().toString());
    sbVert.setString(grid.vert.get().toString());
    sbHorz.setString(grid.horz.get().toString());
    sbElem.setString(grid.getElem().toString());
    sbAlliancePos.setString(dsAlliance.toString() + "-" + String.valueOf(dsLocation));

    if (grid.getElem() == E.CONE) {
      sbElemType.get(E.CONE).setBoolean(true);
    } else {
      sbElemType.get(E.CONE).setBoolean(false);
    }

    if (grid.getElem() == E.CUBE) {
      sbElemType.get(E.CUBE).setBoolean(true);
    } else {
      sbElemType.get(E.CUBE).setBoolean(false);
    }

    readDPad();
  }

  public void setSide(NODEPOS n) {
    grid.setNodePos(n);
  }

  public NODEPOS getSide() {
    return grid.getNodePos();
  }

  public void setState(CRANESTATE state) {
    craneState = state;
  }

  public CRANESTATE getState() {
    return craneState;
  }

  public void setElem(E elem) {
    grid.setElem(elem);
  }

  public E getElem() {
    return grid.getElem();
  }

  public double getGridX() {
    return grid.getRevTurret() + CraneConstants.kTurretBackPos;
  }

  public double getGridY() {
    return grid.getRevArm();
  }

  public double getGridZ() {
    return grid.getRevTilt();
  }

  public H getHorz() {
    return grid.horz.get();
  }

  public V getVert() {
    return grid.vert.get();
  }

  public void readDPad() {
    dpadValue = operator.getPOV();

    if (dpadValue != -1 && lib.deBounce(10)) {
      sbSidePos.get(grid.getNodePos()).setBoolean(false);
      switch (dpadValue) {
        case 0:
          grid.setNodePos(NODEPOS.FRONT);
          break;
        case 90:
          grid.setNodePos(NODEPOS.RIGHT);
          break;
        case 180:
          grid.setNodePos(NODEPOS.BACK);
          break;
        case 270:
          grid.setNodePos(NODEPOS.LEFT);
          break;
        default:
      }
      sbSidePos.get(grid.getNodePos()).setBoolean(true);
    }
  }

  // public void readDPad() {
  //   dpadValue = operator.getPOV();

  //   if (dpadValue != -1 && lib.deBounce(10)) {
  //     sbGridPos.get(grid.vert.get()).get(grid.horz.get()).setBoolean(false);
  //     switch (dpadValue) {
  //       case 0:
  //         grid.vert.prev();
  //         break;
  //       case 45:
  //         grid.vert.prev();
  //         grid.horz.next();
  //         break;
  //       case 90:
  //         grid.horz.next();
  //         break;
  //       case 135:
  //         grid.horz.next();
  //         grid.vert.next();
  //         break;
  //       case 180:
  //         grid.vert.next();
  //         break;
  //       case 225:
  //         grid.horz.prev();
  //         grid.vert.next();
  //         break;
  //       case 270:
  //         grid.horz.prev();
  //         break;
  //       case 315:
  //         grid.horz.prev();
  //         grid.vert.prev();
  //         break;
  //       default:
  //     }
  //     sbGridPos.get(grid.vert.get()).get(grid.horz.get()).setBoolean(true);
  //   }
  // }
}
