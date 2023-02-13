// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.EnumMap;
import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.GridCalcs;
import frc.robot.GridCalcs.H;
import frc.robot.GridCalcs.V;
import frc.robot.GridCalcs.E;

public class Crane extends SubsystemBase {

  public enum CraneState {
    STOW,
    RECEIVE,
    READY,
    NODE,
    MOVING,
    NA
  }

  private CraneState craneState = CraneState.NA;
  private GridCalcs grid = new GridCalcs();
  private XboxController operator;
  private int dpadValue;

  // ==============================================================
  // Define Shuffleboard data
  private final ShuffleboardTab compTab = Shuffleboard.getTab("Competition");
  private final EnumMap<V, Map<H, GenericEntry>> sbGridPos = new EnumMap<>(Map.of(
      V.TOP,
      Map.of(H.LEFT, compTab.addPersistent("Top Left", false).getEntry(),
          H.CENTER, compTab.addPersistent("Top Center", false).getEntry(),
          H.RIGHT, compTab.addPersistent("Top Center", false).getEntry()),
      V.MID,
      Map.of(H.LEFT, compTab.addPersistent("Mid Left", false).getEntry(),
          H.CENTER, compTab.addPersistent("Mid Center", true).getEntry(),
          H.RIGHT, compTab.addPersistent("Mid Center", false).getEntry()),
      V.BOT,
      Map.of(H.LEFT, compTab.addPersistent("Bot Left", false).getEntry(),
          H.CENTER, compTab.addPersistent("BOT Center", false).getEntry(),
          H.RIGHT, compTab.addPersistent("Bot Center", false).getEntry())));

  private final EnumMap<E, GenericEntry> sbElemType = new EnumMap<>(Map.of(
      E.CONE, compTab.addPersistent("Cone", true).getEntry(),
      E.CUBE, compTab.addPersistent("Cube", false).getEntry()));

  /** Creates a new Crane. */
  public Crane(XboxController operator) {
    System.out.println("+++++ Crane Constructor starting +++++");

    this.operator = operator;
    grid.vert.set(V.MID);
    grid.horz.set(H.CENTER);
    grid.setElem(E.CONE);

    System.out.println("+++++ Crane Constructor finished +++++");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    readDPad();
  }

  public void setState(CraneState state) {
    craneState = state;
  }

  public CraneState getState() {
    return craneState;
  }

  public double getGridX() {
    return grid.getX();
  }

  public double getGridY() {
    return grid.getY();
  }

  public double getGridZ() {
    return grid.getZ();
  }

  public void readDPad() {
    dpadValue = operator.getPOV();
    if (dpadValue != -1) {
      sbGridPos.get(grid.vert.get()).get(grid.horz.get()).setBoolean(false);
      switch (dpadValue) {
        case 0:
          grid.vert.prev();
          break;
        case 45:
          grid.vert.prev();
          grid.horz.next();
          break;
        case 90:
          grid.horz.next();
          break;
        case 135:
          grid.horz.next();
          grid.vert.next();
          break;
        case 180:
          grid.vert.next();
          break;
        case 225:
          grid.horz.prev();
          grid.vert.next();
          break;
        case 270:
          grid.horz.prev();
          break;
        case 315:
          grid.horz.prev();
          grid.vert.prev();
          break;
        default:
      }
      sbGridPos.get(grid.vert.get()).get(grid.horz.get()).setBoolean(true);
    }
  }
}
