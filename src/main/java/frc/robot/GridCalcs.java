// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.EnumMap;
import java.util.Map;

/** Add your docs here. */
public class GridCalcs {

	public enum ALLIENCE {
		RED,
		BLUE
	}

	public enum E {
		CONE,
		CUBE
	}

	public enum V {
		TOP,
		MID,
		BOT
	}

	public enum H {
		LEFT,
		CENTER,
		RIGHT
	}

	public enum C {
		X,
		Y,
		Z
	}

	private final EnumMap<C, Double> kRobotArm = new EnumMap<>(Map.of(
			C.X, 0.0,
			C.Y, -8.75,
			C.Z, 39.0));

	public double getRobotArm(C c) {
		return kRobotArm.get(c);
	}

	private final EnumMap<E, Double> kZG = new EnumMap<>(Map.of(
			E.CONE, 12.0,
			E.CUBE, 6.0));

	public double getZG(E e) {
		return kZG.get(e);
	}

	private final EnumMap<H, Double> kXNode = new EnumMap<>(Map.of(
			H.LEFT, -21.25,
			H.CENTER, 0.0,
			H.RIGHT, 21.25));

	public double getXNode(H h) {
		return kXNode.get(h);
	}

	private final EnumMap<V, Double> kYNode = new EnumMap<>(Map.of(
			V.TOP, 39.75,
			V.MID, 22.75,
			V.BOT, 8.0));

	public double getYNode(V v) {
		return kYNode.get(v);
	}

	private final EnumMap<V, Map<E, Double>> kZNode = new EnumMap<>(Map.of(
			V.TOP,
			Map.of(E.CONE, 46.0,
					E.CUBE, 35.5),
			V.MID,
			Map.of(E.CONE, 34.0,
					E.CUBE, 23.5),
			V.BOT,
			Map.of(E.CONE, 5.0,
					E.CUBE, 5.0)));

	public double getZNode(V v, E e) {
		return kZNode.get(v).get(e);
	}

	public double getY() {
		return Math.sqrt(
				Math.pow(kXNode.get(horz) - kRobotArm.get(C.X), 2) +
				Math.pow(kYNode.get(vert) - kRobotArm.get(C.Y), 2) +
				Math.pow(kZNode.get(vert).get(elem) + kZG.get(elem) - kRobotArm.get(C.Z), 2));
	}

	public double getX() {
		return Math.toDegrees(Math.asin(kXNode.get(horz) / getY()));
	}

	public double getZ() {
		return Math.toDegrees(Math.asin((kZNode.get(vert).get(elem) + kZG.get(elem) - kRobotArm.get(C.Z)) / getY()));
	}

	private E elem = null;
	private H horz = null;
	private V vert = null;

	public void setElem(E e) {
		this.elem = e;
	}

	public E getElem() {
		return elem;
	}

	public void setVert(V v) {
		this.vert = v;
	}

	public V getVert() {
		return vert;
	}

	public void setHorz(H h) {
		this.horz = h;
	}

	public H getHorz() {
		return horz;
	}
}