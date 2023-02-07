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

	public class Vert {

		V v;

		public Vert() {
		}

		public Vert(V v) {
			set(v);
		}

		public void set(V v) {
			this.v = v;
		}

		public V get() {
			return (v);
		}

		public V next() {
			// No bounds checking required here, because the last instance overrides
			int ord = v.ordinal();
			if (ord < V.values().length - 1)
				v = V.values()[ord + 1];
			return v;
		}

		public V prev() {
			// No bounds checking required here, because the last instance overrides
			int ord = v.ordinal();
			if (ord > 0)
				v = V.values()[ord - 1];
			return v;
		}
	}

	public enum H {
		LEFT,
		CENTER,
		RIGHT
	}

	public class Horz {

		H h;

		public Horz() {
		}

		public Horz(H h) {
			set(h);
		}

		public void set(H h) {
			this.h = h;
		}

		public H get() {
			return (h);
		}

		public H next() {
			// No bounds checking required here, because the last instance overrides
			int ord = h.ordinal();
			if (ord < H.values().length - 1)
				h = H.values()[ord + 1];
			return h;
		}

		public H prev() {
			// No bounds checking required here, because the last instance overrides
			int ord = h.ordinal();
			if (ord > 0)
				h = H.values()[ord - 1];
			return h;
		}
	}

	public enum C {
		X,
		Y,
		Z
	}

	public final EnumMap<C, Double> kRobotArm = new EnumMap<>(Map.of(
			C.X, 0.0,
			C.Y, -8.75,
			C.Z, 39.3125));

	public double getRobotArm(C c) {
		return kRobotArm.get(c);
	}

	public final EnumMap<E, Double> kZG = new EnumMap<>(Map.of(
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
				Math.pow(kXNode.get(horz.get()) - kRobotArm.get(C.X), 2) +
				Math.pow(kYNode.get(vert.get()) - kRobotArm.get(C.Y), 2) +
				Math.pow(kZNode.get(vert.get()).get(elem) + kZG.get(elem) - kRobotArm.get(C.Z), 2));
	}

	public double getX() {
		return Math.toDegrees(Math.asin(kXNode.get(horz.get()) / getY()));
	}

	public double getZ() {
		return Math.toDegrees(Math.asin((kZNode.get(vert.get()).get(elem) + kZG.get(elem) - kRobotArm.get(C.Z)) / getY()));
	}

	public Horz horz = new Horz();
	public Vert vert = new Vert();

	private E elem = null;

	public void setElem(E e) {
		this.elem = e;
	}

	public E getElem() {
		return elem;
	}
}