// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.EnumMap;
import java.util.Map;

import frc.robot.Constants.E;

/** Add your docs here. */
public class GridCalcs {

	public enum ALLIENCE {
		RED,
		BLUE
	}

	public enum CRANESTATE {
		AUTON,
		STOW,
		RECEIVE,
		HOLD,
		GRIP,
		READY,
		NODE,
		CLEAR2MOVE,
		SUBSTATION,
		MOVING
	}

	public enum CRANEAXIS {
		TURRET,
		TILT,
		ARM
	}

	public enum V {
		TOP,
		MIDDLE,
		BOTTOM
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

	public enum DIR {
		FWD,
		REV
	}

	public enum C {
		TURRET,
		ARM,
		TILT
	}

	public final EnumMap<C, Double> kRobotRevPos = new EnumMap<>(Map.of(
			C.TURRET, 0.0,
			C.ARM, -8.75,
			C.TILT, 39.3125));

	public double getRobotRevPos(C c) {
		return kRobotRevPos.get(c);
	}

	public final EnumMap<C, Double> kRobotFwdPos = new EnumMap<>(Map.of(
			C.TURRET, 0.0,
			C.ARM, -20.75,
			C.TILT, 39.3125));

	public double getRobotFwdPos(C c) {
		return kRobotFwdPos.get(c);
	}

	public final EnumMap<C, Double> kSubStation = new EnumMap<>(Map.of(
			C.TURRET, 0.0,
			C.ARM, 12.5,
			C.TILT, 37.375));

	public double getSubStationPos(C c) {
		double result = 0.0;
		double length = kSubStation.get(C.ARM) - kRobotFwdPos.get(C.ARM);
		double height = kSubStation.get(C.TILT) + (getElem() == E.CONE ? 5.0 : 4.5) - kRobotFwdPos.get(C.TILT);

		switch (c) {
			case TURRET:
				result = kSubStation.get(C.TURRET);
				break;
			case ARM:
				result = Math.sqrt(Math.pow(length, 2) + Math.pow(height, 2));
				break;
			case TILT:
				result = Math.toDegrees(Math.atan(height / length));
				break;
			default:
		}
		return result;
	}

	public final EnumMap<E, Double> kZG = new EnumMap<>(Map.of(
			E.CONE, 12.0,
			E.CUBE, 12.0,
			E.NA, 0.0,
			E.OTHER, 0.0));

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
			V.MIDDLE, 22.75,
			V.BOTTOM, 8.0));

	public double getYNode(V v) {
		return kYNode.get(v);
	}

	private final EnumMap<V, Map<E, Double>> kZNode = new EnumMap<>(Map.of(
			V.TOP,
			Map.of(E.CONE, 46.0,
					E.CUBE, 35.5),
			V.MIDDLE,
			Map.of(E.CONE, 34.0,
					E.CUBE, 23.5),
			V.BOTTOM,
			Map.of(E.CONE, 5.0,
					E.CUBE, 5.0)));

	public double getZNode(V v, E e) {
		return kZNode.get(v).get(e);
	}

	private final EnumMap<CRANESTATE, Map<CRANEAXIS, Double>> kCranePos = new EnumMap<>(Map.of(
			CRANESTATE.STOW,
			Map.of(CRANEAXIS.TURRET, 0.0,
					CRANEAXIS.TILT, -87.0,
					CRANEAXIS.ARM, 25.0),
			CRANESTATE.RECEIVE,
			Map.of(CRANEAXIS.TURRET, 0.0,
					CRANEAXIS.TILT, -78.0,
					CRANEAXIS.ARM, 25.0),
			CRANESTATE.GRIP,
			Map.of(CRANEAXIS.TURRET, 0.0,
					CRANEAXIS.TILT, -78.0,
					CRANEAXIS.ARM, 32.0),
			CRANESTATE.READY,
			Map.of(CRANEAXIS.TURRET, 180.0,
					CRANEAXIS.TILT, 0.0,
					CRANEAXIS.ARM, 25.0),
			CRANESTATE.CLEAR2MOVE,
			Map.of(CRANEAXIS.TURRET, 0.0,
					CRANEAXIS.TILT, -45.0,
					CRANEAXIS.ARM, 25.0)));

	public double getCranePos(CRANESTATE cs, CRANEAXIS ca) {
		return kCranePos.get(cs).get(ca);
	}

	private final EnumMap<V, Map<H, Map<E, Boolean>>> kNodeValid = new EnumMap<>(Map.of(
			V.TOP,
			Map.of(H.LEFT,
					Map.of(E.CUBE, false,
							E.CONE, true),
					H.CENTER,
					Map.of(E.CUBE, true,
							E.CONE, false),
					H.RIGHT,
					Map.of(E.CUBE, false,
							E.CONE, true)),
			V.MIDDLE,
			Map.of(H.LEFT,
					Map.of(E.CUBE, false,
							E.CONE, true),
					H.CENTER,
					Map.of(E.CUBE, true,
							E.CONE, false),
					H.RIGHT,
					Map.of(E.CUBE, false,
							E.CONE, true)),
			V.BOTTOM,
			Map.of(H.LEFT,
					Map.of(E.CUBE, true,
							E.CONE, true),
					H.CENTER,
					Map.of(E.CUBE, true,
							E.CONE, true),
					H.RIGHT,
					Map.of(E.CUBE, true,
							E.CONE, true))));

	public boolean isNodeValid(V v, H h, E e) {
		return kNodeValid.get(v).get(h).get(e);
	}

	private final double fudge = 0.914;

	public double getRevArm() {
		if (isNodeValid(vert.get(), horz.get(), getElem())) {
			return fudge * Math.sqrt(
					Math.pow(kXNode.get(horz.get()) - kRobotRevPos.get(C.TURRET), 2) +
							Math.pow(kYNode.get(vert.get()) - kRobotRevPos.get(C.ARM), 2) +
							Math.pow(kZNode.get(vert.get()).get(elem) - kRobotRevPos.get(C.TILT) + kZG.get(elem), 2));
		} else {
			return Float.NaN;
		}
	}

	public double getRevTurret() {
		if (isNodeValid(vert.get(), horz.get(), getElem())) {
			return Math.toDegrees(Math.asin(
					(kXNode.get(horz.get()) - kRobotRevPos.get(C.TURRET)) /
							(getRevArm() / fudge)));
		} else {
			return Float.NaN;
		}
	}

	public double getRevTilt() {
		if (isNodeValid(vert.get(), horz.get(), getElem())) {
			return Math.toDegrees(
					Math.asin(
							(kZNode.get(vert.get()).get(elem) - kRobotRevPos.get(C.TILT) + kZG.get(elem)) /
									(getRevArm() / fudge)));
		} else {
			return Float.NaN;
		}
	}

	public Horz horz = new Horz();
	public Vert vert = new Vert();

	private E elem = E.CONE;

	public void setElem(E e) {
		this.elem = e;
	}

	public E getElem() {
		return elem;
	}
}