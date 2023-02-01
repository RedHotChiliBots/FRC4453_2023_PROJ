// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.EnumMap;
import java.util.Map;

/** Add your docs here. */
public class GridCalcs {
	public static final class GridConstants {

		public enum ALLIENCE {
			RED,
			BLUE
		}

		public enum GAMEPIECE {
			CONE,
			CUBE
		}

		public enum VERTGRID {
			TOP,
			MID,
			BOT,
		}

		public enum HORZGRID {
			LEFT,
			CENTER,
			RIGHT,
		}

		public enum COORD {
			X,
			Y,
			Z
		}

		private class ROBOTARM {
			private double X;
			private double Y;
			private double Z;

			public ROBOTARM(double X, double Y, double Z) {
				this.Y = X;
				this.Y = Y;
				this.Z = Z;
			}

			public double getX() {
				return this.X;
			}

			public double getY() {
				return this.Y;
			}

			public double getZ() {
				return this.Z;
			}

			public void setX(double X) {
				this.X = X;
			}

			public void setY(double Y) {
				this.Y = Y;
			}

			public void setZ(double Z) {
				this.Z = Z;
			}
		}

		private class GamePiece {
			private double z;

			public GamePiece(double z) {
				this.z = z;
			}

			public double getZ() {
				return this.z;
			}

			public void setZ(double z) {
				this.z = z;
			}
		}

		public class GRIDYZ {
			private double Y;
			private double Z;

			public GRIDYZ(double Y, double Z) {
				this.Y = Y;
				this.Z = Z;
			}

			public double getY() {
				return this.Y;
			}

			public double getZ() {
				return this.Z;
			}

			public void setY(double Y) {
				this.Y = Y;
			}

			public void setZ(double Z) {
				this.Z = Z;
			}
		}

		public class Node {
			private double turretAngle;
			private double tiltAngle;
			private double armLength;

			public Node(double turretAngle, double tiltAngle, double armLength) {
				this.turretAngle = turretAngle;
				this.tiltAngle = tiltAngle;
				this.armLength = armLength;
			}

			public double getTurretAngle() {
				return this.turretAngle;
			}

			public double getTiltAngle() {
				return this.tiltAngle;
			}

			public double getArmLength() {
				return this.armLength;
			}

			public void setTurretAngle(double angle) {
				this.turretAngle = angle;
			}

			public void setTiltAngle(double angle) {
				this.tiltAngle = angle;
			}

			public void setArmLength(double length) {
				this.armLength = length;
			}
		}

		ROBOTARM robotArm = new ROBOTARM(0.0, -8.0, 39.0);

		private static final EnumMap<COORD, Double> kRobotArm = new EnumMap<>(Map.of(
				COORD.X, 0.0,
				COORD.Y, -8.0,
				COORD.Z, 39.0));

		private static final EnumMap<GAMEPIECE, Double> kGamePiece = new EnumMap<>(Map.of(
				GAMEPIECE.CONE, 12.0,
				GAMEPIECE.CUBE, 6.0));

		// private static final EnumMap<GAMEPIECE, EnumMap<VERTGRID, Double>> kGridZ1 = new EnumMap<GAMEPIECE, EnumMap<VERTGRID, Double>>(
		// 		(EnumMap<GAMEPIECE, ? extends EnumMap<VERTGRID, Double>>) Map.of(
		// 				GAMEPIECE.CONE,
		// 				Map.of(VERTGRID.TOP, 46.0,
		// 						VERTGRID.MID, 46.0,
		// 						VERTGRID.BOT, 46.0),
		// 				GAMEPIECE.CUBE,
		// 				Map.of(VERTGRID.TOP, 46.0,
		// 						VERTGRID.MID, 46.0,
		// 						VERTGRID.BOT, 46.0)));

		Node[][] kGrid = new Node[3][3];
	}
}
