// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */

  // Change this to match the name of your camera
  private PhotonCamera camera = new PhotonCamera(VisionConstants.kCameraName);
  private PhotonPipelineResult result;
  private PhotonTrackedTarget target;

  private PIDController distController = new PIDController(VisionConstants.kDistP, VisionConstants.kDistI,
      VisionConstants.kDistD);
  private PIDController turnController = new PIDController(VisionConstants.kTurnP, VisionConstants.kTurnI,
      VisionConstants.kTurnD);

  private double forwardSpeed;
  private double rotationSpeed;

  private final ShuffleboardTab visionTab = Shuffleboard.getTab("Vision");
  private final GenericEntry sbYaw = visionTab.addPersistent("Yaw", 0).getEntry();
  private final GenericEntry sbPitch = visionTab.addPersistent("Pitch", 0).getEntry();
  private final GenericEntry sbArea = visionTab.addPersistent("Area", 0).getEntry();
  private final GenericEntry sbSkew = visionTab.addPersistent("Skew", 0).getEntry();
  private final GenericEntry sbHasTarget = visionTab.addPersistent("Has Target", false).getEntry();
  private final GenericEntry sbDistAtTarget = visionTab.addPersistent("Dist At Target", false).getEntry();
  private final GenericEntry sbTurnAtTarget = visionTab.addPersistent("Turn At Target", false).getEntry();
  private final GenericEntry sbTargetID = visionTab.addPersistent("Target ID", 0.0).getEntry();

  // // Query the latest result from PhotonVision
  // PhotonPipelineResult result = null;

  // // Check if the latest result has any targets.
  // boolean hasTargets = false;

  // // Get a list of currently tracked targets.
  // List<PhotonTrackedTarget> targets = null;

  // // Get the current best target.
  // PhotonTrackedTarget target = null;

  // // Get information from target.
  // double yaw = 0.0;
  // double pitch = 0.0;
  // double area = 0.0;
  // double skew = 0.0;
  // Transform3d pose = null;
  // List<TargetCorner> corners = null;

  // // Get information from target.
  // int targetID = 0;
  // double poseAmbiguity = 0.0;
  // Transform3d bestCameraToTarget = null;
  // Transform3d alternateCameraToTarget = null;

  // double latencySeconds = 0.0;

  public Vision() {
    System.out.println("+++++ Vision Constructor starting +++++");

    // // Set driver mode to on.
    // camera.setDriverMode(true);

    // // Change pipeline to 2
    // camera.setPipelineIndex(2);

    System.out.println("+++++ Vision Constructor finished +++++");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Query the latest result from PhotonVision
    // result = camera.getLatestResult();

    // Get the pipeline latency.
    // latencySeconds = result.getLatencyMillis() / 1000.0;

    // Check if the latest result has any targets.
    // hasTargets = result.hasTargets();

    // Get a list of currently tracked targets.
    // targets = result.getTargets();

    // Get the current best target.
    // target = result.getBestTarget();

    // Get information from target.
    // yaw = target.getYaw();
    // pitch = target.getPitch();
    // area = target.getArea();
    // skew = target.getSkew();
    // pose = target.getBestCameraToTarget();
    // corners = target.getDetectedCorners();

    // Get information from target.
    // targetID = target.getFiducialId();
    // poseAmbiguity = target.getPoseAmbiguity();
    // bestCameraToTarget = target.getBestCameraToTarget();
    // alternateCameraToTarget = target.getAlternateCameraToTarget();

    // if (xboxController.getAButton()) {
    // Vision-alignment mode
    // Query the latest result from PhotonVision

    // Get information from target.
    result = camera.getLatestResult();

    if (result.hasTargets()) {
      target = result.getBestTarget();
      
      sbHasTarget.setBoolean(true);

      sbYaw.setDouble(target.getYaw());
      sbPitch.setDouble(target.getPitch());
      sbArea.setDouble(target.getArea());
      sbSkew.setDouble(target.getSkew());
      sbTargetID.setDouble(target.getFiducialId());
      sbDistAtTarget.setBoolean(atDistTarget());
      sbTurnAtTarget.setBoolean(atTurnTarget());

    } else {
      // If we have no targets, stay still.
      sbHasTarget.setBoolean(false);
    }

    // } else {
    // // Manual Driver Mode
    // forwardSpeed = -xboxController.getRightY();
    // rotationSpeed = xboxController.getLeftX();
    // }
  }

  public double[] trackAprilTag() {

    if (hasTargets()) {

      // First calculate range
      double range = PhotonUtils.calculateDistanceToTargetMeters(
          VisionConstants.kCameraHeight,
          VisionConstants.kTargetHeight,
          VisionConstants.kCameraHeight,
          Units.degreesToRadians(target.getPitch()));

      // Use this range as the measurement we give to the PID controller.
      // -1.0 required to ensure positive PID controller effort _increases_ range
      forwardSpeed = -distController.calculate(range, VisionConstants.kTargetDist);

      // Also calculate angular power
      // -1.0 required to ensure positive PID controller effort _increases_ yaw
      rotationSpeed = -turnController.calculate(target.getYaw(), 0);

    } else {
      forwardSpeed = 0;
      rotationSpeed = 0;
    }

    return new double[] { forwardSpeed, rotationSpeed };
  }

  public boolean hasTargets() {
    return result.hasTargets();
  }

  public boolean atDistTarget() {
    return distController.atSetpoint();
  }

  public boolean atTurnTarget() {
    return turnController.atSetpoint();
  }
}
