// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */

  // Change this to match the name of your camera
  PhotonCamera camera = new PhotonCamera("photonvision");

  // Query the latest result from PhotonVision
  PhotonPipelineResult result = null;

  // Check if the latest result has any targets.
  boolean hasTargets = false;

  // Get a list of currently tracked targets.
  List<PhotonTrackedTarget> targets = null;

  // Get the current best target.
  PhotonTrackedTarget target = null;

  // Get information from target.
  double yaw = 0.0;
  double pitch = 0.0;
  double area = 0.0;
  double skew = 0.0;
  Transform3d pose = null;
  List<TargetCorner> corners = null;

  // Get information from target.
  int targetID = 0;
  double poseAmbiguity = 0.0;
  Transform3d bestCameraToTarget = null;
  Transform3d alternateCameraToTarget = null;

  double latencySeconds = 0.0;

  
  public Vision() {
    // Set driver mode to on.
    camera.setDriverMode(true);

    // Change pipeline to 2
    camera.setPipelineIndex(2);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Query the latest result from PhotonVision
    result = camera.getLatestResult();

    // Get the pipeline latency.
    latencySeconds = result.getLatencyMillis() / 1000.0;

    // Check if the latest result has any targets.
    hasTargets = result.hasTargets();

    // Get a list of currently tracked targets.
    targets = result.getTargets();

    // Get the current best target.
    target = result.getBestTarget();

    // Get information from target.
    yaw = target.getYaw();
    pitch = target.getPitch();
    area = target.getArea();
    skew = target.getSkew();
    pose = target.getBestCameraToTarget();
    corners = target.getDetectedCorners();

    // Get information from target.
    targetID = target.getFiducialId();
    poseAmbiguity = target.getPoseAmbiguity();
    bestCameraToTarget = target.getBestCameraToTarget();
    alternateCameraToTarget = target.getAlternateCameraToTarget();
  }

}
