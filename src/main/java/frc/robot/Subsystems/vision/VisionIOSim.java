// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFields;
import org.photonvision.PhotonCamera;

/** Add your docs here. */
public class VisionIOSim extends VisionIO {

  private static PhotonCamera camera =
      new PhotonCamera("Arducam_IMX298_Camera_2_Top"); // TODO: update name
  private final AprilTagFields apriltags = AprilTagFields.k2024Crescendo;

  public void updateInputs(VisionIOInputs inputs) {
    inputs.PhotonPipelineResult = camera.getLatestResult().toString();
    inputs.HasTargets = camera.getLatestResult().hasTargets();
    if (inputs.HasTargets == true) {

      inputs.latencyMillis = camera.getLatestResult().getLatencyMillis();

      // inputs.Target = camera.getLatestResult().getBestTarget();
      inputs.BestFiducialID = camera.getLatestResult().getBestTarget().getFiducialId();
      inputs.BestCamToTarget = camera.getLatestResult().getBestTarget().getBestCameraToTarget();

      inputs.AltCamToTag = camera.getLatestResult().getBestTarget().getAlternateCameraToTarget();
      inputs.TargetSkew = camera.getLatestResult().getBestTarget().getSkew();
      inputs.PoseAmbiguity = camera.getLatestResult().getBestTarget().getPoseAmbiguity();

      inputs.TargetX = camera.getLatestResult().getBestTarget().getBestCameraToTarget().getX();
      inputs.TargetY = camera.getLatestResult().getBestTarget().getBestCameraToTarget().getY();
      inputs.TargetZ = camera.getLatestResult().getBestTarget().getBestCameraToTarget().getZ();
      inputs.TargetYaw = camera.getLatestResult().getBestTarget().getYaw();
      inputs.TargetPitch = camera.getLatestResult().getBestTarget().getPitch();
      inputs.TargetArea = camera.getLatestResult().getBestTarget().getArea();
    }
  }
}
