// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.vision;

import org.photonvision.PhotonCamera;

/** Add your docs here. */
public class VisionIOArduCam extends VisionIO {
  public static PhotonCamera camera =
      new PhotonCamera("Arducam_IMX298_Camera_2_Top"); // TODO: update camera names
  // (front & back)

  public VisionIOArduCam() {}

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.PhotonPipelineResult = camera.getLatestResult().toString(); // gets latest camera result
    inputs.HasTargets = camera.getLatestResult().hasTargets();
    if (inputs.HasTargets == true) { // gets following info if there is a target

      inputs.latencyMillis = camera.getLatestResult().getLatencyMillis();

      // inputs.Target = camera.getLatestResult().getBestTarget();
      inputs.BestFiducialID = camera.getLatestResult().getBestTarget().getFiducialId();
      inputs.BestCamToTarget = camera.getLatestResult().getBestTarget().getBestCameraToTarget();
      inputs.TargetX = inputs.BestCamToTarget.getX();
      inputs.TargetY = inputs.BestCamToTarget.getY();
      inputs.TargetZ = inputs.BestCamToTarget.getZ();
      inputs.TargetYaw = camera.getLatestResult().getBestTarget().getYaw();
      inputs.TargetPitch = camera.getLatestResult().getBestTarget().getPitch();
      inputs.TargetArea = camera.getLatestResult().getBestTarget().getArea();
    }
  }
}
