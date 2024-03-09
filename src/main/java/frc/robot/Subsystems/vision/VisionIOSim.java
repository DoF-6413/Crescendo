// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

/** Add your docs here. */
public class VisionIOSim implements VisionIO {

  private static PhotonCamera camera = new PhotonCamera("Arducam_Shooter");

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.HasTargets = getPhotonPipelineResult().hasTargets();
    if (inputs.HasTargets == true) {
      inputs.BestFiducialID = camera.getLatestResult().getBestTarget().getFiducialId();
      inputs.BestCamToTarget = camera.getLatestResult().getBestTarget().getBestCameraToTarget();
      inputs.TargetX = camera.getLatestResult().getBestTarget().getBestCameraToTarget().getX();
      inputs.TargetY = camera.getLatestResult().getBestTarget().getBestCameraToTarget().getY();
      inputs.TargetZ = camera.getLatestResult().getBestTarget().getBestCameraToTarget().getZ();
      inputs.TargetYaw = camera.getLatestResult().getBestTarget().getYaw();
      inputs.TargetPitch = camera.getLatestResult().getBestTarget().getPitch();
      inputs.TargetArea = camera.getLatestResult().getBestTarget().getArea();
    }
  }

  @Override
  public PhotonPipelineResult getPhotonPipelineResult() {
    return camera.getLatestResult();
  }
}
