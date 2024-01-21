// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

/** Add your docs here. */
public class VisionIOArduCam implements VisionIO {
  private static PhotonCamera camera =
      new PhotonCamera("Arducam_IMX298_Camera_2_Top"); // TO-DO: update camera names
  // (front & back)

  public VisionIOArduCam() {}

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.HasTargets = getPhotonPipelineResult().hasTargets();
    if (inputs.HasTargets == true) { // gets following info if there is a target
      inputs.BestFiducialID = getPhotonPipelineResult().getBestTarget().getFiducialId();
      inputs.BestCamToTarget = getPhotonPipelineResult().getBestTarget().getBestCameraToTarget();
      inputs.TargetX = inputs.BestCamToTarget.getX();
      inputs.TargetY = inputs.BestCamToTarget.getY();
      inputs.TargetZ = inputs.BestCamToTarget.getZ();
      inputs.TargetYaw = getPhotonPipelineResult().getBestTarget().getYaw();
      inputs.TargetPitch = getPhotonPipelineResult().getBestTarget().getPitch();
      inputs.TargetArea = getPhotonPipelineResult().getBestTarget().getArea();
    }
  }

  @Override
  public PhotonPipelineResult getPhotonPipelineResult() {
    return camera.getLatestResult();
  }
}
