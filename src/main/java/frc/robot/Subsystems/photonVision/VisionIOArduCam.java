// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.photonVision;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionIOArduCam implements VisionIO {
  /** Creates a camera */
  private static PhotonCamera backLeftCam = new PhotonCamera("Back_Left");

  private static PhotonCamera backRightCam = new PhotonCamera("Back_Right");

  public VisionIOArduCam() {}

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // Camera inputs for the Back Left Camera
    inputs.BLHasTargets = getPhotonPipelineResultBL().hasTargets();
    if (inputs.BLHasTargets == true) { // gets following info if there is a target
      inputs.BLBestFiducialID = getPhotonPipelineResultBL().getBestTarget().getFiducialId();
      inputs.BLBestCamToTarget =
          getPhotonPipelineResultBL().getBestTarget().getBestCameraToTarget();
      // inputs.BLTargetX = inputs.BLBestCamToTarget.getX();
      // inputs.BLTargetY = inputs.BLBestCamToTarget.getY();
      // inputs.BLTargetZ = inputs.BLBestCamToTarget.getZ();
      // inputs.BLTargetYaw = getPhotonPipelineResultBL().getBestTarget().getYaw();
      // inputs.BLTargetPitch = getPhotonPipelineResultBL().getBestTarget().getPitch();
      // inputs.BLTargetArea = getPhotonPipelineResultBL().getBestTarget().getArea();
    }

    // Camera inputs for the Back Right Camera
    inputs.BRHasTargets = getPhotonPipelineResultBR().hasTargets();
    if (inputs.BRHasTargets == true) { // gets following info if there is a target
      inputs.BRBestFiducialID = getPhotonPipelineResultBR().getBestTarget().getFiducialId();
      inputs.BRBestCamToTarget =
          getPhotonPipelineResultBR().getBestTarget().getBestCameraToTarget();
      // inputs.BRTargetX = inputs.BRBestCamToTarget.getX();
      // inputs.BRTargetY = inputs.BRBestCamToTarget.getY();
      // inputs.BRTargetZ = inputs.BRBestCamToTarget.getZ();
      // inputs.BRTargetYaw = getPhotonPipelineResultBR().getBestTarget().getYaw();
      // inputs.BRTargetPitch = getPhotonPipelineResultBR().getBestTarget().getPitch();
      // inputs.BRTargetArea = getPhotonPipelineResultBR().getBestTarget().getArea();
    }
  }

  @Override
  public PhotonPipelineResult getPhotonPipelineResultBL() {
    return backLeftCam.getLatestResult();
  }

  @Override
  public PhotonPipelineResult getPhotonPipelineResultBR() {
    return backRightCam.getLatestResult();
  }
}
