// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.vision;

import org.photonvision.PhotonCamera;

/** Add your docs here. */
public class VisionIOArduCam extends VisionIO {
    private static PhotonCamera camera = new PhotonCamera("Arducam_IMX298_Camera_2_Top"); // TO-DO: update camera names
                                                                                          // (front & back)

    public VisionIOArduCam() {

    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.PhotonPipelineResult = camera.getLatestResult(); //gets latest camera result
        inputs.HasTargets = inputs.PhotonPipelineResult.hasTargets();
        if (inputs.HasTargets == true) {                        //gets following info if there is a target
            inputs.Target = inputs.PhotonPipelineResult.getBestTarget();
            inputs.BestFiducialID = inputs.Target.getFiducialId();
            inputs.BestCamToTarget = inputs.Target.getBestCameraToTarget();
            inputs.TargetX = inputs.BestCamToTarget.getX();
            inputs.TargetY = inputs.BestCamToTarget.getY();
            inputs.TargetZ = inputs.BestCamToTarget.getZ();
            inputs.TargetYaw = inputs.Target.getYaw();
            inputs.TargetPitch = inputs.Target.getPitch();
            inputs.TargetArea = inputs.Target.getArea();

        }
    }
}
