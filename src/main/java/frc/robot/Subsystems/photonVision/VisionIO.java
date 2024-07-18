// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.Subsystems.photonVision;

import edu.wpi.first.math.geometry.Transform3d;
import org.littletonrobotics.junction.AutoLog;
import org.photonvision.targeting.PhotonPipelineResult;

public interface VisionIO {

  @AutoLog
  /** creates a vision subsystem */
  public static class VisionIOInputs {
    // Camera inputs for the Back Left Camera
    public boolean BLHasTargets = false; // checks for targets
    public int BLBestFiducialID = 0;
    public Transform3d BLBestCamToTarget = new Transform3d();
    public double BLTargetX = 0.0;
    public double BLTargetY = 0.0;
    public double BLTargetZ = 0.0;
    public double BLTargetYaw = 0.0;
    public double BLTargetPitch = 0.0;
    public double BLTargetArea = 0.0;

    // Camera inputs for the Back Right Camera
    public boolean BRHasTargets = false; // checks for targets
    public int BRBestFiducialID = 0;
    public Transform3d BRBestCamToTarget = new Transform3d();
    public double BRTargetX = 0.0;
    public double BRTargetY = 0.0;
    public double BRTargetZ = 0.0;
    public double BRTargetYaw = 0.0;
    public double BRTargetPitch = 0.0;
    public double BRTargetArea = 0.0;
  }

  public default void updateInputs(VisionIOInputs inputs) {}

  /** Return pipeline from the back left camera */
  public default PhotonPipelineResult getPhotonPipelineResultBL() {
    return null;
  }

  /** returns pipeline from the back right camera */
  public default PhotonPipelineResult getPhotonPipelineResultBR() {
    return null;
  }
}
