// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.Subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;
import org.littletonrobotics.junction.AutoLog;
import org.photonvision.targeting.PhotonPipelineResult;

/** Add your docs here. */
public class VisionIO {

  @AutoLog
  public static class VisionIOInputs {
    public String PhotonPipelineResult = new PhotonPipelineResult().toString();
    public double latencyMillis = 0.0;

    public boolean HasTargets = false; // checks for targets
    // public PhotonTrackedTarget Target = null;
    public int BestFiducialID = 0;
    public Transform3d BestCamToTarget = new Transform3d();

    public Transform3d AltCamToTag = new Transform3d(); // alternative cam to target
    public double TargetSkew = 0.0;
    public double PoseAmbiguity = 0.0;

    public double TargetX = 0.0;
    public double TargetY = 0.0;
    public double TargetZ = 0.0;
    public double TargetYaw = 0.0;
    public double TargetPitch = 0.0;
    public double TargetArea = 0.0;
  }

  public void updateInputs(VisionIOInputs inputs) {}
}
