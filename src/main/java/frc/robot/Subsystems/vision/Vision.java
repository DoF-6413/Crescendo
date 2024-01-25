// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** Add your docs here. */
public class Vision extends SubsystemBase {

  private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
  private final VisionIO VisionIO;
  private List<PhotonTrackedTarget> targeT =
      (List<PhotonTrackedTarget>) new PhotonTrackedTarget(0, 0, 0, 0, 0, null, null, 0, null, null);

  public Vision(VisionIO io) {
    VisionIO = io;
  }

  public void periodic() {
    VisionIO.updateInputs(inputs);
    Logger.processInputs("Vision", inputs);
    targeT =
        (List<PhotonTrackedTarget>)
            new PhotonTrackedTarget(
                inputs.TargetYaw,
                inputs.TargetPitch,
                inputs.TargetArea,
                inputs.TargetSkew,
                inputs.BestFiducialID,
                inputs.BestCamToTarget,
                inputs.AltCamToTag,
                inputs.PoseAmbiguity,
                null,
                null);
  }

  public PhotonPipelineResult getResult() {

    return new PhotonPipelineResult(inputs.latencyMillis, this.targeT);
  }

  public double getTargetX() {

    return inputs.TargetX;
  }

  public double getTargetY() {

    return inputs.TargetY;
  }

  public double getTargetZ() {

    return inputs.TargetZ;
  }

  public double getTargetPitch() {

    return inputs.TargetX;
  }

  public double getTargetYaw() {

    return inputs.TargetX;
  }

  public double getTargetArea() {

    return inputs.TargetArea;
  }

  public boolean hasTargets() {
    return inputs.HasTargets;
  }

  public int getBestFiducialID() {
    return inputs.BestFiducialID;
  }
}
