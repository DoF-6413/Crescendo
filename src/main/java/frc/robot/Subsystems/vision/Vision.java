// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;

/** Add your docs here. */
public class Vision extends SubsystemBase {

  private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
  private final VisionIO VisionIO;

  public Vision(VisionIO io) {
    VisionIO = io;
  }

  public void periodic() {
    if (VisionIO.getPhotonPipelineResult() != null) {
      VisionIO.updateInputs(inputs);
      Logger.processInputs("Vision", inputs);
    }
  }
/**
 * get the photon pipeline result 
*/
  public PhotonPipelineResult getResult() {
    return VisionIO.getPhotonPipelineResult();
  }
/**get the x position that is in that way  of the target */
  public double getTargetX() {

    return inputs.TargetX;
  }
/**
 * get Target 
 */
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
