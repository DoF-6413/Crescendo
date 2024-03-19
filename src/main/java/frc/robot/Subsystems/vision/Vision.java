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
  /** get the photon pipeline result */
  public PhotonPipelineResult getResult() {
    return VisionIO.getPhotonPipelineResult();
  }
  /**
   * @return the x position of the AprilTag
   */
  public double getTargetX() {

    return inputs.TargetX;
  }
  /**
   * @return the y position of the AprilTag
   */
  public double getTargetY() {

    return inputs.TargetY;
  }
  /**
   * @return z potition of the AprilTag
   */
  public double getTargetZ() {

    return inputs.TargetZ;
  }
  /**
   * @return
   */
  public double getTargetPitch() {

    return inputs.TargetX;
  }
  /**
   * @return the yaw of the AprilTag
   */
  public double getTargetYaw() {

    return inputs.TargetX;
  }
  /**
   * @return the area that the AprilTag uses on the screen
   */
  public double getTargetArea() {

    return inputs.TargetArea;
  }
  /***
   *
   * @return if has AprilTag
   */
  public boolean hasAprilTags() {
    return inputs.HasTargets;
  }
  /***
   *
   * @return the fiducial ID of the AprilTag
   */
  public int getBestFiducialID() {
    return inputs.BestFiducialID;
  }
}
