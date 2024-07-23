// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.photonVision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;

/** Add your docs here. */
public class Vision extends SubsystemBase {

  private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
  private final VisionIO io;

  public Vision(VisionIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    if (io.getPhotonPipelineResultBL() != null && io.getPhotonPipelineResultBR() != null) {
      io.updateInputs(inputs);
      Logger.processInputs("Vision", inputs);
    }
  }

  /** Returns the Photon pipeline result from the back left camera */
  public PhotonPipelineResult getResultBL() {
    return io.getPhotonPipelineResultBL();
  }

  /** Returns the Photon pipeline result from the back right camera */
  public PhotonPipelineResult getResultBR() {
    return io.getPhotonPipelineResultBR();
  }

  // /**
  //  * @return the x position of the AprilTag
  //  */
  // public double getTargetX() {

  //   return inputs.TargetX;
  // }
  // /**
  //  * @return the y position of the AprilTag
  //  */
  // public double getTargetY() {

  //   return inputs.TargetY;
  // }
  // /**
  //  * @return z potition of the AprilTag
  //  */
  // public double getTargetZ() {

  //   return inputs.TargetZ;
  // }
  // /**
  //  * @return
  //  */
  // public double getTargetPitch() {

  //   return inputs.TargetX;
  // }
  // /**
  //  * @return the yaw of the AprilTag
  //  */
  // public double getTargetYaw() {

  //   return inputs.TargetX;
  // }
  // /**
  //  * @return the area that the AprilTag uses on the screen
  //  */
  // public double getTargetArea() {

  //   return inputs.TargetArea;
  // }
  // /***
  //  *
  //  * @return if has AprilTag
  //  */
  // public boolean hasAprilTags() {
  //   return inputs.HasTargets;
  // }
  // /***
  //  *
  //  * @return the fiducial ID of the AprilTag
  //  */
  // public int getBestFiducialID() {
  //   return inputs.BestFiducialID;
  // }
}
