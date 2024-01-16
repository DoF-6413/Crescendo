// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.vision;

import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.drive.Drive;



/** Add your docs here. */
public class PoseEstimator extends SubsystemBase {

    public static Vector<N3> StatesStandarDevs = VecBuilder.fill(0.1,0.1,0.1) ;   

    public static Vector<N3> visionMesumentsStandarDevs= VecBuilder.fill(0.1,0.1,0.1) ;   

    private SwerveDrivePoseEstimator PoseEstimator;

    private Field2d field2d;
    private Drive Drive;
    private Vision vision;
    public PhotonPipelineResult PipelineResult;
    public double ResultsTimeStamp;
    public double LastTimeStamp;

public PoseEstimator(SwerveDriveKinematics kinematics,Drive Drive, Vision vision ) {

   field2d = new Field2d();
   this.Drive = Drive;
   this.vision = vision;


   PoseEstimator = new SwerveDrivePoseEstimator(
    kinematics,
    null/* future gyro.get yaw */ ,
    null/*future drive.getSwervemodulePose or something like that */,
    new Pose2d(new Translation2d(),new Rotation2d()));
}


    
    public Pose2d getCurrentPose2d() {
        return PoseEstimator.getEstimatedPosition();
    }

   
}
