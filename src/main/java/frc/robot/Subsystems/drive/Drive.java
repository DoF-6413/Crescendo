// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*; //ChassisSpeeds, SwerveDriveKinematics, SwerveModuleStates
import edu.wpi.first.wpilibj.*; //Timer
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

/** This Runs the full Swerve (All Modules) for all Modes of the Robot */
public class Drive extends SubsystemBase {
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private static final Module[] modules = new Module[4];

  private double maxAngularSpeed;
  
  //swerve kinematics library
  public SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(getModuleTranslations());

  //chassis & swerve modules 
  private ChassisSpeeds setpoint = new ChassisSpeeds();
  private SwerveModuleState[] lastSetpointStates = new SwerveModuleState[] {
    for (int i = 0; i < 4; i++) {
      new SwerveModuleState(); 
    }
  };

  private Timer lastMovementTimer = new Timer();

  public Drive(ModuleIO FLModuleIO, ModuleIO FRModuleIO, ModuleIO BLModuleIO, ModuleIO BRModuleIO) {
    System.out.println("[Init] Creating Drive");
    modules[0] = new Module(FLModuleIO, 0);
    modules[1] = new Module(FRModuleIO, 1);
    modules[2] = new Module(BLModuleIO, 2);
    modules[3] = new Module(BRModuleIO, 3);
  }

  @Override
  public void periodic() {
  }

  public Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(DriveConstants.TRACK_WIDTH / 2.0, DriveConstants.TRACK_WIDTH / 2.0),
      new Translation2d(DriveConstants.TRACK_WIDTH / 2.0, -DriveConstants.TRACK_WIDTH / 2.0),
      new Translation2d(-DriveConstants.TRACK_WIDTH / 2.0, DriveConstants.TRACK_WIDTH / 2.0),
      new Translation2d(-DriveConstants.TRACK_WIDTH / 2.0, -DriveConstants.TRACK_WIDTH / 2.0)
    };
  }
}
