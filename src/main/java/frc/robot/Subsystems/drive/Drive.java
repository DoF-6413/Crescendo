// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.drive;

import java.util.Arrays;

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
    lastMovementTimer.start(); //start timer
    for (var module : modules) { 
      module.setBrakeMode(true); //set brake mode to be true by default
    }
  }

  @Override
  public void periodic() {

    for(int i = 0; i < 4; i++){
      modules[i].periodic();
    }
    
    if (DriverStation.isDisabled()) {
      // Stops driving when disabled
      for (var module : modules) {
        module.stop();
      }
    }

    swerveKinematics = new SwerveDriveKinematics(getModuleTranslations());
    maxAngularSpeed = DriveConstants.MAX_LINEAR_SPEED
    / Arrays.stream(getModuleTranslations())
      .map(translation -> translation.getNorm())
      .max(Double::compare)
      .get();

    
  }
  
  public void runVelocity(ChassisSpeeds speeds) { //set velocity
    DriveConstants.IS_CHARACTERIZING = false; //not really used 
    setpoint = speeds;
  }

  public void setRaw(double x, double y, double rot) { //
    runVelocity(
      ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rot, gyro.getYaw()))
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
