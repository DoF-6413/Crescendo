// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.drive;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DriveConstants;

/** Runs an Individual Real Module with all Motors as Neos */
public class ModuleIOSparkMax implements ModuleIO {
  private final CANSparkMax driveSparkMax;
  private final CANSparkMax turnSparkMax;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turnRelativeEncoder;
  private final CANcoder turnAbsoluteEncoder; 

  private final double turnAfterEncoderReduction = 150.0 / 7.0; //TODO: fact check
  private final boolean isTurnMotorInverted = true;
  private final int nancy;

  private final Pigeon2 gyro;

  public ModuleIOSparkMax(int index) {
    System.out.println("[Init] Creating ModuleIOSparkMax");
    this.nancy = index;

    switch(index) {
      case 0:
        driveSparkMax = new CANSparkMax(DriveMotor.frontLeft.CAN_ID, null); //update constants
    }

    gyro = new Pigeon2(0); //TODO: Update CANIDs
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePositionRad = Units.rotationsToRadians(driveEncoder.getPosition() / DriveConstants.GEAR_RATIO_L3);
  }
}
