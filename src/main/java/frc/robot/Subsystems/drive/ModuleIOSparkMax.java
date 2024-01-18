// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.drive;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.core.CoreCANcoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSensor;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.RobotStateConstants;
import frc.robot.Constants.DriveConstants.*;

/** Runs an Individual Real Module with all Motors as Neos */
public class ModuleIOSparkMax implements ModuleIO {
  private final CANSparkMax driveSparkMax;
  private final CANSparkMax turnSparkMax;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turnRelativeEncoder;
  private final AbsoluteEncoder turnAbsoluteEncoder;

  private final double turnAfterEncoderReduction = 150.0 / 7.0; // TODO: fact check
  private final boolean isTurnMotorInverted = true;
  private final double absoluteEncoderOffset;
  private final int nancy; //<333

  public ModuleIOSparkMax(int index) {
    this.nancy = index;
    System.out.println("[Init] Creating ModuleIOSparkMax" + nancy);

    //sets drive & turn spark maxes, turn encoder, and absolute encoder offset
    switch (index) {
      case 0:
        driveSparkMax = new CANSparkMax(DRIVE_MOTOR.FRONT_LEFT.CAN_ID, MotorType.kBrushless);
        turnSparkMax = new CANSparkMax(TURN_MOTOR.FRONT_LEFT.CAN_ID, MotorType.kBrushless);
        // turnAbsoluteEncoder = new CANcoder(ABSOLUTE_ENCODER.FRONT_LEFT.ENCODER_ID); //
        absoluteEncoderOffset = ABSOLUTE_ENCODER_OFFSET_RAD.FRONT_LEFT.offset;
        break;
      case 1:
        driveSparkMax = new CANSparkMax(DRIVE_MOTOR.FRONT_RIGHT.CAN_ID, MotorType.kBrushless);
        turnSparkMax = new CANSparkMax(TURN_MOTOR.FRONT_RIGHT.CAN_ID, MotorType.kBrushless);
        absoluteEncoderOffset = ABSOLUTE_ENCODER_OFFSET_RAD.FRONT_RIGHT.offset;
        break;
      case 2:
        driveSparkMax = new CANSparkMax(DRIVE_MOTOR.FRONT_LEFT.CAN_ID, MotorType.kBrushless);
        turnSparkMax = new CANSparkMax(TURN_MOTOR.FRONT_LEFT.CAN_ID, MotorType.kBrushless);
        absoluteEncoderOffset = ABSOLUTE_ENCODER_OFFSET_RAD.BACK_LEFT.offset;
        break;
      case 3:
        driveSparkMax = new CANSparkMax(DRIVE_MOTOR.FRONT_LEFT.CAN_ID, MotorType.kBrushless);
        turnSparkMax = new CANSparkMax(TURN_MOTOR.FRONT_LEFT.CAN_ID, MotorType.kBrushless);
        absoluteEncoderOffset = ABSOLUTE_ENCODER_OFFSET_RAD.BACK_RIGHT.offset;
        break;
      default:
        throw new RuntimeException("Invalid module index for ModuleIOSparkMax");
    }

    //ensure configs remain after power cycles
    driveSparkMax.burnFlash();
    turnSparkMax.burnFlash();

    driveSparkMax.setCANTimeout(RobotStateConstants.CAN_CONFIG_TIMEOUT_SEC);
    turnSparkMax.setCANTimeout(RobotStateConstants.CAN_CONFIG_TIMEOUT_SEC);

    driveEncoder = driveSparkMax.getEncoder();
    turnRelativeEncoder = turnSparkMax.getEncoder();

    turnAbsoluteEncoder = turnSparkMax.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

    /** For each drive motor, update values */
    for (int i = 0; i < DRIVE_MOTOR.values().length; i++) {
      driveSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 10);
      turnSparkMax.setInverted(isTurnMotorInverted);

      driveSparkMax.setSmartCurrentLimit(40);
      turnSparkMax.setSmartCurrentLimit(40);

      driveEncoder.setPosition(0.0); // resets position
      driveEncoder.setMeasurementPeriod(10); // sensor reads every 10ms
      driveEncoder.setAverageDepth(2); // Sets velocity calculation process's sampling depth

      // Same for but turn motors
      turnRelativeEncoder.setPosition(0.0);
      turnRelativeEncoder.setMeasurementPeriod(10);
      turnRelativeEncoder.setAverageDepth(2);
    }

    driveSparkMax.setCANTimeout(0);
    turnSparkMax.setCANTimeout(0);

    driveSparkMax.burnFlash();
    turnSparkMax.burnFlash();
  }

  @Override
  /** updates the inputs to be actual values
   * @param inputs from ModuleIOInputsAutoLogged
   */
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePositionRad = Units.rotationsToRadians(driveEncoder.getPosition()
      / DriveConstants.GEAR_RATIO_L3);

    inputs.driveVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(driveEncoder.getVelocity())
      / DriveConstants.DRIVE_AFTER_ENCODER_REDUCTION; //converts  

    inputs.driveAppliedVolts = driveSparkMax.getAppliedOutput() * driveSparkMax.getBusVoltage();

    inputs.driveCurrentAmps = new double[] {driveSparkMax.getOutputCurrent()};
    inputs.driveTempCelcius = new double[] {driveSparkMax.getMotorTemperature()};

    inputs.turnAbsolutePositionRad = MathUtil.angleModulus(
      new Rotation2d(
        Math.toRadians(turnAbsoluteEncoder.getPosition()) + absoluteEncoderOffset //getPosition returns rotations of motor not degrees :((( fix later
      ).getRadians());

      SmartDashboard.putNumber("absolute encoder" + nancy, Math.toRadians(turnAbsoluteEncoder.getPosition()));
      SmartDashboard.putNumber("absolute encoder" + nancy + "with offset", Math.toRadians(turnAbsoluteEncoder.getPosition()) + absoluteEncoderOffset);
    
    inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
    inputs.turnCurrentAmps = new double[] { turnSparkMax.getOutputCurrent() };
    inputs.turnTempCelcius = new double[] { turnSparkMax.getMotorTemperature() };  
  } 
}
