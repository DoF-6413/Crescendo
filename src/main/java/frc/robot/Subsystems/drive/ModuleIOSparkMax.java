// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.drive;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.RobotStateConstants;
import java.util.Optional;

/** Runs an Individual Real Module with all Motors as Neos */
public class ModuleIOSparkMax implements ModuleIO {
  private final CANSparkMax driveSparkMax;
  private final CANSparkMax turnSparkMax;

  private final RelativeEncoder driveRelativeEncoder;
  private final RelativeEncoder turnRelativeEncoder;
  private final CANcoder turnAbsoluteEncoder;

  private final boolean isTurnMotorInverted = true;
  private final double absoluteEncoderOffset;
  private final int swerveModuleNumber;

  public ModuleIOSparkMax(int index) {
    this.swerveModuleNumber = index;
    System.out.println("[Init] Creating ModuleIOSparkMax" + swerveModuleNumber);

    // sets drive & turn spark maxes, turn encoder, and absolute encoder offset
    switch (index) {
      case 0:
        driveSparkMax =
            new CANSparkMax(DriveConstants.DRIVE_MOTOR.FRONT_RIGHT.CAN_ID, MotorType.kBrushless);
        turnSparkMax =
            new CANSparkMax(DriveConstants.TURN_MOTOR.FRONT_RIGHT.CAN_ID, MotorType.kBrushless);
        turnAbsoluteEncoder = new CANcoder(DriveConstants.ABSOLUTE_ENCODER.FRONT_RIGHT.ENCODER_ID);
        absoluteEncoderOffset = DriveConstants.L2_ABSOLUTE_ENCODER_OFFSET_RAD.FRONT_RIGHT.OFFSET;
        break;
      case 1:
        driveSparkMax =
            new CANSparkMax(DriveConstants.DRIVE_MOTOR.FRONT_LEFT.CAN_ID, MotorType.kBrushless);
        turnSparkMax =
            new CANSparkMax(DriveConstants.TURN_MOTOR.FRONT_LEFT.CAN_ID, MotorType.kBrushless);
        turnAbsoluteEncoder = new CANcoder(DriveConstants.ABSOLUTE_ENCODER.FRONT_LEFT.ENCODER_ID);
        absoluteEncoderOffset = DriveConstants.L2_ABSOLUTE_ENCODER_OFFSET_RAD.FRONT_LEFT.OFFSET;
        break;
      case 2:
        driveSparkMax =
            new CANSparkMax(DriveConstants.DRIVE_MOTOR.BACK_LEFT.CAN_ID, MotorType.kBrushless);
        turnSparkMax =
            new CANSparkMax(DriveConstants.TURN_MOTOR.BACK_LEFT.CAN_ID, MotorType.kBrushless);
        turnAbsoluteEncoder = new CANcoder(DriveConstants.ABSOLUTE_ENCODER.BACK_LEFT.ENCODER_ID);
        absoluteEncoderOffset = DriveConstants.L2_ABSOLUTE_ENCODER_OFFSET_RAD.BACK_LEFT.OFFSET;
        break;
      case 3:
        driveSparkMax =
            new CANSparkMax(DriveConstants.DRIVE_MOTOR.BACK_RIGHT.CAN_ID, MotorType.kBrushless);
        turnSparkMax =
            new CANSparkMax(DriveConstants.TURN_MOTOR.BACK_RIGHT.CAN_ID, MotorType.kBrushless);
        turnAbsoluteEncoder = new CANcoder(DriveConstants.ABSOLUTE_ENCODER.BACK_RIGHT.ENCODER_ID);
        absoluteEncoderOffset = DriveConstants.L2_ABSOLUTE_ENCODER_OFFSET_RAD.BACK_RIGHT.OFFSET;
        break;
      default:
        throw new RuntimeException("Invalid module index for ModuleIOSparkMax");
    }

    // set can timeouts from constants
    driveSparkMax.setCANTimeout(RobotStateConstants.CAN_CONFIG_TIMEOUT_SEC);
    turnSparkMax.setCANTimeout(RobotStateConstants.CAN_CONFIG_TIMEOUT_SEC);

    // set drive encoders to match the sparkmax motor controllers
    driveRelativeEncoder = driveSparkMax.getEncoder();
    turnRelativeEncoder = turnSparkMax.getEncoder();

    /** For each drive motor, update values */
    for (int i = 0; i < DriveConstants.DRIVE_MOTOR.values().length; i++) {
      driveSparkMax.setPeriodicFramePeriod(
          PeriodicFrame.kStatus2, DriveConstants.MEASUREMENT_PERIOD_MS);
      turnSparkMax.setInverted(isTurnMotorInverted);

      driveSparkMax.setSmartCurrentLimit(DriveConstants.CUR_LIM_A);
      turnSparkMax.setSmartCurrentLimit(DriveConstants.CUR_LIM_A);

      driveRelativeEncoder.setPosition(0.0); // resets position
      driveRelativeEncoder.setMeasurementPeriod(
          DriveConstants.MEASUREMENT_PERIOD_MS); // sensor reads every 10ms
      driveRelativeEncoder.setAverageDepth(
          2); // sets velocity calculation process's sampling depth (??)

      // Same for but turn motors
      turnRelativeEncoder.setPosition(0.0);
      turnRelativeEncoder.setMeasurementPeriod(DriveConstants.MEASUREMENT_PERIOD_MS);
      turnRelativeEncoder.setAverageDepth(2);
    }

    // ensure configs remain after power cycles
    driveSparkMax.burnFlash();
    turnSparkMax.burnFlash();
  }

  @Override
  /**
   * updates the inputs to be actual values
   *
   * @param inputs from ModuleIOInputsAutoLogged
   */
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePositionRad =
        Units.rotationsToRadians(driveRelativeEncoder.getPosition() / DriveConstants.GEAR_RATIO_L2);

    inputs.driveVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(driveRelativeEncoder.getVelocity())
            / DriveConstants.getGearRatio(false);

    inputs.driveAppliedVolts = driveSparkMax.getAppliedOutput() * driveSparkMax.getBusVoltage();

    inputs.driveCurrentAmps = new double[] {driveSparkMax.getOutputCurrent()};
    inputs.driveTempCelsius = new double[] {driveSparkMax.getMotorTemperature()};

    inputs.turnAbsolutePositionRad =
        MathUtil.angleModulus(
            new Rotation2d(
                    Units.rotationsToRadians(
                            turnAbsoluteEncoder.getAbsolutePosition().getValueAsDouble())
                        + absoluteEncoderOffset // get Position returns rotations of motor not
                    // degrees
                    )
                .getRadians());

    inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
    inputs.turnCurrentAmps = new double[] {turnSparkMax.getOutputCurrent()};
    inputs.turnTempCelsius = new double[] {turnSparkMax.getMotorTemperature()};
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveSparkMax.setVoltage(volts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnSparkMax.setVoltage(volts);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    driveSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    turnSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public Optional<Boolean> isL3() {
    return Optional.of(false);
  }
}
