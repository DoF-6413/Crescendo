// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.drive;

import java.util.Optional;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.ABSOLUTE_ENCODER;
import frc.robot.Constants.DriveConstants.DRIVE_MOTOR;
import frc.robot.Constants.DriveConstants.L2_ABSOLUTE_ENCODER_OFFSET_RAD;
import frc.robot.Constants.DriveConstants.TURN_MOTOR;
import frc.robot.Constants.RobotStateConstants;

/** Runs an Individual Real Module with the Turn Motors as a Neo and Drive Motor as a Krakens */
public class ModuleIOSparkMaxTalonFX implements ModuleIO {
  private final CANSparkMax driveSparkMax;
  private final TalonFX turnTalonFX;

  private final RelativeEncoder driveRelativeEncoder;
  private final CANcoder turnAbsoluteEncoder;

  private final boolean isTurnMotorInverted = true;
  private final double absoluteEncoderOffset;
  private final int swerveModuleNumber;

  public ModuleIOSparkMaxTalonFX(int index) {
    this.swerveModuleNumber = index;
    System.out.println("[Init] Creating ModuleIOSparkMaxTalonFX");

    // sets drive & turn spark maxes, turn encoder, and absolute encoder offset
    switch (index) {
      case 0:
        driveSparkMax = new CANSparkMax(DRIVE_MOTOR.FRONT_RIGHT.CAN_ID, MotorType.kBrushless);
        turnTalonFX = new TalonFX(TURN_MOTOR.FRONT_RIGHT.CAN_ID);
        turnAbsoluteEncoder = new CANcoder(ABSOLUTE_ENCODER.FRONT_RIGHT.ENCODER_ID);
        absoluteEncoderOffset = L2_ABSOLUTE_ENCODER_OFFSET_RAD.FRONT_RIGHT.OFFSET;
        break;
      case 1:
        driveSparkMax = new CANSparkMax(DRIVE_MOTOR.FRONT_LEFT.CAN_ID, MotorType.kBrushless);
        turnTalonFX = new TalonFX(TURN_MOTOR.FRONT_LEFT.CAN_ID);
        turnAbsoluteEncoder = new CANcoder(ABSOLUTE_ENCODER.FRONT_LEFT.ENCODER_ID);
        absoluteEncoderOffset = L2_ABSOLUTE_ENCODER_OFFSET_RAD.FRONT_LEFT.OFFSET;
        break;
      case 2:
        driveSparkMax = new CANSparkMax(DRIVE_MOTOR.BACK_LEFT.CAN_ID, MotorType.kBrushless);
        turnTalonFX = new TalonFX(TURN_MOTOR.BACK_LEFT.CAN_ID);
        turnAbsoluteEncoder = new CANcoder(ABSOLUTE_ENCODER.BACK_LEFT.ENCODER_ID);
        absoluteEncoderOffset = L2_ABSOLUTE_ENCODER_OFFSET_RAD.BACK_LEFT.OFFSET;
        break;
      case 3:
        driveSparkMax = new CANSparkMax(DRIVE_MOTOR.BACK_RIGHT.CAN_ID, MotorType.kBrushless);
        turnTalonFX = new TalonFX(TURN_MOTOR.BACK_RIGHT.CAN_ID);
        turnAbsoluteEncoder = new CANcoder(ABSOLUTE_ENCODER.BACK_RIGHT.ENCODER_ID);
        absoluteEncoderOffset = L2_ABSOLUTE_ENCODER_OFFSET_RAD.BACK_RIGHT.OFFSET;
        break;
      default:
        throw new RuntimeException("Invalid module index for ModuleIOSparkMax");
    }

    // set can timeouts from constants
    driveSparkMax.setCANTimeout(RobotStateConstants.CAN_CONFIG_TIMEOUT_SEC);
    turnTalonFX.setExpiration(RobotStateConstants.CAN_CONFIG_TIMEOUT_SEC);

    // set drive encoders to match the sparkmax motor controllers
    driveRelativeEncoder = driveSparkMax.getEncoder();

    /** For each drive motor, update values */
    for (int i = 0; i < DRIVE_MOTOR.values().length; i++) {
      driveSparkMax.setPeriodicFramePeriod(
          PeriodicFrame.kStatus2, DriveConstants.MEASUREMENT_PERIOD_MS);
      turnTalonFX.setInverted(isTurnMotorInverted);

      driveSparkMax.setSmartCurrentLimit(DriveConstants.SMART_CURRENT_LIMIT);
      CurrentLimitsConfigs currentLimitsConfig =
          new CurrentLimitsConfigs().withStatorCurrentLimit(DriveConstants.SMART_CURRENT_LIMIT);
      turnTalonFX.getConfigurator().apply(currentLimitsConfig);

      driveRelativeEncoder.setPosition(0.0); // resets position
      driveRelativeEncoder.setMeasurementPeriod(
          DriveConstants.MEASUREMENT_PERIOD_MS); // sensor reads every 10ms
      driveRelativeEncoder.setAverageDepth(
          2); // sets velocity calculation process's sampling depth (??)

      // Sets Turn Position to 0
      turnTalonFX.setPosition(0.0);
    }
    // ensure configs remain after power cycles
    driveSparkMax.burnFlash();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePositionRad =
        Units.rotationsToRadians(driveRelativeEncoder.getPosition() / DriveConstants.GEAR_RATIO_L3);

    inputs.driveVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(driveRelativeEncoder.getVelocity())
            / DriveConstants.getGearRatio(true);

    inputs.driveAppliedVolts = driveSparkMax.getAppliedOutput() * driveSparkMax.getBusVoltage();

    inputs.driveCurrentAmps = new double[] {driveSparkMax.getOutputCurrent()};
    inputs.driveTempCelcius = new double[] {driveSparkMax.getMotorTemperature()};

    inputs.turnAbsolutePositionRad =
        MathUtil.angleModulus(
            new Rotation2d(
                    Units.rotationsToRadians(
                            turnAbsoluteEncoder.getAbsolutePosition().getValueAsDouble())
                        + absoluteEncoderOffset // getPosition returns rotations of motor not
                    // degrees
                    )
                .getRadians());

    SmartDashboard.putNumber(
        "absolute encoder" + swerveModuleNumber,
        Units.rotationsToRadians(turnAbsoluteEncoder.getAbsolutePosition().getValueAsDouble()));
    SmartDashboard.putNumber(
        "absolute encoder" + swerveModuleNumber + " with offset",
        Units.rotationsToRadians(turnAbsoluteEncoder.getAbsolutePosition().getValueAsDouble())
            + absoluteEncoderOffset);

    inputs.turnAppliedVolts =
        turnTalonFX.getMotorVoltage().getValueAsDouble()
            * turnTalonFX.getSupplyVoltage().getValueAsDouble();
    inputs.turnCurrentAmps =
        new double[] {
          turnTalonFX.getStatorCurrent().getValueAsDouble(),
          turnTalonFX.getSupplyCurrent().getValueAsDouble()
        };
    inputs.turnTempCelcius = new double[] {turnTalonFX.getDeviceTemp().getValueAsDouble()};
  }

  // sets voltage output of drive sparkmaxes
  public void setDriveVoltage(double volts) {
    driveSparkMax.setVoltage(volts);
  }

  // sets voltage output of turn sparkmaxes
  public void setTurnVoltage(double volts) {
    turnTalonFX.setVoltage(volts);
  }

  // sets BrakeMode for drives
  public void setDriveBrakeMode(boolean enable) {
    driveSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  // set BrakeMode for turn
  public void setTurnBrakeMode(boolean enable) {
    turnTalonFX.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  // returns absolute position from turn absolute encoders
  public double getAbsolutePositionRadians() {
    return Units.degreesToRadians(turnAbsoluteEncoder.getAbsolutePosition().getValueAsDouble());
  }

    @Override
  public Optional<Boolean> isL3() {
    return Optional.of(true);
  }
}
