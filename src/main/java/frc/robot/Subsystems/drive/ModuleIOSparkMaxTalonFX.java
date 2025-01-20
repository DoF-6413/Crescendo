// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.drive;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.RobotStateConstants;

/** Runs an Individual Real Module with the Turn Motors as a Neo and Drive Motor as a Krakens */
public class ModuleIOSparkMaxTalonFX implements ModuleIO {
  private final TalonFX driveTalonFX;
  private final SparkMax turnSparkMax;
  private final SparkMaxConfig turnSparkMaxConfig;
  private final TalonFXConfiguration driveTalonFXConfig;
  
  private final RelativeEncoder turnRelativeEncoder;
  private final CANcoder turnAbsoluteEncoder;
  private final double absoluteEncoderOffsetRad;

  private final VelocityVoltage velocityVoltageRequest;
  private final SparkClosedLoopController turnController;

  public ModuleIOSparkMaxTalonFX(int index) {
    System.out.println("[Init] Creating ModuleIOSparkMaxTalonFX");

    // sets drive & turn SPARK MAXes, turn encoder, and absolute encoder offset
    switch (index) {
      case 0:
        driveTalonFX = new TalonFX(DriveConstants.DRIVE_MOTOR.FRONT_RIGHT.CAN_ID);
        turnSparkMax =
            new SparkMax(DriveConstants.TURN_MOTOR.FRONT_RIGHT.CAN_ID, MotorType.kBrushless);
        turnAbsoluteEncoder = new CANcoder(DriveConstants.ABSOLUTE_ENCODER.FRONT_RIGHT.ENCODER_ID);
        absoluteEncoderOffsetRad = DriveConstants.L3_ABSOLUTE_ENCODER_OFFSET_RAD.FRONT_RIGHT.OFFSET;
        break;
      case 1:
        driveTalonFX = new TalonFX(DriveConstants.DRIVE_MOTOR.FRONT_LEFT.CAN_ID);
        turnSparkMax =
            new SparkMax(DriveConstants.TURN_MOTOR.FRONT_LEFT.CAN_ID, MotorType.kBrushless);
        turnAbsoluteEncoder = new CANcoder(DriveConstants.ABSOLUTE_ENCODER.FRONT_LEFT.ENCODER_ID);
        absoluteEncoderOffsetRad = DriveConstants.L3_ABSOLUTE_ENCODER_OFFSET_RAD.FRONT_LEFT.OFFSET;
        break;
      case 2:
        driveTalonFX = new TalonFX(DriveConstants.DRIVE_MOTOR.BACK_LEFT.CAN_ID);
        turnSparkMax =
            new SparkMax(DriveConstants.TURN_MOTOR.BACK_LEFT.CAN_ID, MotorType.kBrushless);
        turnAbsoluteEncoder = new CANcoder(DriveConstants.ABSOLUTE_ENCODER.BACK_LEFT.ENCODER_ID);
        absoluteEncoderOffsetRad = DriveConstants.L3_ABSOLUTE_ENCODER_OFFSET_RAD.BACK_LEFT.OFFSET;
        break;
      case 3:
        driveTalonFX = new TalonFX(DriveConstants.DRIVE_MOTOR.BACK_RIGHT.CAN_ID);
        turnSparkMax =
            new SparkMax(DriveConstants.TURN_MOTOR.BACK_RIGHT.CAN_ID, MotorType.kBrushless);
        turnAbsoluteEncoder = new CANcoder(DriveConstants.ABSOLUTE_ENCODER.BACK_RIGHT.ENCODER_ID);
        absoluteEncoderOffsetRad = DriveConstants.L3_ABSOLUTE_ENCODER_OFFSET_RAD.BACK_RIGHT.OFFSET;
        break;
      default:
        throw new RuntimeException("Invalid module index for ModuleIOSparkMax");
    }
    // Initialize various Motor objects
    turnSparkMaxConfig = new SparkMaxConfig();
    driveTalonFXConfig = new TalonFXConfiguration();
    velocityVoltageRequest = new VelocityVoltage(0);
    turnController = turnSparkMax.getClosedLoopController();

    // set can timeouts from constants
    driveTalonFX.setExpiration(RobotStateConstants.CAN_CONFIG_TIMEOUT_SEC);
    turnSparkMax.setCANTimeout(RobotStateConstants.CAN_CONFIG_TIMEOUT_SEC);

    // set drive encoders to match the SPARK MAX motor controllers
    turnRelativeEncoder = turnSparkMax.getEncoder();

    /** For each drive motor, update values */
    driveTalonFXConfig.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
    driveTalonFXConfig.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
    turnSparkMaxConfig.inverted(DriveConstants.INVERT_TURN_SPARK_MAX);
    turnSparkMaxConfig.idleMode(IdleMode.kCoast);

    // Configure Current Limits
    driveTalonFXConfig.CurrentLimits.withStatorCurrentLimit(DriveConstants.CUR_LIM_A);
    driveTalonFXConfig.CurrentLimits.withSupplyCurrentLimitEnable(DriveConstants.ENABLE_CUR_LIM);
    driveTalonFXConfig.CurrentLimits.withStatorCurrentLimit(DriveConstants.CUR_LIM_A);
    driveTalonFXConfig.CurrentLimits.withStatorCurrentLimitEnable(DriveConstants.ENABLE_CUR_LIM);
    turnSparkMaxConfig.smartCurrentLimit(DriveConstants.CUR_LIM_A, DriveConstants.CUR_LIM_A);

    driveTalonFX.setPosition(0.0); // resets position
    driveTalonFXConfig.MotorOutput.withControlTimesyncFreqHz(DriveConstants.MEASUREMENT_PERIOD_HZ);

    // Sets Turn Position to 0
    turnRelativeEncoder.setPosition(0.0);

    // Set PID Configuration
    driveTalonFXConfig.Slot0.withKP(DriveConstants.DRIVE_KP_KRAKEN);
    driveTalonFXConfig.Slot0.withKI(DriveConstants.DRIVE_KI_KRAKEN);
    driveTalonFXConfig.Slot0.withKD(DriveConstants.DRIVE_KD_KRAKEN);
    driveTalonFXConfig.Slot0.withKS(DriveConstants.DRIVE_KS_KRAKEN);
    driveTalonFXConfig.Slot0.withKV(DriveConstants.DRIVE_KV_KRAKEN);
  
    // Apply configuration
    driveTalonFX.getConfigurator().apply(driveTalonFXConfig);
    turnSparkMax.configure(turnSparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePositionRad =
        Units.rotationsToRadians(
            driveTalonFX.getPosition().getValueAsDouble() / DriveConstants.GEAR_RATIO_L3);
    inputs.driveVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(
                driveTalonFX.getVelocity().getValueAsDouble() * 60)
            / DriveConstants.GEAR_RATIO_L3;
    inputs.driveVelocityRadPerSecAbs =
        Math.abs(
            Units.rotationsPerMinuteToRadiansPerSecond(
                    driveTalonFX.getVelocity().getValueAsDouble() * 60)
                / DriveConstants.GEAR_RATIO_L3);

    inputs.driveAppliedVolts = driveTalonFX.getMotorVoltage().getValueAsDouble();

    inputs.driveCurrentAmps = driveTalonFX.getStatorCurrent().getValueAsDouble();
    inputs.driveTempCelsius = driveTalonFX.getDeviceTemp().getValueAsDouble();

    // getPosition returns rotations of motor, not the turn angle
    inputs.turnAbsolutePositionRad =
        MathUtil.angleModulus(
            new Rotation2d(
                    Units.rotationsToRadians(
                            turnAbsoluteEncoder.getAbsolutePosition().getValueAsDouble())
                        + absoluteEncoderOffsetRad)
                .getRadians());

    inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
    inputs.turnCurrentAmps = turnSparkMax.getOutputCurrent();
    inputs.turnTempCelsius = turnSparkMax.getMotorTemperature();
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveTalonFX.setVoltage(volts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnSparkMax.setVoltage(volts);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    driveTalonFX.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    turnSparkMaxConfig.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    turnSparkMax.configure(turnSparkMaxConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void setDriveVelocity(double velocity) {
    driveTalonFX.setControl(velocityVoltageRequest.withVelocity(velocity));
  }
}
