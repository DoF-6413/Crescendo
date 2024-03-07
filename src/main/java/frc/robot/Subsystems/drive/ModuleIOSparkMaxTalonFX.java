// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.drive;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.RobotStateConstants;
import java.util.Optional;

/** Runs an Individual Real Module with the Turn Motors as a Neo and Drive Motor as a Krakens */
public class ModuleIOSparkMaxTalonFX implements ModuleIO {
  private final TalonFX driveTalonFX;
  private final CANSparkMax turnSparkMax;

  private final RelativeEncoder turnRelativeEncoder;
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
        driveTalonFX = new TalonFX(DriveConstants.DRIVE_MOTOR.FRONT_RIGHT.CAN_ID);
        turnSparkMax =
            new CANSparkMax(DriveConstants.TURN_MOTOR.FRONT_RIGHT.CAN_ID, MotorType.kBrushless);
        turnAbsoluteEncoder = new CANcoder(DriveConstants.ABSOLUTE_ENCODER.FRONT_RIGHT.ENCODER_ID);
        absoluteEncoderOffset = DriveConstants.L3_ABSOLUTE_ENCODER_OFFSET_RAD.FRONT_RIGHT.OFFSET;
        break;
      case 1:
        driveTalonFX = new TalonFX(DriveConstants.DRIVE_MOTOR.FRONT_LEFT.CAN_ID);
        turnSparkMax =
            new CANSparkMax(DriveConstants.TURN_MOTOR.FRONT_LEFT.CAN_ID, MotorType.kBrushless);
        turnAbsoluteEncoder = new CANcoder(DriveConstants.ABSOLUTE_ENCODER.FRONT_LEFT.ENCODER_ID);
        absoluteEncoderOffset = DriveConstants.L3_ABSOLUTE_ENCODER_OFFSET_RAD.FRONT_LEFT.OFFSET;
        break;
      case 2:
        driveTalonFX = new TalonFX(DriveConstants.DRIVE_MOTOR.BACK_LEFT.CAN_ID);
        turnSparkMax =
            new CANSparkMax(DriveConstants.TURN_MOTOR.BACK_LEFT.CAN_ID, MotorType.kBrushless);
        turnAbsoluteEncoder = new CANcoder(DriveConstants.ABSOLUTE_ENCODER.BACK_LEFT.ENCODER_ID);
        absoluteEncoderOffset = DriveConstants.L3_ABSOLUTE_ENCODER_OFFSET_RAD.BACK_LEFT.OFFSET;
        break;
      case 3:
        driveTalonFX = new TalonFX(DriveConstants.DRIVE_MOTOR.BACK_RIGHT.CAN_ID);
        turnSparkMax =
            new CANSparkMax(DriveConstants.TURN_MOTOR.BACK_RIGHT.CAN_ID, MotorType.kBrushless);
        turnAbsoluteEncoder = new CANcoder(DriveConstants.ABSOLUTE_ENCODER.BACK_RIGHT.ENCODER_ID);
        absoluteEncoderOffset = DriveConstants.L3_ABSOLUTE_ENCODER_OFFSET_RAD.BACK_RIGHT.OFFSET;
        break;
      default:
        throw new RuntimeException("Invalid module index for ModuleIOSparkMax");
    }

    // set can timeouts from constants
    driveTalonFX.setExpiration(RobotStateConstants.CAN_CONFIG_TIMEOUT_SEC);
    turnSparkMax.setCANTimeout(RobotStateConstants.CAN_CONFIG_TIMEOUT_SEC);

    // set drive encoders to match the sparkmax motor controllers
    turnRelativeEncoder = turnSparkMax.getEncoder();

    /** For each drive motor, update values */
    for (int i = 0; i < DriveConstants.DRIVE_MOTOR.values().length; i++) {
      // todo: drive?
      // turnSparkMax.setPeriodicFramePeriod(
      //     PeriodicFrame.kStatus2, DriveConstants.MEASUREMENT_PERIOD_MS);
      turnSparkMax.setInverted(isTurnMotorInverted);
      driveTalonFX.setInverted(false); // TODO: Make constant

      CurrentLimitsConfigs currentLimitsConfig =
          new CurrentLimitsConfigs().withSupplyCurrentLimit(DriveConstants.CUR_LIM_A);
      currentLimitsConfig.withSupplyCurrentLimitEnable(true);
      currentLimitsConfig.withStatorCurrentLimit(DriveConstants.CUR_LIM_A);
      currentLimitsConfig.withStatorCurrentLimitEnable(true);
      SmartDashboard.putBoolean("StatorEnabled", currentLimitsConfig.StatorCurrentLimitEnable);
      SmartDashboard.putBoolean("SupplyEnabled", currentLimitsConfig.SupplyCurrentLimitEnable);
      SmartDashboard.putNumber("StatorCurr", currentLimitsConfig.StatorCurrentLimit);
      SmartDashboard.putNumber("SupplyCurr", currentLimitsConfig.SupplyCurrentLimit);
      driveTalonFX.getConfigurator().apply(currentLimitsConfig);
      // turnSparkMax.setSmartCurrentLimit(DriveConstants.CUR_LIM_A);
      turnSparkMax.setSmartCurrentLimit(DriveConstants.CUR_LIM_A, DriveConstants.CUR_LIM_A);

      driveTalonFX.setPosition(0.0); // resets position
      // driveTalonFX.setMeasurementPeriod(
      //     DriveConstants.MEASUREMENT_PERIOD_MS); // sensor reads every 10ms
      // driveTalonFX.setAverageDepth(
      //     2); // sets velocity calculation process's sampling depth (??)

      // Sets Turn Position to 0
      turnRelativeEncoder.setPosition(0.0);
    }
    // ensure configs remain after power cycles
    turnSparkMax.burnFlash();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePositionRad =
        Units.rotationsToRadians(
            driveTalonFX.getPosition().getValueAsDouble() / DriveConstants.GEAR_RATIO_L3);
    inputs.driveVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(driveTalonFX.getVelocity().getValueAsDouble() * 60)
            / DriveConstants.getGearRatio(true);

    // unit conversions, Kraken getVelocity returns rotations per sec, multiply by 60 to get RPM
    inputs.driveVelocityRadPerSecAbs =
        Math.abs(
            Units.rotationsPerMinuteToRadiansPerSecond(
                    driveTalonFX.getVelocity().getValueAsDouble() * 60)
                / DriveConstants.getGearRatio(true));

    inputs.driveAppliedVolts =
        driveTalonFX.getMotorVoltage().getValueAsDouble()
            * driveTalonFX.getSupplyVoltage().getValueAsDouble();

    inputs.driveCurrentAmps = new double[] {driveTalonFX.getStatorCurrent().getValueAsDouble()};
    inputs.driveTempCelsius = new double[] {driveTalonFX.getDeviceTemp().getValueAsDouble()};

    inputs.turnAbsolutePositionRad =
        MathUtil.angleModulus(
            new Rotation2d(
                    Units.rotationsToRadians(
                            turnAbsoluteEncoder.getAbsolutePosition().getValueAsDouble())
                        + absoluteEncoderOffset // getPosition returns rotations of motor not
                    // degrees
                    )
                .getRadians());

    inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
    inputs.turnCurrentAmps = new double[] {turnSparkMax.getOutputCurrent()};
    inputs.turnTempCelsius = new double[] {turnSparkMax.getMotorTemperature()};
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
    turnSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public Optional<Boolean> isL3() {
    return Optional.of(true);
  }
}
