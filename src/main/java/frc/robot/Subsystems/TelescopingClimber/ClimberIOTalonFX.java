package frc.robot.Subsystems.TelescopingClimber;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ClimberIOTalonFX implements ClimberIO{
    
  private final TalonFX climberMotor;

  public ClimberIOTalonFX() {
    System.out.println("[Init] Creating ClimberIOTalonFX");

    climberMotor = new TalonFX(ClimberConstants.MOTOR_ID);

    climberMotor.setInverted(ClimberConstants.IS_INVERTED);

    climberMotor.setNeutralMode(NeutralModeValue.Brake);

    final CurrentLimitsConfigs currentLimitsConfigs = 
        new CurrentLimitsConfigs().withStatorCurrentLimit(ClimberConstants.CUR_LIM_A);
    currentLimitsConfigs.withSupplyCurrentLimit(ClimberConstants.CUR_LIM_A);
    climberMotor.getConfigurator().apply(currentLimitsConfigs);
    currentLimitsConfigs.withStatorCurrentLimitEnable(true);
    currentLimitsConfigs.withSupplyCurrentLimitEnable(true);
  }

  @Override
  /** All the inputs for the Climber motor */
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.climberPositionMeters = (climberMotor.getPosition().getValueAsDouble() / ClimberConstants.GEAR_RATIO) * (2 * Math.PI) * ClimberConstants.SHAFT_RADIUS_M;
    inputs.climberVelocityMetersPerSecond = (climberMotor.getVelocity().getValueAsDouble() / ClimberConstants.GEAR_RATIO) * (2 * Math.PI) * ClimberConstants.SHAFT_RADIUS_M;
    inputs.climberAppliedVolts = climberMotor.getMotorVoltage().getValueAsDouble();
    inputs.climberCurrentAmps = new double[] {climberMotor.getStatorCurrent().getValueAsDouble()};
    inputs.climberTempCelsius = new double[] {climberMotor.getDeviceTemp().getValueAsDouble()};
  }

  @Override
  public void setClimberBrakeMode(boolean enable) {
    if (enable) {
    climberMotor.setNeutralMode(NeutralModeValue.Brake);
  } else {
    climberMotor.setNeutralMode(NeutralModeValue.Coast);
  }
}

  @Override
  public void setClimberVoltage(double volts) {
    climberMotor.setVoltage(volts);
  }

  @Override
  public void setClimberPercentSpeed(double percent) {
    climberMotor.set(percent);
  }
}
