package frc.robot.Subsystems.feeder;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class FeederIOTalonFX implements FeederIO {
  private final TalonFX feederMotor;

  public FeederIOTalonFX() {
    System.out.println("[Init] Creating FeederIOTalonFX");

    // Feeder motor ID
    feederMotor = new TalonFX(FeederConstants.FEEDER_MOTOR_ID);

    // Sets the inverted status of the Feeder motor to false
    feederMotor.setInverted(FeederConstants.IS_INVERTED);

    // Sets the feeder motor to brake mode by default
    feederMotor.setNeutralMode(NeutralModeValue.Brake);

    // Configure current limiting to the Feeder motor
    final CurrentLimitsConfigs currentLimitsConfigs =
        new CurrentLimitsConfigs().withStatorCurrentLimit(FeederConstants.CUR_LIM_A);
    feederMotor.getConfigurator().apply(currentLimitsConfigs);
  }

  @Override
  /** Updates all the logged values for the Feeder motpor */
  public void updateInputs(FeederIOInputs inputs) {
    // Gets the velocity of the Feeder motor, in RPM, and divides it by the
    // gear ratio to obatin the RPM of the feeder itself
    inputs.feederRPM = feederMotor.getVelocity().getValueAsDouble() / FeederConstants.GEAR_RATIO;
    inputs.feederAppliedVolts = feederMotor.getMotorVoltage().getValueAsDouble();
    inputs.feederCurrentAmps = new double[] {feederMotor.getStatorCurrent().getValueAsDouble()};
    inputs.feederTempCelsius = new double[] {feederMotor.getDeviceTemp().getValueAsDouble()};
  }

  @Override
  public void setMotorVoltage(double volts) {
    feederMotor.setVoltage(volts);
  }

  @Override
  public void setPercentSpeed(double percent) {
    feederMotor.set(percent);
  }

  @Override
  public void setBrakeMode(boolean enable) {
    if (enable) {
      feederMotor.setNeutralMode(NeutralModeValue.Brake);
    } else {
      feederMotor.setNeutralMode(NeutralModeValue.Coast);
    }
  }
}
