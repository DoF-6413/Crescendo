package frc.robot.Subsystems.climber;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;

public class ClimberIOSparkMax implements ClimberIO {

  private final CANSparkMax leftClimberMotor;
  private final RelativeEncoder leftClimberEncoder;

  private final CANSparkMax rightClimberMotor;
  private final RelativeEncoder rightClimberEncoder;

  /** Runs the real life Climbers with CANSpark Speed Controllers and NEO motor */
  public ClimberIOSparkMax() {
    System.out.println("[Init] Creating ClimberIOSparkMax");

    leftClimberMotor = new CANSparkMax(ClimberConstants.LEFT_CAN_ID, MotorType.kBrushless);
    leftClimberEncoder = leftClimberMotor.getEncoder();

    rightClimberMotor = new CANSparkMax(ClimberConstants.RIGHT_CAN_ID, MotorType.kBrushless);
    rightClimberEncoder = rightClimberMotor.getEncoder();

    leftClimberMotor.setInverted(ClimberConstants.LEFT_IS_INVERTED);
    rightClimberMotor.setInverted(ClimberConstants.RIGHT_IS_INVERTED);

    leftClimberMotor.setIdleMode(IdleMode.kBrake);
    rightClimberMotor.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    // Updates the Left Climber Motor inputs
    inputs.leftClimberAppliedVolts =
        leftClimberMotor.getAppliedOutput() * leftClimberMotor.getBusVoltage();
    // Converts rotations to radians, divides by gear ratio then multiplies by drum
    // radius to get
    // position in meters
    inputs.leftClimberPositionMeters =
        Units.rotationsToRadians(leftClimberEncoder.getPosition() / ClimberConstants.GEAR_RATIO)
            * ClimberConstants.DRUM_RADIUS_M;

    // Converts RPM to rad/s, divides by gear ratio then multiplies by drum radius
    // to get velocity
    // in m/s
    inputs.leftClimberVelocityMetersPerSecond =
        Units.rotationsPerMinuteToRadiansPerSecond(
                leftClimberEncoder.getVelocity() / ClimberConstants.GEAR_RATIO)
            * ClimberConstants.DRUM_RADIUS_M;

    inputs.leftClimberCurrentAmps = new double[] {leftClimberMotor.getOutputCurrent()};

    inputs.leftClimberTempCelsius = new double[] {leftClimberMotor.getMotorTemperature()};

    // Updates the Right Climber Motor inputs
    inputs.rightClimberAppliedVolts =
        rightClimberMotor.getAppliedOutput() * rightClimberMotor.getBusVoltage();
    // Converts rotaions to Radians and then divides it by gear ratio then
    // multiplies by drum radius position in meters
    inputs.rightClimberPositionMeters =
        Units.rotationsToRadians(rightClimberEncoder.getPosition() / ClimberConstants.GEAR_RATIO)
            * ClimberConstants.DRUM_RADIUS_M;

    // Converts RPM to rad/s, divides by gear ratio then multiplies by drum radius
    // to get velocity
    // in m/s
    inputs.rightClimberVelocityMetersPerSecond =
        Units.rotationsPerMinuteToRadiansPerSecond(
                rightClimberEncoder.getVelocity() / ClimberConstants.GEAR_RATIO)
            * ClimberConstants.DRUM_RADIUS_M;
    inputs.rightClimberCurrentAmps = new double[] {rightClimberMotor.getOutputCurrent()};
    inputs.rightClimberTempCelsius = new double[] {rightClimberMotor.getMotorTemperature()};
  }

  @Override
  public void setBothClimberVoltage(double volts) {
    leftClimberMotor.setVoltage(volts);
    rightClimberMotor.setVoltage(volts);
  }

  @Override
  public void setLeftClimberVoltage(double volts) {
    leftClimberMotor.setVoltage(volts);
  }

  @Override
  public void setRightClimberVoltage(double volts) {
    rightClimberMotor.setVoltage(volts);
  }

  @Override
  public void setBothClimberPercentSpeed(double percent) {
    leftClimberMotor.set(percent);
    rightClimberMotor.set(percent);
  }

  @Override
  public void setLeftClimberPercentSpeed(double percent) {
    leftClimberMotor.set(percent);
  }

  @Override
  public void setRightClimberPercentSpeed(double percent) {
    rightClimberMotor.set(percent);
  }

  @Override
  public void setBrakeMode(boolean enable) {
    if (enable) {
      leftClimberMotor.setIdleMode(IdleMode.kBrake);
      rightClimberMotor.setIdleMode(IdleMode.kBrake);
    } else {
      leftClimberMotor.setIdleMode(IdleMode.kCoast);
      rightClimberMotor.setIdleMode(IdleMode.kCoast);
    }
  }
}
