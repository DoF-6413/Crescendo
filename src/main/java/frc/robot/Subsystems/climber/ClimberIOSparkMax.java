package frc.robot.Subsystems.climber;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.ClimberConstants;

/** Climber Motor Control */
public class ClimberIOSparkMax implements ClimberIO {

  private final CANSparkMax leftClimberMotor;
  private final RelativeEncoder leftClimberEncoder;

  private final CANSparkMax rightClimberMotor;
  private final RelativeEncoder rightClimberEncoder;

  /** Creates the Motor and Encoder for the Climber */
  public ClimberIOSparkMax() {
    System.out.println("[Init] Creating ClimberIOSparkMax");

    leftClimberMotor = new CANSparkMax(ClimberConstants.LEFT_CLIMBER_CANID, MotorType.kBrushless);
    leftClimberEncoder = leftClimberMotor.getEncoder();

    rightClimberMotor = new CANSparkMax(ClimberConstants.RIGHT_CLIMBER_CANID, MotorType.kBrushless);
    rightClimberEncoder = rightClimberMotor.getEncoder();

    leftClimberMotor.setInverted(ClimberConstants.LEFT_CLIMBER_INVERTED);
    rightClimberMotor.setInverted(ClimberConstants.RIGHT_CLIMBER_INVERTED);
  }

  /** Updates the printed values for the Climber */
  public void updateInputs(ClimberIOInputs inputs) {
    // Updates the Left Climber Motor inputs
    // Converts rotations to radians, divides by gear ratio then multiplies by drum radius to get
    // position in meters
    inputs.leftClimberPositionMeters =
        Units.rotationsToRadians(
                leftClimberEncoder.getPosition() / ClimberConstants.CLIMBER_GEAR_RATIO)
            * ClimberConstants.CLIMBER_DRUM_RADIUS_M;

    // Converts RPM to rad/s, divides by gear ratio then multiplies by drum radius to get velocity
    // in m/s
    inputs.leftClimberVelocityMetersPerSecond =
        Units.rotationsPerMinuteToRadiansPerSecond(
                leftClimberEncoder.getVelocity() / ClimberConstants.CLIMBER_GEAR_RATIO)
            * ClimberConstants.CLIMBER_DRUM_RADIUS_M;

    inputs.leftClimberAppliedVolts =
        leftClimberMotor.getAppliedOutput()
            * leftClimberMotor.getBusVoltage(); // Applied Voltage of the Left Climber Motor
    inputs.leftClimberCurrentAmps =
        new double[] {leftClimberMotor.getOutputCurrent()}; // Amps used by the Left Climber Motor
    inputs.leftClimberTempCelcius =
        new double[] {
          leftClimberMotor.getMotorTemperature()
        }; // Tempature (Celcius) of the Left Climber Motor

    // Updates the Right Climber Motor inputs
    inputs.rightClimberPositionMeters =
        Units.rotationsToRadians(rightClimberEncoder.getPosition())
            / ClimberConstants
                .CLIMBER_GEAR_RATIO; // Converts rotaions to Radians and then divides it by gear
    // ratio
    inputs.rightClimberVelocityMetersPerSecond =
        Units.rotationsPerMinuteToRadiansPerSecond(
                rightClimberEncoder.getVelocity() / ClimberConstants.CLIMBER_GEAR_RATIO)
            * ClimberConstants.CLIMBER_DRUM_RADIUS_M;
    ; // Converts RPM to Radians per Second and then divides it by
    // the gear ratio
    inputs.rightClimberAppliedVolts =
        rightClimberMotor.getBusVoltage(); // Applied Voltage of the Right Climber Motor
    inputs.rightClimberCurrentAmps =
        new double[] {rightClimberMotor.getOutputCurrent()}; // Amps used by the Right
    // Climber Motor
    inputs.rightClimberTempCelcius =
        new double[] {
          rightClimberMotor.getMotorTemperature()
        }; // Tempature (Celcius) of the Right Climber Motor
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
}
