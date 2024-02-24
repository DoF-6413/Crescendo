package frc.robot.Subsystems.utbintake;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

/** UTB Intake motor controller */
public class UTBIntakeIOSparkMax implements UTBIntakeIO {
  private final CANSparkMax utbIntakeMotor;
  private final RelativeEncoder utbIntakeEncoder;

  /** Creates the Motor and Encoder for the Under the Bumper (UTB) Intake */
  public UTBIntakeIOSparkMax() {
    System.out.println("[Init] Creating UTBIntakeIOSparkMax");

    /** Creates the Motor and Encoder for the UTB Intake */
    utbIntakeMotor = new CANSparkMax(UTBIntakeConstants.CAN_ID, MotorType.kBrushless);
    utbIntakeEncoder = utbIntakeMotor.getEncoder();

    /** defaults to brake mode on initialization */
    utbIntakeMotor.setIdleMode(IdleMode.kBrake);
  }

  /** Updates the printed values for the UTB Intake */
  public void updateInputs(UTBIntakeIOInputs inputs) {
    // Converts rotaions to Radians and then divides it by the gear ratio
    inputs.utbIntakeRPM = utbIntakeEncoder.getVelocity() / UTBIntakeConstants.GEAR_RATIO;
    inputs.utbIntakeAppliedVolts =
        utbIntakeMotor.getAppliedOutput() * utbIntakeMotor.getBusVoltage();
    inputs.utbIntakeCurrentAmps = new double[] {utbIntakeMotor.getOutputCurrent()};
    inputs.utbIntakeTempCelsius = new double[] {utbIntakeMotor.getMotorTemperature()};
  }

  @Override
  public void setUTBIntakeVoltage(double voltage) {
    utbIntakeMotor.setVoltage(voltage);
  }

  @Override
  public void setUTBIntakePercentSpeed(double percent) {
    utbIntakeMotor.set(percent);
  }

  @Override
  public void setUTBIntakeBrakeMode(boolean isEnabled) {
    if (isEnabled) {
      utbIntakeMotor.setIdleMode(IdleMode.kBrake);
    } else {
      utbIntakeMotor.setIdleMode(IdleMode.kCoast);
    }
  }

  @Override
  public void enableUTB(boolean auxYIsPressed) {
    if (auxYIsPressed == true) {
      setUTBIntakePercentSpeed(100);
    } else {
      setUTBIntakePercentSpeed(100);
    }
  }
}
