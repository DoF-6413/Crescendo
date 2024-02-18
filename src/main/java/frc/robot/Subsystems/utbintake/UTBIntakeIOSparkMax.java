package frc.robot.Subsystems.utbintake;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

/** utbIntake motor controller */
public class UTBIntakeIOSparkMax implements UTBIntakeIO {
  private final CANSparkMax utbIntakeMotor;
  private final RelativeEncoder utbIntakeEncoder;

  /** Creates the Motor and Encoder for the utb Intake */
  public UTBIntakeIOSparkMax() {
    System.out.println("[Init] Creating UTBIntakeIO");
    utbIntakeMotor = new CANSparkMax(UTBIntakeConstants.CAN_ID, MotorType.kBrushless);
    utbIntakeEncoder = utbIntakeMotor.getEncoder();
  }

  /** Updates the printed values for the utb Intake */
  public void updateInputs(UTBIntakeIOInputs inputs) {
    // Converts rotaions to Radians and then divides it by the gear ratio
    inputs.utbIntakeRPM = utbIntakeEncoder.getVelocity() / UTBIntakeConstants.GEAR_RATIO;

    inputs.utbIntakeAppliedVolts =
        utbIntakeMotor.getAppliedOutput() * utbIntakeMotor.getBusVoltage();

    inputs.utbIntakeCurrentAmps =
        new double[] {utbIntakeMotor.getOutputCurrent()}; // amps used by intake
  }

  /**
   * Sets the voltage of the utb Intake motor
   *
   * @param voltage
   */
  public void setutbIntakeVoltage(double voltage) {
    utbIntakeMotor.setVoltage(voltage);
  }

  /**
   * Sets the utb Intake to a percentage of its maximum speed
   *
   * @param percent A value between -1 and 1
   */
  public void setutbIntakePercentSpeed(double percent) {
    utbIntakeMotor.set(percent);
  }
}
