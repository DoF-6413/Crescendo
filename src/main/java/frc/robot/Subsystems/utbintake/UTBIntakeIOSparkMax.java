package frc.robot.Subsystems.utbintake;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.UTBIntakeConstants;

/** UTBIntake motor controller */
public class UTBIntakeIOSparkMax implements UTBIntakeIO {
  private CANSparkMax utbIntakeMotor;
  private RelativeEncoder utbIntakeEncoder;

  /** Creates the Motor and Encoder for the UTB Intake */
  public UTBIntakeIOSparkMax() {
    System.out.println("[Init] Creating UTBIntakeIO");
    utbIntakeMotor = new CANSparkMax(UTBIntakeConstants.UTB_INTAKE_CANID, MotorType.kBrushless);
    utbIntakeEncoder = utbIntakeMotor.getEncoder();
  }

  /** Updates the printed values for the UTB Intake */
  public void updateInputs(UTBIntakeIOInputs inputs) {
    inputs.utbIntakeVelocityRadPerSec =
        Units.rotationsToRadians(utbIntakeEncoder.getPosition())
            / UTBIntakeConstants
                .GEAR_RATIO; // Converts rotaions to Radians and then divides it by the gear ratio
    inputs.utbIntakeAppliedVolts =
        utbIntakeMotor.getAppliedOutput()
            * utbIntakeMotor.getBusVoltage(); // Applied voltage of intake
    inputs.utbIntakeCurrentAmps =
        new double[] {utbIntakeMotor.getOutputCurrent()}; // Amps used by intake
  }

  /**
   * Sets the voltage of the UTB Intake motor
   *
   * @param voltage
   */
  public void setUTBIntakeVoltage(double voltage) {
    utbIntakeMotor.setVoltage(voltage);
  }

  /**
   * Sets the UTB Intake to a percentage of its maximum speed
   *
   * @param percent A value between -1 and 1
   */
  public void setUTBIntakePercentSpeed(double percent) {
    utbIntakeMotor.set(percent);
  }
}
