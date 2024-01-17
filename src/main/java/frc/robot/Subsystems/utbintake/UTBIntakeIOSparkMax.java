package frc.robot.Subsystems.utbintake;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.UTBIntakeConstants;

/**UTBIntake motor controller*/
public class UTBIntakeIOSparkMax implements UTBIntakeIO {
  private CANSparkMax UTBIntakeMotor;
  private RelativeEncoder UTBIntakeEncoder;

  /**Creates the Motor and Encoder for the UTB Intake */
  public UTBIntakeIOSparkMax() {
    System.out.println("[Init] Creating UTBIntakeIO");
    UTBIntakeMotor = new CANSparkMax(UTBIntakeConstants.UTB_INTAKE_CANID, MotorType.kBrushless);
    UTBIntakeEncoder = UTBIntakeMotor.getEncoder();
  }

  /**Updates the printed values for the UTB Intake */
  public void updateInputs(UTBIntakeIOInputs inputs) {
    inputs.UTBIntakeVelocityRadPerSec = Units.rotationsToRadians(UTBIntakeEncoder.getPosition()) / UTBIntakeConstants.GEAR_RATIO; // Converts rotaions to Radians and then divides it by the gear ratio
    inputs.UTBIntakeAppliedVolts = UTBIntakeMotor.getAppliedOutput() * UTBIntakeMotor.getBusVoltage(); // Applied voltage of intake
    inputs.UTBIntakeCurrentAmps = new double[] {UTBIntakeMotor.getOutputCurrent()}; // Amps used by intake
  }

  /**
   * Sets the voltage for the UTB Intake
   * @param voltage
   */
  public void setUTBIntakeVoltage(double voltage) {
    UTBIntakeMotor.setVoltage(voltage);
  }
}
