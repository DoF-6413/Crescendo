package frc.robot.Subsystems.utbintake;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

/** UTB Intake motor controller */
public class UTBIntakeIOSparkMax implements UTBIntakeIO {
  private final CANSparkMax topUTBIntakeMotor;
  private final CANSparkMax bottomUTBIntakeMotor;
  private final RelativeEncoder topUTBIntakeEncoder;
  private final RelativeEncoder bottomUTBIntakeEncoder;

  /** Creates the Motor and Encoder for the Under the Bumper (UTB) Intake */
  public UTBIntakeIOSparkMax() {
    System.out.println("[Init] Creating UTBIntakeIOSparkMax");

    /** Creates the Motor and Encoder for the UTB Intake */
    topUTBIntakeMotor = new CANSparkMax(UTBIntakeConstants.TOP_CAN_ID, MotorType.kBrushless);
    bottomUTBIntakeMotor = new CANSparkMax(UTBIntakeConstants.BOTTOM_CAN_ID, MotorType.kBrushless);
    topUTBIntakeEncoder = topUTBIntakeMotor.getEncoder();
    bottomUTBIntakeEncoder = bottomUTBIntakeMotor.getEncoder();

    topUTBIntakeMotor.setInverted(UTBIntakeConstants.IS_TOP_INVERTED);
    bottomUTBIntakeMotor.setInverted(UTBIntakeConstants.IS_BOTTOM_INVERTED);

    /** Defaults to brake mode on initialization */
    topUTBIntakeMotor.setIdleMode(IdleMode.kBrake);
    bottomUTBIntakeMotor.setIdleMode(IdleMode.kBrake);

    /** Saves the configuration to the SPARKMAX */
    topUTBIntakeMotor.burnFlash();
    bottomUTBIntakeMotor.burnFlash();
  }

  /** Updates the printed values for the UTB Intake */
  public void updateInputs(UTBIntakeIOInputs inputs) {
    // Converts rotaions to Radians and then divides it by the gear ratio
    inputs.topUTBIntakeRPM = topUTBIntakeEncoder.getVelocity() / UTBIntakeConstants.GEAR_RATIO;
    inputs.topUTBIntakeAppliedVolts =
        topUTBIntakeMotor.getAppliedOutput() * topUTBIntakeMotor.getBusVoltage();
    inputs.topUTBIntakeCurrentAmps = topUTBIntakeMotor.getOutputCurrent();
    inputs.topUTBIntakeTempCelsius = topUTBIntakeMotor.getMotorTemperature();

    inputs.bottomUTBIntakeRPM =
        bottomUTBIntakeEncoder.getVelocity() / UTBIntakeConstants.GEAR_RATIO;
    inputs.bottomUTBIntakeAppliedVolts =
        bottomUTBIntakeMotor.getAppliedOutput() * bottomUTBIntakeMotor.getBusVoltage();
    inputs.bottomUTBIntakeCurrentAmps = bottomUTBIntakeMotor.getOutputCurrent();
    inputs.bottomUTBIntakeTempCelsius = bottomUTBIntakeMotor.getMotorTemperature();
  }

  @Override
  public void setVoltage(double voltage) {
    topUTBIntakeMotor.setVoltage(voltage);
    bottomUTBIntakeMotor.setVoltage(voltage);
  }

  @Override
  public void setPercentSpeed(double percent) {
    topUTBIntakeMotor.set(percent);
    bottomUTBIntakeMotor.set(percent);
  }

  @Override
  public void setBrakeMode(boolean enable) {
    topUTBIntakeMotor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    bottomUTBIntakeMotor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }
}
