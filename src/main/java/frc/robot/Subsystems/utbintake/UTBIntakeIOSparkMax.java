package frc.robot.Subsystems.utbintake;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

/** UTB Intake motor controller */
public class UTBIntakeIOSparkMax implements UTBIntakeIO {
  private final SparkMax topUTBIntakeMotor;
  private final SparkMax bottomUTBIntakeMotor;
  private final RelativeEncoder topUTBIntakeEncoder;
  private final RelativeEncoder bottomUTBIntakeEncoder;
  private final SparkMaxConfig topUTBConfig;
  private final SparkMaxConfig bottomUTBConfig;


  /** Creates the Motor and Encoder for the Under the Bumper (UTB) Intake */
  public UTBIntakeIOSparkMax() {
    System.out.println("[Init] Creating UTBIntakeIOSparkMax");

    /** Creates the Motor and Encoder for the UTB Intake */
    topUTBIntakeMotor = new SparkMax(UTBIntakeConstants.TOP_CAN_ID, MotorType.kBrushless);
    bottomUTBIntakeMotor = new SparkMax(UTBIntakeConstants.BOTTOM_CAN_ID, MotorType.kBrushless);
    topUTBIntakeEncoder = topUTBIntakeMotor.getEncoder();
    bottomUTBIntakeEncoder = bottomUTBIntakeMotor.getEncoder();
    topUTBConfig = new SparkMaxConfig();
    bottomUTBConfig = new SparkMaxConfig();
    topUTBConfig.idleMode(IdleMode.kBrake);
    bottomUTBConfig.idleMode(IdleMode.kBrake);
    topUTBConfig.inverted(UTBIntakeConstants.IS_TOP_INVERTED);
    bottomUTBConfig.inverted(UTBIntakeConstants.IS_BOTTOM_INVERTED);
    topUTBConfig.smartCurrentLimit(UTBIntakeConstants.CUR_LIM_A);
    bottomUTBConfig.smartCurrentLimit(UTBIntakeConstants.CUR_LIM_A);

    /** Default inversion status of the motors */
    topUTBConfig.inverted(UTBIntakeConstants.IS_TOP_INVERTED);
    bottomUTBConfig.inverted(UTBIntakeConstants.IS_BOTTOM_INVERTED);

    /** Defaults to brake mode on initialization */
    topUTBConfig.idleMode(IdleMode.kBrake);
    bottomUTBConfig.idleMode(IdleMode.kBrake);

    /** Sets the current limit of the motors */
    topUTBConfig.smartCurrentLimit(UTBIntakeConstants.CUR_LIM_A);
    bottomUTBConfig.smartCurrentLimit(UTBIntakeConstants.CUR_LIM_A);

    /** Saves the configuration to the SPARKMAX */
    topUTBIntakeMotor.configure(topUTBConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    bottomUTBIntakeMotor.configure(bottomUTBConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /** Updates the printed values for the UTB Intake */
  public void updateInputs(UTBIntakeIOInputs inputs) {
    // Converts rotaions to Radians and then divides it by the gear ratio
    inputs.topUTBIntakeRPM = topUTBIntakeEncoder.getVelocity() / UTBIntakeConstants.GEAR_RATIO_TOP;
    inputs.topUTBIntakeAppliedVolts =
        topUTBIntakeMotor.getAppliedOutput() * topUTBIntakeMotor.getBusVoltage();
    inputs.topUTBIntakeCurrentAmps = topUTBIntakeMotor.getOutputCurrent();
    inputs.topUTBIntakeTempCelsius = topUTBIntakeMotor.getMotorTemperature();

    inputs.bottomUTBIntakeRPM =
        bottomUTBIntakeEncoder.getVelocity() / UTBIntakeConstants.GEAR_RATIO_BOTTOM;
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
    topUTBConfig.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    bottomUTBConfig.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    topUTBIntakeMotor.configure(topUTBConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    bottomUTBIntakeMotor.configure(bottomUTBConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
}
