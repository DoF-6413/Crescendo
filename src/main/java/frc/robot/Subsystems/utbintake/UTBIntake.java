package frc.robot.Subsystems.utbintake;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.PIDController;
import org.littletonrobotics.junction.Logger;

public class UTBIntake extends SubsystemBase {

  private final UTBIntakeIO io;
  private final UTBIntakeIOInputsAutoLogged inputs = new UTBIntakeIOInputsAutoLogged();
  /** shuffleboard tabs: utbIntake */
  private final ShuffleboardTab utbIntakeTab = Shuffleboard.getTab("UTBIntake");

  private GenericEntry utbIntakekp;
  private GenericEntry utbIntakeki;
  private GenericEntry utbIntakekd;
  private GenericEntry utbIntakeSetpointSetter;

  /** utb intake pid controller */
  private static PIDController utbIntakePIDController;

  private double utbIntakeSetpoint = 0.0;

  /**
   * creates an Under the Bumper (UTB) Intake, the subsystem that intakes pieces from under the
   * bumper
   */
  public UTBIntake(UTBIntakeIO io) {
    System.out.println("[Init] Creating UTB Intake");
    this.io = io;

    /** creates a new PIDController for the UTBIntake */
    utbIntakePIDController =
        new PIDController(UTBIntakeConstants.KP, UTBIntakeConstants.KI, UTBIntakeConstants.KD);

    /** sets tolerance and setpoint for UTBIntake PIDController */
    utbIntakePIDController.setSetpoint(utbIntakeSetpoint);
    utbIntakePIDController.setTolerance(utbIntakeSetpoint * UTBIntakeConstants.TOLERANCE_PERCENT);

    utbIntakekp = utbIntakeTab.add("UTBIntakekp", 0.0).getEntry();
    utbIntakeki = utbIntakeTab.add("UTBIntakeki", 0.0).getEntry();
    utbIntakekd = utbIntakeTab.add("UTBIntakekd", 0.0).getEntry();
    utbIntakeSetpointSetter = utbIntakeTab.add("UTBIntakeSetpoint", 0.0).getEntry();
  }

  @Override
  /** Periodically updates the inputs and outputs of the utb Intake */
  public void periodic() {
    this.updateInputs();
    Logger.processInputs("UTBIntake", inputs);

    // updates UTB Intake voltage from PID calculations
    setUTBIntakeVoltage(
        utbIntakePIDController.calculateForVoltage(
            inputs.utbIntakeRPM, UTBIntakeConstants.MAX_RPM));

    // TODO: delete once PID values are finalized
    if (UTBIntakeConstants.KP != utbIntakekp.getDouble(0.0)
        || UTBIntakeConstants.KI != utbIntakeki.getDouble(0.0)
        || UTBIntakeConstants.KD != utbIntakekd.getDouble(0.0)) {
      updatePIDController();
    }

    if (utbIntakeSetpoint != utbIntakeSetpointSetter.getDouble(0.0)) {
      updateSetpoint();
    }
  }

  /** updates PID values if SmartDashboard gets updated */
  public void updatePIDController() {
    UTBIntakeConstants.KP = utbIntakekp.getDouble(0.0);
    UTBIntakeConstants.KI = utbIntakeki.getDouble(0.0);
    UTBIntakeConstants.KD = utbIntakekd.getDouble(0.0);

    utbIntakePIDController.setPID(
        UTBIntakeConstants.KP, UTBIntakeConstants.KI, UTBIntakeConstants.KD);
  }

  /** updates setpoint if SmartDashboard gets updated */
  public void updateSetpoint() {
    utbIntakeSetpoint = utbIntakeSetpointSetter.getDouble(0.0);
    utbIntakePIDController.setSetpoint(utbIntakeSetpoint);
  }

  /** Updates the inputs for the utb Intake */
  public void updateInputs() {
    io.updateInputs(inputs);
  }

  /** Sets intake voltage for the utb Intake :) */
  public void setUTBIntakeVoltage(double volts) {
    io.setUTBIntakeVoltage(volts);
  }

  /** Sets intake speed for the utb Intake :) */
  public void setUTBIntakePercentSpeed(double percent) {
    io.setUTBIntakePercentSpeed(percent);
  }

  /** Returns the speed of the wheels for the intake found under the bumpers in RPM */
  public double getUTBIntakeRPM() {
    return inputs.utbIntakeRPM;
  }

  /**
   * Sets brake mode
   *
   * @param isEnabled boolean for is brake mode true or false
   */
  public void setUTBIntakeBrakeMode(boolean enable) {
    io.setUTBIntakeBrakeMode(enable);
  }
}
