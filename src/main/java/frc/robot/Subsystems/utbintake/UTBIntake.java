package frc.robot.Subsystems.utbintake;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.PIDController;
import org.littletonrobotics.junction.Logger;

public class UTBIntake extends SubsystemBase {

  private final UTBIntakeIO io;
  private final UTBIntakeIOInputsAutoLogged inputs = new UTBIntakeIOInputsAutoLogged();
  /** shuffleboard tabs: UTBIntake */
  private final ShuffleboardTab UTBIntakeTab = Shuffleboard.getTab("UTBIntake");

  private GenericEntry utbIntakekp;
  private GenericEntry utbIntakeki;
  private GenericEntry utbIntakekd;
  // private GenericEntry utbIntakeSetpointSetter;

  /** utb intake pid controller */
  private final PIDController utbIntakePIDController;

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

    utbIntakekp = UTBIntakeTab.add("UTBIntakekp", 0.0).getEntry();
    utbIntakeki = UTBIntakeTab.add("UTBIntakeki", 0.0).getEntry();
    utbIntakekd = UTBIntakeTab.add("UTBIntakekd", 0.0).getEntry();
    // utbIntakeSetpointSetter = UTBIntakeTab.add("UTBIntakeSetpoint", 0.0).getEntry();
  }

  @Override
  /** Periodically updates the inputs and outputs of the UTB Intake */
  public void periodic() {
    this.updateInputs();
    Logger.processInputs("UTBIntake", inputs);

    // updates UTB Intake voltage from PID calculations
    setUTBIntakeVoltage(
        utbIntakePIDController.calculateForVoltage(
            inputs.utbIntakeRPM, UTBIntakeConstants.MAX_RPM));

    // // TODO: delete once PID values are finalized
    // if (UTBIntakeConstants.KP != utbIntakekp.getDouble(0.0)
    //     || UTBIntakeConstants.KI != utbIntakeki.getDouble(0.0)
    //     || UTBIntakeConstants.KD != utbIntakekd.getDouble(0.0)) {
    //   updatePIDController();
    // }

    // if (utbIntakeSetpoint != utbIntakeSetpointSetter.getDouble(0.0)) {
    //   updateSetpoint();
    // }
  }

  /** updates PID values if SmartDashboard gets updated */
  // public void updatePIDController() {
  //   UTBIntakeConstants.KP = utbIntakekp.getDouble(0.0);
  //   UTBIntakeConstants.KI = utbIntakeki.getDouble(0.0);
  //   UTBIntakeConstants.KD = utbIntakekd.getDouble(0.0);

  //   utbIntakePIDController.setPID(
  //       UTBIntakeConstants.KP, UTBIntakeConstants.KI, UTBIntakeConstants.KD);
  // }

  // /** updates setpoint if SmartDashboard gets updated */
  // public void updateSetpoint() {
  //   utbIntakeSetpoint = utbIntakeSetpointSetter.getDouble(0.0);
  //   utbIntakePIDController.setSetpoint(utbIntakeSetpoint);
  // }

  /** Updates the inputs for the UTB Intake */
  public void updateInputs() {
    io.updateInputs(inputs);
  }

  /** Sets intake voltage for the UTB Intake :) */
  public void setUTBIntakeVoltage(double volts) {
    io.setUTBIntakeVoltage(volts);
  }

  /** Sets intake speed for the UTB Intake :) */
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

  /** Sets the speed of the UTB Intake to predetermined speed (currently 1000 RPM) */
  public void enableUTBPID(boolean enable) {
    if (enable == true) {
      // setUTBIntakePercentSpeed(15);
      utbIntakePIDController.setSetpoint(-2000);
    } else {
      // setUTBIntakePercentSpeed(0);
      utbIntakePIDController.setSetpoint(0);
      setUTBIntakeVoltage(0);
    }
  }

  public void enableUTB(boolean enable) {
    if (enable == true) {
      setUTBIntakePercentSpeed(0.75);
    } else {
      setUTBIntakePercentSpeed(0);
    }
  }

  public void disableUTB() {
    utbIntakePIDController.setSetpoint(0);
    setUTBIntakeVoltage(0);
  }

  public void setUTBSetpoint(double setpoint) {
    utbIntakePIDController.setSetpoint(setpoint);
    utbIntakePIDController.calculateForVoltage(inputs.utbIntakeRPM, UTBIntakeConstants.MAX_RPM);
  }
}
