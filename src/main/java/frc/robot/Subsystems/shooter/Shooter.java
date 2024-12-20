// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotStateConstants;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  // Creates the PID & FF Contollers for both Shooter Motors
  private final PIDController topShooterPIDController;
  private final PIDController bottomShooterPIDController;
  private SimpleMotorFeedforward topShooterFeedforward;
  private SimpleMotorFeedforward bottomShooterFeedforward;

  /** The desired RPM for the Shooter */
  private double setpointRPM = 0.0;

  /** Used to toggle PID calculations for RPM */
  private static boolean isPIDEnabled = true;
  /** Used to toggle test features */
  private static boolean isTestingEnabled = false;

  /** Creates a new Shooter Subsystem */
  public Shooter(ShooterIO io) {
    System.out.println("[Init] Creating Shooter");
    this.io = io;

    // Initializes the Shooter PID Contollers
    topShooterPIDController =
        new PIDController(ShooterConstants.KP, ShooterConstants.KI, ShooterConstants.KD);
    bottomShooterPIDController =
        new PIDController(ShooterConstants.KP, ShooterConstants.KI, ShooterConstants.KD);
    topShooterPIDController.setSetpoint(setpointRPM);
    bottomShooterPIDController.setSetpoint(setpointRPM);
    // Sets the tolerance of the setpoint
    topShooterPIDController.setTolerance(ShooterConstants.RPM_TOLERANCE);
    bottomShooterPIDController.setTolerance(ShooterConstants.RPM_TOLERANCE);

    // Initalizes the Shooter FF Contollers
    topShooterFeedforward =
        new SimpleMotorFeedforward(ShooterConstants.KS, ShooterConstants.KV, ShooterConstants.KA);
    bottomShooterFeedforward =
        new SimpleMotorFeedforward(ShooterConstants.KS, ShooterConstants.KV, ShooterConstants.KA);

    // Puts adjustable PID and FF values onto the SmartDashboard for testing mode
    SmartDashboard.putNumber("shooterkP", 0.00275);
    SmartDashboard.putNumber("shooterkI", 0.0);
    SmartDashboard.putNumber("shooterkD", 0.0);
    SmartDashboard.putNumber("shooterkS", 0.0);
    SmartDashboard.putNumber("shooterkV", 0.0175);
    SmartDashboard.putNumber("shooterkA", 0.0);
  }

  @Override
  public void periodic() {
    this.updateInputs();
    Logger.processInputs("Shooter", inputs);

    // Sets the voltage of the Shooter Motors using PID
    if (isPIDEnabled) {
      setTopVoltage(
          topShooterPIDController.calculate(inputs.topShooterMotorRPM)
              + (topShooterFeedforward.calculate(inputs.topShooterMotorRPM)
                  / RobotStateConstants.BATTERY_VOLTAGE));
      setBottomVoltage(
          bottomShooterPIDController.calculate(inputs.bottomShooterMotorRPM)
              + (bottomShooterFeedforward.calculate(inputs.bottomShooterMotorRPM)
                  / RobotStateConstants.BATTERY_VOLTAGE));
    }

    // Enables test values along with printing other useful measurements for testing
    if (isTestingEnabled) {
      testPIDFFValues();
      SmartDashboard.putNumber(
          "Shoot RPM Diff",
          Math.abs(inputs.topShooterMotorRPM) - Math.abs(inputs.bottomShooterMotorRPM));
      SmartDashboard.putNumber("TopShooter Error", topShooterPIDController.getPositionError());
      SmartDashboard.putNumber(
          "BottomShooter Error", bottomShooterPIDController.getPositionError());
      SmartDashboard.putNumber("ShooterAvgVel", getAverageVelocityRPM());
      SmartDashboard.putNumber(
          "ShooterAvgVelError",
          (topShooterPIDController.getPositionError()
                  + bottomShooterPIDController.getPositionError())
              / 2);
      SmartDashboard.putBoolean("BothAtSetpoint", bothAtSetpoint());
    }
  }

  /** Updates the set of loggable inputs for both Shooter Motors */
  public void updateInputs() {
    io.updateInputs(inputs);
  }

  /**
   * Sets the Brake Mode for the Shooter (Brake means motor holds position, Coast means easy to
   * move)
   *
   * @param enable if enable, it sets brake mode, else it sets coast mode
   */
  public void setBrakeMode(boolean enable) {
    io.setBrakeMode(enable);
  }

  /**
   * Sets BOTH Shooter Motors at a percentage of its max speed.
   *
   * <p>A positve number spins the Top Shooter Motor CCW and the Bottom Shooter Motor CW and vice
   * versa for a negative number
   *
   * @param percent -1 to 1
   */
  public void setBothPercentSpeed(double percent) {
    io.setBothPercentSpeed(percent);
  }

  /**
   * Sets BOTH Shooter Motors at a percentage of its max speed.
   *
   * <p>A positve number spins the Top Shooter Motor CCW and CCW for a negative number
   *
   * @param percent -1 to 1
   */
  public void setTopPercentSpeed(double percent) {
    io.setTopPercentSpeed(percent);
  }

  /**
   * Sets BOTH Shooter Motors at a percentage of its max speed.
   *
   * <p>A positve number spins the Motor CW and CCW for a negative number
   *
   * @param percent -1 to 1
   */
  public void setBottomPercentSpeed(double percent) {
    io.setBottomPercentSpeed(percent);
  }

  /**
   * Sets BOTH Shooter Motors at the specified Voltage
   *
   * @param volts -12 to 12
   */
  public void setBothVoltage(double volts) {
    io.setBothVoltage(volts);
  }

  /**
   * Sets the voltage of the Top Shooter Motor
   *
   * @param volts -12 to 12
   */
  public void setTopVoltage(double volts) {
    io.setTopVoltage(volts);
  }

  /**
   * Sets the voltage of the Bottom Shooter Motor
   *
   * @param volts -12 to 12
   */
  public void setBottomVoltage(double volts) {
    io.setBottomVoltage(volts);
  }

  /** Returns whether BOTH Shooter motors are at their setpoint */
  public boolean bothAtSetpoint() {
    return bottomShooterPIDController.atSetpoint() && topShooterPIDController.atSetpoint();
  }

  /** Returns the average velocity, in RPM, of the Shooter motors */
  public double getAverageVelocityRPM() {
    return (inputs.topShooterMotorRPM + inputs.bottomShooterMotorRPM) / 2;
  }

  /**
   * Sets the PID setpoint of the Shooter
   *
   * @param setpoint RPM
   */
  public void setSetpoint(double setpoint) {
    topShooterPIDController.setSetpoint(setpoint);
    bottomShooterPIDController.setSetpoint(setpoint);
  }

  /**
   * Sets the range the RPM of the Shooter motors can be within the setpoint
   *
   * @param tolerance RPM
   */
  public void setTolerance(double tolerance) {
    topShooterPIDController.setTolerance(tolerance);
    bottomShooterPIDController.setTolerance(tolerance);
  }

  /** Returns the current RPM setpoint of the Shooter */
  public double getSetpoint() {
    return topShooterPIDController.getSetpoint();
  }

  /**
   * Toggles whether the PID controller is used to for setting the voltage of the Shooter motors
   * based on an RPM setpoint
   *
   * @param enable True = Enable, False = Disable
   */
  public void enablePID(boolean enable) {
    isPIDEnabled = enable;
  }

  /**
   * Toggles whether the PID values are used from Constants or SmartDashboard inputs
   *
   * @param enable True = Enable, False = Disable
   */
  public void enableTesting(boolean enable) {
    isTestingEnabled = enable;
  }

  /** Updates the PID values for the Shooter from SmartDashboard */
  public void updatePIDController(double kp, double ki, double kd) {
    ShooterConstants.KP = kp;
    ShooterConstants.KI = ki;
    ShooterConstants.KD = kd;
    topShooterPIDController.setPID(ShooterConstants.KP, ShooterConstants.KI, ShooterConstants.KD);
    bottomShooterPIDController.setPID(
        ShooterConstants.KP, ShooterConstants.KI, ShooterConstants.KD);
  }

  /** Updates the Feedforward values for the Shooter from SmartDashboard */
  public void updateFFController(double ks, double kv, double ka) {
    ShooterConstants.KS = ks;
    ShooterConstants.KV = kv;
    ShooterConstants.KA = ka;
    topShooterFeedforward =
        new SimpleMotorFeedforward(ShooterConstants.KS, ShooterConstants.KV, ShooterConstants.KA);
    bottomShooterFeedforward =
        new SimpleMotorFeedforward(ShooterConstants.KS, ShooterConstants.KV, ShooterConstants.KA);
  }

  /** Updates PID and FF values from SmartDashboard */
  public void testPIDFFValues() {
    if (ShooterConstants.KP != SmartDashboard.getNumber("shooterkP", 0.00275)
        || ShooterConstants.KI != SmartDashboard.getNumber("shooterkI", 0.0)
        || ShooterConstants.KD != SmartDashboard.getNumber("shooterkD", 0.0)) {
      updatePIDController(
          SmartDashboard.getNumber("shooterkP", 0.00275),
          SmartDashboard.getNumber("shooterkI", 0.0),
          SmartDashboard.getNumber("shooterkD", 0.0));
    }

    if (ShooterConstants.KS != SmartDashboard.getNumber("shooterkS", 0.0)
        || ShooterConstants.KV != SmartDashboard.getNumber("shooterkV", 0.0175)
        || ShooterConstants.KA != SmartDashboard.getNumber("shooterkA", 0.0)) {
      updateFFController(
          SmartDashboard.getNumber("shooterkS", 0.0),
          SmartDashboard.getNumber("shooterkV", 0.0175),
          SmartDashboard.getNumber("shooterkA", 0.0));
    }
  }
}
