// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotStateConstants;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private final ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter");
  private final GenericEntry shooterkP;
  private final GenericEntry shooterkI;
  private final GenericEntry shooterkD;
  private final GenericEntry shooterkS;
  private final GenericEntry shooterkV;
  private final GenericEntry shooterkA;
  private final GenericEntry shooterSetpoint;

  // Creates the PID & FF Contollers for both shooter motors
  private final PIDController topShooterPIDController;
  private final PIDController bottomShooterPIDController;
  private SimpleMotorFeedforward topShooterFeedforward;
  private SimpleMotorFeedforward bottomShooterFeedforward;

  // The desired RPM for the shooter
  private double setpointRPM = 0.0;

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
    // topShooterFeedforward.maxAchievableAcceleration(
    //     RobotStateConstants.BATTERY_VOLTAGE, inputs.topShooterMotorRPM);
    // bottomShooterFeedforward.maxAchievableAcceleration(
    //     RobotStateConstants.BATTERY_VOLTAGE, inputs.bottomShooterMotorRPM);

    // Puts adjustable PID & FF values and setpoints onto the ShuffleBoard
    shooterkP = shooterTab.add("shooterkP", 0.0).getEntry();
    shooterkI = shooterTab.add("shooterkI", 0.0).getEntry();
    shooterkD = shooterTab.add("shooterkD", 0.0).getEntry();
    shooterSetpoint = shooterTab.add("shooterSetpoint", 0.0).getEntry();
    shooterkS = shooterTab.add("shooterkS", 0.0).getEntry();
    shooterkV = shooterTab.add("shooterkV", 0.0).getEntry();
    shooterkA = shooterTab.add("shooterkA", 0.0).getEntry();
  }

  @Override
  public void periodic() {
    this.updateInputs();
    Logger.processInputs("Shooter", inputs);

    // Sets the voltage of the Shooter Motors using PID
    setTopPercentSpeed(
        topShooterPIDController.calculate(inputs.topShooterMotorRPM)
            + (topShooterFeedforward.calculate(inputs.topShooterMotorRPM)
                / RobotStateConstants.BATTERY_VOLTAGE));
    setBottomPercentSpeed(
        bottomShooterPIDController.calculate(inputs.bottomShooterMotorRPM)
            + (bottomShooterFeedforward.calculate(inputs.bottomShooterMotorRPM)
                / RobotStateConstants.BATTERY_VOLTAGE));

    if (ShooterConstants.KP != shooterkP.getDouble(0.0)
        || ShooterConstants.KI != shooterkI.getDouble(0.0)
        || ShooterConstants.KD != shooterkD.getDouble(0.0)) {
      updatePIDController();
    }

    if (setpointRPM != shooterSetpoint.getDouble(0.0)) {
      updateSetpoint();
    }

    if (ShooterConstants.KS != shooterkS.getDouble(0.0)
        || ShooterConstants.KV != shooterkV.getDouble(0.0)
        || ShooterConstants.KA != shooterkA.getDouble(0.0)) {
      updateFFController();
    }

    SmartDashboard.putBoolean("BothAtSetpoint", bothAtSetpoint());
    // SmartDashboard.putBoolean("TopAtSetpoint", topAtSetpoint());
    // SmartDashboard.putBoolean("BottomAtSetpoint", bottomAtSetpoint());

  }

  /** Updates the PID values for the Shooter from ShuffleBoard */
  public void updatePIDController() {
    ShooterConstants.KP = shooterkP.getDouble(0.0);
    ShooterConstants.KI = shooterkI.getDouble(0.0);
    ShooterConstants.KD = shooterkD.getDouble(0.0);
    topShooterPIDController.setPID(ShooterConstants.KP, ShooterConstants.KI, ShooterConstants.KD);
    bottomShooterPIDController.setPID(
        ShooterConstants.KP, ShooterConstants.KI, ShooterConstants.KD);
  }

  /** Updates the Setpoint for the Shooter from ShuffleBoard */
  public void updateSetpoint() {
    setpointRPM = shooterSetpoint.getDouble(0.0);
    topShooterPIDController.setSetpoint(setpointRPM);
    bottomShooterPIDController.setSetpoint(setpointRPM);
  }

  /** Updates the Feedforward values for the Shooter from ShuffleBoard */
  public void updateFFController() {
    ShooterConstants.KS = shooterkS.getDouble(0.0);
    ShooterConstants.KV = shooterkV.getDouble(0.0);
    ShooterConstants.KA = shooterkA.getDouble(0.0);
    topShooterFeedforward =
        new SimpleMotorFeedforward(ShooterConstants.KS, ShooterConstants.KV, ShooterConstants.KA);
    bottomShooterFeedforward =
        new SimpleMotorFeedforward(ShooterConstants.KS, ShooterConstants.KV, ShooterConstants.KA);
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
  public void setBothsVoltage(double volts) {
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

  /** Returns where the Top Shooter RPM is within the setpoint, including tolerance */
  public boolean topAtSetpoint() {
    return topShooterPIDController.atSetpoint();
  }

  /** Returns where the Bottom Shooter RPM is within the setpoint, including tolerance */
  public boolean bottomAtSetpoint() {
    return bottomShooterPIDController.atSetpoint();
  }

  /** Returns whether BOTH Shooter motors are at their setpoint */
  public boolean bothAtSetpoint() {
    return bottomAtSetpoint() && topAtSetpoint();
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
}
