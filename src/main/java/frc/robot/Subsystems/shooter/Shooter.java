// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.shooter;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.PIDController;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private final ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter");
  private GenericEntry shooterTopkP;
  private GenericEntry shooterTopkI;
  private GenericEntry shooterTopkD;
  private GenericEntry shooterBottomkP;
  private GenericEntry shooterBottomkI;
  private GenericEntry shooterBottomkD;
  private GenericEntry shooterSetpointSetter;

  // Creates the PID Contollers for both shooter motors
  private final PIDController topShooterPIDController;
  private final PIDController bottomShooterPIDController;

  // The desired RPM for the shooter
  private double setpointRPM = 0.0;

  public Shooter(ShooterIO io) {

    System.out.println("[Init] Creating Shooter");
    this.io = io;

    topShooterPIDController =
        new PIDController(
            ShooterConstants.TOP_KP, ShooterConstants.TOP_KI, ShooterConstants.TOP_KD);
    bottomShooterPIDController =
        new PIDController(
            ShooterConstants.BOTTOM_KP, ShooterConstants.BOTTOM_KI, ShooterConstants.BOTTOM_KD);

    topShooterPIDController.setSetpoint(setpointRPM);
    bottomShooterPIDController.setSetpoint(setpointRPM);

    // Sets the tolerance of the setpoint, allowing the RPM of the motors to be within 200 RPM of
    // the setpoint
    topShooterPIDController.setTolerance(setpointRPM * ShooterConstants.TOLERANCE_PERCENT);
    bottomShooterPIDController.setTolerance(setpointRPM * ShooterConstants.TOLERANCE_PERCENT);

    // Puts adjustable PID values and setpoints onto the SmartDashboard
    shooterTopkP = shooterTab.add("shooterTopkp", 0.0).getEntry();
    shooterTopkI = shooterTab.add("shooterTopki", 0.0).getEntry();
    shooterTopkD = shooterTab.add("shooterTopkd", 0.0).getEntry();
    shooterBottomkP = shooterTab.add("shooterBottomkp", 0.0).getEntry();
    shooterBottomkI = shooterTab.add("shooterBottomki", 0.0).getEntry();
    shooterBottomkD = shooterTab.add("shooterBottomkd", 0.0).getEntry();
    shooterSetpointSetter = shooterTab.add("shooterSetpoint", 0.0).getEntry();
  }

  @Override
  public void periodic() {
    this.updateInputs();
    Logger.processInputs("Shooter", inputs);

    // Sets the voltage of the Shooter Motors using PID
    setTopShooterMotorVoltage(
        topShooterPIDController.calculateForVoltage(
            inputs.topShooterMotorRPM, ShooterConstants.MAX_VALUE));
    setBottomShooterMotorVoltage(
        -bottomShooterPIDController.calculateForVoltage(
            Math.abs(inputs.bottomShooterMotorRPM), ShooterConstants.MAX_VALUE));

    if (ShooterConstants.TOP_KP != shooterTopkP.getDouble(0.0)
        || ShooterConstants.TOP_KI != shooterTopkI.getDouble(0.0)
        || ShooterConstants.TOP_KD != shooterTopkD.getDouble(0.0)
        || ShooterConstants.BOTTOM_KP != shooterBottomkP.getDouble(0.0)
        || ShooterConstants.BOTTOM_KI != shooterBottomkI.getDouble(0.0)
        || ShooterConstants.BOTTOM_KD != shooterBottomkD.getDouble(0.0)) {
      updatePIDController();
    }

    if (setpointRPM != shooterSetpointSetter.getDouble(0.0)) {
      updateSetpoint();
    }

    // Returns whether or not motors have reached setpoint
    SmartDashboard.putBoolean("shooterTopAtSetpoint", topAtSetpoint());
    SmartDashboard.putBoolean("shooterBottomAtSetpoint", bottomAtSetpoint());

    // SmartDashboard.putboolean("!!Tempature Warning!!", exceedsTemperature());
  }

  // Updates the PID values to what they are set to on the SmartDashboard
  public void updatePIDController() {
    ShooterConstants.TOP_KP = shooterTopkP.getDouble(0.0);
    ShooterConstants.TOP_KI = shooterTopkI.getDouble(0.0);
    ShooterConstants.TOP_KD = shooterTopkD.getDouble(0.0);
    ShooterConstants.BOTTOM_KP = shooterBottomkP.getDouble(0.0);
    ShooterConstants.BOTTOM_KI = shooterBottomkI.getDouble(0.0);
    ShooterConstants.BOTTOM_KD = shooterBottomkD.getDouble(0.0);
    topShooterPIDController.setPID(
        ShooterConstants.TOP_KP, ShooterConstants.TOP_KI, ShooterConstants.TOP_KD);
    bottomShooterPIDController.setPID(
        ShooterConstants.BOTTOM_KP, ShooterConstants.BOTTOM_KI, ShooterConstants.BOTTOM_KD);
  }

  // Updates the setpoint to what is typed on the SmartDashboard
  public void updateSetpoint() {
    setpointRPM = shooterSetpointSetter.getDouble(0.0);
    topShooterPIDController.setSetpoint(setpointRPM);
    bottomShooterPIDController.setSetpoint(setpointRPM);
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
  public void setShooterBrakeMode(boolean enable) {
    io.setShooterBrakeMode(enable);
  }

  /**
   * Sets BOTH Shooter Motors at a percentage of its max speed.
   *
   * <p>A positve number spins the Top Shooter Motor CCW and the Bottom Shooter Motor CW and vice
   * versa for a negative number
   *
   * @param percent -1 to 1
   */
  public void setShooterMotorPercentSpeed(double percent) {
    io.setBothShooterMotorPercentSpeed(percent);
  }

  /**
   * Sets BOTH Shooter Motors at the specified Voltage
   *
   * <p>A positve number spins the Top Shooter Motor CCW and the Bottom Shooter Motor CW and vice
   * versa for a negative number
   *
   * @param volts -12 to 12
   */
  public void setBothShooterMotorsVoltage(double volts) {
    io.setBothShooterMotorsVoltage(volts);
  }

  /**
   * Sets the voltage of the Top Shooter Motor
   *
   * @param volts -12 to 12
   */
  public void setTopShooterMotorVoltage(double volts) {
    io.setTopShooterMotorVoltage(volts);
  }

  /**
   * Sets the voltage of the Bottom Shooter Motor
   *
   * @param volts -12 to 12
   */
  public void setBottomShooterMotorVoltage(double volts) {
    io.setBottomShooterMotorVoltage(volts);
  }

  /** Returns where the Top Shooter RPM is within the setpoint, including tolerance */
  public boolean topAtSetpoint() {
    return topShooterPIDController.atSetpoint(inputs.topShooterMotorRPM);
  }

  /** Returns where the Bottom Shooter RPM is within the setpoint, including tolerance */
  public boolean bottomAtSetpoint() {
    return bottomShooterPIDController.atSetpoint(inputs.bottomShooterMotorRPM);
  }

  public void enableShooter(boolean auxAIsPressed) {
    if (auxAIsPressed) {
      io.setBothShooterMotorPercentSpeed(100);
    } else {
      io.setBothShooterMotorPercentSpeed(0);
    }
  }

  // TODO: Create a tempature shutoff/warning
  // note 2.8.24: probably also check if the last x array values are over some set temp; 100 is
  // arbitrary
  // 2.12.24: crashes in Sim, not tested on real hardware
  // public boolean exceedsTemperature() {
  //   if (inputs.topShooterTempCelsius[inputs.topShooterTempCelsius.length - 1] > 100) {
  //     return true;
  //   }
  //   return false;
  // }
}
