// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.PIDController;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  // Creates the PID Contollers for both shooter motors
  private final PIDController topShooterPID;
  private final PIDController bottomShooterPID;

  // The desired RPM for the shooter
  private double setpointRPM = 0.0;

  public Shooter(ShooterIO io) {

    System.out.println("[Init] Creating Shooter");
    this.io = io;

    topShooterPID =
        new PIDController(
            ShooterConstants.TOP_KP, ShooterConstants.TOP_KI, ShooterConstants.TOP_KD);
    bottomShooterPID =
        new PIDController(
            ShooterConstants.BOTTOM_KP, ShooterConstants.BOTTOM_KI, ShooterConstants.BOTTOM_KD);

    topShooterPID.setSetpoint(setpointRPM);
    bottomShooterPID.setSetpoint(setpointRPM);

    // Sets the tolerence of the setpoint, allowing the RPM of the motors to be within 200 RPM of
    // the setpoint
    topShooterPID.setTolerance(setpointRPM * ShooterConstants.TOLERANCE_PERCENT);
    bottomShooterPID.setTolerance(setpointRPM * ShooterConstants.TOLERANCE_PERCENT);

    // Puts adjustable PID values and setpoints onto the SmartDashboard
    SmartDashboard.putNumber("shooterTopKP", 0.0);
    SmartDashboard.putNumber("shooterTopKI", 0.0);
    SmartDashboard.putNumber("shooterTopKD", 0.0);
    SmartDashboard.putNumber("shooterBottomKP", 0.0);
    SmartDashboard.putNumber("shooterBottomKI", 0.0);
    SmartDashboard.putNumber("shooterBottomKD", 0.0);
    SmartDashboard.putNumber("shooterSetpoint", 0.0);
  }

  @Override
  public void periodic() {
    this.updateInputs();
    Logger.processInputs("Shooter", inputs);

    // Sets the voltage of the Shooter Motors using PID
    setTopShooterMotorVoltage(
        topShooterPID.calculateForVoltage(inputs.topShooterMotorRPM, ShooterConstants.MAX_RPM));
    setBottomShooterMotorVoltage(
        -bottomShooterPID.calculateForVoltage(
            Math.abs(inputs.bottomShooterMotorRPM), ShooterConstants.MAX_RPM));

    if (ShooterConstants.TOP_KP != SmartDashboard.getNumber("shooterTopKP", 0.0)
        || ShooterConstants.TOP_KI != SmartDashboard.getNumber("shooterTopKI", 0.0)
        || ShooterConstants.TOP_KD != SmartDashboard.getNumber("shooterTopKD", 0.0)
        || ShooterConstants.BOTTOM_KP != SmartDashboard.getNumber("shooterBottomKP", 0.0)
        || ShooterConstants.BOTTOM_KI != SmartDashboard.getNumber("shooterBottomKI", 0.0)
        || ShooterConstants.BOTTOM_KD != SmartDashboard.getNumber("shooterBottomKD", 0.0)) {
      updatePIDController();
    }

    if (setpointRPM != SmartDashboard.getNumber("shooterSetpoint", 0.0)) {
      updateSetpoint();
    }
    // Puts the difference between the setpoint and current RPM on the SmartDashboard
    SmartDashboard.putNumber("shooterTopError", setpointRPM - inputs.topShooterMotorRPM);
    SmartDashboard.putNumber(
        "shooterBottomError", setpointRPM - Math.abs(inputs.bottomShooterMotorRPM));
    SmartDashboard.putNumber(
        "shooterMotorsRPMDifference",
        inputs.topShooterMotorRPM - Math.abs(inputs.bottomShooterMotorRPM));

    // Returns whether or not motors have reached setpoint
    SmartDashboard.putBoolean("shooterTopAtSetpoint", topAtSetpoint());
    SmartDashboard.putBoolean("shooterBottomAtSetpoint", bottomAtSetpoint());

    // Gets the current PID values that the PID contollers are set to
    SmartDashboard.putNumber("shooterTopCurrentkP", topShooterPID.getP());
    SmartDashboard.putNumber("shooterTopCurrentkI", topShooterPID.getI());
    SmartDashboard.putNumber("shooterTopCurrentkD", topShooterPID.getD());
    SmartDashboard.putNumber("shooterBottomCurrentkP", bottomShooterPID.getP());
    SmartDashboard.putNumber("shooterBottomCurrentkI", bottomShooterPID.getI());
    SmartDashboard.putNumber("shooterBottomCurrentkD", bottomShooterPID.getD());

    // Gets the current setpoint that the PID contollers are set to
    SmartDashboard.putNumber("shooterTopCurrentSetpoint", topShooterPID.getSetpoint());
    SmartDashboard.putNumber("shooterBottomCurrentSetpoint", -bottomShooterPID.getSetpoint());

    // SmartDashboard.putBoolean("!!Tempature Warning!!", exceedsTemperature());
  }

  // Updates the PID values to what they are set to on the SmartDashboard
  public void updatePIDController() {
    ShooterConstants.TOP_KP = SmartDashboard.getNumber("shooterTopKP", 0.0);
    ShooterConstants.TOP_KI = SmartDashboard.getNumber("shooterTopKI", 0.0);
    ShooterConstants.TOP_KD = SmartDashboard.getNumber("shooterTopKD", 0.0);
    ShooterConstants.BOTTOM_KP = SmartDashboard.getNumber("shooterBottomKP", 0.0);
    ShooterConstants.BOTTOM_KI = SmartDashboard.getNumber("shooterBottomKI", 0.0);
    ShooterConstants.BOTTOM_KD = SmartDashboard.getNumber("shooterBottomKD", 0.0);
    topShooterPID.setPID(ShooterConstants.TOP_KP, ShooterConstants.TOP_KI, ShooterConstants.TOP_KD);
    bottomShooterPID.setPID(
        ShooterConstants.BOTTOM_KP, ShooterConstants.BOTTOM_KI, ShooterConstants.BOTTOM_KD);
  }

  // Updates the setpoint to what is typed on the SmartDashboard
  public void updateSetpoint() {
    setpointRPM = SmartDashboard.getNumber("shooterSetpoint", 0.0);
    topShooterPID.setSetpoint(setpointRPM);
    bottomShooterPID.setSetpoint(setpointRPM);
  }

  public void updateInputs() {
    io.updateInputs(inputs);
  }

  /**
   * Sets the Brake Mode for the Shooter (Brake means motor holds position, Coast means easy to move) 
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
    return topShooterPID.atSetpoint(inputs.topShooterMotorRPM);
  }

  /** Returns where the Bottom Shooter RPM is within the setpoint, including tolerance */
  public boolean bottomAtSetpoint() {
    return bottomShooterPID.atSetpoint(inputs.bottomShooterMotorRPM);
  }

  // TODO: Create a tempature shutoff/warning
  // note 2.8.24: probably also check if the last x array values are over some set temp; 100 is
  // arbitrary
  // 2.12.24: crashes in Sim, not tested on real hardware
  // public boolean exceedsTemperature() {
  //   if (inputs.topShooterTempCelcius[inputs.topShooterTempCelcius.length - 1] > 100) {
  //     return true;
  //   }
  //   return false;
  // }
}
