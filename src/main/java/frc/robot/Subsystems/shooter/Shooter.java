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
  private static PIDController topShooterPID;
  private static PIDController bottomShooterPID;

  // The desired RPM for the shooter
  private double setpointRPM = 0.0;

  public Shooter(ShooterIO io) {

    System.out.println("[Init] Creating Shooter");
    this.io = io;

    topShooterPID =
        new PIDController(
            ShooterConstants.TOP_KP,
            ShooterConstants.TOP_KI,
            ShooterConstants.TOP_KD
            );
    bottomShooterPID =
        new PIDController(
            ShooterConstants.BOTTOM_KP,
            ShooterConstants.BOTTOM_KI,
            ShooterConstants.BOTTOM_KD
            );

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
    SmartDashboard.putNumber("TopError", setpointRPM - inputs.topShooterMotorRPM);
    SmartDashboard.putNumber("BottomError", setpointRPM - Math.abs(inputs.bottomShooterMotorRPM));
    SmartDashboard.putNumber(
        "Top Bottom Motor RPM Difference",
        inputs.topShooterMotorRPM - Math.abs(inputs.bottomShooterMotorRPM));

    // Sets the voltage of the Shooter Motors using PID
      setTopShooterMotorVoltage(topShooterPID.calculateForVoltage(inputs.topShooterMotorRPM, 6350));
      setBottomShooterMotorVoltage(
          -bottomShooterPID.calculateForVoltage(Math.abs(inputs.bottomShooterMotorRPM), 6350));

    // Returns whether or not motors have reached setpoint
    SmartDashboard.putBoolean("TopAtSetpoint", topAtSetpoint());
    SmartDashboard.putBoolean("BottomAtSetpoint", bottomAtSetpoint());

    // Gets the current PID values that the PID contollers are set to
    SmartDashboard.putNumber("topCurrentkP", topShooterPID.getP());
    SmartDashboard.putNumber("topCurrentkI", topShooterPID.getI());
    SmartDashboard.putNumber("topCurrentkD", topShooterPID.getD());
    SmartDashboard.putNumber("bottomCurrentkP", bottomShooterPID.getP());
    SmartDashboard.putNumber("bottomCurrentkI", bottomShooterPID.getI());
    SmartDashboard.putNumber("bottomCurrentkD", bottomShooterPID.getD());

    // Gets the current setpoint that the PID contollers are set to
    SmartDashboard.putNumber("Top PID Controller Setpoint", topShooterPID.getSetpoint());
    SmartDashboard.putNumber("Bottom PID Controller Setpoint", -bottomShooterPID.getSetpoint());

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
    bottomShooterPID.setPID(ShooterConstants.BOTTOM_KP, ShooterConstants.BOTTOM_KI, ShooterConstants.BOTTOM_KD);
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

  public void setShooterBreakMode(boolean enable) {
    io.setShooterBreakMode(enable);
  }

  public void setShooterMotorPercentSpeed(double percent) {
    io.setBothShooterMotorPercentSpeed(percent);
  }

  public void setBothShooterMotorsVoltage(double volts) {
    io.setBothShooterMotorsVoltage(volts);
  }

  public void setTopShooterMotorVoltage(double volts) {
    io.setTopShooterMotorVoltage(volts);
  }

  public void setBottomShooterMotorVoltage(double volts) {
    io.setBottomShooterMotorVoltage(volts);
  }

  public boolean topAtSetpoint() {
    return topShooterPID.atSetpoint(inputs.topShooterMotorRPM);
  }

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
