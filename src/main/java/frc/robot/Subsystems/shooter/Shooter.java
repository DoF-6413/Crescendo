// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  // Creates the PID Contollers for both Shooter Motors
  private static PIDController topShooterPID;
  private static PIDController bottomShooterPID;

  private static SimpleMotorFeedforward topFeedForward;
  private static SimpleMotorFeedforward bottomFeedForward;

  // The desired RPM for the Shooter Motors
  private double setpointRPM = 4850; // The RPM when the motors run at 75% speed

  // TODO: delete
  private double topShooterkp;
  private double topShooterki = 0.0;
  private double topShooterkd = 0.0;
  private double bottomShooterkp = 0.0;
  private double bottomShooterki = 0.0;
  private double bottomShooterkd = 0.0;

  private double top_ff_kS = 0.0;
  private double top_ff_kV = 0.0;
  private double bottom_ff_kS = 0.0;
  private double bottom_ff_kV = 0.0;

  public Shooter(ShooterIO io) {
    System.out.println("[Init] Creating Shooter");
    this.io = io;

    topShooterPID = new PIDController(topShooterkp, topShooterki, topShooterkd);
    bottomShooterPID = new PIDController(bottomShooterkp, bottomShooterki, bottomShooterkd);

    topFeedForward = new SimpleMotorFeedforward(top_ff_kS, top_ff_kV);
    bottomFeedForward = new SimpleMotorFeedforward(bottom_ff_kS, bottom_ff_kV);

    topShooterPID.setSetpoint(setpointRPM);
    topShooterPID.disableContinuousInput();

    // Creates adjustable PID values and Setpoint on the Shuffleboard
    SmartDashboard.putNumber("topShooterkp", 0);
    SmartDashboard.putNumber("topShooterki", 0);
    SmartDashboard.putNumber("topShooterkd", 0);
    SmartDashboard.putNumber("bottomShooterkp", 0);
    SmartDashboard.putNumber("bottomShooterki", 0);
    SmartDashboard.putNumber("bottomShooterkd", 0);

    SmartDashboard.putNumber("topFF_KS", 0);
    SmartDashboard.putNumber("topFF_KV", 0);
    SmartDashboard.putNumber("bottomFF_KS", 0);
    SmartDashboard.putNumber("bottomFF_KV", 0);
    // SmartDashboard.putNumber("setpoint", 0);
  }

  @Override
  public void periodic() {
    this.updateInputs();
    Logger.processInputs("Shooter", inputs);

    // Updates the PID values if they are changed on the Shuffleboard
    if (topShooterkp != SmartDashboard.getNumber("topShooterkp", 0)
        || topShooterki != SmartDashboard.getNumber("topShooterki", 0)
        || topShooterkd != SmartDashboard.getNumber("topShooterkd", 0)
        || bottomShooterkp != SmartDashboard.getNumber("bottomShooterkp", 0)
        || bottomShooterki != SmartDashboard.getNumber("bottomShooterki", 0)
        || bottomShooterkd != SmartDashboard.getNumber("bottomShooterkd", 0)) {
      updatePIDController();
    }

    if (top_ff_kS != SmartDashboard.getNumber("topFF_kS", 0)
        || top_ff_kV != SmartDashboard.getNumber("topFF_kV", 0)
        || bottom_ff_kS != SmartDashboard.getNumber("bottom_ff_kS", 0)
        || bottom_ff_kV != SmartDashboard.getNumber("bottom_ff_kV", 0)) {
      updateFF();
    }

    // Updates the setpoint if it is changed on the Shuffleboard
    // if (setpointRPM != SmartDashboard.getNumber("setpoint", 0)) {
    //   updateSetpoint();
    // }

    // Sets the tolerence of the setpoint
    topShooterPID.setTolerance(
        200); // Allows the RPM of the motors can be within 5% RPM of the goal
    bottomShooterPID.setTolerance(
        setpointRPM * 0.05); // Allows the RPM of the motors can be within 5% RPM of the goal

    // Puts the desired RPM on the Shuffleboard
    SmartDashboard.putNumber("Setpoint Put Number", setpointRPM);

    // Puts the difference between the setpoint and current RPM on the Shuffleboard
    SmartDashboard.putNumber(
        "TopVelocityError",
        topShooterPID
            .getPositionError()); // The difference between the setpoint and RPM as calculated by
    // the PID controller
    SmartDashboard.putNumber(
        "BottomVelocityError",
        bottomShooterPID
            .getPositionError()); // The difference between the setpoint and RPM as calculated by
    // the PID controller

    // Sets the voltage of the Shooter Motors using the PID controller
    if (topShooterPID.calculate(getTopRPM(), setpointRPM) <= 0) {
      io.setTopShooterMotorVoltage(0);
      System.out.println("0: " + topFeedForward.calculate(setpointRPM));
    } else {
      io.setTopShooterMotorVoltage(
          topShooterPID.calculate(getTopRPM(), setpointRPM) + top_ff_kS + top_ff_kV * getTopRPM());
      // topFeedForward.calculate(setpointRPM)
      System.out.println("top shooter motor voltage" + top_ff_kS + top_ff_kV * getTopRPM());
    }

    if (bottomShooterPID.calculate(getBottomRPM(), setpointRPM) <= 0) {
      io.setBottomShooterMotorVoltage(0);
    } else {
      io.setTopShooterMotorVoltage(
          bottomShooterPID.calculate(getBottomRPM(), setpointRPM)
              + bottom_ff_kS
              + bottom_ff_kV * getBottomRPM());
    }

    // Returns whether or not the Motors have reached the setpoint
    SmartDashboard.putBoolean("TopAtSetpoint", topAtSetpoint());
    SmartDashboard.putBoolean("BottomAtSetpoint", bottomAtSetpoint());
  }

  /** Updates the PID values based on what they are set to on the Shuffleboard */
  public void updatePIDController() {
    topShooterkp = SmartDashboard.getNumber("topShooterkp", 0);
    topShooterki = SmartDashboard.getNumber("topShooterki", 0);
    topShooterkd = SmartDashboard.getNumber("topShooterkd", 0);
    bottomShooterkp = SmartDashboard.getNumber("bottomShooterkp", 0);
    bottomShooterki = SmartDashboard.getNumber("bottomShooterki", 0);
    bottomShooterkd = SmartDashboard.getNumber("bottomShooterkd", 0);
    topShooterPID = new PIDController(topShooterkp, topShooterki, topShooterkd);
    bottomShooterPID = new PIDController(bottomShooterkp, bottomShooterki, bottomShooterkd);
  }

  public void updateFF() {
    top_ff_kS = SmartDashboard.getNumber("top_ff_kS", 0);
    top_ff_kV = SmartDashboard.getNumber("top_ff_kV", 0);
    bottom_ff_kS = SmartDashboard.getNumber("bottom_ff_kS", 0);
    bottom_ff_kV = SmartDashboard.getNumber("bottom_ff_kV", 0);
    topFeedForward = new SimpleMotorFeedforward(top_ff_kS, top_ff_kV);
    bottomFeedForward = new SimpleMotorFeedforward(bottom_ff_kS, bottom_ff_kV);
  }

  public void updateInputs() {
    io.updateInputs(inputs);
  }

  public void setShooterMotorsVoltage(double volts) {
    io.setBothShooterMotorsVoltage(volts);
  }

  public void setShooterBreakMode(boolean enable) {
    io.setShooterBreakMode(enable);
  }

  public void setShooterMotorPercentSpeed(double percent) {
    io.setBothShooterMotorPercentSpeed(percent);
  }

  public double getTopRPM() {
    return Math.abs(inputs.topShooterMotorRPM);
  }

  public double getBottomRPM() {
    return Math.abs(inputs.bottomShooterMotorRPM);
  }

  /** Updates the setpoint based on what it is set to on the Shuffleboard */
  // public void updateSetpoint() {
  //   setpointRPM = SmartDashboard.getNumber("setpoint", 0);
  //   topShooterPID.setSetpoint(setpointRPM);
  // }

  public boolean topAtSetpoint() {
    return topShooterPID.atSetpoint();
  }

  public boolean bottomAtSetpoint() {
    return bottomShooterPID.atSetpoint();
  }
}
