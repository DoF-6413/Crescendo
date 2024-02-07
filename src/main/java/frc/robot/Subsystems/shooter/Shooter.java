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

  // Creates the PID Contollers for both Shooter Motors
  private static PIDController topShooterPID;
  private static PIDController bottomShooterPID;

  // The desired RPM for the Shooter Motors
  private double setpointRPM = 1000.0; // The RPM when the motors run at 75% speed is 4850

  // TODO: delete once the proper PID values have been determined along with the smartdashboard put
  // numbers
  private double topShooterkp = 1.0;
  private double topShooterki = 0.0;
  private double topShooterkd = 0.0;
  private double bottomShooterkp = 1.5;
  private double bottomShooterki = 0.0;
  private double bottomShooterkd = 0.0;

  public Shooter(ShooterIO io) {

    System.out.println("[Init] Creating Shooter");
    this.io = io;

    topShooterPID = new PIDController(topShooterkp, topShooterki, topShooterkd);
    bottomShooterPID = new PIDController(bottomShooterkp, bottomShooterki, bottomShooterkd);

    topShooterPID.setSetpoint(setpointRPM);
    bottomShooterPID.setSetpoint(setpointRPM);

    // Sets the tolerence of the setpoint, allowing the RPM of the motors to be within 200 RPM of
    // the setpoint
    // topShooterPID.setTolerance(200);
    // bottomShooterPID.setTolerance(200);

    // Creates adjustable PID values and Setpoint on the Shuffleboard
    // SmartDashboard.putNumber("topShooterkp", 0.0);
    // SmartDashboard.putNumber("topShooterki", 0.0);
    // SmartDashboard.putNumber("topShooterkd", 0.0);
    // SmartDashboard.putNumber("bottomShooterkp", 0.0);
    // SmartDashboard.putNumber("bottomShooterki", 0.0);
    // SmartDashboard.putNumber("bottomShooterkd", 0.0);
    // SmartDashboard.putNumber("setpoint", 0.0);
  }

  @Override
  public void periodic() {
    this.updateInputs();
    Logger.processInputs("Shooter", inputs);

    // Updates the PID values if they are changed on the Shuffleboard
    // if (topShooterkp != SmartDashboard.getNumber("topShooterkp", 0.0)
    //     || topShooterki != SmartDashboard.getNumber("topShooterki", 0.0)
    //     || topShooterkd != SmartDashboard.getNumber("topShooterkd", 0.0)
    //     || bottomShooterkp != SmartDashboard.getNumber("bottomShooterkp", 0.0)
    //     || bottomShooterki != SmartDashboard.getNumber("bottomShooterki", 0.0)
    //     || bottomShooterkd != SmartDashboard.getNumber("bottomShooterkd", 0.0)) {
    //   updatePIDController();
    // }

    // Updates the setpoint if it is changed on the Shuffleboard
    // if (setpointRPM != SmartDashboard.getNumber("setpoint", 0.0)) {
    //   updateSetpoint();
    //   System.out.println("setpoint: " + setpointRPM);
    // }

    // Puts the difference between the setpoint and current RPM on the Shuffleboard as calculated by
    // the WPI PID Controller
    SmartDashboard.putNumber("TopError", setpointRPM - inputs.topShooterMotorRPM);
    SmartDashboard.putNumber("BottomError", setpointRPM - Math.abs(inputs.bottomShooterMotorRPM));
    SmartDashboard.putNumber(
        "MotorRPMDifference", inputs.topShooterMotorRPM - Math.abs(inputs.bottomShooterMotorRPM));

    // Sets the desired RPM that the motors should run at
    // topShooterPID.setSetpoint(setpointRPM);
    // bottomShooterPID.setSetpoint(setpointRPM);
    // Sets the voltage of the Shooter Motors using PID
    if (inputs.topShooterMotorRPM < 0.0) {
      setTopShooterMotorVoltage(0.0);
    } else {
      setTopShooterMotorVoltage(
          // TODO: fix through pid
          topShooterPID.calculateForVoltage(
              inputs.topShooterMotorRPM,
              6350)); // Max Value: 2-6-24 with a 12.3V battery, max RPM was ~6350
      // System.out.println(
      //     "top pid: " + topShooterPID.calculateForVoltage(inputs.topShooterMotorRPM, 6350));
    }

    if (inputs.bottomShooterMotorRPM > 0.0) {
      setBottomShooterMotorVoltage(0.0);
    } else {
      setBottomShooterMotorVoltage(
          -bottomShooterPID.calculateForVoltage(inputs.bottomShooterMotorRPM, 6350));
      // System.out.println(
      //     "bottom shooter pid: "
      //         + bottomShooterPID.calculateForVoltage(inputs.bottomShooterMotorRPM, 6350));
      // // ((setpointRPM * 12) / 6800)
      //     + (bottomShooterkp * (setpointRPM - inputs.bottomShooterMotorRPM)));
      //   setTopShooterMotorVoltage(
      //       (setpointRPM * 12) / 6800 + topShooterkp * (setpointRPM -
      // inputs.topShooterMotorRPM));
    }

    // Returns whether or not the Motors have reached the setpoint
    // SmartDashboard.putBoolean("TopAtSetpoint", topAtSetpoint());
    // SmartDashboard.putBoolean("BottomAtSetpoint", bottomAtSetpoint());

    // Gets the current PID values that the PID contoller is set to
    SmartDashboard.putNumber("topCurrentkP", topShooterPID.getP());
    SmartDashboard.putNumber("topCurrentkI", topShooterPID.getI());
    SmartDashboard.putNumber("topCurrentkD", topShooterPID.getD());
    SmartDashboard.putNumber("bottomCurrentkP", bottomShooterPID.getP());
    SmartDashboard.putNumber("bottomCurrentkI", bottomShooterPID.getI());
    SmartDashboard.putNumber("bottomCurrentkD", bottomShooterPID.getD());

    // Gets the current setpoint that the PID contoller is set to
    SmartDashboard.putNumber("Top PID Controller Setpoint", topShooterPID.getSetpoint());
    SmartDashboard.putNumber("Bottom PID Controller Setpoint", -bottomShooterPID.getSetpoint());
  }

  /** Updates the PID values based on what they are set to on the Shuffleboard */
  // public void updatePIDController() {
  //   topShooterkp = SmartDashboard.getNumber("topShooterkp", 0.0);
  //   topShooterki = SmartDashboard.getNumber("topShooterki", 0.0);
  //   topShooterkd = SmartDashboard.getNumber("topShooterkd", 0.0);
  //   bottomShooterkp = SmartDashboard.getNumber("bottomShooterkp", 0.0);
  //   bottomShooterki = SmartDashboard.getNumber("bottomShooterki", 0.0);
  //   bottomShooterkd = SmartDashboard.getNumber("bottomShooterkd", 0.0);
  //   topShooterPID.setPID(topShooterkp, topShooterki, topShooterkd);
  //   bottomShooterPID.setPID(bottomShooterkp, bottomShooterki, bottomShooterkd);
  // }

  /** Updates the setpoint based on what it is set to on the Shuffleboard */
  // public void updateSetpoint() {
  //   setpointRPM = SmartDashboard.getNumber("setpoint", 0.0);
  //   topShooterPID.setSetpoint(setpointRPM);
  //   bottomShooterPID.setSetpoint(setpointRPM);
  // }

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

  // public boolean topAtSetpoint() {
  //   return topShooterPID.atSetpoint();
  // }

  // public boolean bottomAtSetpoint() {
  //   return bottomShooterPID.atSetpoint();
  // }
}
