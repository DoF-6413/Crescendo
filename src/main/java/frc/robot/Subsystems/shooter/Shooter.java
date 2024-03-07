// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.shooter;

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
  }

  @Override
  public void periodic() {
    this.updateInputs();
    Logger.processInputs("Shooter", inputs);

    // Sets the voltage of the Shooter Motors using PID
    setTopShooterMotorVoltage(
        topShooterPIDController.calculateForVoltage(
            inputs.topShooterMotorRPM, ShooterConstants.MAX_RPM));
    setBottomShooterMotorVoltage(
        bottomShooterPIDController.calculateForVoltage(
            inputs.bottomShooterMotorRPM, ShooterConstants.MAX_RPM));

    SmartDashboard.putNumber("ShooterTopSetpoint", topShooterPIDController.getSetpoint());
    SmartDashboard.putNumber("ShooterBottomSetpoint", bottomShooterPIDController.getSetpoint());

    // SmartDashboard.putBoolean("!!Tempature Warning!!", exceedsTemperature());

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

  public boolean allAtSetpoint() {
    return bottomAtSetpoint() && topAtSetpoint();
  }

  public void enableShooter(boolean enable) {
    if (enable) {
      topShooterPIDController.setSetpoint(setpointRPM);
      bottomShooterPIDController.setSetpoint(setpointRPM);
    } else {
      topShooterPIDController.setSetpoint(0);
      bottomShooterPIDController.setSetpoint(0);
      setTopShooterMotorVoltage(0);
      setBottomShooterMotorVoltage(0);
    }
  }

  public double returnDesiredAngle(double x) {
    double closestX, closestTheta;
    if (ShooterConstants.LOOKUP_TABLE_X_M_VS_THETA_DEG[0][5] > x) {
      int i = 1;
      closestX = ShooterConstants.LOOKUP_TABLE_X_M_VS_THETA_DEG[0][i];
      closestTheta = ShooterConstants.LOOKUP_TABLE_X_M_VS_THETA_DEG[1][i];

      // do closest theta is 0 if you are 5 m away of the speaker(you cant
      // shoot)//TODO:when we change the max change the 5 to the new max

      // since the table is sorted, find the index of the first value where the distance value
      // exceeds
      while (ShooterConstants.LOOKUP_TABLE_X_M_VS_THETA_DEG[0][i] < x) {
        i++;
      }

      // finds which x value in the table the actual distance is closer to and return that
      if (Math.abs(ShooterConstants.LOOKUP_TABLE_X_M_VS_THETA_DEG[0][i - 1] - x)
          < Math.abs(ShooterConstants.LOOKUP_TABLE_X_M_VS_THETA_DEG[0][i] - x)) {
        closestX = ShooterConstants.LOOKUP_TABLE_X_M_VS_THETA_DEG[0][i - 1];
        closestTheta =
            (ShooterConstants.LOOKUP_TABLE_X_M_VS_THETA_DEG[1][i - 1]
                    + ShooterConstants.LOOKUP_TABLE_X_M_VS_THETA_DEG[2][i - 1])
                / 2;
      } else {
        closestX = ShooterConstants.LOOKUP_TABLE_X_M_VS_THETA_DEG[0][i];
        closestTheta =
            (ShooterConstants.LOOKUP_TABLE_X_M_VS_THETA_DEG[1][i]
                    + ShooterConstants.LOOKUP_TABLE_X_M_VS_THETA_DEG[2][i])
                / 2;
      }

      // returns the closest Theta based on the lookup table
      return closestTheta;
    } else {
      closestTheta = 0;
      return closestTheta;
    } 
  }
  public void disableShooter() {
    topShooterPIDController.setSetpoint(0);
    bottomShooterPIDController.setSetpoint(0);
  }

  public void setSetpoint(double setpoint) {
    topShooterPIDController.setSetpoint(setpoint);
    bottomShooterPIDController.setSetpoint(setpoint);
    topShooterPIDController.setTolerance(setpointRPM * ShooterConstants.TOLERANCE_PERCENT);
    bottomShooterPIDController.setTolerance(setpointRPM * ShooterConstants.TOLERANCE_PERCENT);
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
