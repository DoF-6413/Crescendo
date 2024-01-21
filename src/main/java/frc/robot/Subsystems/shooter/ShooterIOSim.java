package frc.robot.Subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.*;

public class ShooterIOSim implements ShooterIO {
  // new flywheel simulation
  private FlywheelSim topShooterFlywheel =
      new FlywheelSim(
          DCMotor.getFalcon500(2),
          ShooterConstants.GEAR_RATIO,
          ShooterConstants.SHOOTER_J_KG_METERS_SQUARED);
  private FlywheelSim bottomShooterFlywheel =
      new FlywheelSim(
          DCMotor.getFalcon500(2),
          ShooterConstants.GEAR_RATIO,
          ShooterConstants.SHOOTER_J_KG_METERS_SQUARED);
  private double flywheelPositionRad = 0.0;

  public ShooterIOSim() {
    System.out.println("[Init] Creating ShooterIOSim");
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // update periodically
    topShooterFlywheel.update(RobotStateConstants.LOOP_PERIODIC_SEC);
    bottomShooterFlywheel.update(RobotStateConstants.LOOP_PERIODIC_SEC);

    // updates inputs
    inputs.topShooterMotorRPM = topShooterFlywheel.getAngularVelocityRPM();
    inputs.topShooterAppliedVolts = 0;
    inputs.topShooterCurrentAmps = new double[] {Math.abs(topShooterFlywheel.getCurrentDrawAmps())};
    inputs.topShooterTempCelcius = new double[] {};

    inputs.bottomShooterMotorRPM = bottomShooterFlywheel.getAngularVelocityRPM();
    inputs.bottomShooterAppliedVolts = 0;
    inputs.bottomShooterCurrentAmps =
        new double[] {Math.abs(bottomShooterFlywheel.getCurrentDrawAmps())};
    inputs.bottomShooterTempCelcius = new double[] {};
  }

  public void setTopShooterMotorPercentSpeed(double percent) {
    topShooterFlywheel.setInputVoltage(ShooterConstants.APPLIED_VOLTS * percent);
  }

  public void setBottomShooterMotorPercentSpeed(double percent) {
    bottomShooterFlywheel.setInputVoltage(ShooterConstants.APPLIED_VOLTS * percent);
  }
}
