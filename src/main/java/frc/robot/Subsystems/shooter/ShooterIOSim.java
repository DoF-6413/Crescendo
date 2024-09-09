package frc.robot.Subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.RobotStateConstants;

public class ShooterIOSim implements ShooterIO {
  // Creating flywheels
  private final FlywheelSim topShooterFlywheel =
      new FlywheelSim(
          DCMotor.getFalcon500(1), ShooterConstants.GEAR_RATIO, ShooterConstants.MOI_KG_M2);

  private final FlywheelSim bottomShooterFlywheel =
      new FlywheelSim(
          DCMotor.getFalcon500(1), ShooterConstants.GEAR_RATIO, ShooterConstants.MOI_KG_M2);

  public ShooterIOSim() {
    System.out.println("[Init] Creating ShooterIOSim");
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // Updates the Shooter motors periodically
    topShooterFlywheel.update(RobotStateConstants.LOOP_PERIODIC_SEC);
    bottomShooterFlywheel.update(RobotStateConstants.LOOP_PERIODIC_SEC);

    // Updates logged inputs of the simulated Shooter Flywheels
    inputs.topShooterMotorRPM = topShooterFlywheel.getAngularVelocityRPM();
    inputs.topShooterAppliedVolts = 0.0;
    inputs.topShooterCurrentAmps = Math.abs(topShooterFlywheel.getCurrentDrawAmps());

    inputs.bottomShooterMotorRPM = bottomShooterFlywheel.getAngularVelocityRPM();
    inputs.bottomShooterAppliedVolts = 0.0;
    inputs.bottomShooterCurrentAmps =
        Math.abs(bottomShooterFlywheel.getCurrentDrawAmps());
  }

  @Override
  public void setBothPercentSpeed(double percent) {
    topShooterFlywheel.setInputVoltage(RobotStateConstants.BATTERY_VOLTAGE * percent);
    bottomShooterFlywheel.setInputVoltage(RobotStateConstants.BATTERY_VOLTAGE * percent);
  }

  @Override
  public void setTopPercentSpeed(double percent) {
    topShooterFlywheel.setInputVoltage(RobotStateConstants.BATTERY_VOLTAGE * percent);
  }

  @Override
  public void setBottomPercentSpeed(double percent) {
    bottomShooterFlywheel.setInputVoltage(RobotStateConstants.BATTERY_VOLTAGE * percent);
  }

  @Override
  public void setBothVoltage(double volts) {
    topShooterFlywheel.setInputVoltage(volts);
    bottomShooterFlywheel.setInputVoltage(volts);
  }

  @Override
  public void setTopVoltage(double volts) {
    topShooterFlywheel.setInputVoltage(volts);
  }

  @Override
  public void setBottomVoltage(double volts) {
    bottomShooterFlywheel.setInputVoltage(volts);
  }
}
