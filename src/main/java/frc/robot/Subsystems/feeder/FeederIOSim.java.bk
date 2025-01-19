package frc.robot.Subsystems.feeder;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.RobotStateConstants;

public class FeederIOSim implements FeederIO {
  // Creating flywheels
  private final FlywheelSim feederFlywheelSim =
      new FlywheelSim(
          DCMotor.getFalcon500(1), FeederConstants.GEAR_RATIO, FeederConstants.MOI_KG_M2);

  public FeederIOSim() {
    System.out.println("[Init] Creating FeederIOSim");
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    // Updates the Feeder motors periodically
    feederFlywheelSim.update(RobotStateConstants.LOOP_PERIODIC_SEC);

    // Updates logged inputs of the simulated Feeder Flywheels
    inputs.feederRPM = feederFlywheelSim.getAngularVelocityRPM();
    inputs.feederAppliedVolts = 0.0;
    inputs.feederCurrentAmps = Math.abs(feederFlywheelSim.getCurrentDrawAmps());
  }

  @Override
  public void setPercentSpeed(double percent) {
    feederFlywheelSim.setInputVoltage(RobotStateConstants.BATTERY_VOLTAGE * percent);
  }

  @Override
  public void setMotorVoltage(double volts) {
    feederFlywheelSim.setInputVoltage(volts);
  }
}
