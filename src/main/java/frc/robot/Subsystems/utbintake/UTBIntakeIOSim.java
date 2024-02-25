package frc.robot.Subsystems.utbintake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.RobotStateConstants;

public class UTBIntakeIOSim implements UTBIntakeIO {
  /** creates a flywheel simulation for the UTB intake motor */
  private final FlywheelSim utbFlywheelSim;

  public UTBIntakeIOSim() {
    System.out.println("[Init] Creating UTBIntakeIOSim");
    utbFlywheelSim =
        new FlywheelSim(
            DCMotor.getNEO(1), UTBIntakeConstants.GEAR_RATIO, UTBIntakeConstants.MOI_KG_M2);
  }

  @Override
  public void updateInputs(UTBIntakeIOInputs inputs) {
    utbFlywheelSim.update(RobotStateConstants.LOOP_PERIODIC_SEC);
    inputs.utbIntakeRPM = utbFlywheelSim.getAngularVelocityRPM();
    inputs.utbIntakeAppliedVolts = 0.0;
    inputs.utbIntakeCurrentAmps = new double[] {Math.abs(utbFlywheelSim.getCurrentDrawAmps())};
  }

  @Override
  public void setUTBIntakeVoltage(double volts) {
    utbFlywheelSim.setInputVoltage(volts);
  }

  @Override
  public void setUTBIntakePercentSpeed(double percent) {
    utbFlywheelSim.setInputVoltage(percent * RobotStateConstants.BATTERY_VOLTAGE);
  }

  @Override
  public void enableUTB(boolean auxYIsPressed) {
    if (auxYIsPressed == true) {
      setUTBIntakePercentSpeed(100);
    } else {
      setUTBIntakePercentSpeed(100);
    }
  }
}
