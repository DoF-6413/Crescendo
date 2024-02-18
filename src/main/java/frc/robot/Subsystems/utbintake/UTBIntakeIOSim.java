package frc.robot.Subsystems.utbintake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.RobotStateConstants;

public class UTBIntakeIOSim implements UTBIntakeIO {
  private FlywheelSim utbFlywheelSim =
      new FlywheelSim(
          DCMotor.getNEO(1), UTBIntakeConstants.GEAR_RATIO, UTBIntakeConstants.UTB_MOI_KG_M2);

  public UTBIntakeIOSim() {
    System.out.println("[Init] Creating UTBIntakeIOSim");
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
}
