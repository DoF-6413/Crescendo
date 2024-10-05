package frc.robot.Subsystems.utbintake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.RobotStateConstants;

public class UTBIntakeIOSim implements UTBIntakeIO {
  /** creates a flywheel simulation for the UTB intake motor */
  private final FlywheelSim topUTBFlywheelSim;
  private final FlywheelSim bottomUTBFlywheelSim;

  public UTBIntakeIOSim() {
    System.out.println("[Init] Creating UTBIntakeIOSim");
    topUTBFlywheelSim =
        new FlywheelSim(
            DCMotor.getNEO(1), UTBIntakeConstants.GEAR_RATIO, UTBIntakeConstants.MOI_KG_M2);
    bottomUTBFlywheelSim =
        new FlywheelSim(
            DCMotor.getNEO(1), UTBIntakeConstants.GEAR_RATIO, UTBIntakeConstants.MOI_KG_M2);
  }

  @Override
  public void updateInputs(UTBIntakeIOInputs inputs) {
    topUTBFlywheelSim.update(RobotStateConstants.LOOP_PERIODIC_SEC);
    bottomUTBFlywheelSim.update(RobotStateConstants.LOOP_PERIODIC_SEC);

    inputs.topUTBIntakeRPM = topUTBFlywheelSim.getAngularVelocityRPM();
    inputs.topUTBIntakeCurrentAmps = Math.abs(topUTBFlywheelSim.getCurrentDrawAmps());

    inputs.bottomUTBIntakeRPM = bottomUTBFlywheelSim.getAngularVelocityRPM();
    inputs.bottomUTBIntakeCurrentAmps = Math.abs(bottomUTBFlywheelSim.getCurrentDrawAmps());
  }

  @Override
  public void setVoltage(double volts) {
    topUTBFlywheelSim.setInputVoltage(volts);
    bottomUTBFlywheelSim.setInputVoltage(volts);
  }

  @Override
  public void setPercentSpeed(double percent) {
    topUTBFlywheelSim.setInputVoltage(percent * RobotStateConstants.BATTERY_VOLTAGE);
    bottomUTBFlywheelSim.setInputVoltage(percent * RobotStateConstants.BATTERY_VOLTAGE);
  }
}
