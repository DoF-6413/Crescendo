package frc.robot.Subsystems.utbintake;

import javax.print.Doc;
import javax.print.DocPrintJob;

import com.fasterxml.jackson.core.json.DupDetector;

import edu.wpi.first.hal.simulation.DIODataJNI;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Subsystems.utbintake.*;
import frc.robot.Constants;
import frc.robot.Constants.UTBIntakeConstants;

public class UTBIntakeIOSim implements UTBIntakeIO {
    private FlywheelSim intakeMotorSim;
    private double utbIntakeAppliedVolts = 0.0;

    public UTBIntakeIOSim() {
        intakeMotorSim = new FlywheelSim(DCMotor.getNEO(1), Constants.UTBIntakeConstants.GEAR_RATIO,
            Constants.UTBIntakeConstants.UTB_MOMENT_OF_INERTIA_KGMETERSSQUARED);
    }

    @Override
    public void updateInputs(UTBIntakeIOInputs inputs) {
        inputs.UTBIntakepositionrad += intakeMotorSim.getAngularVelocityRadPerSec() * Constants.RobotStateConstants.LOOP_PERIODIC_SEC;
        inputs.utbIntakeVelocityRadPerSec = intakeMotorSim.getAngularVelocityRadPerSec();
        inputs.utbIntakeAppliedVolts = 0.0;
        inputs.utbIntakeCurrentAmps = new double[] {Math.abs(intakeMotorSim.getCurrentDrawAmps())};
    }

    @Override
    public void setUTBIntakeVoltage(double volts) {
        intakeMotorSim.setInputVoltage(volts);
    }
}
