package frc.robot.Subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.ClimberConstants;

/** Climber Motor Control */
public class ClimberIOSparkMax implements ClimberIO {

    private CANSparkMax leftClimberMotor;
    private RelativeEncoder leftClimberEncoder;

    private CANSparkMax rightClimberMotor;
    private RelativeEncoder rightClimberEncoder;

    /** Creates the Motor and Encoder for the Climber */
    public ClimberIOSparkMax() {
        System.out.println("[Init] Creating ClimberIOSparkMax");

        leftClimberMotor = new CANSparkMax(ClimberConstants.LEFT_CLIMBER_CANID, MotorType.kBrushless);
        leftClimberEncoder = leftClimberMotor.getEncoder();

        rightClimberMotor = new CANSparkMax(ClimberConstants.RIGHT_CLIMBER_CANID, MotorType.kBrushless);
        rightClimberEncoder = rightClimberMotor.getEncoder();
    }

    /** Updates the printed values for the Climber */
    public void updateInputs(ClimberIOInputs inputs) {
        // Updates the Left Climber Motor inputs
        inputs.leftClimberPositionRad = Units.rotationsToRadians(leftClimberEncoder.getPosition()) / ClimberConstants.CLIMBER_GEAR_RATIO; // Converts rotaions to Radians and then divides it by the gear ratio
        inputs.leftClimberVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(leftClimberEncoder.getVelocity()) / ClimberConstants.CLIMBER_GEAR_RATIO; // Converts RPM to Radians per Second and then divides it by the gear ratio
        inputs.leftClimberAppliedVolts = leftClimberMotor.getAppliedOutput() * leftClimberMotor.getBusVoltage(); 
        inputs.leftClimberCurrentAmps = new double[] {leftClimberMotor.getOutputCurrent()};
        inputs.leftClimberTempCelcius = new double[] {leftClimberMotor.getMotorTemperature()};
        
        // Updates the Right Climber Motor inputs
        inputs.rightClimberPositionRad = Units.rotationsToRadians(rightClimberEncoder.getPosition()) / ClimberConstants.CLIMBER_GEAR_RATIO; // Converts rotaions to Radians and then divides it by the gear ratio
        inputs.rightClimberVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(rightClimberEncoder.getVelocity()) / ClimberConstants.CLIMBER_GEAR_RATIO; // Converts RPM to Radians per Second and then divides it by the gear ratio
        inputs.rightClimberAppliedVolts = rightClimberMotor.getBusVoltage();
        inputs.rightClimberCurrentAmps = new double[] {rightClimberMotor.getOutputCurrent()};
        inputs.rightClimberTempCelcius = new double[] {rightClimberMotor.getMotorTemperature()};
    }
}
