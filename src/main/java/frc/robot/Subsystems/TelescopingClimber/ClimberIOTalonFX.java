package frc.robot.Subsystems.TelescopingClimber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ClimberIOTalonFX implements ClimberIO{
    
  private final CANSparkMax climberMotor;
  private final RelativeEncoder climberEncoder;

  public ClimberIOTalonFX() {
    System.out.println("[Init] Creating ClimberIOTalonFX");

    climberMotor = new CANSparkMax(ClimberConstants.MOTOR_ID, MotorType.kBrushless);
    climberEncoder = climberMotor.getEncoder();
  }
}
