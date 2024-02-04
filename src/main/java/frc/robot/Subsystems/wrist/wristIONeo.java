// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//2 motors
//neo


package frc.robot.Subsystems.wrist;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.wristNeoConstants;

/** Add your docs here. */
public class wristIONeo implements WristIO{

    //first wrist motor

    private final CANSparkMax firstWristMotor;
    private final RelativeEncoder firstWristEncoder;

    //second wrist motor(the closest to the shooter)

    private final CANSparkMax secondWristMotor;
    private final RelativeEncoder secondWristEncoder;


    public wristIONeo(){
        firstWristMotor = new CANSparkMax(0,MotorType.kBrushless);
        firstWristEncoder = firstWristMotor.getEncoder();

        secondWristMotor = new CANSparkMax(1, MotorType.kBrushless);
        secondWristEncoder = secondWristMotor.getEncoder();

        firstWristMotor.setIdleMode(IdleMode.kBrake);
        firstWristMotor.setSmartCurrentLimit(21321);//TODO:update the limit 
       
        secondWristMotor.setIdleMode(IdleMode.kBrake);
        secondWristMotor.setSmartCurrentLimit(123213);//TODO:update the limit 
    

    }
        public void setFirstWristMotorSpeed(double speed) {
            firstWristMotor.setVoltage(speed);
        }
        public void setSecondWristMotorSpeed(double speed) {
            secondWristMotor.setVoltage(speed);
        }
        public void updateInputs(WristIOInputs inputs){
           //first wrist motor
          
            inputs.firstWristTurnAppliedVolts = firstWristMotor.getBusVoltage();
            inputs.firstWristTurnPositionRad = Units.rotationsToRadians(firstWristEncoder.getPosition()) / wristNeoConstants.FIRST_MOTOR_GEAR_RATIO;//TODO: update gear ratio
            inputs.firstWristTempCelcius = firstWristMotor.getMotorTemperature();
            inputs.firstWristTurnVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(firstWristEncoder.getVelocity());
            inputs.firstWristTurnCurrentAmps = firstWristMotor.getOutputCurrent();

    //second wrist motor(the closest to the shooter)

            inputs.secondWristTurnAppliedVolts = secondWristMotor.getBusVoltage();
            inputs.secondWristTurnPositionRad = Units.rotationsToRadians(secondWristEncoder.getPosition()) / wristNeoConstants.SECOND_MOTOR_GEAR_RATIO;//TODO: update gear ratio
            inputs.secondWristTempCelcius = secondWristMotor.getMotorTemperature();
            inputs.secondWristTurnVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(secondWristEncoder.getVelocity());
            inputs.secondWristTurnCurrentAmps = secondWristMotor.getOutputCurrent();

        }
        }
