// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.LEDs;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class LEDs extends SubsystemBase {

  public final PWM pwm;
  public final double yellow = 0.69;
  public final double violet = 0.91;
  public final double blue = 0.87; // blue allience
  public final double whithe = 0.93; // auto
  public final double green = 0.77;
  public final double red = 0.61; // red allience
  public final double blinkRed = -0.1; // >>>30sec
  public final double LEDsoff = 0;

  public LEDs() {
    pwm = new PWM(1);
  }

  public void setLEDs(double color) {
    pwm.setSpeed(color);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (DriverStation.isAutonomous()) {
      setLEDs(whithe);
    } else {
      if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
        if (DriverStation.getMatchTime() > 5) {
          setLEDs(blue);
        } else {
          setLEDs(blinkRed);
        }
      } else {
        if (DriverStation.getMatchTime() > 5) {
          setLEDs(red);
        } else {
          setLEDs(blinkRed);
        }
      }
    }
  }
}
