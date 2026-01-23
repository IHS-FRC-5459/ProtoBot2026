// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSub extends SubsystemBase {
  TalonFX motor;
  /** Creates a new Climb. */
  public ClimbSub() {
    motor = new TalonFX(Motors.climbId, canbus);
  }

  public void setPower(double power) {
    motor.set(-power);
  }

  public double getPower() {
    return -motor.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
