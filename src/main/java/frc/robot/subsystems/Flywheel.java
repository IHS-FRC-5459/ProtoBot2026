// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheel extends SubsystemBase {
  TalonFX motor;
  private double goal;
  /** Creates a new Flywheel. */
  public Flywheel() {
    motor = new TalonFX(Motors.flywheelId, canbus);
  }

  public void setGoal(double goal) {
    this.goal = goal;
  }

  public double getGoal() {
    return this.goal;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
