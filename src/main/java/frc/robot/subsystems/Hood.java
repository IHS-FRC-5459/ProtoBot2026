// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Motors;

public class Hood extends SubsystemBase {
  private SparkMax motor;
  private double goal;

  public Hood() {
    motor = new SparkMax(Motors.hoodId, MotorType.kBrushless);
  }

  public void setGoal(double goal) {
    this.goal = goal;
  }

  public double getGoal() {
    return this.goal;
  }

  public void setVoltage(double volts) {
    motor.setVoltage(-volts);
  }

  public double getVoltage() {
    return -motor.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
