// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSub extends SubsystemBase {
  /** Creates a new Intake. */
  private SparkMax controller;

  public IntakeSub() {
    controller = new SparkMax(Motors.intakeId, MotorType.kBrushed);
  }

  public void setVoltage(double volts) {
    controller.setVoltage(volts);
  }

  public double getSpeed() {
    return controller.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
