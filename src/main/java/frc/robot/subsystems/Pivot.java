// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Just simple constant controller
public class Pivot extends SubsystemBase {
  TalonFX motor;
  Encoder m_encoder;
  private double goal;
  private final double deadspace = 20;
  private final double upGoal = 0;
  private final double downGoal = -100;
  /** Creates a new Pivot. */
  public Pivot() {
    motor = new TalonFX(Motors.pivotId, canbus);
    m_encoder = new Encoder(Sensors.Ports.PivotEncoderPort1, Sensors.Ports.PivotEncoderPort2);
    m_encoder.setDistancePerPulse(1.0 / 360.0 * 2.0 * Math.PI * 1.5);
    resetEncoder();
  }

  private void updateMotorOutput() {
    double delta = getEncoder() - goal;
    if (Math.abs(delta) < deadspace) { // Deadspace
      return;
    }
    if (delta < 0) {
      setVoltage(MathUtil.clamp(delta, -1, -0.3));
    } else {
      setVoltage(MathUtil.clamp(delta, 0.3, 1));
    }
  }

  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  public double getEncoder() {
    return -m_encoder.getDistance();
  }

  private void setGoal(double goal) {
    this.goal = goal;
  }

  public double getGoal() {
    return this.goal;
  }

  public void resetEncoder() {
    m_encoder.reset();
  }

  public void goUp() {
    setGoal(upGoal);
  }

  public void goDown() {
    setGoal(downGoal);
  }

  public void goOpposite() {
    if (getGoal() == upGoal) {
      goDown();
    } else {
      goUp();
    }
  }

  @Override
  public void periodic() {
    updateMotorOutput();
    // This method will be called once per scheduler run
  }
}
