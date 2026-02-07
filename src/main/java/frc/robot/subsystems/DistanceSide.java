// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Sensors.Distance;
import java.util.LinkedList;
import java.util.Queue;
import org.littletonrobotics.junction.Logger;

public class DistanceSide extends SubsystemBase {
  Queue<Double> queue = new LinkedList<>();
  int badNum = 0;
  private final int queueSize = 18;
  private final int maxBadMeasurements = 20;
  private TimeOfFlight sensor;
  /** Creates a new DistanceSide. */
  public DistanceSide() {
    sensor = new TimeOfFlight(Distance.climbSideId);
  }

  public double get() {
    double total = 0;
    int numZeroes = 0;
    for (double d : queue) {
      total += d;
      if (d == 0) {
        badNum++;
        numZeroes++;
      } else {
        badNum = 0;
      }
    }
    if (total == 0) {
      return -1;
    }
    return (total / (queue.size() - numZeroes)) / 1000;
  }

  public boolean isValid() {
    return badNum <= maxBadMeasurements;
  }

  @Override
  public void periodic() {
    queue.add(sensor.getRange());
    if (queue.size() > queueSize) {
      queue.remove();
    }
    Logger.recordOutput("distance/distance side", get());
    // This method will be called once per scheduler run
  }
}
