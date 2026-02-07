// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSiD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Colors;

public class LED extends SubsystemBase {
  private final int framerate = 20;
  private final int rainbowFramerate = 100;
  private final int startIndex = 0;
  private final int endIndex = 100;
  // status booleans
  private boolean intaking,
      shooting,
      elevatorGoingUp,
      elevatorGoingDown,
      passing,
      canShoot,
      isAngry,
      isHappy,
      isRainbow;
  private CANdle candle;

  public LED(CANdle candle) {
    this.candle = candle;
  }

  // sets the colors of an LED
  private void setLEDs(RGBWColor color) {
    SolidColor solidColor = new SolidColor(startIndex, endIndex);
    solidColor = solidColor.withColor(color);
    candle.setControl(solidColor);
  }

  private void setLEDs(RGBWColor color, double frameRate) {
    SingleFadeAnimation animation = new SingleFadeAnimation(startIndex, endIndex);
    animation = animation.withColor(color);
    animation = animation.withFrameRate(frameRate);
    candle.setControl(animation);
  }

  private void doRainbow() {
    RainbowAnimation animation = new RainbowAnimation(startIndex, endIndex);
    animation.withFrameRate(rainbowFramerate);
    candle.setControl(animation);
  }

  public void setIntaking(boolean val) {
    intaking = val;
  }

  public void setShooting(boolean val) {
    shooting = val;
  }

  public void setElevatorGoingUp(boolean val) {
    elevatorGoingUp = val;
  }

  public void setElevatorGoingDown(boolean val) {
    elevatorGoingDown = val;
  }

  public void setPassing(boolean val) {
    passing = val;
  }

  public void setCanShoot(boolean val) {
    canShoot = val;
  }

  public void setIsAngry(boolean val) {
    isAngry = val;
  }

  public void setIsHappy(boolean val) {
    isHappy = val;
  }

  public void setIsRainbow(boolean val) {
    isRainbow = val;
  }

  private void updateLEDs() {
    // This is just an exaple cascenario
    if (shooting) {
      setLEDs(Colors.green, framerate);
    } else if (elevatorGoingUp) {
      setLEDs(Colors.purple, framerate);
    } else if (elevatorGoingDown) {
      setLEDs(Colors.red, framerate);
    } else if (passing) {
      setLEDs(Colors.blue, framerate);
    } else if (intaking) {
      setLEDs(Colors.yellow, framerate);
    } else if (canShoot) {
      setLEDs(Colors.green);
    } else if (isAngry) {
      setLEDs(Colors.red);
    } else if (isHappy) {
      setLEDs(Colors.yellow);
    } else if (isRainbow) {
      doRainbow();
    } else {
      setLEDs(Colors.blank);
    }
  }

  // @Override
  public void periodic() {
    updateLEDs();
    // int[] color = Constants.Colors.red;
    // setLED(color);
  }
}
