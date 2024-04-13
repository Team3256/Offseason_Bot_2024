// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.climb;

import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;
import frc.robot.utils.SinglePositionSubsystem;

public class ClimbTest extends SinglePositionSubsystem {
  public ClimbTest() {
    super(ClimbConstants.kUseClimbMotionMagic, ClimbConstants.kCurrentThreshold, 0);
    super.configureRealHardware(
        ClimbConstants.kLeftClimbMotorID,
        NeutralModeValue.Brake,
        ClimbConstants.kS,
        ClimbConstants.kV,
        ClimbConstants.kP,
        ClimbConstants.kI,
        ClimbConstants.kD,
        ClimbConstants.motionMagicVelocity,
        ClimbConstants.motionMagicAcceleration,
        ClimbConstants.motionMagicJerk,
        ClimbConstants.enableStatorLimit,
        ClimbConstants.statorLimit);
  }

  public void setLeftMotor(double position) {
    super.setDegrees(position);
  }

  public void lock() {
    super.setControlStaticBrake();
  }

  public double getLeftPosition() {
    return super.getDegrees();
  }

  public void off() {
    super.off();
    if (Constants.FeatureFlags.kDebugEnabled) {
      System.out.println("Climb off");
    }
  }

  public void leftArmZero() {
    if (Constants.FeatureFlags.kDebugEnabled) {
      System.out.println("[LeftClimb] Setting zero position to: " + getLeftPosition());
    }
    super.zero();
  }

  public void setLeftOutputVoltage(double voltage) {
    super.setOutputVoltage(voltage);
    if (Constants.FeatureFlags.kDebugEnabled) {
      System.out.println("Left Climb voltage set to: " + voltage);
    }
  }

  public double getLeftVelocity() {
    return super.getVelocity();
  }

  public double getLeftCurrent() {
    return super.getCurrent();
  }

  public boolean isLeftClimbSpiking() {
    return super.isCurrentSpiking();

    // public void up() {
    // // Sets both arms to max height (4ft as per Crescendo rules)
    // leftArm.release();
    // rightArm.release();
    // }
    //
    // public void down() {
    // // Sets both arms to min height
    // leftArm.setPosition(0);
    // rightArm.setPosition(0);
    // }
    //
    // public void setLeftPosition(double meters) {
    // leftArm.setPosition(convertMetersToRotations(meters));
    // }
    //
    // public void setRightPosition(double meters) {
    // rightArm.setPosition(convertMetersToRotations(meters));
    // }

  }
}
