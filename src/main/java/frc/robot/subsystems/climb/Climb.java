// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.climb;

import static frc.robot.Constants.FeatureFlags.kDebugEnabled;
import static frc.robot.subsystems.climb.ClimbConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.drivers.MonitoredTalonFX;
import frc.robot.utils.PhoenixUtil;
import frc.robot.utils.TalonUtil;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import org.littletonrobotics.junction.AutoLogOutput;

/// Manages a pair of `TelescopicArm`s, one on each side of the robot.
/// Also needs a Pigeon2 gyro to measure the tilt of the robot
/// so that we can adjust the telescopic arms to keep the robot level.
public class Climb extends SubsystemBase implements Loggable {

  private final PositionVoltage m_requestLeftPostionVoltage =
      new PositionVoltage(0).withEnableFOC(false).withSlot(0);
  private final DynamicMotionMagicVoltage m_requestLeftMotionMagicVoltage =
      new DynamicMotionMagicVoltage(
              0,
              ClimbConstants.climbVelocity,
              ClimbConstants.climbAcceleration,
              ClimbConstants.climbJerk)
          .withEnableFOC(true)
          .withSlot(0);
  private final PositionVoltage m_requestRightPostionVoltage =
      new PositionVoltage(0).withEnableFOC(true).withSlot(0);
  private final DynamicMotionMagicVoltage m_requestRightMotionMagicVoltage =
      new DynamicMotionMagicVoltage(
              0,
              ClimbConstants.climbVelocity,
              ClimbConstants.climbAcceleration,
              ClimbConstants.climbJerk)
          .withEnableFOC(true)
          .withSlot(0);
  private MonitoredTalonFX leftMotor;

  public double leftUpDirection = 1;
  public double rightUpDirection = 1;

  @Log.ToString private Command currentCommand = getCurrentCommand();

  public void DEFAULT() {}

  public Climb() {
    if (RobotBase.isReal()) {
      configureRealHardware();
    } else {
      configureRealHardware();
    }
  }

  private void configureRealHardware() {
    leftMotor = new MonitoredTalonFX(ClimbConstants.kLeftClimbMotorID, "rio");
    leftMotor.setNeutralMode(NeutralModeValue.Brake);

    setMotorConfigs(kS, kV, kP, kI, kD, enableStatorLimit, statorLimit);
  }

  // @Config(name = "Climb PID")
  public void setMotorConfigs(
      double kS,
      double kV,
      double kP,
      double kI,
      double kD,
      boolean enableStatorLimit,
      double statorLimit) {
    var talonFXConfiguration = new TalonFXConfiguration();
    PhoenixUtil.checkErrorAndRetry(() -> leftMotor.getConfigurator().refresh(talonFXConfiguration));
    talonFXConfiguration.Slot0.kS = kS;
    talonFXConfiguration.Slot0.kV = kV;
    talonFXConfiguration.Slot0.kP = kP;
    talonFXConfiguration.Slot0.kI = kI;
    talonFXConfiguration.Slot0.kD = kD;
    talonFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = enableStatorLimit;
    talonFXConfiguration.CurrentLimits.StatorCurrentLimit = statorLimit;
    TalonUtil.applyAndCheckConfiguration(leftMotor, talonFXConfiguration);
  }

  @Config(name = "Set Left Climb")
  public void setLeftMotor(double position) {
    if (ClimbConstants.kUseClimbMotionMagic) {
      leftMotor.setControl(m_requestLeftMotionMagicVoltage.withPosition(position));
    } else {
      leftMotor.setControl(m_requestLeftPostionVoltage.withPosition(position));
    }
  }

  public void lock() {
    if (RobotBase.isReal()) {
      leftMotor.setControl(new StaticBrake());
    }
  }

  @Log.Graph(name = "Left Climb")
  @AutoLogOutput
  public double getLeftPosition() {
    return ((leftMotor.getRotorPosition().getValue()) * 360);
  }

  public void off() {
    leftMotor.setControl(new NeutralOut());
    if (kDebugEnabled) {
      System.out.println("Climb off");
    }
  }

  public void leftArmZero() {
    if (Constants.FeatureFlags.kDebugEnabled) {
      System.out.println("[LeftClimb] Setting zero position to: " + getLeftPosition());
    }
    leftMotor.setPosition(0);
  }

  @Config(name = "Set Left Climb Voltage")
  public void setLeftOutputVoltage(double voltage) {
    leftMotor.setVoltage(voltage);
    if (Constants.FeatureFlags.kDebugEnabled) {
      System.out.println("Left Climb voltage set to: " + voltage);
    }
  }

  @AutoLogOutput
  public double getLeftVelocity() {
    return leftMotor.getRotorVelocity().getValue();
  }

  @AutoLogOutput
  public double getLeftCurrent() {
    return Math.abs(leftMotor.getStatorCurrent().getValue());
  }

  @AutoLogOutput
  public boolean isLeftClimbSpiking() {
    return getLeftCurrent() > ClimbConstants.kCurrentThreshold;
  }

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

  public void flipLeftUpDirection() {
    leftUpDirection = -1 * leftUpDirection;
  }

  public void flipRightUpDirection() {
    rightUpDirection = -1 * rightUpDirection;
  }

  public double getLeftUpDirection() {
    return leftUpDirection;
  }

  public double getRightUpDirection() {
    return rightUpDirection;
  }

  @Override
  public void periodic() {}
}
