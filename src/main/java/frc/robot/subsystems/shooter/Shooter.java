// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter;

import static frc.robot.Constants.FeatureFlags.kDebugEnabled;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.drivers.MonitoredTalonFX;
import frc.robot.utils.PhoenixUtil;
import frc.robot.utils.TalonUtil;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import org.littletonrobotics.junction.AutoLogOutput;

public class Shooter extends SubsystemBase implements Loggable {
  // @Log.MotorController(name = "Shooter Motor")
  private MonitoredTalonFX shooterMotor;
  private MonitoredTalonFX shooterMotorFollower;

  private TalonFXSimState shooterMotorSim;
  private FlywheelSim flywheelSimModel;

  final VelocityVoltage velocityShooterRequest =
      new VelocityVoltage(0)
          .withEnableFOC(ShooterConstants.kUseFOC)
          .withSlot(0)
          .withFeedForward(
              0); // Set FF to 0 here - makes sure the shooter doesn't spin up when we don't want
  // it to
  // final Follower velocityShooterFollowerRequest =
  // new Follower(ShooterConstants.kShooterMotorID, false);
  final VelocityVoltage velocityShooterFollowerRequest =
      new VelocityVoltage(0).withEnableFOC(ShooterConstants.kUseFOC).withSlot(0);

  final VelocityTorqueCurrentFOC velocityShooterRequestFOC =
      new VelocityTorqueCurrentFOC(0).withSlot(0);
  final VelocityTorqueCurrentFOC velocityShooterFollowerRequestFOC =
      new VelocityTorqueCurrentFOC(0).withSlot(0);

  final MotionMagicVelocityVoltage motionMagicShooterRequest =
      new MotionMagicVelocityVoltage(0).withEnableFOC(ShooterConstants.kUseFOC).withSlot(0);
  final MotionMagicVelocityVoltage motionMagicShooterFollowerRequest =
      new MotionMagicVelocityVoltage(0).withEnableFOC(ShooterConstants.kUseFOC).withSlot(0);
  @Log.ToString private Command currentCommand = getCurrentCommand();

  public void DEFAULT() {}

  public Shooter() {
    if (RobotBase.isReal()) {
      configureRealHardware();
    } else {
      configureSimHardware();
    }
    System.out.println("Shooter initialized");
  }

  private void configureRealHardware() {
    shooterMotor = new MonitoredTalonFX(ShooterConstants.kShooterMotorID);
    shooterMotorFollower = new MonitoredTalonFX(ShooterConstants.kShooterMotorFollowerID);

    setShootConfigs(
        ShooterConstants.kShooterKS,
        ShooterConstants.kShooterKV,
        ShooterConstants.kShooterKP,
        ShooterConstants.kShooterKI,
        ShooterConstants.kShooterKD,
        InvertedValue.Clockwise_Positive,
        NeutralModeValue.Coast);
    setShootConfigs(
        ShooterConstants.kShooterKS,
        ShooterConstants.kShooterKV,
        ShooterConstants.kShooterKP,
        ShooterConstants.kShooterKI,
        ShooterConstants.kShooterKD,
        InvertedValue.CounterClockwise_Positive,
        NeutralModeValue.Coast);
  }

  // @Config(name = "Shoot PID values")
  private void setShootConfigs(
      double kS,
      double kV,
      double kP,
      double kI,
      double kD,
      InvertedValue inverted,
      NeutralModeValue neutral) {
    var motorConfig = new TalonFXConfiguration();
    PhoenixUtil.checkErrorAndRetry(() -> shooterMotor.getConfigurator().refresh(motorConfig));
    motorConfig.Slot0.kS = kS; // Add 0.05 V output to overcome static friction
    motorConfig.Slot0.kV = kV; // A velocity target of 1 rps results in 0.12 V output
    motorConfig.Slot0.kP = kP; // An error of 1 rps results in 0.11 V output
    motorConfig.Slot0.kI = kI; // no output for integrated error
    motorConfig.Slot0.kD = kD;
    motorConfig.MotorOutput.Inverted = inverted;
    motorConfig.MotorOutput.NeutralMode = neutral;
    motorConfig.CurrentLimits.StatorCurrentLimit = 60;

    TalonUtil.applyAndCheckConfiguration(shooterMotor, motorConfig);
  }

  private void setShootFollowerConfigs(
      double kS,
      double kV,
      double kP,
      double kI,
      double kD,
      InvertedValue inverted,
      NeutralModeValue neutral) {
    var motorConfig = new TalonFXConfiguration();
    PhoenixUtil.checkErrorAndRetry(() -> shooterMotor.getConfigurator().refresh(motorConfig));
    motorConfig.Slot0.kS = kS; // Add 0.05 V output to overcome static friction
    motorConfig.Slot0.kV = kV; // A velocity target of 1 rps results in 0.12 V output
    motorConfig.Slot0.kP = kP; // An error of 1 rps results in 0.11 V output
    motorConfig.Slot0.kI = kI; // no output for integrated error
    motorConfig.Slot0.kD = kD;
    motorConfig.MotorOutput.Inverted = inverted;
    motorConfig.MotorOutput.NeutralMode = neutral;
    motorConfig.CurrentLimits.StatorCurrentLimit = 60;

    TalonUtil.applyAndCheckConfiguration(shooterMotorFollower, motorConfig);
  }

  private void setShootConfigs(double kS, double kV, double kP, double kI, double kD) {
    var motorConfig = new TalonFXConfiguration();
    PhoenixUtil.checkErrorAndRetry(() -> shooterMotor.getConfigurator().refresh(motorConfig));
    motorConfig.Slot0.kS = kS; // Add 0.05 V output to overcome static friction
    motorConfig.Slot0.kV = kV; // A velocity target of 1 rps results in 0.12 V output
    motorConfig.Slot0.kP = kP; // An error of 1 rps results in 0.11 V output
    motorConfig.Slot0.kI = kI; // no output for integrated error
    motorConfig.Slot0.kD = kD;
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    TalonUtil.applyAndCheckConfiguration(shooterMotor, motorConfig);
  }

  // @Config(name = "Feed PID values")

  private void configureSimHardware() {
    configureRealHardware();
    shooterMotorSim = shooterMotor.getSimState();
    flywheelSimModel = new FlywheelSim(DCMotor.getFalcon500(1), 1, 0.1207277);
  }

  /* setters */
  @Config(name = "Set Shooter Voltage", defaultValueNumeric = 0.0)
  public void setInputShooterVoltage(double voltage) {
    shooterMotor.setVoltage(voltage);
    if (kDebugEnabled) {
      System.out.println("Shooter voltage set to: " + voltage);
    }
  }

  @Config(name = "Set Shooter Follower Voltage", defaultValueNumeric = 0.0)
  public void setInputShooterFollowerVoltage(double voltage) {
    shooterMotorFollower.setVoltage(voltage);
    if (kDebugEnabled) {
      System.out.println("Shooter follower voltage set to: " + voltage);
    }
  }

  @Config(name = "Set Shooter Rps", defaultValueNumeric = 0.0)
  public void setShooterRps(double shooterRps) {
    if (kDebugEnabled) {
      System.out.println("Shooter velocity set to: " + shooterRps);
    }
    if (ShooterConstants.kUseShooterMotionMagic) {
      shooterMotor.setControl(motionMagicShooterRequest.withVelocity(shooterRps));

    } else {
      shooterMotor.setControl(
          velocityShooterRequest.withVelocity(shooterRps).withAcceleration(100));
    }
  }

  @Config(name = "Set Shooter Follower Rps", defaultValueNumeric = 0.0)
  public void setShooterFollowerRps(double shooterRps) {
    if (kDebugEnabled) {
      System.out.println("Shooter velocity set to: " + shooterRps);
    }
    if (ShooterConstants.kUseShooterMotionMagic) {
      shooterMotorFollower.setControl(motionMagicShooterFollowerRequest.withVelocity(shooterRps));

    } else {
      shooterMotorFollower.setControl(velocityShooterFollowerRequest.withVelocity(shooterRps));
    }
  }

  public void off() {
    shooterMotor.setControl(new NeutralOut());
    shooterMotorFollower.setControl(new NeutralOut());
    if (kDebugEnabled) {
      System.out.println("Shooter off");
    }
  }

  @Log.BooleanBox(name = "Shooter Off")
  @AutoLogOutput
  public boolean getOff() {
    return shooterMotor.getControlMode().getValue() == ControlModeValue.NeutralOut;
  }

  // getters
  @Log.Graph(name = "Shooter Motor Rps")
  @AutoLogOutput
  public double getShooterRps() {
    return shooterMotor.getVelocity().getValue();
  }

  @Log.Graph(name = "Shoot Motor Current")
  @AutoLogOutput
  public double getShooterCurrent() {
    return shooterMotor.getSupplyCurrent().getValue();
  }

  @AutoLogOutput
  public double getShooterFollowerCurrent() {
    return shooterMotorFollower.getSupplyCurrent().getValue();
  }

  // @Log.Graph(name = "Shooter Motor Voltage")
  @AutoLogOutput
  public double getShooterMotorVoltage() {
    return shooterMotor.getMotorVoltage().getValue();
  }

  // @Log.Graph(name = "Shooter Motor Stator Voltage")
  // @AutoLogOutput
  // public double getShooterStatorVoltage() {
  // return shooterMotor.getStatorCurrent().getValue();
  // }

  // @Log.Graph(name = "Feeder Motor Rps"

  /* workers */
  @Override
  public void periodic() {
    currentCommand = getCurrentCommand();
  }

  // @Log.Graph(name = "Feeder Stator Voltage")

  // @Log.Graph(name = "Follower Motor Rps")
  @AutoLogOutput
  public double getShooterFollowerRps() {
    return shooterMotorFollower.getVelocity().getValue();
  }

  // @Log.Graph(name = "Follower Motor Voltage")
  @AutoLogOutput
  public double getShooterMotorFollowerVoltage() {
    return shooterMotorFollower.getMotorVoltage().getValue();
  }

  // @Log.Graph(name = "Follower Stator Voltage")
  @AutoLogOutput
  public double getShooterFollowerSupplyVoltage() {
    return shooterMotorFollower.getStatorCurrent().getValue();
  }

  // workers

  @Override
  public void simulationPeriodic() {
    flywheelSimModel.setInputVoltage(shooterMotorSim.getMotorVoltage());
    flywheelSimModel.update(Robot.defaultPeriodSecs);
    shooterMotorSim.setRotorVelocity(flywheelSimModel.getAngularVelocityRPM() * 60); // rpm to rps
  }
}
