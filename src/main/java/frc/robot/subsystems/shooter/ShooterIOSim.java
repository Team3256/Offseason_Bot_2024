// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterIOSim implements ShooterIO {
  private final TalonFX shooterMotor = new TalonFX(ShooterConstants.kShooterMotorID);
  private final TalonFX shooterMotorFollower =
      new TalonFX(ShooterConstants.kShooterMotorFollowerID);

  private final TalonFXSimState shooterMotorSimState = shooterMotor.getSimState();
  private final TalonFXSimState followerMotorSimState = shooterMotorFollower.getSimState();

  private final DCMotorSim shooterMotorModel =
      new DCMotorSim(
          DCMotor.getKrakenX60Foc(1),
          ShooterConstants.kGearingRatio,
          ShooterConstants.kShooterInertia);
  private final DCMotorSim followerMotorModel =
      new DCMotorSim(
          DCMotor.getKrakenX60Foc(1),
          ShooterConstants.kFollowerGearingRatio,
          ShooterConstants.kFollowerShooterInertia);

  final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
  final MotionMagicVelocityVoltage motionMagicRequest =
      new MotionMagicVelocityVoltage(0).withSlot(0);
  private final VoltageOut voltageReq = new VoltageOut(0);

  final VelocityVoltage velocityRequestFollower = new VelocityVoltage(0).withSlot(0);
  final MotionMagicVelocityVoltage motionMagicRequestFollower =
      new MotionMagicVelocityVoltage(0).withSlot(0);

  public ShooterIOSim() {
    var motorConfig = ShooterConstants.motorConfigs;
    PhoenixUtil.checkErrorAndRetry(() -> shooterMotor.getConfigurator().refresh(motorConfig));
    TalonUtil.applyAndCheckConfiguration(shooterMotor, motorConfig);

    var motorConfigFollower = ShooterConstants.followerMotorConfigs;
    PhoenixUtil.checkErrorAndRetry(
        () -> shooterMotorFollower.getConfigurator().refresh(motorConfigFollower));
    TalonUtil.applyAndCheckConfiguration(shooterMotorFollower, motorConfigFollower);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    shooterMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    followerMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    var motorVoltage = shooterMotorSimState.getMotorVoltage();
    var followerMotorVoltage = followerMotorSimState.getMotorVoltage();

    shooterMotorModel.setInputVoltage(motorVoltage);
    shooterMotorModel.update(0.02); // assume 20ms loop
    followerMotorModel.setInputVoltage(followerMotorVoltage);
    followerMotorModel.update(0.02); // assume 20ms loop

    shooterMotorSimState.setRawRotorPosition(shooterMotorModel.getAngularPositionRad());
    shooterMotorSimState.setRotorVelocity(
        Units.radiansToRotations(shooterMotorModel.getAngularVelocityRadPerSec()));
    inputs.shooterMotorVoltage = motorVoltage;
    inputs.shooterMotorVelocity = shooterMotorModel.getAngularVelocityRadPerSec();
    inputs.shooterMotorStatorCurrent = shooterMotorModel.getCurrentDrawAmps();
    inputs.shooterMotorSupplyCurrent = shooterMotorSimState.getSupplyCurrent();
    inputs.shooterMotorTemperature = 69;
    inputs.shooterMotorReferenceSlope = 0; // ???

    followerMotorSimState.setRawRotorPosition(followerMotorModel.getAngularPositionRad());
    followerMotorSimState.setRotorVelocity(
        Units.radiansToRotations(followerMotorModel.getAngularVelocityRadPerSec()));
    inputs.shooterMotorFollowerVoltage = followerMotorVoltage;
    inputs.shooterMotorFollowerVelocity = followerMotorModel.getAngularVelocityRadPerSec();
    inputs.shooterMotorFollowerStatorCurrent = followerMotorModel.getCurrentDrawAmps();
    inputs.shooterMotorFollowerSupplyCurrent = followerMotorSimState.getSupplyCurrent();
    inputs.shooterMotorFollowerTemperature = 69;
    inputs.shooterMotorFollowerReferenceSlope = 0; // ???
  }

  @Override
  public void setShooterVoltage(double voltage) {
    shooterMotor.setVoltage(voltage);
  }

  @Override
  public void setShooterVelocity(double velocity) {
    if (ShooterConstants.kUseMotionMagic) {
      shooterMotor.setControl(motionMagicRequest.withVelocity(velocity));
    } else {
      shooterMotor.setControl(velocityRequest.withVelocity(velocity));
    }
  }

  @Override
  public void setShooterFollowerVoltage(double voltage) {
    shooterMotorFollower.setVoltage(voltage);
  }

  @Override
  public void setShooterFollowerVelocity(double velocity) {
    if (ShooterConstants.kUseMotionMagic) {
      shooterMotorFollower.setControl(motionMagicRequestFollower.withVelocity(velocity));
    } else {
      shooterMotorFollower.setControl(velocityRequestFollower.withVelocity(velocity));
    }
  }

  @Override
  public void off() {
    shooterMotor.setControl(new NeutralOut());
    shooterMotorFollower.setControl(new NeutralOut());
  }

  @Override
  public TalonFX getMotor() {
    return shooterMotor;
  }

  @Override
  public VoltageOut getVoltageRequest() {
    return voltageReq;
  }
}
