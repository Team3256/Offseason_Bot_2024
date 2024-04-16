// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Robot;
import frc.robot.drivers.MonitoredTalonFX;
import frc.robot.helpers.Conversions;
import frc.robot.subsystems.swerve.helpers.SwerveModuleConstants;
import frc.robot.utils.TalonUtil;
import org.littletonrobotics.junction.AutoLogOutput;

public class SwerveModule {
  public int moduleNumber;
  private Rotation2d angleOffset;

  private MonitoredTalonFX mAngleMotor;
  private MonitoredTalonFX mDriveMotor;
  private CANcoder angleEncoder;

  private final SimpleMotorFeedforward driveFeedForward =
      new SimpleMotorFeedforward(
          SwerveConstants.driveKS, SwerveConstants.driveKV, SwerveConstants.driveKA);

  /* drive motor control requests */
  private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0).withEnableFOC(true);
  private final VelocityVoltage driveVelocity = new VelocityVoltage(0).withEnableFOC(true);

  /* angle motor control requests */
  private final PositionVoltage anglePosition = new PositionVoltage(0).withEnableFOC(true);
  private SwerveModuleConstants swerveModuleConstants;

  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;
    this.angleOffset = moduleConstants.angleOffset;
    this.swerveModuleConstants = moduleConstants;

    /* Angle Encoder Config */
    angleEncoder = new CANcoder(moduleConstants.cancoderID, "mani");
    angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCANcoderConfig);
    // configCANCoder();
    /* Angle Motor Config */
    mAngleMotor = new MonitoredTalonFX(moduleConstants.angleMotorID, "mani");
    TalonUtil.applyAndCheckConfiguration(mAngleMotor, Robot.ctreConfigs.swerveAngleFXConfig);
    resetToAbsolute();

    /* Drive Motor Config */
    mDriveMotor = new MonitoredTalonFX(moduleConstants.driveMotorID, "mani");
    TalonUtil.applyAndCheckConfiguration(mDriveMotor, Robot.ctreConfigs.swerveDriveFXConfig);
    mDriveMotor.getConfigurator().setPosition(0.0);
  }

  public void configCANCoder() {
    angleEncoder.getConfigurator().apply(new CANcoderConfiguration());
    Robot.ctreConfigs.swerveCANcoderConfig.MagnetSensor.SensorDirection =
        SwerveConstants.cancoderInvert;
    Robot.ctreConfigs.swerveCANcoderConfig.MagnetSensor.MagnetOffset = angleOffset.getDegrees();
    Robot.ctreConfigs.swerveCANcoderConfig.MagnetSensor.AbsoluteSensorRange =
        AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCANcoderConfig);
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
    mAngleMotor.setControl(anglePosition.withPosition(desiredState.angle.getRotations()));
    setSpeed(desiredState, isOpenLoop);
  }

  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    if (isOpenLoop) {
      driveDutyCycle.Output =
          desiredState.speedMetersPerSecond / SwerveConstants.maxTranslationalVelocity;
      mDriveMotor.setControl(driveDutyCycle);
    } else {

      driveVelocity.Velocity =
          Conversions.MPSToRPS(
              desiredState.speedMetersPerSecond, SwerveConstants.wheelCircumference);
      driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
      mDriveMotor.setControl(driveVelocity);
    }
  }

  public Rotation2d getCANcoder() {
    return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue());
  }

  public void resetToAbsolute() {
    double absolutePosition = getCANcoder().getRotations() - angleOffset.getRotations();
    mAngleMotor.setPosition(absolutePosition);
  }

  @AutoLogOutput(key = "SwerveModule{moduleNumber}/DriveMotorCurrent")
  public double getDriveMotorCurrent() {
    return mDriveMotor.getSupplyCurrent().getValue();
  }

  @AutoLogOutput(key = "SwerveModule{moduleNumber}/DriveMotorVoltage")
  public double getDriveMotorVoltage() {
    return mDriveMotor.getMotorVoltage().getValue();
  }

  @AutoLogOutput(key = "SwerveModule{moduleNumber}/AngleMotorCurrent")
  public double getAngleMotorCurrent() {
    return mAngleMotor.getSupplyCurrent().getValue();
  }

  @AutoLogOutput(key = "SwerveModule{moduleNumber}/AngleMotorSupply")
  public double getAngleMotorVoltage() {
    return mAngleMotor.getSupplyVoltage().getValue();
  }

  @AutoLogOutput(key = "SwerveModule{moduleNumber}/State")
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        Conversions.RPSToMPS(
            mDriveMotor.getVelocity().getValue(), SwerveConstants.wheelCircumference),
        Rotation2d.fromRotations(mAngleMotor.getPosition().getValue()));
  }

  @AutoLogOutput(key = "SwerveModule{moduleNumber}/Position")
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        Conversions.rotationsToMeters(
            mDriveMotor.getPosition().getValue(), SwerveConstants.wheelCircumference),
        Rotation2d.fromRotations(mAngleMotor.getPosition().getValue()));
  }

  public void off() {}
}
