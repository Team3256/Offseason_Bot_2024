// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import frc.robot.subsystems.swerve.SwerveConstants;

public final class CTREConfigs {
  public TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();
  public TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
  public CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();

  public CTREConfigs() {
    /** Swerve CANCoder Configuration */
    swerveCANcoderConfig.MagnetSensor.SensorDirection = SwerveConstants.cancoderInvert;

    /** Swerve Angle Motor Configurations */
    /* Motor Inverts and Neutral Mode */
    swerveAngleFXConfig.MotorOutput.Inverted = SwerveConstants.angleMotorInvert;
    swerveAngleFXConfig.MotorOutput.NeutralMode = SwerveConstants.angleNeutralMode;

    /* Gear Ratio and Wrapping Config */
    swerveAngleFXConfig.Feedback.SensorToMechanismRatio = SwerveConstants.angleGearRatio;
    swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;

    /* Current Limiting */
    swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimitEnable =
        SwerveConstants.angleEnableCurrentLimit;
    swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimit = SwerveConstants.angleCurrentLimit;
    swerveAngleFXConfig.CurrentLimits.SupplyCurrentThreshold =
        SwerveConstants.angleCurrentThreshold;
    swerveAngleFXConfig.CurrentLimits.SupplyTimeThreshold =
        SwerveConstants.angleCurrentThresholdTime;
    swerveAngleFXConfig.CurrentLimits.StatorCurrentLimit = 50;
    swerveAngleFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    /* PID Config */
    swerveAngleFXConfig.Slot0.kP = SwerveConstants.angleKP;
    swerveAngleFXConfig.Slot0.kI = SwerveConstants.angleKI;
    swerveAngleFXConfig.Slot0.kD = SwerveConstants.angleKD;

    /** Swerve Drive Motor Configuration */
    /* Motor Inverts and Neutral Mode */
    swerveDriveFXConfig.MotorOutput.Inverted = SwerveConstants.driveMotorInvert;
    swerveDriveFXConfig.MotorOutput.NeutralMode = SwerveConstants.driveNeutralMode;

    /* Gear Ratio Config */
    swerveDriveFXConfig.Feedback.SensorToMechanismRatio = SwerveConstants.driveGearRatio;

    /* Current Limiting */
    swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimitEnable =
        SwerveConstants.driveEnableCurrentLimit;
    swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit = SwerveConstants.driveCurrentLimit;
    swerveDriveFXConfig.CurrentLimits.SupplyCurrentThreshold =
        SwerveConstants.driveCurrentThreshold;
    swerveDriveFXConfig.CurrentLimits.SupplyTimeThreshold =
        SwerveConstants.driveCurrentThresholdTime;
    swerveDriveFXConfig.CurrentLimits.StatorCurrentLimit = 60;
    swerveDriveFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    /* PID Config */
    swerveDriveFXConfig.Slot0.kP = SwerveConstants.driveKP;
    swerveDriveFXConfig.Slot0.kI = SwerveConstants.driveKI;
    swerveDriveFXConfig.Slot0.kD = SwerveConstants.driveKD;
    swerveDriveFXConfig.Slot0.kV = SwerveConstants.driveKV;
    swerveDriveFXConfig.Slot0.kA = SwerveConstants.driveKA;
    swerveDriveFXConfig.Slot0.kS = SwerveConstants.driveKS;

    /* Open and Closed Loop Ramping */
    swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = SwerveConstants.openLoopRamp;
    swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = SwerveConstants.openLoopRamp;

    swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod =
        SwerveConstants.closedLoopRamp;
    swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod =
        SwerveConstants.closedLoopRamp;
  }
}
