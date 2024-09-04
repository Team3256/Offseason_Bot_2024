// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.swerve.kit;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
  @AutoLog
  public static class ModuleIOInputs {
    public double driveMotorVoltage = 0.0;
    public double driveMotorPosition = 0.0;
    public double driveMotorVelocity = 0.0;
    public double driveMotorStatorCurrent = 0.0;
    public double driveMotorSupplyCurrent = 0.0;
    public double driveMotorTemperature = 0.0;

    public double angleMotorVoltage = 0.0;
    public double angleMotorPosition = 0.0;
    public double angleMotorVelocity = 0.0;
    public double angleMotorStatorCurrent = 0.0;
    public double angleMotorSupplyCurrent = 0.0;
    public double angleMotorTemperature = 0.0;

    public SwerveModuleState currentState = new SwerveModuleState();
    public SwerveModulePosition currentPosition = new SwerveModulePosition();
  }

  public default void updateInputs(ModuleIOInputs inputs) {}

  public SwerveModulePosition getPosition(boolean refresh);

  public void apply(
      SwerveModuleState state,
      SwerveModule.DriveRequestType driveRequestType,
      SwerveModule.SteerRequestType steerRequestType);

  public void applyCharacterization(Rotation2d steerTarget, VoltageOut driveRequest);

  public void applyCharacterization(Rotation2d steerTarget, TorqueCurrentFOC driveRequest);

  public StatusCode configNeutralMode(NeutralModeValue neutralMode);

  public SwerveModulePosition getCachedPosition();

  public SwerveModuleState getCurrentState();

  public SwerveModuleState getTargetState();

  public BaseStatusSignal[] getSignals();

  public void resetPosition();

  public TalonFX getDriveMotor();

  public TalonFX getSteerMotor();

  public CANcoder getCANcoder();

  public static enum DriveRequestType {
    OpenLoopVoltage,
    Velocity;

    private DriveRequestType() {}
  }

  public static enum SteerRequestType {
    MotionMagic,
    MotionMagicExpo;

    private SteerRequestType() {}
  }

  public static enum ClosedLoopOutputType {
    Voltage,
    TorqueCurrentFOC;

    private ClosedLoopOutputType() {}
  }
}
