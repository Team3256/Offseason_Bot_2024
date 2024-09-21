// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.swerve.kit;

import org.littletonrobotics.junction.Logger;

//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by FernFlower decompiler)
//

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;

public interface KitSwerveRequest {
  StatusCode apply(SwerveControlRequestParameters var1, ModuleIO... var2);

  public static class SysIdSwerveSteerGains implements KitSwerveRequest {
    public final MutableMeasure<Voltage> VoltsToApply;
    private VoltageOut m_voltRequest;

    public SysIdSwerveSteerGains() {
      this.VoltsToApply = MutableMeasure.mutable(Units.Volts.of(0.0));
      this.m_voltRequest = new VoltageOut(0.0);
    }

    public StatusCode apply(SwerveControlRequestParameters parameters, ModuleIO... modulesToApply) {
      for (int i = 0; i < modulesToApply.length; ++i) {
        modulesToApply[i]
            .getSteerMotor()
            .setControl(this.m_voltRequest.withOutput(this.VoltsToApply.in(Units.Volts)));
        modulesToApply[i].getDriveMotor().setControl(this.m_voltRequest.withOutput(0.0));
      }

      return StatusCode.OK;
    }

    public SysIdSwerveSteerGains withVolts(Measure<Voltage> Volts) {
      this.VoltsToApply.mut_replace(Volts);
      return this;
    }
  }

  public static class SysIdSwerveRotation implements KitSwerveRequest {
    public final MutableMeasure<Voltage> VoltsToApply;
    private VoltageOut m_voltRequest;

    public SysIdSwerveRotation() {
      this.VoltsToApply = MutableMeasure.mutable(Units.Volts.of(0.0));
      this.m_voltRequest = new VoltageOut(0.0);
    }

    public StatusCode apply(SwerveControlRequestParameters parameters, ModuleIO... modulesToApply) {
      for (int i = 0; i < modulesToApply.length; ++i) {
        modulesToApply[i].applyCharacterization(
            parameters.swervePositions[i].getAngle().plus(Rotation2d.fromDegrees(90.0)),
            this.m_voltRequest.withOutput(this.VoltsToApply.in(Units.Volts)));
      }

      return StatusCode.OK;
    }

    public SysIdSwerveRotation withVolts(Measure<Voltage> Volts) {
      this.VoltsToApply.mut_replace(Volts);
      return this;
    }
  }

  public static class SysIdSwerveTranslation implements KitSwerveRequest {
    public final MutableMeasure<Voltage> VoltsToApply;
    private VoltageOut m_voltRequest;

    public SysIdSwerveTranslation() {
      this.VoltsToApply = MutableMeasure.mutable(Units.Volts.of(0.0));
      this.m_voltRequest = new VoltageOut(0.0);
    }

    public StatusCode apply(SwerveControlRequestParameters parameters, ModuleIO... modulesToApply) {
      for (int i = 0; i < modulesToApply.length; ++i) {
        modulesToApply[i].applyCharacterization(
            Rotation2d.fromDegrees(0.0),
            this.m_voltRequest.withOutput(this.VoltsToApply.in(Units.Volts)));
      }

      return StatusCode.OK;
    }

    public SysIdSwerveTranslation withVolts(Measure<Voltage> Volts) {
      this.VoltsToApply.mut_replace(Volts);
      return this;
    }
  }

  public static class ApplyChassisSpeeds implements KitSwerveRequest {
    public ChassisSpeeds Speeds = new ChassisSpeeds();
    public Translation2d CenterOfRotation = new Translation2d(0.0, 0.0);
    public SwerveModule.DriveRequestType DriveRequestType;
    public SwerveModule.SteerRequestType SteerRequestType;

    public ApplyChassisSpeeds() {
      this.DriveRequestType = DriveRequestType.OpenLoopVoltage;
      this.SteerRequestType = SteerRequestType.MotionMagic;
    }

    public StatusCode apply(SwerveControlRequestParameters parameters, ModuleIO... modulesToApply) {
      SwerveModuleState[] states =
          parameters.kinematics.toSwerveModuleStates(this.Speeds, this.CenterOfRotation);

      for (int i = 0; i < modulesToApply.length; ++i) {
        modulesToApply[i].apply(states[i], this.DriveRequestType, this.SteerRequestType);
      }

      return StatusCode.OK;
    }

    public ApplyChassisSpeeds withSpeeds(ChassisSpeeds speeds) {
      this.Speeds = speeds;
      return this;
    }

    public ApplyChassisSpeeds withCenterOfRotation(Translation2d centerOfRotation) {
      this.CenterOfRotation = centerOfRotation;
      return this;
    }

    public ApplyChassisSpeeds withDriveRequestType(SwerveModule.DriveRequestType driveRequestType) {
      this.DriveRequestType = driveRequestType;
      return this;
    }

    public ApplyChassisSpeeds withSteerRequestType(SwerveModule.SteerRequestType steerRequestType) {
      this.SteerRequestType = steerRequestType;
      return this;
    }
  }

  public static class RobotCentric implements KitSwerveRequest {
    public double VelocityX = 0.0;
    public double VelocityY = 0.0;
    public double RotationalRate = 0.0;
    public double Deadband = 0.0;
    public double RotationalDeadband = 0.0;
    public Translation2d CenterOfRotation = new Translation2d();
    public SwerveModule.DriveRequestType DriveRequestType;
    public SwerveModule.SteerRequestType SteerRequestType;

    public RobotCentric() {
      this.DriveRequestType = DriveRequestType.OpenLoopVoltage;
      this.SteerRequestType = SteerRequestType.MotionMagic;
    }

    public StatusCode apply(SwerveControlRequestParameters parameters, ModuleIO... modulesToApply) {
      double toApplyX = this.VelocityX;
      double toApplyY = this.VelocityY;
      double toApplyOmega = this.RotationalRate;
      if (Math.sqrt(toApplyX * toApplyX + toApplyY * toApplyY) < this.Deadband) {
        toApplyX = 0.0;
        toApplyY = 0.0;
      }

      if (Math.abs(toApplyOmega) < this.RotationalDeadband) {
        toApplyOmega = 0.0;
      }

      ChassisSpeeds speeds = new ChassisSpeeds(toApplyX, toApplyY, toApplyOmega);
      SwerveModuleState[] states =
          parameters.kinematics.toSwerveModuleStates(speeds, this.CenterOfRotation);

      for (int i = 0; i < modulesToApply.length; ++i) {
        modulesToApply[i].apply(states[i], this.DriveRequestType, this.SteerRequestType);
      }

      return StatusCode.OK;
    }

    public RobotCentric withVelocityX(double velocityX) {
      this.VelocityX = velocityX;
      return this;
    }

    public RobotCentric withVelocityY(double velocityY) {
      this.VelocityY = velocityY;
      return this;
    }

    public RobotCentric withRotationalRate(double rotationalRate) {
      this.RotationalRate = rotationalRate;
      return this;
    }

    public RobotCentric withDeadband(double deadband) {
      this.Deadband = deadband;
      return this;
    }

    public RobotCentric withRotationalDeadband(double rotationalDeadband) {
      this.RotationalDeadband = rotationalDeadband;
      return this;
    }

    public RobotCentric withCenterOfRotation(Translation2d centerOfRotation) {
      this.CenterOfRotation = centerOfRotation;
      return this;
    }

    public RobotCentric withDriveRequestType(SwerveModule.DriveRequestType driveRequestType) {
      this.DriveRequestType = driveRequestType;
      return this;
    }

    public RobotCentric withSteerRequestType(SwerveModule.SteerRequestType steerRequestType) {
      this.SteerRequestType = steerRequestType;
      return this;
    }
  }

  public static class PointWheelsAt implements KitSwerveRequest {
    public Rotation2d ModuleDirection = new Rotation2d();
    public SwerveModule.DriveRequestType DriveRequestType;
    public SwerveModule.SteerRequestType SteerRequestType;

    public PointWheelsAt() {
      this.DriveRequestType = DriveRequestType.OpenLoopVoltage;
      this.SteerRequestType = SteerRequestType.MotionMagic;
    }

    public StatusCode apply(SwerveControlRequestParameters parameters, ModuleIO... modulesToApply) {
      for (int i = 0; i < modulesToApply.length; ++i) {
        SwerveModuleState state = new SwerveModuleState(0.0, this.ModuleDirection);
        modulesToApply[i].apply(state, this.DriveRequestType, this.SteerRequestType);
      }

      return StatusCode.OK;
    }

    public PointWheelsAt withModuleDirection(Rotation2d moduleDirection) {
      this.ModuleDirection = moduleDirection;
      return this;
    }

    public PointWheelsAt withDriveRequestType(SwerveModule.DriveRequestType driveRequestType) {
      this.DriveRequestType = driveRequestType;
      return this;
    }

    public PointWheelsAt withSteerRequestType(SwerveModule.SteerRequestType steerRequestType) {
      this.SteerRequestType = steerRequestType;
      return this;
    }
  }

  public static class Idle implements KitSwerveRequest {
    public Idle() {}

    public StatusCode apply(SwerveControlRequestParameters parameters, ModuleIO... modulesToApply) {
      return StatusCode.OK;
    }
  }

  public static class FieldCentricFacingAngle implements KitSwerveRequest {
    public double VelocityX = 0.0;
    public double VelocityY = 0.0;
    public Rotation2d TargetDirection = new Rotation2d();
    public double Deadband = 0.0;
    public double RotationalDeadband = 0.0;
    public Translation2d CenterOfRotation = new Translation2d();
    public SwerveModule.DriveRequestType DriveRequestType;
    public SwerveModule.SteerRequestType SteerRequestType;
    public PhoenixPIDController HeadingController;
    public ForwardReference ForwardReference;

    public FieldCentricFacingAngle() {
      this.DriveRequestType = DriveRequestType.OpenLoopVoltage;
      this.SteerRequestType = SteerRequestType.MotionMagic;
      this.HeadingController = new PhoenixPIDController(0.0, 0.0, 0.0);
      this.ForwardReference = KitSwerveRequest.ForwardReference.OperatorPerspective;
    }

    public StatusCode apply(SwerveControlRequestParameters parameters, ModuleIO... modulesToApply) {
      double toApplyX = this.VelocityX;
      double toApplyY = this.VelocityY;
      Rotation2d angleToFace = this.TargetDirection;
      if (this.ForwardReference == KitSwerveRequest.ForwardReference.OperatorPerspective) {
        Translation2d tmp = new Translation2d(toApplyX, toApplyY);
        tmp = tmp.rotateBy(parameters.operatorForwardDirection);
        toApplyX = tmp.getX();
        toApplyY = tmp.getY();
        angleToFace = angleToFace.rotateBy(parameters.operatorForwardDirection);
      }

      double rotationRate =
          this.HeadingController.calculate(
              parameters.currentPose.getRotation().getRadians(),
              angleToFace.getRadians(),
              parameters.timestamp);
      double toApplyOmega = rotationRate;
      if (Math.sqrt(toApplyX * toApplyX + toApplyY * toApplyY) < this.Deadband) {
        toApplyX = 0.0;
        toApplyY = 0.0;
      }

      if (Math.abs(toApplyOmega) < this.RotationalDeadband) {
        toApplyOmega = 0.0;
      }

      ChassisSpeeds speeds =
          ChassisSpeeds.discretize(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  toApplyX, toApplyY, toApplyOmega, parameters.currentPose.getRotation()),
              parameters.updatePeriod);
      SwerveModuleState[] states =
          parameters.kinematics.toSwerveModuleStates(speeds, this.CenterOfRotation);

      for (int i = 0; i < modulesToApply.length; ++i) {
        modulesToApply[i].apply(states[i], this.DriveRequestType, this.SteerRequestType);
      }

      return StatusCode.OK;
    }

    public FieldCentricFacingAngle withVelocityX(double velocityX) {
      this.VelocityX = velocityX;
      return this;
    }

    public FieldCentricFacingAngle withVelocityY(double velocityY) {
      this.VelocityY = velocityY;
      return this;
    }

    public FieldCentricFacingAngle withTargetDirection(Rotation2d targetDirection) {
      this.TargetDirection = targetDirection;
      return this;
    }

    public FieldCentricFacingAngle withDeadband(double deadband) {
      this.Deadband = deadband;
      return this;
    }

    public FieldCentricFacingAngle withRotationalDeadband(double rotationalDeadband) {
      this.RotationalDeadband = rotationalDeadband;
      return this;
    }

    public FieldCentricFacingAngle withCenterOfRotation(Translation2d centerOfRotation) {
      this.CenterOfRotation = centerOfRotation;
      return this;
    }

    public FieldCentricFacingAngle withDriveRequestType(
        SwerveModule.DriveRequestType driveRequestType) {
      this.DriveRequestType = driveRequestType;
      return this;
    }

    public FieldCentricFacingAngle withSteerRequestType(
        SwerveModule.SteerRequestType steerRequestType) {
      this.SteerRequestType = steerRequestType;
      return this;
    }
  }

  public static class FieldCentric implements KitSwerveRequest {
    public double VelocityX = 0.0;
    public double VelocityY = 0.0;
    public double RotationalRate = 0.0;
    public double Deadband = 0.0;
    public double RotationalDeadband = 0.0;
    public Translation2d CenterOfRotation = new Translation2d();
    public SwerveModule.DriveRequestType DriveRequestType;
    public SwerveModule.SteerRequestType SteerRequestType;
    public ForwardReference ForwardReference;
    protected SwerveModuleState[] m_lastAppliedState;

    public FieldCentric() {
      this.DriveRequestType = DriveRequestType.OpenLoopVoltage;
      this.SteerRequestType = SteerRequestType.MotionMagic;
      this.ForwardReference = KitSwerveRequest.ForwardReference.OperatorPerspective;
      this.m_lastAppliedState = null;
    }

    public StatusCode apply(SwerveControlRequestParameters parameters, ModuleIO... modulesToApply) {
      double toApplyX = this.VelocityX;
      double toApplyY = this.VelocityY;
      if (this.ForwardReference == KitSwerveRequest.ForwardReference.OperatorPerspective) {
        Translation2d tmp = new Translation2d(toApplyX, toApplyY);
        tmp = tmp.rotateBy(parameters.operatorForwardDirection);
        toApplyX = tmp.getX();
        toApplyY = tmp.getY();
      }

      double toApplyOmega = this.RotationalRate;
      if (Math.sqrt(toApplyX * toApplyX + toApplyY * toApplyY) < this.Deadband) {
        toApplyX = 0.0;
        toApplyY = 0.0;
      }

      if (Math.abs(toApplyOmega) < this.RotationalDeadband) {
        toApplyOmega = 0.0;
      }

      ChassisSpeeds speeds =
          ChassisSpeeds.discretize(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  toApplyX, toApplyY, toApplyOmega, parameters.currentPose.getRotation()),
              parameters.updatePeriod);
      SwerveModuleState[] states =
          parameters.kinematics.toSwerveModuleStates(speeds, this.CenterOfRotation);
      for (int i = 0; i < modulesToApply.length; ++i) {
        modulesToApply[i].apply(states[i], this.DriveRequestType, this.SteerRequestType);
      }

      return StatusCode.OK;
    }

    public FieldCentric withVelocityX(double velocityX) {
      this.VelocityX = velocityX;
      return this;
    }

    public FieldCentric withVelocityY(double velocityY) {
      this.VelocityY = velocityY;
      return this;
    }

    public FieldCentric withRotationalRate(double rotationalRate) {
      this.RotationalRate = rotationalRate;
      return this;
    }

    public FieldCentric withDeadband(double deadband) {
      this.Deadband = deadband;
      return this;
    }

    public FieldCentric withRotationalDeadband(double rotationalDeadband) {
      this.RotationalDeadband = rotationalDeadband;
      return this;
    }

    public FieldCentric withCenterOfRotation(Translation2d centerOfRotation) {
      this.CenterOfRotation = centerOfRotation;
      return this;
    }

    public FieldCentric withDriveRequestType(SwerveModule.DriveRequestType driveRequestType) {
      this.DriveRequestType = driveRequestType;
      return this;
    }

    public FieldCentric withSteerRequestType(SwerveModule.SteerRequestType steerRequestType) {
      this.SteerRequestType = steerRequestType;
      return this;
    }
  }

  public static class SwerveDriveBrake implements KitSwerveRequest {
    public SwerveModule.DriveRequestType DriveRequestType;
    public SwerveModule.SteerRequestType SteerRequestType;

    public SwerveDriveBrake() {
      this.DriveRequestType = DriveRequestType.OpenLoopVoltage;
      this.SteerRequestType = SteerRequestType.MotionMagic;
    }

    public StatusCode apply(SwerveControlRequestParameters parameters, ModuleIO... modulesToApply) {
      for (int i = 0; i < modulesToApply.length; ++i) {
        SwerveModuleState state =
            new SwerveModuleState(0.0, parameters.swervePositions[i].getAngle());
        modulesToApply[i].apply(state, this.DriveRequestType, this.SteerRequestType);
      }

      return StatusCode.OK;
    }

    public SwerveDriveBrake withDriveRequestType(SwerveModule.DriveRequestType driveRequestType) {
      this.DriveRequestType = driveRequestType;
      return this;
    }

    public SwerveDriveBrake withSteerRequestType(SwerveModule.SteerRequestType steerRequestType) {
      this.SteerRequestType = steerRequestType;
      return this;
    }
  }

  public static class SwerveControlRequestParameters {
    public SwerveDriveKinematics kinematics;
    public ChassisSpeeds currentChassisSpeed;
    public Pose2d currentPose;
    public double timestamp;
    public Translation2d[] swervePositions;
    public Rotation2d operatorForwardDirection;
    public double updatePeriod;

    public SwerveControlRequestParameters() {}
  }

  public static enum ForwardReference {
    RedAlliance,
    OperatorPerspective;

    private ForwardReference() {}
  }
}
