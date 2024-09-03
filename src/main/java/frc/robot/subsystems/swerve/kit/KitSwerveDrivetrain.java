// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.swerve.kit;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.*;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import java.util.function.Consumer;

public class KitSwerveDrivetrain {
  protected final boolean IsOnCANFD;
  protected final double UpdateFrequency;
  protected final int ModuleCount;
  protected final ModuleIO[] Modules;
  protected final Pigeon2 m_pigeon2;
  protected final StatusSignal<Double> m_yawGetter;
  protected final StatusSignal<Double> m_angularVelocity;
  protected SwerveDriveKinematics m_kinematics;
  protected SwerveDrivePoseEstimator m_odometry;
  protected SwerveModulePosition[] m_modulePositions;
  protected SwerveModuleState[] m_moduleStates;
  protected Translation2d[] m_moduleLocations;
  protected OdometryThread m_odometryThread;
  protected Rotation2d m_fieldRelativeOffset;
  protected Rotation2d m_operatorForwardDirection;
  protected KitSwerveRequest m_requestToApply;
  protected KitSwerveRequest.SwerveControlRequestParameters m_requestParameters;
  protected final ReadWriteLock m_stateLock;
  protected final KitSimSwerveDrivetrain m_simDrive;
  protected Consumer<SwerveDriveState> m_telemetryFunction;
  protected final SwerveDriveState m_cachedState;

  protected boolean checkIsOnCanFD(String canbusName) {
    return CANBus.isNetworkFD(canbusName);
  }

  public KitSwerveDrivetrain(
      SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
    this(driveTrainConstants, 0.0, modules);
  }

  public KitSwerveDrivetrain(
      SwerveDrivetrainConstants driveTrainConstants,
      double OdometryUpdateFrequency,
      SwerveModuleConstants... modules) {
    this(
        driveTrainConstants,
        OdometryUpdateFrequency,
        VecBuilder.fill(0.1, 0.1, 0.1),
        VecBuilder.fill(0.9, 0.9, 0.9),
        modules);
  }

  public KitSwerveDrivetrain(
      SwerveDrivetrainConstants driveTrainConstants,
      double OdometryUpdateFrequency,
      Matrix<N3, N1> odometryStandardDeviation,
      Matrix<N3, N1> visionStandardDeviation,
      SwerveModuleConstants... modules) {
    this.m_requestToApply = new KitSwerveRequest.Idle();
    this.m_requestParameters = new KitSwerveRequest.SwerveControlRequestParameters();
    this.m_stateLock = new ReentrantReadWriteLock();
    this.m_telemetryFunction = null;
    this.m_cachedState = new SwerveDriveState();
    this.IsOnCANFD = this.checkIsOnCanFD(driveTrainConstants.CANbusName);
    if (OdometryUpdateFrequency == 0.0) {
      this.UpdateFrequency = this.IsOnCANFD ? 250.0 : 100.0;
    } else {
      this.UpdateFrequency = OdometryUpdateFrequency;
    }

    this.ModuleCount = modules.length;
    this.m_pigeon2 = new Pigeon2(driveTrainConstants.Pigeon2Id, driveTrainConstants.CANbusName);
    this.m_yawGetter = this.m_pigeon2.getYaw().clone();
    this.m_angularVelocity = this.m_pigeon2.getAngularVelocityZWorld().clone();
    this.Modules = new ModuleIO[this.ModuleCount];
    this.m_modulePositions = new SwerveModulePosition[this.ModuleCount];
    this.m_moduleStates = new SwerveModuleState[this.ModuleCount];
    this.m_moduleLocations = new Translation2d[this.ModuleCount];
    int iteration = 0;
    SwerveModuleConstants[] var8 = modules;
    int var9 = modules.length;

    for (int var10 = 0; var10 < var9; ++var10) {
      SwerveModuleConstants module = var8[var10];
      this.Modules[iteration] = new ModuleIOTalonFX(module, driveTrainConstants.CANbusName);
      this.m_moduleLocations[iteration] = new Translation2d(module.LocationX, module.LocationY);
      this.m_modulePositions[iteration] = this.Modules[iteration].getPosition(true);
      this.m_moduleStates[iteration] = this.Modules[iteration].getCurrentState();
      ++iteration;
    }

    this.m_kinematics = new SwerveDriveKinematics(this.m_moduleLocations);
    this.m_odometry =
        new SwerveDrivePoseEstimator(
            this.m_kinematics,
            new Rotation2d(),
            this.m_modulePositions,
            new Pose2d(),
            odometryStandardDeviation,
            visionStandardDeviation);
    this.m_fieldRelativeOffset = new Rotation2d();
    this.m_operatorForwardDirection = new Rotation2d();
    this.m_simDrive =
        new KitSimSwerveDrivetrain(
            this.m_moduleLocations, this.m_pigeon2, driveTrainConstants, modules);
    this.m_odometryThread = new OdometryThread();
    this.m_odometryThread.start();
  }

  public OdometryThread getDaqThread() {
    return this.m_odometryThread;
  }

  public void setControl(KitSwerveRequest request) {
    try {
      this.m_stateLock.writeLock().lock();
      this.m_requestToApply = request;
    } finally {
      this.m_stateLock.writeLock().unlock();
    }
  }

  public StatusCode configNeutralMode(NeutralModeValue neutralMode) {
    StatusCode status = StatusCode.OK;
    ModuleIO[] var3 = this.Modules;
    int var4 = var3.length;

    for (int var5 = 0; var5 < var4; ++var5) {
      ModuleIO module = var3[var5];
      StatusCode moduleStatus = module.configNeutralMode(neutralMode);
      if (status.isOK()) {
        status = moduleStatus;
      }
    }

    return status;
  }

  public void tareEverything() {
    try {
      this.m_stateLock.writeLock().lock();

      for (int i = 0; i < this.ModuleCount; ++i) {
        this.Modules[i].resetPosition();
        this.m_modulePositions[i] = this.Modules[i].getPosition(true);
      }

      this.m_odometry.resetPosition(
          Rotation2d.fromDegrees((Double) this.m_yawGetter.getValue()),
          this.m_modulePositions,
          new Pose2d());
    } finally {
      this.m_stateLock.writeLock().unlock();
    }
  }

  public void seedFieldRelative() {
    try {
      this.m_stateLock.writeLock().lock();
      this.m_fieldRelativeOffset = this.getState().Pose.getRotation();
    } finally {
      this.m_stateLock.writeLock().unlock();
    }
  }

  public void setOperatorPerspectiveForward(Rotation2d fieldDirection) {
    this.m_operatorForwardDirection = fieldDirection;
  }

  public void seedFieldRelative(Pose2d location) {
    try {
      this.m_stateLock.writeLock().lock();
      this.m_odometry.resetPosition(
          Rotation2d.fromDegrees((Double) this.m_yawGetter.getValue()),
          this.m_modulePositions,
          location);
      this.m_cachedState.Pose = location;
    } finally {
      this.m_stateLock.writeLock().unlock();
    }
  }

  public boolean odometryIsValid() {
    boolean var1;
    try {
      this.m_stateLock.readLock().lock();
      var1 = this.m_odometryThread.odometryIsValid();
    } finally {
      this.m_stateLock.readLock().unlock();
    }

    return var1;
  }

  public ModuleIO getModule(int index) {
    return index >= this.Modules.length ? null : this.Modules[index];
  }

  public SwerveDriveState getState() {
    SwerveDriveState var1;
    try {
      this.m_stateLock.readLock().lock();
      var1 = this.m_cachedState;
    } finally {
      this.m_stateLock.readLock().unlock();
    }

    return var1;
  }

  public Rotation3d getRotation3d() {
    return this.m_pigeon2.getRotation3d();
  }

  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    try {
      this.m_stateLock.writeLock().lock();
      this.m_odometry.addVisionMeasurement(
          visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    } finally {
      this.m_stateLock.writeLock().unlock();
    }
  }

  public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
    try {
      this.m_stateLock.writeLock().lock();
      this.m_odometry.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
    } finally {
      this.m_stateLock.writeLock().unlock();
    }
  }

  public void setVisionMeasurementStdDevs(Matrix<N3, N1> visionMeasurementStdDevs) {
    try {
      this.m_stateLock.writeLock().lock();
      this.m_odometry.setVisionMeasurementStdDevs(visionMeasurementStdDevs);
    } finally {
      this.m_stateLock.writeLock().unlock();
    }
  }

  public void updateSimState(double dtSeconds, double supplyVoltage) {
    this.m_simDrive.update(dtSeconds, supplyVoltage, this.Modules);
  }

  public void registerTelemetry(Consumer<SwerveDriveState> telemetryFunction) {
    try {
      this.m_stateLock.writeLock().lock();
      this.m_telemetryFunction = telemetryFunction;
    } finally {
      this.m_stateLock.writeLock().unlock();
    }
  }

  public Pigeon2 getPigeon2() {
    return this.m_pigeon2;
  }

  public class OdometryThread {
    protected static final int START_THREAD_PRIORITY = 1;
    protected final Thread m_thread = new Thread(this::run);
    protected volatile boolean m_running = false;
    protected final BaseStatusSignal[] m_allSignals;
    protected final MedianFilter peakRemover = new MedianFilter(3);
    protected final LinearFilter lowPass = LinearFilter.movingAverage(50);
    protected double lastTime = 0.0;
    protected double currentTime = 0.0;
    protected double averageLoopTime = 0.0;
    protected int SuccessfulDaqs = 0;
    protected int FailedDaqs = 0;
    protected int lastThreadPriority = 1;
    protected volatile int threadPriorityToSet = 1;

    public OdometryThread() {
      this.m_thread.setDaemon(true);
      this.m_allSignals = new BaseStatusSignal[KitSwerveDrivetrain.this.ModuleCount * 4 + 2];

      for (int i = 0; i < KitSwerveDrivetrain.this.ModuleCount; ++i) {
        BaseStatusSignal[] signals = KitSwerveDrivetrain.this.Modules[i].getSignals();
        this.m_allSignals[i * 4 + 0] = signals[0];
        this.m_allSignals[i * 4 + 1] = signals[1];
        this.m_allSignals[i * 4 + 2] = signals[2];
        this.m_allSignals[i * 4 + 3] = signals[3];
      }

      this.m_allSignals[this.m_allSignals.length - 2] = KitSwerveDrivetrain.this.m_yawGetter;
      this.m_allSignals[this.m_allSignals.length - 1] = KitSwerveDrivetrain.this.m_angularVelocity;
    }

    public void start() {
      this.m_running = true;
      this.m_thread.start();
    }

    public void stop() {
      this.stop(0L);
    }

    public void stop(long millis) {
      this.m_running = false;

      try {
        this.m_thread.join(millis);
      } catch (InterruptedException var4) {
        Thread.currentThread().interrupt();
      }
    }

    public void run() {
      BaseStatusSignal.setUpdateFrequencyForAll(
          KitSwerveDrivetrain.this.UpdateFrequency, this.m_allSignals);
      Threads.setCurrentThreadPriority(true, 1);

      while (this.m_running) {
        StatusCode status;
        if (KitSwerveDrivetrain.this.IsOnCANFD) {
          status =
              BaseStatusSignal.waitForAll(
                  2.0 / KitSwerveDrivetrain.this.UpdateFrequency, this.m_allSignals);
        } else {
          Timer.delay(1.0 / KitSwerveDrivetrain.this.UpdateFrequency);
          status = BaseStatusSignal.refreshAll(this.m_allSignals);
        }

        try {
          KitSwerveDrivetrain.this.m_stateLock.writeLock().lock();
          this.lastTime = this.currentTime;
          this.currentTime = Utils.getCurrentTimeSeconds();
          this.averageLoopTime =
              this.lowPass.calculate(this.peakRemover.calculate(this.currentTime - this.lastTime));
          if (status.isOK()) {
            ++this.SuccessfulDaqs;
          } else {
            ++this.FailedDaqs;
          }

          for (int i = 0; i < KitSwerveDrivetrain.this.ModuleCount; ++i) {
            KitSwerveDrivetrain.this.m_modulePositions[i] =
                KitSwerveDrivetrain.this.Modules[i].getPosition(false);
            KitSwerveDrivetrain.this.m_moduleStates[i] =
                KitSwerveDrivetrain.this.Modules[i].getCurrentState();
          }

          double yawDegrees =
              BaseStatusSignal.getLatencyCompensatedValue(
                  KitSwerveDrivetrain.this.m_yawGetter, KitSwerveDrivetrain.this.m_angularVelocity);
          KitSwerveDrivetrain.this.m_odometry.update(
              Rotation2d.fromDegrees(yawDegrees), KitSwerveDrivetrain.this.m_modulePositions);
          ChassisSpeeds speeds =
              KitSwerveDrivetrain.this.m_kinematics.toChassisSpeeds(
                  KitSwerveDrivetrain.this.m_moduleStates);
          KitSwerveDrivetrain.this.m_requestParameters.currentPose =
              KitSwerveDrivetrain.this
                  .m_odometry
                  .getEstimatedPosition()
                  .relativeTo(new Pose2d(0.0, 0.0, KitSwerveDrivetrain.this.m_fieldRelativeOffset));
          KitSwerveDrivetrain.this.m_requestParameters.kinematics =
              KitSwerveDrivetrain.this.m_kinematics;
          KitSwerveDrivetrain.this.m_requestParameters.swervePositions =
              KitSwerveDrivetrain.this.m_moduleLocations;
          KitSwerveDrivetrain.this.m_requestParameters.currentChassisSpeed = speeds;
          KitSwerveDrivetrain.this.m_requestParameters.timestamp = this.currentTime;
          KitSwerveDrivetrain.this.m_requestParameters.updatePeriod =
              1.0 / KitSwerveDrivetrain.this.UpdateFrequency;
          KitSwerveDrivetrain.this.m_requestParameters.operatorForwardDirection =
              KitSwerveDrivetrain.this.m_operatorForwardDirection;
          KitSwerveDrivetrain.this.m_requestToApply.apply(
              KitSwerveDrivetrain.this.m_requestParameters, KitSwerveDrivetrain.this.Modules);
          KitSwerveDrivetrain.this.m_cachedState.FailedDaqs = this.FailedDaqs;
          KitSwerveDrivetrain.this.m_cachedState.SuccessfulDaqs = this.SuccessfulDaqs;
          KitSwerveDrivetrain.this.m_cachedState.Pose =
              KitSwerveDrivetrain.this.m_odometry.getEstimatedPosition();
          KitSwerveDrivetrain.this.m_cachedState.speeds = speeds;
          KitSwerveDrivetrain.this.m_cachedState.OdometryPeriod = this.averageLoopTime;
          if (KitSwerveDrivetrain.this.m_cachedState.ModuleStates == null) {
            KitSwerveDrivetrain.this.m_cachedState.ModuleStates =
                new SwerveModuleState[KitSwerveDrivetrain.this.Modules.length];
          }

          if (KitSwerveDrivetrain.this.m_cachedState.ModuleTargets == null) {
            KitSwerveDrivetrain.this.m_cachedState.ModuleTargets =
                new SwerveModuleState[KitSwerveDrivetrain.this.Modules.length];
          }

          for (int ix = 0; ix < KitSwerveDrivetrain.this.Modules.length; ++ix) {
            KitSwerveDrivetrain.this.m_cachedState.ModuleStates[ix] =
                KitSwerveDrivetrain.this.Modules[ix].getCurrentState();
            KitSwerveDrivetrain.this.m_cachedState.ModuleTargets[ix] =
                KitSwerveDrivetrain.this.Modules[ix].getTargetState();
          }

          if (KitSwerveDrivetrain.this.m_telemetryFunction != null) {
            KitSwerveDrivetrain.this.m_telemetryFunction.accept(
                KitSwerveDrivetrain.this.m_cachedState);
          }
        } finally {
          KitSwerveDrivetrain.this.m_stateLock.writeLock().unlock();
        }

        if (this.threadPriorityToSet != this.lastThreadPriority) {
          Threads.setCurrentThreadPriority(true, this.threadPriorityToSet);
          this.lastThreadPriority = this.threadPriorityToSet;
        }
      }
    }

    public boolean odometryIsValid() {
      return this.SuccessfulDaqs > 2;
    }

    public void setThreadPriority(int priority) {
      this.threadPriorityToSet = priority;
    }
  }

  public static class SwerveDriveState {
    public int SuccessfulDaqs;
    public int FailedDaqs;
    public Pose2d Pose;
    public ChassisSpeeds speeds;
    public SwerveModuleState[] ModuleStates;
    public SwerveModuleState[] ModuleTargets;
    public double OdometryPeriod;

    public SwerveDriveState() {}
  }
}
