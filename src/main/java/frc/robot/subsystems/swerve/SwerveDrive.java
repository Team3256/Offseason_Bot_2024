// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeatureFlags;
import frc.robot.Robot;
import frc.robot.autos.AutoConstants;
import frc.robot.drivers.MonitoredPigeon2;
import frc.robot.limelight.Limelight;
import frc.robot.limelight.LimelightHelpers;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.apache.commons.math3.analysis.interpolation.LinearInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class SwerveDrive extends SubsystemBase implements Loggable {

  private static PolynomialSplineFunction distanceToStdDevXTranslation;
  private static PolynomialSplineFunction distanceToStdDevYTranslation;
  private static PolynomialSplineFunction distancetostdDevAngle;

  public void DEFAULT() {}

  static {
    if (FeatureFlags.kLocalizationStdDistanceBased) {
      double[] trainDistance = new double[SwerveConstants.kSwervePoseEstimatorStdData.size()];
      double[] trainStdDevXTranslation =
          new double[SwerveConstants.kSwervePoseEstimatorStdData.size()];
      double[] trainStdDevYTranslation =
          new double[SwerveConstants.kSwervePoseEstimatorStdData.size()];
      double[] trainStdDevAngle = new double[SwerveConstants.kSwervePoseEstimatorStdData.size()];
      for (int i = 0; i < SwerveConstants.kSwervePoseEstimatorStdData.size(); i++) {
        trainDistance[i] = SwerveConstants.kSwervePoseEstimatorStdData.get(i).distance;
        trainStdDevXTranslation[i] =
            SwerveConstants.kSwervePoseEstimatorStdData.get(i).stdDevXTranslation;
        trainStdDevYTranslation[i] =
            SwerveConstants.kSwervePoseEstimatorStdData.get(i).stdDevYTranslation;
        trainStdDevAngle[i] = SwerveConstants.kSwervePoseEstimatorStdData.get(i).stdDevAngle;
      }
      distanceToStdDevXTranslation =
          new LinearInterpolator().interpolate(trainDistance, trainStdDevXTranslation);
      distanceToStdDevYTranslation =
          new LinearInterpolator().interpolate(trainDistance, trainStdDevYTranslation);
      distancetostdDevAngle = new LinearInterpolator().interpolate(trainDistance, trainStdDevAngle);
    }
  }

  private final SwerveDrivePoseEstimator poseEstimator;
  private final Field2d limelightLocalizationField = new Field2d();
  // private Pose2d PP_currentPose = new Pose2d();
  // private Pose2d PP_targetPose = new Pose2d();
  public SwerveDriveOdometry swerveOdometry;
  public SwerveModule[] mSwerveMods;
  public MonitoredPigeon2 gyro;
  public ChassisSpeeds curChassisSpeeds = new ChassisSpeeds();
  private Thread odometryThread;
  public static final Lock odometryLock = new ReentrantLock();

  private List<Double> distanceData = new ArrayList<Double>();
  private List<Double> poseXData = new ArrayList<Double>();
  private List<Double> poseYData = new ArrayList<Double>();
  private List<Double> poseThetaData = new ArrayList<Double>();
  private double lastLocalizeTime = 0;

  public Pose2d simPose = new Pose2d();

  public SwerveDrive() {
    gyro = new MonitoredPigeon2(24, "mani");
    // gyro.getConfigurator().apply(new Pigeon2Configuration());
    var allianceBruh = DriverStation.getAlliance();
    if (allianceBruh.isPresent()) {
      if (allianceBruh.get() == DriverStation.Alliance.Blue) {
        gyro.setYaw(180);
        LimelightHelpers.setPipelineIndex("limelight", 1);
      } else {
        gyro.setYaw(0);
        LimelightHelpers.setPipelineIndex("limelight", 0);
      }
    }
    mSwerveMods =
        new SwerveModule[] {
          new SwerveModule(0, SwerveConstants.Mod0.constants),
          new SwerveModule(1, SwerveConstants.Mod1.constants),
          new SwerveModule(2, SwerveConstants.Mod2.constants),
          new SwerveModule(3, SwerveConstants.Mod3.constants)
        };

    /*
     * By pausing init for a second before setting module offsets, we avoid a bug
     * with inverting motors.
     * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
     */
    System.out.println("Waiting for one second before setting module offsets...");
    Timer.delay(1.0);
    resetModulesToAbsolute();

    swerveOdometry =
        new SwerveDriveOdometry(
            SwerveConstants.swerveKinematics, getGyroYaw(), getModulePositions());
    // old rotation 2 0 0.025

    AutoBuilder.configureHolonomic(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds, // Robot relative
        (ChassisSpeeds chassisSpeeds) -> { // Robot relative
          drive(
              new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond),
              chassisSpeeds.omegaRadiansPerSecond,
              false, // Robot relative
              false); // Closed loop
        },
        new HolonomicPathFollowerConfig(
            new PIDConstants(5, 0, 0), // Translation PID constants OLD: 29 0 0
            new PIDConstants(3.5, 0, 0), // Rotation PID constants OLD: 7.5 0 0.75
            AutoConstants.kMaxSpeedMetersPerSecond, // Max module speed, in m/s
            AutoConstants
                .kDriveBaseRadiusMeters, // Drive base radius in meters. Distance from robot center
            // to furthest module.
            new ReplanningConfig(
                false, false) // Default path replanning config. See the API for the options
            // here
            ),
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get()
                == DriverStation.Alliance
                    .Blue; // flip path if we are blue alliance since paths were
            // drawn for red
          }
          return false;
        },
        this);

    // configureAutoBuilder(10, 0.075, 0.15, 0.1, 0, 0); // kPTrans: 5.75

    poseEstimator =
        new SwerveDrivePoseEstimator(
            SwerveConstants.swerveKinematics,
            getGyroYaw(),
            new SwerveModulePosition[] {
              mSwerveMods[0].getPosition(),
              mSwerveMods[1].getPosition(),
              mSwerveMods[2].getPosition(),
              mSwerveMods[3].getPosition()
            },
            new Pose2d(),
            VecBuilder.fill(0.5, 0.5, 0.02),
            VecBuilder.fill(0.10, 0.10, 0.5));
    // new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.5, 0.5, 0.02), // Current state
    // X, Y,
    // theta.
    // new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.10, 0.10, 0.5));

    SmartDashboard.putData("Limelight Localization Field", limelightLocalizationField);

    odometryThread = new Thread(this::localizeThreadCaller);
    odometryThread.start();
  }

  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    curChassisSpeeds =
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(), translation.getY(), rotation, getHeading())
            : new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
    SwerveModuleState[] swerveModuleStates =
        SwerveConstants.swerveKinematics.toSwerveModuleStates(curChassisSpeeds);
    if (FeatureFlags.kSwerveVelocityLimitingEnabled) {
      SwerveDriveKinematics.desaturateWheelSpeeds(
          swerveModuleStates, SwerveConstants.maxTranslationalVelocity);
    }
    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  // public void drive(Translation2d translation, double rotation, boolean
  // fieldRelative, boolean isOpenLoop) {
  // SwerveModuleState[] swerveModuleStates =
  // SwerveConstants.swerveKinematics.toSwerveModuleStates(
  // fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
  // translation.getX(),
  // translation.getY(),
  // rotation,
  // getHeading())
  // : new ChassisSpeeds(
  // translation.getX(),
  // translation.getY(),
  // rotation));
  // SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates,
  // SwerveConstants.maxTranslationalVelocity);

  // for (SwerveModule mod : mSwerveMods) {
  // mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
  // }
  // }

  @Config(name = "Set Angular Velocity")
  public void setAngularVelocity(double radPerSec) {
    System.out.println("setAngularVelocity:" + radPerSec);
    drive(new Translation2d(), radPerSec, true, true);
  }

  @Config(name = "Set Velocity")
  public void setVelocity(double velocity) {
    System.out.println("setIntakeVelocity:" + velocity);
    drive(new Translation2d(velocity, 0), 0, false, true);
  }

  @AutoLogOutput
  public ChassisSpeeds getChassisSpeeds() {
    return curChassisSpeeds;
  }

  public void resetGyro() {
    gyro.setYaw(0);
    Timer.delay(0.125);
    System.out.println("Gyro reset!");
  }

  public void flipGyro() {
    gyro.setYaw(180);
    Timer.delay(0.125);
    System.out.println("Gyro flipped!");
  }

  public boolean CANTest() {
    return true;
  }

  public void getDistanceToSpeaker() {
    // System.out.println(Limelight.getBotpose("limelight").length);
    Pose2d ourPose = this.getPose();

    // double x = visionBotPose[0] -
    // Constants.AutoConstants.kBlueSpeakerLocation.getX();
    // double y = visionBotPose[1] -
    // Constants.AutoConstants.kBlueSpeakerLocation.getY();

    Pose2d scoringLocation;
    Optional<DriverStation.Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent() && ally.get() == DriverStation.Alliance.Red) {
      scoringLocation = AutoConstants.kRedSpeakerLocation;
    } else if (ally.isPresent()) {
      scoringLocation = AutoConstants.kBlueSpeakerLocation;
    } else {
      scoringLocation = AutoConstants.kRedSpeakerLocation;
    }
    double distance = ourPose.getTranslation().getDistance(scoringLocation.getTranslation());
    // double distance = Math.sqrt(x*x+y*y);
    System.out.println("dist to speaker:" + distance);
    Logger.recordOutput("BruhPose", ourPose);
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, SwerveConstants.maxTranslationalVelocity);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  @AutoLogOutput
  public Pose2d stupidIdiotLimelightDumb() {
    Pose2d limelightPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight").pose;
    return limelightPose;
  }

  public void localize(String networkTablesName) {
    // Check if we're running on the main thread
    // StaticThreadChecker.checkCurrentThread();
    if (!Limelight.hasValidTargets(networkTablesName)) return;
    /*
     * From the Limelight docs about using WPILib's Pose
     * Estimator:
     * https://docs.limelightvision.io/docs/docs-limelight/pipeline-
     * apriltag/apriltag-robot-localization#using-wpilibs-pose-estimator
     * In 2024, most of the WPILib Ecosystem transitioned to a single-origin
     * coordinate system. In 2023, your coordinate system origin changed based on
     * your alliance color.
     *
     * For 2024 and beyond, the origin of your coordinate system should always be
     * the "blue" origin. FRC teams should always use botpose_wpiblue for
     * pose-related functionality
     */
    // LimelightHelpers.SetRobotOrientation(
    // networkTablesName,
    // poseEstimator.getEstimatedPosition().getRotation().getDegrees(),
    // 0,
    // 0,
    // 0,
    // 0,
    // 0);

    double tl = Limelight.getLatency_Pipeline(networkTablesName);
    Pose2d limelightPose =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(networkTablesName).pose;

    double[] aprilTagLocation = Limelight.getTargetPose_RobotSpace(networkTablesName);
    double aprilTagDistance = new Translation2d(aprilTagLocation[0], aprilTagLocation[2]).getNorm();
    if (FeatureFlags.kLocalizationDataCollectionMode) {
      distanceData.add(aprilTagDistance);
      poseXData.add(limelightPose.getX());
      poseYData.add(limelightPose.getY());
      poseThetaData.add(limelightPose.getRotation().getRadians());
    }

    if (FeatureFlags.kLocalizationStdDistanceBased) {
      // we should probably use std-devs for champs bc it helps filter out "bad" data,
      // but it needs to be done for EVERY SINGLE LIMELIGHT!
      if (FeatureFlags.kDebugEnabled) {
        SmartDashboard.putNumber("April Tag Distance", aprilTagDistance);
        SmartDashboard.putNumber("X std", getStdDevXTranslation(aprilTagDistance));
        SmartDashboard.putNumber("Y std", getStdDevYTranslation(aprilTagDistance));
        SmartDashboard.putNumber("Theta std", getStdDevAngle(aprilTagDistance));
      }

      // poseEstimator.setVisionMeasurementStdDevs(
      // VecBuilder.fill(
      // getStdDevXTranslation(aprilTagDistance),
      // getStdDevYTranslation(aprilTagDistance),
      // getStdDevAngle(aprilTagDistance)));
      poseEstimator.addVisionMeasurement(
          limelightPose, Timer.getFPGATimestamp() - Units.millisecondsToSeconds(tl));

      // poseEstimator.addVisionMeasurement(
      // limelightPose,
      // Timer.getFPGATimestamp() - Units.millisecondsToSeconds(tl),
      // new MatBuilder<>(Nat.N3(), Nat.N1())
      // .fill(
      // getStdDevXTranslation(aprilTagDistance),
      // getStdDevYTranslation(aprilTagDistance),
      // getStdDevAngle(aprilTagDistance)));
    } else {
      poseEstimator.addVisionMeasurement(
          limelightPose, Timer.getFPGATimestamp() - Units.millisecondsToSeconds(tl));
    }

    if (FeatureFlags.kDebugEnabled) {
      limelightLocalizationField.setRobotPose(limelightPose);
      SmartDashboard.putNumber("Lime Light pose x " + networkTablesName, limelightPose.getX());
      SmartDashboard.putNumber("Lime Light pose y " + networkTablesName, limelightPose.getY());
      SmartDashboard.putNumber("Lime Light pose theta", limelightPose.getRotation().getDegrees());
    }
  }

  public void localizeThreadCaller() {
    // localizeThreadCaller is called from the main thread:
    // new Thread(this::localizeThread).start();
    // so localize will update the pose estimator in a separate thread
    // and it'll reflect over to the main thread's poseEstimator
    // when the main thread calls getPose()// using while true instead of
    // periodic()ly creating a new thread!
    if (DriverStation.isTeleopEnabled()) {
      localize("limelight");
      // localize("limelight-left");
      // localize("limelight-right");
      // Timer.delay(SwerveConstants.kVisionSecondsBetweenLocalize);
    }
  }

  public double clampDistanceForInterpolation(double distance) {
    return MathUtil.clamp(
        distance,
        SwerveConstants.kSwervePoseEstimatorMinValue,
        SwerveConstants.kSwervePoseEstimatorMaxValue);
  }

  public double getStdDevXTranslation(double distance) {
    return distanceToStdDevXTranslation.value(clampDistanceForInterpolation(distance));
  }

  public double getStdDevYTranslation(double distance) {
    return distanceToStdDevYTranslation.value(clampDistanceForInterpolation(distance));
  }

  public double getStdDevAngle(double distance) {
    return distancetostdDevAngle.value(clampDistanceForInterpolation(distance));
  }

  @AutoLogOutput
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  @AutoLogOutput
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : mSwerveMods) {
      positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }

  @AutoLogOutput
  // @Log(name = "Pose")
  public Pose2d getPose() {
    if (Robot.isSimulation()) return simPose;
    if (FeatureFlags.kSwerveUseVisionForPoseEst) {
      return poseEstimator.getEstimatedPosition();
    } else {
      return swerveOdometry.getPoseMeters();
    }
  }

  public void updatePose(Rotation2d gyroYaw, SwerveModulePosition[] swerveModulePositions) {
    if (Robot.isSimulation()) {
      ChassisSpeeds fieldRelative =
          ChassisSpeeds.fromRobotRelativeSpeeds(curChassisSpeeds, getGyroYaw());
      simPose =
          simPose.transformBy(
              new Transform2d(
                  fieldRelative.vxMetersPerSecond * Robot.defaultPeriodSecs,
                  fieldRelative.vyMetersPerSecond * Robot.defaultPeriodSecs,
                  Rotation2d.fromRadians(
                      fieldRelative.omegaRadiansPerSecond * Robot.defaultPeriodSecs)));
    } else if (FeatureFlags.kSwerveUseVisionForPoseEst) {
      poseEstimator.update(getGyroYaw(), getModulePositions());
    } else {
      swerveOdometry.update(getGyroYaw(), getModulePositions());
    }
  }

  @Config(name = "Set Pose")
  public void setPose(Pose2d pose) {
    if (FeatureFlags.kSwerveUseVisionForPoseEst) {
      poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
    } else {
      swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }
  }

  @AutoLogOutput(key = "robotVelocity")
  public double getVelocity() {
    ChassisSpeeds something = SwerveConstants.swerveKinematics.toChassisSpeeds(getModuleStates());
    return Math.hypot(something.vxMetersPerSecond, something.vyMetersPerSecond);
  }

  @AutoLogOutput(key = "robotRotationalVelocity")
  public double getRotationalVelocity() {
    ChassisSpeeds something = SwerveConstants.swerveKinematics.toChassisSpeeds(getModuleStates());
    return something.omegaRadiansPerSecond;
  }

  @Config(name = "Set Heading")
  public void setHeading(Rotation2d heading) {
    if (FeatureFlags.kSwerveUseVisionForPoseEst) {
      poseEstimator.resetPosition(
          getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    } else {
      swerveOdometry.resetPosition(
          getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }
  }

  public void zeroHeading() {
    if (FeatureFlags.kSwerveUseVisionForPoseEst) {
      poseEstimator.resetPosition(
          getGyroYaw(),
          getModulePositions(),
          new Pose2d(getPose().getTranslation(), new Rotation2d()));
    } else {
      swerveOdometry.resetPosition(
          getGyroYaw(),
          getModulePositions(),
          new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }
  }

  @AutoLogOutput
  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  @AutoLogOutput
  // @Log(name = "Gyro Yaw")
  public Rotation2d getGyroYaw() {
    Rotation2d yaw;
    if (Robot.isReal()) yaw = Rotation2d.fromDegrees(gyro.getYaw().getValue() % 360);
    else yaw = simPose.getRotation();
    return yaw;
  }

  public void resetModulesToAbsolute() {
    for (SwerveModule mod : mSwerveMods) {
      mod.resetToAbsolute();
    }
  }

  public static double joystickToSpeed(double joystickValue, double maxValue) {
    if (FeatureFlags.kQuadraticDrive) {
      return Math.signum(joystickValue) * joystickValue * joystickValue * maxValue;
    } else {
      return joystickValue * maxValue;
    }
  }

  // Docs: https://pathplanner.dev/pplib-build-an-auto.html#configure-autobuilder
  public void configureAutoBuilder(
      double translationKP,
      double translationKI,
      double translationKD,
      double rotationKP,
      double rotationKI,
      double rotationKD) {
    // Auto must drive in robot relative and closed loop mode
    AutoBuilder.configureHolonomic(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds, // Robot relative
        (ChassisSpeeds chassisSpeeds) -> { // Robot relative
          drive(
              new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond),
              chassisSpeeds.omegaRadiansPerSecond,
              false, // Robot relative
              false); // Closed loop
        },
        new HolonomicPathFollowerConfig(
            new PIDConstants(
                translationKP, translationKI, translationKD, 0), // Translation PID constants
            new PIDConstants(rotationKP, rotationKI, rotationKD), // Rotation PID constants
            AutoConstants.kMaxSpeedMetersPerSecond, // Max module speed, in m/s
            AutoConstants
                .kDriveBaseRadiusMeters, // Drive base radius in meters. Distance from robot center
            // to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options
            // here
            ),
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get()
                == DriverStation.Alliance
                    .Blue; // flip path if we are blue alliance since paths were
            // drawn for red
          }
          return false;
        },
        this);

    // PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
    // PP_currentPose = pose;

    // });
    // PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
    // PP_targetPose = pose;
    // });
    // // PathPlannerLogging.setLogActivePathCallback((pose) -> {
    // // Logger.recordOutput("Auto-TargetPose", pose);
    // // })
    // // Pathfinding.setPathfinder(new LocalADStarAK());
    // PathPlannerLogging.setLogActivePathCallback(
    // (activePath) -> {
    // // Logger.recordOutput("AutoTrajectory", activePath.toArray(new
    // // Pose2d[activePath.size()]));
    // autoFieldLog.getObject("path").setPoses(activePath);
    // });
    // PathPlannerLogging.setLogTargetPoseCallback(
    // (targetPose) -> {
    // Logger.recordOutput("Auto/TrajectorySetpoint", targetPose);
    // });
  }

  public boolean isAlignedToSpeaker() {
    // Get the current pose
    Pose2d currentPose = getPose();
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      Pose2d speakerPose;
      if (alliance.get() == DriverStation.Alliance.Red) {
        // Pose of speaker
        speakerPose = AutoConstants.kRedSpeakerLocation;
      } else {
        // Pose of speaker
        speakerPose = AutoConstants.kBlueSpeakerLocation;
      }
      if (speakerPose == null) {
        return false;
      }
      // Get the distance to the speaker
      double distanceToSpeaker =
          currentPose.getTranslation().getDistance(speakerPose.getTranslation());

      // If the distance to the speaker is less than the threshold, return true
      return distanceToSpeaker < AutoConstants.kSpeakerAlignmentThreshold;
    } else {
      return false;
    }
  }

  public void off() {
    for (var module : mSwerveMods) {
      module.off();
    }
  }

  @Override
  public void periodic() {
    // System.out.println(getGyroYaw().toString());
    // update last period values
    // ChassisSpeeds cs =
    // Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    // double actualVel = Math.hypot(cs.vxMetersPerSecond,cs.vyMetersPerSecond);
    // actualAccel = (actualVel - prevActualVel)/Robot.defaultPeriodSecs;
    // prevActualVel = actualVel;

    // Logger.recordOutput("currentPose", PP_currentPose);
    // Logger.recordOutput("targetPose", PP_targetPose);

    odometryLock.lock();
    updatePose(getGyroYaw(), getModulePositions());
    odometryLock.unlock();
    // if (Timer.getFPGATimestamp() - lastLocalizeTime >
    // SwerveConstants.kVisionSecondsBetweenLocalize) {
    // Logger.recordOutput("Last localize time", lastLocalizeTime);
    // localize("limelight-top");
    // lastLocalizeTime = Timer.getFPGATimestamp();
    // }
    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
    }
    // Is the odometry thread still alive?
    if (!odometryThread.isAlive()) {
      if (FeatureFlags.kDebugEnabled) {
        System.out.println("Odometry thread is dead! Restarting...");
      }
      // odometryThread = new Thread(this::localizeThreadCaller);
      // odometryThread.start();
    }
    this.localizeThreadCaller();
  }
}
