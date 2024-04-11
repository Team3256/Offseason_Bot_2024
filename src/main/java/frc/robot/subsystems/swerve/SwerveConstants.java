// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import frc.robot.TrainingDataPoint;
import frc.robot.subsystems.swerve.helpers.COTSTalonFXSwerveConstants;
import frc.robot.subsystems.swerve.helpers.SwerveModuleConstants;
import java.util.List;

public final class SwerveConstants {
  // CAN
  public static final int pigeonID = 24;

  // Motor
  public static final COTSTalonFXSwerveConstants
      chosenModule = // TODO: This must be tuned to specific robot
      COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(
              COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L3);

  // PID
  public static final double angleKP = chosenModule.angleKP;
  public static final double angleKI = chosenModule.angleKI;
  public static final double angleKD = chosenModule.angleKD;
  public static final double driveKP = 0.3; // 0.030 *12
  public static final double driveKI = 0;
  public static final double driveKD = 0; // 0.00001*12
  public static final double driveKF = 0.0;
  public static final double aziDrivekP = 0.02;
  public static final double aziDrivekI = 0;
  public static final double aziDrivekD = 0.015; // 0.01 -> 0.005

  public static final double aDrivekF = 0.00;

  // Note drive
  public static final double noteTargetTurnX = 0.5;
  public static final double noteTargetTurnY = 1;
  public static final double noteTargetDriveX = 0.5;
  public static final double noteTargetDriveY = 0.3;

  public static final double noteAngleTolerance = 5;
  public static final double noteAngularVelocityTolerance = 2;
  public static final double noteDistTolerance = 0.04;
  public static final double noteVelocityTolerance = 0.1;

  public static final double noteDrivekP = 0.7;
  public static final double noteDrivekI = 0;
  public static final double noteDrivekD = 0;
  public static final double noteDrivekF = 0.1;
  public static final double noteTurnkP = 0.025;
  public static final double noteTurnkI = 0;
  public static final double noteTurnkD = 0;
  public static final double noteTurnkF = 0;

  /* Drive Motor Characterization Values From SysID */
  public static final double driveKS = 0; // TODO: Tune
  public static final double driveKV = 0.13;
  public static final double driveKA = 0;

  // Localization

  // Localization will run every [kLocalizationSeconds] seconds.
  public static final double kVisionSecondsBetweenLocalize = Robot.defaultPeriodSecs * 10;

  // Pose Estimation
  public static final List<TrainingDataPoint> kSwervePoseEstimatorStdData =
      List.of(
          new TrainingDataPoint(0.804213, 7.8637125, 3.167025, 0.157388), // impossible location
          new TrainingDataPoint(1.093281, 9.14025, 3.2641875, 0.167762), // 1 foot
          new TrainingDataPoint(1.431827, 4.66685, 2.0580375, 0.130962), // 2 foot
          new TrainingDataPoint(1.743492, 6.7381125, 3.246825, 0.158225), // 3 foot
          new TrainingDataPoint(2.028354, 6.0687, 3.2111875, 0.171350), // 4 foot
          new TrainingDataPoint(2.336431, 6.591775, 3.2745625, 0.166477), // 5 foot
          new TrainingDataPoint(2.665142, 7.11455, 3.798075, 0.167438), // 6 foot
          new TrainingDataPoint(2.953581, 7.4191, 3.9353125, 0.159327), // 7 foot
          new TrainingDataPoint(3.282424, 6.574225, 3.6709625, 0.161348), // 8 foot
          new TrainingDataPoint(3.560149, 6.7152875, 5.2717375, 0.186038),
          new TrainingDataPoint(7.128826, 16.0333875, 30.8555875, 0.505372)); // 9 foot

  public static final double kSwervePoseEstimatorMinValue = 0.804213;
  public static final double kSwervePoseEstimatorMaxValue = 7.128826;

  /* Drivetrain Constants */
  public static final double trackWidth = Units.inchesToMeters(20.75); // Drive base x TODO: Tune
  public static final double wheelBase = Units.inchesToMeters(20.75); // Drive base y TODO: Tune
  /*
   * We calculate the drive base radius using the Pythagorean theorem
   * Assuming we know the sides of the robot, which is square.
   * We can calculate the distance from the center of the robot to the corner.
   *
   *
   * wheelBase
   * <--------->
   * O---------O
   * | |
   * | | trackWidth is the top down distance - from the "O" to the "O"
   * | |
   * | |
   * O---------O
   * <--------->
   *
   * /|
   * / | trackWidth/2
   * / |
   * / |
   * C | driveBaseRadius
   * \ |
   * \ | wheelBase/2
   * \ |
   * \|
   *
   * The line from 'C' to any 'O' represents the hypotenuse of a right-angled
   * triangle
   * formed by 'trackWidth / 2' and 'wheelBase / 2'. Using the Pythagorean
   * theorem, we
   * can calculate the length of the hypotenuse, which is the 'driveBaseRadius'.
   */
  public static final double driveBaseRadius = Math.hypot(trackWidth / 2, wheelBase / 2);
  public static final double wheelCircumference = Units.inchesToMeters(3.869) * Math.PI;

  // public static final double LEGACY_wheelCircumference =
  // chosenModule.wheelCircumference
  /*
   * Swerve Kinematics
   * Don't change if using traditional 4 module swerve
   */
  public static final SwerveDriveKinematics swerveKinematics =
      new SwerveDriveKinematics(
          new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
          new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
          new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
          new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));
  /* Module Gear Ratios */
  public static final double driveGearRatio = chosenModule.driveGearRatio;
  public static final double angleGearRatio = chosenModule.angleGearRatio;
  /* Motor Inverts */
  public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
  public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;
  /* Angle Encoder Invert */
  public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;
  /* Swerve Current Limiting */
  public static final int angleCurrentLimit = 25;
  public static final int angleCurrentThreshold = 40;
  public static final double angleCurrentThresholdTime = 0.1;
  public static final boolean angleEnableCurrentLimit = true;
  public static final int driveCurrentLimit = 50; // 35
  public static final int driveCurrentThreshold = 90;
  public static final double driveCurrentThresholdTime = 0.1;
  public static final boolean driveEnableCurrentLimit = true;
  /*
   * These values are used by the drive falcon to ramp in open loop and closed
   * loop driving.
   * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
   */
  public static final double openLoopRamp = 0.25;
  public static final double closedLoopRamp = 0.0;
  /* Swerve Profiling Values */
  public static final double maxTranslationalVelocity = 4.6; // TODO: Tune
  public static final double slowMaxTranslationalVelocity =
      maxTranslationalVelocity * 0.2; // TODO: Tune
  public static final double maxAngularVelocity = 6; // TODO: Tune
  public static final double slowMaxAngularVelocity = maxAngularVelocity * 0.5; // TODO: Tune
  /* Neutral Modes */
  public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
  public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;
  public static double crosshairAngleKP = 0.15;
  public static double crosshairAngleKI = 0;
  public static double crosshairAngleKD = 0.005;
  public static double crosshairTurnTolerance = 5;
  public static double crosshairTurnToleranceVel = 5;

  public static double translationNoteKP = 0.15;
  public static double translationNoteKI = 0;
  public static double translationNoteKD = 0.005;
  public static double translationNoteTolerance = 0;
  public static double translationNoteToleranceVel = 5;

  public static double strafeNoteKP = 0.15;
  public static double strafeNoteKI = 0;
  public static double strafeNoteKD = 0.005;
  public static double strafeNoteTolerance = 0;
  public static double strafeNoteToleranceVel = 5;

  public static final double kPixelToDegreesMagicNumber = 69;

  // We KNOW the robot is a square, so we can assert this. REMOVE THIS IF ROBOT
  // CHANGES!!!
  {
    assert (wheelBase == trackWidth);
  }

  /* Module Specific Constants */
  /* Front Left Module - Module 0 */
  public static final class Mod0 { // TODO: This must be tuned to specific robot
    public static final int driveMotorID = 1;
    public static final int angleMotorID = 8;
    public static final int canCoderID = 2;
    public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-154.072 + 180);
    public static final SwerveModuleConstants constants =
        new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
  }

  /* Front Right Module - Module 1 */
  public static final class Mod1 { // TODO: This must be tuned to specific robot
    public static final int driveMotorID = 6;
    public static final int angleMotorID = 7;
    public static final int canCoderID = 9;
    public static final Rotation2d angleOffset = Rotation2d.fromDegrees(97.470 + 180);
    public static final SwerveModuleConstants constants =
        new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
  }

  /* Back Left Module - Module 2 */
  public static final class Mod2 { // TODO: This must be tuned to specific robot
    public static final int driveMotorID = 2;
    public static final int angleMotorID = 4;
    public static final int canCoderID = 1;
    public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-132.85 + 180);
    public static final SwerveModuleConstants constants =
        new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
  }

  /* Back Right Module - Module 3 */
  public static final class Mod3 { // TODO: This must be tuned to specific robot
    public static final int driveMotorID = 5;
    public static final int angleMotorID = 0;
    public static final int canCoderID = 0;
    public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-149.45 + 180);
    public static final SwerveModuleConstants constants =
        new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
  }

  public static final class AzimuthConstants {
    // Quick assignments
    private static final double meow = (1 / 2);
    private static final double joshua = (Math.sqrt(3) / 2);
    private static final double abraham = 0.0;
    private static final double upputuri = 1.0;

    // AMP
    public static final double ampXRed = upputuri;
    public static final double ampXBlue = -upputuri;
    public static final double ampY = abraham;

    // SPEAKER
    public static final double subwooferFrontX = abraham;
    public static final double subwooferLeftX = -meow;
    public static final double subwooferRightX = meow;
    public static final double subwooferFrontY = -upputuri;
    public static final double subwooferSideY = -joshua;

    // SOURCE
    public static final double sourceXRed = -joshua;
    public static final double sourceXBlue = joshua;
    public static final double sourceY = meow;

    // Angles
    public static final double aziAmpRed = (Math.atan2(ampXRed, ampY) * 180 / Math.PI);
    public static final double aziAmpBlue = (Math.atan2(ampXBlue, ampY) * 180 / Math.PI);
    public static final double aziSubwooferFront =
        (Math.atan2(subwooferFrontX, subwooferFrontY) * 180 / Math.PI) + 180;
    public static final double aziSubwooferLeft =
        (Math.atan2(subwooferLeftX, subwooferSideY) * 180 / Math.PI) - 120;
    public static final double aziSubwooferRight =
        (Math.atan2(subwooferRightX, subwooferSideY) * 180 / Math.PI) + 120;
    public static final double aziSourceRed =
        (Math.atan2(sourceXRed, sourceY) * 180 / Math.PI) - 225;
    public static final double aziSourceBlue =
        (Math.atan2(sourceXBlue, sourceY) * 180 / Math.PI) - 225;
    public static final double test = (Math.atan2(ampXRed, ampY) * 180 / Math.PI);
    public static final double aziCommandTimeOut = 0.75;
  }
}
