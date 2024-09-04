// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.pivotshooter.PivotShooterConstants.*;
import static frc.robot.subsystems.swerve.AzimuthConstants.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.FeatureFlags;
import frc.robot.autos.routines.AutoRoutines;
import frc.robot.helpers.XboxStalker;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.ampbar.AmpBar;
import frc.robot.subsystems.ampbar.AmpBarIOTalonFX;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.led.commands.*;
import frc.robot.subsystems.pivotintake.PivotIntake;
import frc.robot.subsystems.pivotintake.PivotIntakeConstants;
import frc.robot.subsystems.pivotintake.PivotIntakeIOTalonFX;
import frc.robot.subsystems.pivotshooter.PivotShooter;
import frc.robot.subsystems.pivotshooter.PivotShooterIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveTelemetry;
import frc.robot.subsystems.swerve.TunerConstants;
import frc.robot.subsystems.swerve.kit.KitSwerveRequest;
import frc.robot.subsystems.swerve.requests.SwerveFieldCentricFacingAngle;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOLimelight;
import org.littletonrobotics.junction.Logger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private SendableChooser<Command> autoChooser;
  /* Controllers */
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);
  private final CommandXboxController test = new CommandXboxController(2);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int secondaryAxis = XboxController.Axis.kRightY.value;

  /* Subsystems */
  private boolean isRed;

  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;

  private double MaxSpeed =
      TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // My drivetrain

  private final KitSwerveRequest.FieldCentric drive =
      new KitSwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * Constants.stickDeadband)
          .withRotationalDeadband(
              MaxAngularRate * Constants.rotationalDeadband) // Add a 10% deadband
          .withDriveRequestType(
              SwerveModule.DriveRequestType.OpenLoopVoltage); // I want field-centric

  private SwerveFieldCentricFacingAngle azi =
      new SwerveFieldCentricFacingAngle()
          .withDeadband(MaxSpeed * .1) // TODO: update deadband
          .withRotationalDeadband(MaxAngularRate * .1) // TODO: update deadband
          .withHeadingController(SwerveConstants.azimuthController)
          .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

  // driving in open loop
  private final SwerveTelemetry swerveTelemetry = new SwerveTelemetry(MaxSpeed);
  public Shooter shooter = new Shooter(FeatureFlags.kShooterEnabled, new ShooterIOTalonFX());
  public Intake intake = new Intake(FeatureFlags.kIntakeEnabled, new IntakeIOTalonFX());
  public AmpBar ampbar = new AmpBar(FeatureFlags.kAmpBarEnabled, new AmpBarIOTalonFX());
  public PivotIntake pivotIntake =
      new PivotIntake(FeatureFlags.kPivotIntakeEnabled, new PivotIntakeIOTalonFX());
  public Climb climb = new Climb(FeatureFlags.kClimbEnabled, new ClimbIOTalonFX());
  public Vision vision = new Vision(new VisionIOLimelight());
  public PivotShooter pivotShooter =
      new PivotShooter(FeatureFlags.kPivotShooterEnabled, new PivotShooterIOTalonFX());
  public LED led = new LED();

  public Superstructure superstructure =
      new Superstructure(ampbar, climb, intake, pivotIntake, pivotShooter, shooter, vision);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Cancel any previous commands running
    CommandScheduler.getInstance().cancelAll();
    test.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
    test.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));

    test.leftTrigger()
        .onTrue(
            drivetrain.applyRequest(() -> azi.withTargetDirection(Rotation2d.fromDegrees(180))));

    test.y().whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    test.a().whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    test.b().whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
    test.x().whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    /* Run checks */
    configureCheeks();

    configureAutos();
    configureBindings();
  }

  private void configureAutos() {
    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption("Do Nothing", new InstantCommand());
    autoChooser.addOption(
        "5 Note test",
        AutoRoutines.center5Note(drivetrain, intake, shooter, pivotShooter, pivotIntake));
    // Autos
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {
    // Operator
    operator.a().onTrue(superstructure.setState(Superstructure.StructureState.PREINTAKE));
    operator.x().onTrue(superstructure.setState(Superstructure.StructureState.PRESUB));
    operator.b().onTrue(superstructure.setState(Superstructure.StructureState.PREPODIUM));
    operator.y().onTrue(superstructure.setState(Superstructure.StructureState.HOMED));
    operator.povLeft().onTrue(pivotIntake.homePosition());
    operator
        .povRight()
        .onTrue(
            pivotIntake.setPosition(
                PivotIntakeConstants.kPivotGroundPos * PivotIntakeConstants.kPivotMotorGearing));
    operator.povDown().onTrue(pivotShooter.homePosition());
    operator.povUp().onTrue(superstructure.setState(Superstructure.StructureState.PREFEED));
    operator.rightTrigger().and(operator.leftTrigger()).whileTrue(pivotIntake.manualZero());
    operator.rightBumper().and(operator.leftBumper()).whileTrue(pivotShooter.manualZero());
    operator.leftTrigger().onTrue(superstructure.setState(Superstructure.StructureState.PREAMP));
    operator
        .rightTrigger()
        .onTrue(
            shooter.setVelocity(
                ShooterConstants.kShooterSubwooferRPS,
                ShooterConstants.kShooterFollowerSubwooferRPS));
    operator
        .leftBumper()
        .whileTrue(superstructure.setState(Superstructure.StructureState.OUTTAKE))
        .toggleOnFalse(superstructure.setState(Superstructure.StructureState.IDLE));
    operator.rightBumper().onTrue(superstructure.setState(Superstructure.StructureState.SHOOT));
    operator
        .rightTrigger()
        .and(operator.leftBumper())
        .or(operator.leftTrigger().and(operator.rightBumper()))
        .onTrue(
            intake.setVoltage(
                IntakeConstants.kIntakeIntakeVoltage, IntakeConstants.kPassthroughIntakeVoltage));
    operator
        .axisGreaterThan(translationAxis, 0.5)
        .onTrue(superstructure.setState(Superstructure.StructureState.PRECLIMB));

    // Intake / outtake overrides
    driver
        .x()
        .whileTrue(
            intake.setVoltage(
                IntakeConstants.kIntakeIntakeVoltage, IntakeConstants.kPassthroughIntakeVoltage));
    driver
        .b()
        .whileTrue(
            intake.setVoltage(
                IntakeConstants.kIntakeIntakeVoltage, -IntakeConstants.kPassthroughIntakeVoltage));
    //    driver.rightTrigger().whileTrue(intake.intakeIn());

    // operator.povDown().onTrue(new IntakeOff(intake));

    operator
        .axisLessThan(translationAxis, -0.5)
        .onTrue(superstructure.setState(Superstructure.StructureState.CLIMB));

    Trigger noteInPassthrough = new Trigger(() -> intake.isBeamBroken());
    noteInPassthrough
        .onTrue(
            Commands.run(() -> operator.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 1)))
        .toggleOnFalse(
            Commands.run(() -> operator.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0)));
  }

  public void setAllianceCol(boolean col) {
    isRed = col;
  }

  public void configureSwerve() {

    // default command
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with
                    // negative Y (forward)
                    .withVelocityY(
                        -driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate)));

    /* Right stick absolute angle mode on trigger hold,
    robot adjusts heading to the angle right joystick creates */
    driver
        .rightTrigger()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    drive
                        .withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with
                        // negative Y (forward)
                        .withVelocityY(
                            -driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-driver.getRightX() * MaxAngularRate)));

    azi.withTargetDirection(new Rotation2d(driver.getRightX(), driver.getRightY()));

    // Slows translational and rotational speed to 30%
    driver
        .leftTrigger()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    drive
                        .withVelocityX(-driver.getLeftY() * (MaxSpeed * 0.3))
                        .withVelocityY(-driver.getLeftX() * (MaxSpeed * 0.3))
                        .withRotationalRate(
                            -driver.getLeftTriggerAxis() * (MaxAngularRate * 0.3))));

    // Reset robot heading on button press
    driver.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    // Azimuth angle bindings. isRed == true for red alliance presets. isRed != true for blue.
    if (isRed == true) {
      driver
          .rightBumper()
          .whileTrue(drivetrain.applyRequest(() -> azi.withTargetDirection(aziSourceRed)));
      driver.a().whileTrue(drivetrain.applyRequest(() -> azi.withTargetDirection(aziAmpRed)));
      driver
          .povRight()
          .whileTrue(drivetrain.applyRequest(() -> azi.withTargetDirection(aziFeederRed)));
    } else {
      driver
          .rightBumper()
          .whileTrue(drivetrain.applyRequest(() -> azi.withTargetDirection(aziSourceBlue)));
      driver.a().whileTrue(drivetrain.applyRequest(() -> azi.withTargetDirection(aziAmpBlue)));
      driver
          .povRight()
          .whileTrue(drivetrain.applyRequest(() -> azi.withTargetDirection(aziFeederBlue)));
    }

    // Universal azimuth bindings
    driver
        .leftBumper()
        .whileTrue(drivetrain.applyRequest(() -> azi.withTargetDirection(aziSubwooferFront)));
    driver
        .povDownLeft()
        .whileTrue(drivetrain.applyRequest(() -> azi.withTargetDirection(aziSubwooferLeft)));
    driver
        .povDownRight()
        .whileTrue(drivetrain.applyRequest(() -> azi.withTargetDirection(aziSubwooferRight)));
    driver.povDown().whileTrue(drivetrain.applyRequest(() -> azi.withTargetDirection(aziCleanUp)));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(swerveTelemetry::telemeterize);
  }

  // operator.x().onTrue(new AutoScoreAmp(swerveDrive, shooter, intake));
  // operator.b().onTrue(new AutoScoreSpeaker(swerveDrive, shooter, intake));
  // }

  public void disableRumble() {
    driver.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
    operator.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
  }

  public void configureLED() {
    int[][] ledList = new int[][] {new int[] {2, 3}, new int[] {1, 1}};

    led = new LED();
    led.setDefaultCommand(new CoordinatesButItsMultiple(led, ledList, 100, 0, 0, 10));
  }

  private void configureCheeks() {
    // Here, put checks for the configuration of the robot
    if (DriverStation.isFMSAttached()) {
      // FMS-attached checks
      if (!FeatureFlags.kSwerveEnabled) {
        // Swerve is disabled, but the robot is FMS-attached. this *probably* shouldn't
        // happen!
        System.out.println(
            "Swerve is disabled, but the robot is FMS-attached. This probably shouldn't happen!");

        DriverStation.reportError(
            "Swerve is disabled, but the robot is FMS-attached. This probably shouldn't happen!",
            false);
        // You can forcibly disable the robot by throwing an exception here, or by
        // calling
        // DriverStationJNI.observeUserProgramDisabled();
      }
      if (FeatureFlags.kDebugEnabled) {
        System.err.println(
            "Robot is FMS-attached & debug mode is enabled. This is not recommended!");
        DriverStation.reportWarning(
            "Robot is FMS-attached & debug mode is enabled. This is not recommended.", false);
      }
    } else {

    }
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void periodic(double dt) {
    XboxStalker.stalk(driver, operator);
    Logger.recordOutput("Note pose", vision.getNotePose(drivetrain.getState().Pose));
    superstructure.periodic();
  }
}
