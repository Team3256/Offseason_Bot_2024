// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj.RobotBase.isReal;
import static frc.robot.Constants.azimuthStickDeadband;
import static frc.robot.subsystems.pivotintake.PivotIntakeConstants.kPivotGroundAngleDeg;
import static frc.robot.subsystems.pivotshooter.PivotShooterConstants.*;
import static frc.robot.subsystems.swerve.SwerveConstants.AzimuthConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.FeatureFlags;
import frc.robot.autos.commands.AutoScoreAmp;
import frc.robot.autos.commands.AutoScoreSpeaker;
import frc.robot.autos.commands.IntakeSequence;
import frc.robot.autos.commands.MoveToNote;
import frc.robot.autos.commands.RotateToNote;
import frc.robot.commands.PitRoutine;
import frc.robot.helpers.XboxStalker;
import frc.robot.limelight.Limelight;
import frc.robot.limelight.LimelightHelpers;
import frc.robot.robotviz.RobotViz;
import frc.robot.subsystems.ampbar.AmpBar;
import frc.robot.subsystems.ampbar.commands.AmpPosition;
import frc.robot.subsystems.ampbar.commands.StowPosition;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.commands.DehookClimb;
import frc.robot.subsystems.climb.commands.DownClimb;
import frc.robot.subsystems.climb.commands.UpClimb;
import frc.robot.subsystems.climb.commands.ZeroClimb;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.*;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.led.LEDConstants;
import frc.robot.subsystems.led.commands.*;
import frc.robot.subsystems.pivotintake.PivotIntake;
import frc.robot.subsystems.pivotintake.PivotIntakeConstants;
import frc.robot.subsystems.pivotintake.commands.PivotIntakeSetAngle;
import frc.robot.subsystems.pivotintake.commands.PivotIntakeSlamAndVoltage;
import frc.robot.subsystems.pivotintake.commands.PivotIntakeZero;
import frc.robot.subsystems.pivotshooter.PivotShooter;
import frc.robot.subsystems.pivotshooter.commands.*;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.ShootAmp;
import frc.robot.subsystems.shooter.commands.ShootSpeaker;
import frc.robot.subsystems.shooter.commands.ShootSubwoofer;
import frc.robot.subsystems.shooter.commands.ShooterOff;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.commands.*;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import java.util.ArrayList;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /* Controllers */
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);
  // private final CommandXboxController tester = new CommandXboxController(2);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;
  private final int secondaryAxis = XboxController.Axis.kRightY.value;

  /* Swerve Helpers */
  private boolean isRed = true;

  /* Subsystems */
  @Log
  @Config
  public SwerveDrive swerveDrive;
  @Log
  @Config
  public Shooter shooter;
  @Log
  @Config
  public Intake intake;
  @Log
  @Config
  public AmpBar ampbar;
  @Log
  @Config
  public PivotIntake pivotIntake;
  @Log
  @Config
  public Climb climb;

  @Log
  @Config
  public PivotShooter pivotShooter;
  @Log
  @Config
  public LED led;

  @Config.Command(name = "Auto Score Speaker")
  private Command autoScoreSpeaker;

  @Config.Command(name = "Auto Score Amp")
  private Command autoScoreAmp;

  @Config.Command(name = "Climb Zero")
  private Command zeroClimb;

  /* Auto */
  private SendableChooser<Command> autoChooser;

  /* Viz */
  private RobotViz robotViz;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Cancel any previous commands running
    CommandScheduler.getInstance().cancelAll();

    // Setup subsystems & button-bindings
    if (FeatureFlags.kPivotShooterEnabled) {
      configurePivotShooter();
    }
    if (FeatureFlags.kShooterEnabled) {
      configureShooter();
    }
    if (FeatureFlags.kPivotEnabled) {
      configurePivot();
    }
    if (FeatureFlags.kIntakeEnabled) {
      configureIntake();
    }
    if (FeatureFlags.kSwerveEnabled) {
      configureSwerve();
    }
    if (FeatureFlags.kClimbEnabled) {
      configureClimb();
    }
    // If all the subsystems are enabled, configure "operator" autos
    if (FeatureFlags.kIntakeEnabled && FeatureFlags.kShooterEnabled && FeatureFlags.kPivotEnabled) {
      configureOperatorAutos();
    }

    if (FeatureFlags.kLEDEnabled) {
      configureLED();
    }

    if (FeatureFlags.kSwerveEnabled
        && FeatureFlags.kIntakeEnabled
        && FeatureFlags.kShooterEnabled
        && FeatureFlags.kPivotEnabled) {
      autoScoreSpeaker = new AutoScoreSpeaker(swerveDrive, shooter, intake);
      autoScoreAmp = new AutoScoreAmp(swerveDrive, shooter, intake);
    }
    // Named commands
    {
      // Auto named commands
      NamedCommands.registerCommand("test intake", new IntakeSetVoltage(intake, 12).withTimeout(1));
      // NamedCommands.registerCommand( // intake ground note, stow to feeder chamber
      // "intake sequence new",
      // new SequentialCommandGroup(
      // new ParallelCommandGroup(
      // new PivotSetAngle(pivotIntake, PivotConstants.kPivotGroundAngleDeg)
      // .withTimeout(0.75),
      // new IntakeIn(intake)),
      // new ParallelCommandGroup(
      // new PivotSlamAndVoltage(pivotIntake).withTimeout(0.75),
      // new ScheduleCommand(new ShootSpeaker(shooter)))));

      NamedCommands.registerCommand( // shoot preloaded note to speaker, use at match start
          "preload speaker",
          new SequentialCommandGroup(
              new PivotShooterZero(pivotShooter),
              new ParallelDeadlineGroup(
                  new SequentialCommandGroup(
                      new WaitCommand(0.6), // TODO: maybe need to tune this too
                      new IntakeAndPassthroughButItEnds(intake)), // TODO: tune time in withTimeout
                  new ShootSubwoofer(shooter),
                  new PivotShootSubwoofer(pivotShooter))
          // new PivotShooterSlamAndVoltage(pivotShooter)));
          ));
      NamedCommands.registerCommand( // intake ground note, stow to feeder chamber
          "intake sequence",
          new ParallelCommandGroup(
              new PivotIntakeSetAngle(pivotIntake, PivotIntakeConstants.kPivotGroundAngleDeg),
              new IntakeIn(intake),
              // new PivotShooterSlamAndVoltage(pivotShooter),
              // new PivotShootSubwoofer(pivotShooter),
              new ShootSubwoofer(shooter)));
      NamedCommands.registerCommand( // outtake note to feeder
          "outtake speaker",
          new SequentialCommandGroup(
              // new ScheduleCommand(new PivotShootSubwoofer(pivotShooter)).asProxy(),
              new ParallelCommandGroup(
                  new IntakeInOverride(intake).withTimeout(2),
                  new ShootSubwoofer(shooter)))); // TODO: tune time in withTimeout
      NamedCommands.registerCommand("aim subwoofer", new PivotShootSubwoofer(pivotShooter));

      NamedCommands.registerCommand( // outtake note to feeder
          "safety",
          new ParallelCommandGroup(
              new IntakeIn(intake).withTimeout(1),
              new ShootAmp(shooter))); // TODO: tune time in withTimeout
      NamedCommands.registerCommand(
          "aim wing center", new PivotShooterSetAngle(pivotShooter, kWingNoteCenterPreset));
      NamedCommands.registerCommand(
          "aim wing side", new PivotShooterSetAngle(pivotShooter, kWingNoteSidePreset));
      NamedCommands.registerCommand(
          "aim wing far side", new PivotShooterSetAngle(pivotShooter, kWingNoteFarSidePreset));
      NamedCommands.registerCommand(
          "aim truss", new PivotShooterSetAngle(pivotShooter, kTrussSourceSidePreset));
      NamedCommands.registerCommand(
          "aim half truss wing", new PivotShooterSetAngle(pivotShooter, kHalfWingPodiumPreset));
      NamedCommands.registerCommand(
          "zero pivot shooter", new PivotShooterSlamAndVoltage(pivotShooter));

      NamedCommands.registerCommand( // rev shooter to speaker presets
          "rev speaker", new ShootSpeaker(shooter));
      NamedCommands.registerCommand( // rev shooter to amp presets
          "rev amp", new ShootAmp(shooter));
      NamedCommands.registerCommand( // modular pivot down, use for sabotage
          "pivot down",
          new PivotIntakeSetAngle(pivotIntake, kPivotGroundAngleDeg).withTimeout(0.75));
      NamedCommands.registerCommand(
          "stow", new PivotIntakeSlamAndVoltage(pivotIntake).withTimeout(0.75));
      NamedCommands.registerCommand( // intake with no stow, use for sabotage
          "intake", new IntakeIn(intake));
      NamedCommands.registerCommand( // shoot preloaded note to amp, use at match start
          "preload amp",
          new SequentialCommandGroup(
              new PivotIntakeZero(pivotIntake),
              new ParallelDeadlineGroup(
                  new SequentialCommandGroup(
                      new WaitCommand(0.8), // TODO: maybe need to tune this too
                      new IntakeOut(intake).withTimeout(1.5)), // TODO: tune time in withTimeout
                  new ShootAmp(shooter))));
      NamedCommands.registerCommand(
          "scheduled shoot speaker", new ScheduleCommand(new ShootSpeaker(shooter)));
      NamedCommands.registerCommand(
          "align to note",
          new SequentialCommandGroup(
              new RotateToNote(swerveDrive), new MoveToNote(swerveDrive, intake)));
      NamedCommands.registerCommand(
          "lmao",
          new RepeatCommand(
              new SequentialCommandGroup(
                  new PivotIntakeSetAngle(pivotIntake, kPivotGroundAngleDeg).withTimeout(0.75),
                  new PivotIntakeSlamAndVoltage(pivotIntake))));
    }

    /* Run checks */
    configureCheeks();

    // operator.povLeft().onTrue(cancelCommand);
    // Configure the auto
    if (FeatureFlags.kSwerveEnabled) {
      autoChooser = AutoBuilder.buildAutoChooser();
    } else {
      autoChooser = new SendableChooser<>();
    }
    // Autos
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // 2D visualization
    if (FeatureFlags.kRobotVizEnabled) {
      if (FeatureFlags.kDebugEnabled) {
        System.out.println("Robot Viz Enabled");
      }
      robotViz = new RobotViz(swerveDrive, shooter, intake, pivotIntake);
    }
  }

  private void configurePivotShooter() {
    pivotShooter = new PivotShooter();
    // operator.b().onTrue(new bruh(pivotShooter));
    // operator.x().onTrue(new SequentialCommandGroup(new
    // PivotShootSubwoofer(pivotShooter)));
    operator.povUp().onTrue(new PivotShooterZero(pivotShooter));
  }

  private void configureIntake() {
    intake = new Intake();
    // intake.setDefaultCommand(new IntakeSetVoltage(intake, 0));
    // operator.rightBumper().whileTrue(new IntakeInOverride(intake));
    // We assume intake is already enabled, so if pivot is enabled as
    // use IntakeOutWithArm
    operator.rightBumper().whileTrue(new IntakeAndPassthrough(intake));
    if (FeatureFlags.kPivotEnabled) {
      operator.leftBumper().whileTrue(new IntakeOutArmOff(intake, pivotIntake));
      driver.rightTrigger().whileTrue(new IntakeOutArmOff(intake, pivotIntake));
    } else {
      operator.leftBumper().whileTrue(new IntakeOut(intake));
      driver.rightTrigger().whileTrue(new IntakeOut(intake));
    }

    // operator.povDown().onTrue(new IntakeOff(intake));
  }

  private void configurePivot() {
    pivotIntake = new PivotIntake();
    operator.povRight().onTrue(new PivotIntakeSetAngle(pivotIntake, kPivotGroundAngleDeg));
    operator.povLeft().onTrue(new PivotIntakeSlamAndVoltage(pivotIntake));
  }

  private void configureClimb() {
    climb = new Climb();
    // zeroClimb = new ZeroClimb(climb); // NEED FOR SHUFFLEBOARD
    operator.povDown().onTrue(new ZeroClimb(climb));
    // new Trigger(() -> operator.getRawAxis(translationAxis) < -0.5).onTrue(new
    // UpClimb(climb));
    new Trigger(() -> operator.getRawAxis(translationAxis) > 0.5).onTrue(new DownClimb(climb));
    if (this.ampbar != null && this.pivotShooter != null) {
      new Trigger(() -> operator.getRawAxis(translationAxis) < -0.5)
          .onTrue(
              Commands.sequence(
                  new ParallelCommandGroup(
                      new AmpPosition(ampbar),
                      new PivotShooterSetAngle(pivotShooter, 12 / 138.33))
                      .withTimeout(1),
                  new UpClimb(climb)));
    } else {
      new Trigger(() -> Math.abs(operator.getRawAxis(secondaryAxis)) > 0.5)
          .onTrue(new DehookClimb(climb));
    }
    // Josh: HangSequence is broken and Rhea does not want to use it; we should
    // rmove this
    // later.
    // new ScheduleCommand(new HangSequence(climb, operator)).schedule();
    // operator.povDownLeft().onTrue(new TestClimbFlip(climb));
  }

  private void configureSwerve() {
    swerveDrive = new SwerveDrive();
    operator.b().whileTrue(new StrafeNoteTuner(swerveDrive, true, false));
    operator.x().whileTrue(new TranslationNoteTuner(swerveDrive, true, false));

    swerveDrive.setDefaultCommand(
        new TeleopSwerve(
            swerveDrive,
            () -> driver.getRawAxis(translationAxis),
            () -> driver.getRawAxis(strafeAxis),
            () -> driver.getRawAxis(rotationAxis)));
    driver
        .leftTrigger()
        .whileTrue(
            new TeleopSwerveLimited(
                swerveDrive,
                () -> driver.getRawAxis(translationAxis),
                () -> driver.getRawAxis(strafeAxis),
                () -> driver.getRawAxis(rotationAxis)));

    // driver.povDown().whileTrue(new EjectNote(swerveDrive, pivotIntake, intake));
    driver
        .leftBumper()
        .whileTrue(
            new NoMoreRotation(
                swerveDrive,
                () -> driver.getRawAxis(translationAxis),
                () -> driver.getRawAxis(strafeAxis),
                true,
                true));

    driver.y().onTrue(new ZeroGyro(swerveDrive));
    driver.y().onTrue(new ForceResetModulePositions(swerveDrive));

    Command aziRed = new InstantCommand(() -> isRed = true);
    Command aziBlue = new InstantCommand(() -> isRed = false);

    driver.povLeft().onTrue(aziRed);
    driver.povRight().onTrue(aziBlue);

    if (isRed) /* RED ALLIANCE PRESETS */ {
      driver // AMP
          .a()
          .onTrue(
              new Azimuth(
                  swerveDrive,
                  driver::getLeftY,
                  driver::getLeftX,
                  () -> azimuthStickDeadband + 0.1,
                  () -> azimuthStickDeadband + 0.1,
                  () -> aziAmpRed,
                  () -> true,
                  true,
                  true)
                  .withTimeout(aziCommandTimeOut));
      driver // SUBWOOFER FRONT
          .povDown()
          .onTrue(
              new Azimuth(
                  swerveDrive,
                  driver::getLeftY,
                  driver::getLeftX,
                  () -> azimuthStickDeadband + 0.1,
                  () -> azimuthStickDeadband + 0.1,
                  () -> aziSubwooferFront,
                  () -> true,
                  true,
                  true)
                  .withTimeout(aziCommandTimeOut));
      driver // SUBWOOFER RIGHT
          .b()
          .onTrue(
              new Azimuth(
                  swerveDrive,
                  driver::getLeftY,
                  driver::getLeftX,
                  () -> azimuthStickDeadband + 0.1,
                  () -> azimuthStickDeadband + 0.1,
                  () -> aziSubwooferRight,
                  () -> true,
                  true,
                  true)
                  .withTimeout(aziCommandTimeOut));
      driver // SUBWOOFER LEFT
          .x()
          .onTrue(
              new Azimuth(
                  swerveDrive,
                  driver::getLeftY,
                  driver::getLeftX,
                  () -> azimuthStickDeadband + 0.1,
                  () -> azimuthStickDeadband + 0.1,
                  () -> aziSubwooferLeft,
                  () -> true,
                  true,
                  true)
                  .withTimeout(aziCommandTimeOut));
      driver // SOURCE
          .rightBumper()
          .onTrue(
              new Azimuth(
                  swerveDrive,
                  driver::getLeftY,
                  driver::getLeftX,
                  () -> azimuthStickDeadband + 0.1,
                  () -> azimuthStickDeadband + 0.1,
                  () -> aziSourceRed,
                  () -> true,
                  true,
                  true)
                  .withTimeout(aziCommandTimeOut));

    } else /* BLUE ALLIANCE PRESETS */ {
      driver // AMP
          .a()
          .onTrue(
              new Azimuth(
                  swerveDrive,
                  driver::getLeftY,
                  driver::getLeftX,
                  () -> azimuthStickDeadband + 0.1,
                  () -> azimuthStickDeadband + 0.1,
                  () -> aziAmpBlue,
                  () -> true,
                  true,
                  true)
                  .withTimeout(aziCommandTimeOut));
      driver // SUBWOOFER FRONT
          .povDown()
          .onTrue(
              new Azimuth(
                  swerveDrive,
                  driver::getLeftY,
                  driver::getLeftX,
                  () -> azimuthStickDeadband + 0.1,
                  () -> azimuthStickDeadband + 0.1,
                  () -> aziSubwooferFront,
                  () -> true,
                  true,
                  true)
                  .withTimeout(aziCommandTimeOut));
      driver // SUBWOOFER RIGHT
          .b()
          .onTrue(
              new Azimuth(
                  swerveDrive,
                  driver::getLeftY,
                  driver::getLeftX,
                  () -> azimuthStickDeadband + 0.1,
                  () -> azimuthStickDeadband + 0.1,
                  () -> aziSubwooferRight,
                  () -> true,
                  true,
                  true)
                  .withTimeout(aziCommandTimeOut));
      driver // SUBWOOFER LEFT
          .x()
          .onTrue(
              new Azimuth(
                  swerveDrive,
                  driver::getLeftY,
                  driver::getLeftX,
                  () -> azimuthStickDeadband + 0.1,
                  () -> azimuthStickDeadband + 0.1,
                  () -> aziSubwooferLeft,
                  () -> true,
                  true,
                  true)
                  .withTimeout(aziCommandTimeOut));
      driver // SOURCE
          .rightBumper()
          .onTrue(
              new Azimuth(
                  swerveDrive,
                  driver::getLeftY,
                  driver::getLeftX,
                  () -> azimuthStickDeadband + 0.1,
                  () -> azimuthStickDeadband + 0.1,
                  () -> aziSourceBlue,
                  () -> true,
                  true,
                  true)
                  .withTimeout(aziCommandTimeOut));
    }

    /* Tester bind */
    driver
        .povUp()
        .onTrue(
            new Azimuth(
                swerveDrive,
                driver::getLeftY,
                driver::getLeftX,
                () -> azimuthStickDeadband + 0.1,
                () -> azimuthStickDeadband + 0.1,
                () -> test,
                () -> true,
                true,
                true)
                .withTimeout(aziCommandTimeOut));
  }

  private void configureShooter() {
    if (isReal()) {
      shooter = new Shooter();
    } else {
      shooter = new Shooter();
      shooter = new Shooter();
    }
    // new Trigger(() -> Math.abs(shooter.getShooterRps() - 100) <= 5)
    // .onTrue(
    // new InstantCommand(
    // () -> {
    // operator.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 100);
    // }))
    // .onFalse(
    // new InstantCommand(
    // () -> {
    // operator.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
    // }));
    if (FeatureFlags.kAmpBarEnabled) {
      ampbar = new AmpBar();
      operator
          .rightTrigger()
          .onTrue(Commands.parallel(new ShootSpeaker(shooter), new StowPosition(ampbar)));
      operator
          .leftTrigger()
          .onTrue(
              new ParallelCommandGroup(
                  new ShootAmp(shooter),
                  new AmpPosition(ampbar),
                  new PivotShooterAmp(pivotShooter)));
      operator
          .y()
          .onTrue(
              new ParallelCommandGroup(
                  new ShooterOff(shooter),
                  new StowPosition(ampbar),
                  new PivotShooterSlamAndVoltage(pivotShooter)));
    } else {
      operator.rightTrigger().onTrue(new ShootSpeaker(shooter));
      operator.leftTrigger().onTrue(new ShootAmp(shooter));
      operator
          .y()
          .onTrue(
              new ParallelCommandGroup(
                  new ShooterOff(shooter), new PivotShooterSlamAndVoltage(pivotShooter)));
    }
  }

  private void configureOperatorAutos() {
    operator.a().onTrue(new IntakeSequence(intake, pivotIntake, pivotShooter, shooter, ampbar));
    // operator.x().onTrue(new AutoScoreAmp(swerveDrive, shooter, intake));
    // operator.b().onTrue(new AutoScoreSpeaker(swerveDrive, shooter, intake));
  }

  public void disableRumble() {
    driver.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
    operator.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
  }

  public void configureLED() {
    ArrayList<int[]> ledList = new ArrayList<int[]>();
    ledList.add(new int[] { 2, 4 });
    ledList.add(new int[] { 6, 9 });

    led = new LED();
    led.setDefaultCommand(new SetLEDsFromBinaryString(led, LEDConstants.based));

    /*
     * Intake LED, flashes RED while intake is down and running,
     * flashes GREEN on successful intake
     */
    if (FeatureFlags.kIntakeEnabled) {
      Trigger intakeDetectedNote = new Trigger(intake::isBeamBroken);
      // intakeDetectedNote.whileTrue(new SetSuccessfulIntake(led));

      intakeDetectedNote
          .onTrue(
              new InstantCommand(
                  () -> {
                    operator.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 50);
                  }))
          .onFalse(
              new InstantCommand(
                  () -> {
                    operator.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
                  }));
      intakeDetectedNote
          .onTrue(
              new InstantCommand(
                  () -> {
                    driver.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 50);
                  }))
          .onFalse(
              new InstantCommand(
                  () -> {
                    driver.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
                  }));

      // This boolean is true when velocity is LESS than 0.
      // Trigger intakeRunning = new Trigger(intake::isMotorSpinning);
      // intakeRunning.whileTrue(new SetGroundIntakeRunning(led));
    }

    /*
     * Shooter LED, solid ORANGE while shooter is running, flashes ORANGE if note
     * fed
     */
    // if (FeatureFlags.kShooterEnabled) {

    // Trigger shooterRunning = new Trigger(
    // () -> (shooter.getShooterFollowerRps() > 75 || shooter.getShooterRps() >
    // 75));
    // shooterRunning.whileTrue(new SetSpeakerScore(led));
    // }
    // if (FeatureFlags.kSwerveEnabled) {
    // if (DriverStation.isAutonomousEnabled() ||
    // FeatureFlags.kSwerveUseVisionForPoseEst) {
    // Trigger swerveSpeakerAligned = new Trigger(swerveDrive::isAlignedToSpeaker);
    // swerveSpeakerAligned.whileTrue(new SetRobotAligned(led));
    // }
    // if (DriverStation.isTeleopEnabled()) {
    // // if (DriverStation.isTeleopEnabled()) {
    // // Trigger azimuthRan =
    // // new Trigger(
    // // () ->
    // // (driver.leftBumper().getAsBoolean() ||
    // driver.rightBumper().getAsBoolean())
    // // || driver.x().getAsBoolean()
    // // || driver.b().getAsBoolean()
    // // || driver.a().getAsBoolean()
    // // || driver.povUp().getAsBoolean());
    // // azimuthRan.whileTrue(new SetAzimuthRan(led));
    // // }
    // }
    // if (FeatureFlags.kClimbEnabled) {
    // Trigger climbRunning =
    // new Trigger(
    // () -> ((climb.getLeftCurrent() > 0) || (operator.povDown().getAsBoolean())));
    // }
    // }
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

  private void systemCheeks() {
    if (FeatureFlags.kSwerveEnabled && swerveDrive != null) {
      swerveDrive.CANTest();
    }
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  /* Test Routines */
  public boolean CANTest() {
    return swerveDrive.CANTest();
  }

  public void runPitTestRoutine() {
    Command pitRoutine = new PitRoutine(swerveDrive, climb, intake, pivotIntake, shooter);
    pitRoutine.schedule();
  }

  public void ccccccc() {
    Command shoot = new ShootSpeaker(shooter);
    shoot.schedule();
  }

  public void periodic(double dt) {
    if (FeatureFlags.kRobotVizEnabled) {
      robotViz.update(dt);
    }
    XboxStalker.stalk(driver, operator);
    // System.out.println(Limelight.getBotpose("limelight").length);

    double ty = Limelight.getTY("limelight");

    // how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees = 21.936;

    // distance from the center of the Limelight lens to the floor
    double limelightLensHeightInches = 15.601;

    // distance from the target to the floor
    double goalHeightInches = 56.375;

    double angleToGoalDegrees = limelightMountAngleDegrees + ty;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    // calculate distance le distanceFromLimelightToGoalInches =

    // (goalHeightInches - limelightLensHeightInches) /
    // Math.tan(angleToGoalRadians);
    LimelightHelpers.setPriorityTagID("limelight", 4);
    // System.out.println("Distance: " + distanceFromLimelightToGoalInches);
  }

  public void shootSpeaker() {
    Command shoot = new ScheduleCommand(new ShootSpeaker(shooter));
    shoot.schedule();
  }
}
