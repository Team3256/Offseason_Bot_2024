// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.function.BiConsumer;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {


  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    RobotController.setBrownoutVoltage(5.6); // we ball
    // if (Constants.kOverrideBrownOutVoltage) {
    // RobotController.setBrownoutVoltage(Constants.kOverridenBrownOutVoltage);
    // System.out.println(
    // "*** Config set: new BrownOut voltage is "
    // + Constants.kOverridenBrownOutVoltage
    // + " (current battery voltage is "
    // + RobotController.getBatteryVoltage()
    // + ")");
    // if (Constants.kOverridenBrownOutVoltage <= 5.0) {
    // System.out.println("**** BROWNOUT VOLTAGE LESS THAN 5 VOLTS! DO NOT DO
    // THIS!".repeat(9));
    // }
    // }

    m_robotContainer = new RobotContainer();
    if (Constants.kEnableOBlog) {
      io.github.oblarg.oblog.Logger.configureLoggingAndConfig(m_robotContainer, false);
    } else {
      System.out.println("OBlog is disabled -- not configuring logging and config.");
    }
    // We enable the Logger here to start it directly after RobotContainer. Also
    // fixes
    // random issues.
    // Lucas: Must start logger after creating RobotContainer
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME); // Set a metadata value
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);

    if (isReal()) {
      System.out.println("Robot is real, forcing Robot mode to REAL");
      // Logger.addDataReceiver(new WPILOGWriter("/U/logs"));
      Logger.addDataReceiver(
          new WPILOGWriter("/home/lvuser/logs")); // Log to a USB stick ("/U/logs")
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
      new PowerDistribution(
          1, PowerDistribution.ModuleType.kRev); // Enables power distribution logging
    } else if (isSimulation()) {
      // DriverStation.silenceJoystickConnectionWarning(true);
      Logger.addDataReceiver(new WPILOGWriter(""));
      Logger.addDataReceiver(new NT4Publisher());
    } else {
      // Unknown mode
      Logger.addDataReceiver(new WPILOGWriter(""));
      Logger.addDataReceiver(new NT4Publisher());
    }

    // Logger.disableDeterministicTimestamps() // See "Deterministic Timestamps" in
    // the
    // "Understanding Data Flow" page

    // Log active commands
    Map<String, Integer> commandCounts = new HashMap<>();
    BiConsumer<Command, Boolean> logCommandFunction =
        (Command command, Boolean active) -> {
          String name = command.getName();
          int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
          commandCounts.put(name, count);

          Logger.recordOutput(
              "CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()), active);
          Logger.recordOutput("CommandsAll/" + name, count > 0);
        };
    CommandScheduler.getInstance()
        .onCommandInitialize(
            (Command command) -> {
              logCommandFunction.accept(command, true);
            });
    CommandScheduler.getInstance()
        .onCommandFinish(
            (Command command) -> {
              logCommandFunction.accept(command, false);
            });
    CommandScheduler.getInstance()
        .onCommandInterrupt(
            (Command command) -> {
              logCommandFunction.accept(command, false);
            });

    if (Constants.kEnableAdvKit) {
      Logger.start();
    }
    FollowPathCommand.warmupCommand().schedule();

    // FollowPathCommand warmupCommand = new FollowPathCommand();
    // warmupCommand.schedule();
    // Start logging! No more data receivers, replay sources, or
    // metadata values
    // may
    // be added.
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    if (Constants.kEnableOBlog) {
      io.github.oblarg.oblog.Logger.updateEntries();
    }
    m_robotContainer.periodic(Robot.defaultPeriodSecs);
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    if (Constants.kDisableSubsystemsOnDisableInit) {
      if (m_robotContainer.climb != null) {
        m_robotContainer.climb.off();
      }
      if (m_robotContainer.intake != null) {
        m_robotContainer.intake.off();
      }
      if (m_robotContainer.pivotIntake != null) {
        m_robotContainer.pivotIntake.off();
      }
      if (m_robotContainer.shooter != null) {
        m_robotContainer.shooter.off();
      }
      if (m_robotContainer.led != null) {
        m_robotContainer.led.reset();
      }
      if (m_robotContainer.ampbar != null) {
        m_robotContainer.ampbar.off();
      }
      m_robotContainer.disableRumble();
    }
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {;
//    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
//    // m_robotContainer.shootSpeaker();
//
//    // schedule the autonomous command (example)
//    if (m_autonomousCommand != null) {
//      System.out.println("hehe auto startie " + Timer.getFPGATimestamp());
//      m_autonomousCommand.schedule();
//    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
//    if (m_autonomousCommand != null) {
//      m_autonomousCommand.cancel();
//    }
    CommandScheduler.getInstance().cancelAll();

    boolean isRedAlliance = true;
    Optional<DriverStation.Alliance> ally = Optional.of(DriverStation.getAlliance().get());
    if (ally.get() == DriverStation.Alliance.Red) {
      isRedAlliance = true;
    }
    if (ally.get() == DriverStation.Alliance.Blue) {
      isRedAlliance = false;
    } else {
      isRedAlliance = true;
    }

    m_robotContainer.setAllianceCol(isRedAlliance);
    m_robotContainer.configureSwerve();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    // Run CAN Test
    // m_robotContainer.CANTest();
    m_robotContainer.runPitTestRoutine();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  @Override
  public void driverStationConnected() {
    // idk some config
  }
}
