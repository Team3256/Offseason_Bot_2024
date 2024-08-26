package frc.robot.subsystems.swerve.commands;

import static frc.robot.Constants.stickDeadband;
import static frc.robot.subsystems.swerve.SwerveConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.helpers.DebugCommandBase;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Azimuth extends DebugCommandBase {
  private Translation2d translation;
  private boolean fieldRelative;
  private boolean openLoop;

  private CommandSwerveDrivetrain swerveSubsystem;
  private DoubleSupplier translationAxis;
  private DoubleSupplier strafeAxis;
  private DoubleSupplier setpointAngle;
  private BooleanSupplier manualRotating;
  private PIDController azimuthController;

  /** Driver control */
  public Azimuth(
      CommandSwerveDrivetrain swerveSubsystem,
      DoubleSupplier translationAxis,
      DoubleSupplier strafeAxis,
      DoubleSupplier setpointAngle,
      BooleanSupplier manualRotating,
      boolean fieldRelative,
      boolean openLoop) {
    this.swerveSubsystem = swerveSubsystem;
    addRequirements(swerveSubsystem);

    this.translationAxis = translationAxis;
    this.strafeAxis = strafeAxis;
    this.setpointAngle = setpointAngle;
    this.manualRotating = manualRotating;
    this.fieldRelative = fieldRelative;
    this.openLoop = openLoop;
    this.azimuthController = new PIDController(0, 0, 0);
    azimuthController.enableContinuousInput(-180, 180);
    azimuthController.setTolerance(5, 5);
  }

  @Override
  public void execute() {
    // Obtains axis values for x and y for translation command
    double yAxis = -translationAxis.getAsDouble();
    double xAxis = -strafeAxis.getAsDouble();

    // Safety area, insures that joystick movement will not be tracked within a
    // certain area,
    // prevents unintentional drifting
    yAxis = (Math.abs(yAxis) < stickDeadband) ? 0 : yAxis;
    xAxis = (Math.abs(xAxis) < stickDeadband) ? 0 : xAxis;

    translation = new Translation2d(yAxis, xAxis).times(4.6);

    // Converts from coordinates to angle, sets joystick forward input as 0,
    // converts angle to
    // degrees
    double azimuthAngle = setpointAngle.getAsDouble();

    // PID controller takes current robot position (getYaw) and compares to the
    // azimuth angle to
    // calculate error

    double rotationPIDOutput =
        azimuthController.calculate(swerveSubsystem.getPigeon2().getAngle(), azimuthAngle);

    Logger.recordOutput("rotationOutput", rotationPIDOutput);
    Logger.recordOutput("rotationOutputDeg", Units.radiansToDegrees(rotationPIDOutput));
    Logger.recordOutput("rotationSetpoint", azimuthAngle);

    translation = new Translation2d(yAxis, xAxis).times(4.6);
    rotationPIDOutput = MathUtil.clamp(rotationPIDOutput, -7.6, 7.6);

    swerveSubsystem.drive(translation, rotationPIDOutput, fieldRelative, openLoop);
  }

  @Override
  public boolean isFinished() {
    // return manualRotating.getAsBoolean() || azimuthController.atSetpoint();
    return azimuthController.atSetpoint();
  }
}
