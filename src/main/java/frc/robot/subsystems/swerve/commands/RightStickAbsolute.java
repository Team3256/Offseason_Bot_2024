package frc.robot.subsystems.swerve.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.helpers.DebugCommandBase;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class RightStickAbsolute extends DebugCommandBase{
  private Translation2d translation;
  private boolean fieldRelative;
  private boolean openLoop;

  private CommandSwerveDrivetrain swerveSubsystem;
  private DoubleSupplier translationAxis;
  private DoubleSupplier strafeAxis;
  private DoubleSupplier rotationXAxis;
  private DoubleSupplier rotationYAxis;
  private DoubleSupplier setpointAngle;
  private BooleanSupplier manualRotating;
  private PIDController angleController;

  /** Driver control */
  public RightStickAbsolute(
      CommandSwerveDrivetrain swerveSubsystem,
      DoubleSupplier translationAxis,
      DoubleSupplier strafeAxis,
      DoubleSupplier rotationXAxis,
      DoubleSupplier rotationYAxis,
      BooleanSupplier manualRotating,
      boolean fieldRelative,
      boolean openLoop) {
    this.swerveSubsystem = swerveSubsystem;
    addRequirements(swerveSubsystem); //TODO: fix this

    this.translationAxis = translationAxis;
    this.strafeAxis = strafeAxis;
    this.rotationXAxis = rotationXAxis;
    this.rotationYAxis = rotationYAxis;
    this.manualRotating = manualRotating;
    this.fieldRelative = fieldRelative;
    this.openLoop = openLoop;
    this.angleController = new PIDController(0, 0, 0);
    angleController.enableContinuousInput(-180, 180);
  }

  public DoubleSupplier getRightStickAngle(double x, double y){
    DoubleSupplier angle = () -> (Math.atan2(y, x) * (180 / Math.PI));
    return angle;
  }
  
  @Override
  public void execute() {
    double yAxis = -translationAxis.getAsDouble();
    double xAxis = -strafeAxis.getAsDouble();

    double rAxisX = rotationXAxis.getAsDouble();
    double rAxisY = -rotationYAxis.getAsDouble();

    yAxis = (Math.abs(yAxis) < 0.1) ? 0 : yAxis;
    xAxis = (Math.abs(xAxis) < 0.1) ? 0 : xAxis;
    rAxisX = (Math.abs(rAxisX) < 0.3) ? 0 : rAxisX;
    rAxisY = (Math.abs(rAxisY) < 0.3) ? 0 : rAxisY;

    translation = new Translation2d(yAxis, xAxis).times(4.6);
    if (rAxisX == 0 && rAxisY == 0) {
      swerveSubsystem.drive(translation, 0, fieldRelative, openLoop); // TODO: implement new drive method
      return;
    }

    setpointAngle = getRightStickAngle(rAxisX, rAxisY);
    double azimuthAngle = setpointAngle.getAsDouble();

    double rotationPIDOutput =
        angleController.calculate(
          swerveSubsystem.getPigeon2().getAngle(), azimuthAngle);

    translation = new Translation2d(yAxis, xAxis).times(4.6);
    rotationPIDOutput = MathUtil.clamp(rotationPIDOutput, -7.6, 7.6);

    swerveSubsystem.drive(translation, rotationPIDOutput, fieldRelative, openLoop);
  }

  @Override
  public boolean isFinished() {
    return manualRotating.getAsBoolean() || angleController.atSetpoint();
  }
}
