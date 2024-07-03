// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.pivotshooter;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.vision.Vision;
import org.littletonrobotics.junction.Logger;

public class PivotShooter extends SubsystemBase {

  private final PivotShooterIO pivotShooterIO;
  private final PivotShooterIOInputsAutoLogged pivotShooterIOAutoLogged =
      new PivotShooterIOInputsAutoLogged();

  private final SysIdRoutine m_sysIdRoutine;

  private final InterpolatingDoubleTreeMap aprilTagMap =
      new InterpolatingDoubleTreeMap() {
        {
          put(0.0, 0.0);
          put(1.0, 1.0);
        }
      };

  public PivotShooter(PivotShooterIO pivotShooterIO) {

    this.pivotShooterIO = pivotShooterIO;
    m_sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.2).per(Seconds.of(1)), // Use default ramp rate (1 V/s)
                Volts.of(6), // Reduce dynamic step voltage to 4 to prevent brownout
                null, // Use default timeout (10 s)
                // Log state with Phoenix SignalLogger class
                (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                (volts) ->
                    pivotShooterIO
                        .getMotor()
                        .setControl(pivotShooterIO.getVoltageRequest().withOutput(volts.in(Volts))),
                null,
                this));
  }

  @Override
  public void periodic() {
    pivotShooterIO.updateInputs(pivotShooterIOAutoLogged);
    Logger.processInputs(this.getClass().getName(), pivotShooterIOAutoLogged);
  }

  public Command setPosition(double position) {
    return new StartEndCommand(
        () -> pivotShooterIO.setPosition(position * PivotShooterConstants.kPivotMotorGearing),
        () -> {},
        this);
  }

  public Command setVoltage(double voltage) {
    return new StartEndCommand(
        () -> pivotShooterIO.setVoltage(voltage), () -> pivotShooterIO.setVoltage(0), this);
  }

  public Command off() {
    return new StartEndCommand(() -> pivotShooterIO.off(), () -> {}, this);
  }

  public Command slamZero() {
    return new Command() {
      @Override
      public void initialize() {
        pivotShooterIO.setVoltage(PivotShooterConstants.kPivotSlamShooterVoltage);
      }

      @Override
      public void end(boolean interrupted) {
        pivotShooterIO.off();
        if (!interrupted) {
          pivotShooterIO.zero();
        }
      }

      @Override
      public boolean isFinished() {
        return pivotShooterIOAutoLogged.pivotShooterMotorStatorCurrent
            > PivotShooterConstants.kPivotSlamStallCurrent;
      }
    };
  }

  public Command slamAndPID() {
    return new SequentialCommandGroup(this.setPosition(0), this.slamZero());
  }

  public Command zero() {
    return new StartEndCommand(() -> pivotShooterIO.zero(), () -> {}, this);
  }

  public Command bruh(Vision vision) {
    return new RunCommand(
        () ->
            pivotShooterIO.setPosition(
                aprilTagMap.get(
                        (vision.getLastCenterLimelightY() - vision.getLastLastCenterLimelightY())
                            + vision.getCenterLimelightY())
                    * PivotShooterConstants.kPivotMotorGearing),
        this);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }
}
