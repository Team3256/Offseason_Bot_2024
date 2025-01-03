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
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.vision.Vision;
import frc.robot.utils.DisableSubsystem;
import org.littletonrobotics.junction.Logger;

public class PivotShooter extends DisableSubsystem {

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

  public PivotShooter(boolean disabled, PivotShooterIO pivotShooterIO) {
    super(disabled);

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
    super.periodic();
    pivotShooterIO.updateInputs(pivotShooterIOAutoLogged);
    Logger.processInputs(this.getClass().getSimpleName(), pivotShooterIOAutoLogged);
  }

  public Command setPosition(double position) {
    return this.run(() -> pivotShooterIO.setPosition(position)).finallyDo(pivotShooterIO::off);
  }

  public Command setVoltage(double voltage) {
    return this.run(() -> pivotShooterIO.setVoltage(voltage)).finallyDo(pivotShooterIO::off);
  }

  public Command off() {

    return this.runOnce(pivotShooterIO::off);
  }

  public Command slamZero() {
    return this.run(() -> pivotShooterIO.setVoltage(PivotShooterConstants.kPivotSlamShooterVoltage))
        .until(
            () ->
                pivotShooterIOAutoLogged.pivotShooterMotorStatorCurrent
                    > PivotShooterConstants.kPivotSlamStallCurrent)
        .andThen(this.zero());
  }

  public Command slamAndPID() {

    return Commands.sequence(this.setPosition(0), this.slamZero());
  }

  public Command zero() {
    return this.runOnce(pivotShooterIO::zero);
  }

  public Command bruh(Vision vision) {
    return this.run(
        () -> {
          pivotShooterIO.setPosition(
              aprilTagMap.get(
                      (vision.getLastCenterLimelightY() - vision.getLastLastCenterLimelightY())
                          + vision.getCenterLimelightY())
                  * PivotShooterConstants.kPivotMotorGearing);
        });
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

  public double getPosition() {
    return pivotShooterIOAutoLogged.pivotShooterMotorPosition;
  }
}
