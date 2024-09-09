// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.utils.DisableSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Climb extends DisableSubsystem {

  private final ClimbIO climbIO;
  private final ClimbIOInputsAutoLogged climbIOAutoLogged = new ClimbIOInputsAutoLogged();
  private final SysIdRoutine m_sysIdRoutine;

  private Rotation2d goal = new Rotation2d();

  public Climb(boolean disabled, ClimbIO climbIO) {
    super(disabled);
    this.climbIO = climbIO;
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
                    climbIO
                        .getMotor()
                        .setControl(climbIO.getVoltageRequest().withOutput(volts.in(Volts))),
                null,
                this));
  }

  @Override
  public void periodic() {
    super.periodic();
    climbIO.updateInputs(climbIOAutoLogged);
    Logger.processInputs(this.getClass().getSimpleName(), climbIOAutoLogged);
  }

  public Command setPosition(double position) {
    return this.run(
        () -> {
          climbIO.setPosition(position * ClimbConstants.gearRatio);
          this.goal = Rotation2d.fromRotations(position * ClimbConstants.gearRatio);
        });
  }

  public Command setVoltage(double voltage) {
    return this.run(() -> climbIO.setVoltage(voltage)).finallyDo(climbIO::off);
  }

  @AutoLogOutput
  public boolean isAtGoal() {
    return MathUtil.isNear(goal.getRotations(), climbIOAutoLogged.climbMotorPosition, 0.5)
        && MathUtil.isNear(0, climbIOAutoLogged.climbMotorVelocity, 0.1);
  }

  public Command off() {

    return this.runOnce(climbIO::off);
  }

  public Command zero() {

    return this.runOnce(climbIO::zero);
  }

  public Command extendClimber() {
    return setPosition(ClimbConstants.kClimbUpPosition);
  }

  public Command retractClimber() {
    return setPosition(ClimbConstants.kClimbDownPosition);
  }

  public boolean isExtended() {
    return MathUtil.isNear(
            ClimbConstants.kClimbUpPosition, climbIOAutoLogged.climbMotorPosition, 0.5)
        && MathUtil.isNear(0, climbIOAutoLogged.climbMotorVelocity, 0.1);
  }

  public boolean isRetracted() {
    return MathUtil.isNear(
            ClimbConstants.kClimbDownPosition, climbIOAutoLogged.climbMotorPosition, 0.5)
        && MathUtil.isNear(0, climbIOAutoLogged.climbMotorVelocity, 0.1);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }
}
