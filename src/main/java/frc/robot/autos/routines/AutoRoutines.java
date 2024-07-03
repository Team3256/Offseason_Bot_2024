package frc.robot.autos.routines;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.pivotintake.PivotIntake;
import frc.robot.subsystems.pivotintake.PivotIntakeConstants;
import frc.robot.subsystems.pivotshooter.PivotShooter;
import frc.robot.subsystems.pivotshooter.PivotShooterConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class AutoRoutines {
    public static Command ampWing1Note (CommandSwerveDrivetrain swerve, Intake intake, Shooter shooter, PivotShooter pivotShooter, PivotIntake pivotIntake) { // This is an example on how to make a choreo auto, I have no clue if its gonna work
        return Commands.sequence(
                swerve.runChoreoTraj(Choreo.getTrajectory("AmpSingleNote.1")).alongWith(pivotIntake.setPosition(PivotIntakeConstants.kPivotGroundPos).andThen(intake.intakeIn().andThen(pivotIntake.setPosition(0)))),
                shooter.setVelocity(ShooterConstants.kShooterSubwooferRPS, ShooterConstants.kShooterFollowerSubwooferRPS).alongWith(pivotShooter.setPosition(PivotShooterConstants.kSubWooferPreset).andThen(
                        intake.setPassthroughVoltage(IntakeConstants.kPassthroughIntakeVoltage)))
        );
    }
}
