// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

public final class IntakeConstants {
  /* CAN */
  public static final int kIntakeMotorID = 33;
  public static final String kIntakeCANBus = "rio";

  public static final double kPassthroughIntakeVoltage = -8;
  public static final double kIntakeIntakeVoltage = 12;

  /* PID */
  public static final double kIntakeKV = 0.1; // TODO: Tune PID values
  public static final double kIntakeKA = 0; // only for feedforward but idk if we gonna use it
  public static final double kIntakeKS = 0;
  public static final double kIntakeKP = 1;
  public static final double kIntakeKI = 0;
  public static final double kIntakeKD = 0;

  public static final int kIntakeCurrentThreshold = 16;
  public static final double kIntakeToFDistanceThreshold =
      450; // todo: tune | meters probably needs to be tuned

  /* Presets */
  public static final double kIntakeNoteRPM = 6000; // 1000
  public static final double kOuttakeNoteRPM = -6000;

  /* Misc */
  public static final boolean kUseIntakeMotionMagic = false;

  // Time of Flight constants
  public static final boolean kUseBeamBreak = true;
  public static final boolean kUseTimedCurrentSpike = true;
  public static final double kIntakeVelocitySpiking = 94;
  public static final double kIntakeSequenceTimeThreshold = 0.3; // seconds
  public static final double kTimeOfFlightRangingTime = 48;
  public static final int kTimeOfFlightID = 0;
  public static final boolean kDelayBeamBreak = false;
  public static final double kBeamBreakDelayTime = 0;
  public static final double kMotorCurrentSpikingTime = 0.8;

  public static final int kIntakeBeamBreakDIO = 0;
  public static int kPassthroughMotorID = 35;

  public static final double kPassthroughkS = 0.0;
  public static final double kPassthroughkV = 0.0;
  public static final double kPassthroughkA = 0.0;
  public static final double kPassthroughkP = 1.0;
  public static final double kPassthroughkI = 0.0;
  public static final double kPassthroughkD = 0.0;
  public static boolean kUsePassthroughMotionMagic = false;

  public static final double kPassthroughVelocitySpiking = 94;
  public static double kPassthroughOuttakeSpeed;
}
