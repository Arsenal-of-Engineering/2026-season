// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final double kAimingDegreesPerRotation = 6.48;
  public static final int kAimingMotorCanId = 13;
  public static final int kAngleMotorCanId = 15;
  public static final int kShooterMotorCanId = 11;
  public static final int kConveyorMotorCanId = 10;
  public static final int kIntakeMotorCanId = 9;
  public static final int kFeederMotorCanId = 12;
  public static final int kPacerMotorCanId = 14;
  public static final int kJoystickId = 1;
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
