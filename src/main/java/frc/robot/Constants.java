// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class JoystickConstants {
    // Controllers
    public static final int DRIVER_PORT = 0;
    public static final int OPERATOR_PORT = 1;
    // XboxOne Joysticks (axes)
    public static final int LEFT_STICK_X = 0;
    public static final int LEFT_STICK_Y = 1;
    public static final int LEFT_TRIGGER = 2;
    public static final int RIGHT_TRIGGER = 3;
    public static final int RIGHT_STICK_X = 4;
    public static final int RIGHT_STICK_Y = 5;
    // XboxOne Buttons
    public static final int A = 1;
    public static final int B = 2;
    public static final int X = 3;
    public static final int Y = 4;
    public static final int BUMPER_LEFT = 5;
    public static final int BUMPER_RIGHT = 6;
    public static final int LOGO_LEFT = 7;
    public static final int LOGO_RIGHT = 8;
    public static final int LEFT_STICK_BUTTON = 9;
    public static final int RIGHT_STICK_BUTTON = 10;
    // Joystick controller axis
    public static final int LOGITECH_UP = 0;
    public static final int LOGITECH_DOWN = 1;

  }

  public static final class CANIDs {
    // Drivetrain sparkmaxes
    // changed to match 2024Kitbot when testing
    public static final int kfrontLeftID = 10; // formerly 4;
    public static final int kfrontRightID = 13; // formerly 3;
    public static final int kbackLeftID = 11; // formerly 2;
    public static final int kbackRightID = 12; // formerly 1;
    // Arm sparkmaxes
    public static final int kArmID = 5;
    // Intake sparkmaxes
    public static final int kIntakeID = 6;
  }

  public static final class AutoConstants {
    public static final double kRamseteB = 2D;
    public static final double kRamseteZeta = 0.7D;
  }

  public static final class DriveTrainConstants {
    public static final double kDistancePerWheelRevolutionMeters = Units.inchesToMeters(Math.PI * 6.0);
    public static final double kGearReduction = 10.71;
    public static final double ksVolts = 0.25568;
    public static final double kvVoltSecondsPerMeter = 2.8971;
    public static final double kaVoltSecondsSquaredPerMeter = 0.56826;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
        Units.inchesToMeters(23));
    public static final double kPDriveVel = 0;
    // XBOX or JOYSTICK as value
    public static final String driveController = "JOYSTICK";
    // 0.9 for XBOX is best and 0.65 for JOYSTICK is best
    public static final double turnReduction = 0.65;
    // keep the same for both controller types
    public static final double directionReduction = .85;
    public static final double kRampRate = 0.5;
  }

  public static final class ArmConstants {
    public static final double kArmP = 3;
    public static final double kArmI = 0;
    public static final double kArmD = 0;
    public static final double kArmMin = 0.00;
    public static final double kArmMax = 0.25;
    public static final double kArmHighRot = 35;
    public static final double kArmMidRot = 20;
    public static final double kArmStored = 0;
  }
}
