// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import frc.lib.drivers.PearadoxSparkMax;
import frc.robot.Constants.CANIDs;


public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */
  private PearadoxSparkMax _frontLeft = new PearadoxSparkMax(CANIDs.kfrontLeftID, MotorType.kBrushless, IdleMode.kBrake, 55, false);
  private PearadoxSparkMax _frontRight = new PearadoxSparkMax(CANIDs.kfrontRightID, MotorType.kBrushless, IdleMode.kBrake, 55, false);
  private PearadoxSparkMax _backLeft = new PearadoxSparkMax(CANIDs.kbackLeftID, MotorType.kBrushless, IdleMode.kBrake, 55, false, _frontLeft);
  private PearadoxSparkMax _backRight = new PearadoxSparkMax(CANIDs.kbackRightID, MotorType.kBrushless, IdleMode.kBrake, 55, false, _frontRight);
  private DifferentialDrive _drive = new DifferentialDrive(_frontLeft , _frontRight);

  private DifferentialDriveOdometry _odometry;
  private RelativeEncoder _frontLeftEncoder;
  private RelativeEncoder _frontRightEncoder;
  private RelativeEncoder _backLeftEncoder;
  private RelativeEncoder _backRightEncoder;


  private ADIS16470_IMU _gyro;

  public DriveTrain(ADIS16470_IMU gyro) {


    _frontLeftEncoder = _frontLeft.getEncoder();
    _frontRightEncoder = _frontRight.getEncoder();
    _backLeftEncoder = _backLeft.getEncoder();
    _backRightEncoder = _backRight.getEncoder();

    _gyro = gyro;

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public void teleopDrive(Joystick controller) {
    double axis2 = controller.getRawAxis(2);
    double axis1 = controller.getRawAxis(1);
    drive(axis2, axis1);
  }

  public void drive(double rotation, double direction) {
    _drive.arcadeDrive(rotation, direction);
  }
  public RelativeEncoder getEncoder() {
    return _frontLeft.getEncoder();
  }

  public Pose2d getPose() {
    return _odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(_frontLeftEncoder.getVelocity() / 60, -_frontRightEncoder.getVelocity() / 60);
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    _odometry.resetPosition(Rotation2d.fromDegrees(_gyro.getAngle()), _frontLeftEncoder.getPosition(), -_frontRightEncoder.getPosition(), pose);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    _frontLeft.setVoltage(leftVolts);
    _backLeft.setVoltage(leftVolts);
    _frontRight.setVoltage(-rightVolts);
    _backRight.setVoltage(-rightVolts);

    _drive.feed();
  }

  public void resetEncoders() {
    _frontLeftEncoder.setPosition(0);
    _frontRightEncoder.setPosition(0);
  }

  public void zeroHeading() {
    _gyro.reset();
  }
}
