// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */
  private CANSparkMax _frontLeft = new CANSparkMax(4, MotorType.kBrushless);
  private CANSparkMax _frontRight = new CANSparkMax(3, MotorType.kBrushless);
  private CANSparkMax _backLeft = new CANSparkMax(2, MotorType.kBrushless);
  private CANSparkMax _backRight = new CANSparkMax(1, MotorType.kBrushless);
  private DifferentialDrive _drive = new DifferentialDrive(_frontLeft , _frontRight);

  private RelativeEncoder _frontLeftEncoder;
  private RelativeEncoder _frontRightEncoder;
  private RelativeEncoder _backLeftEncoder;
  private RelativeEncoder _backRightEncoder;

  private ADIS16470_IMU _gyro;

  public DriveTrain(ADIS16470_IMU gyro) {
    _frontLeft.restoreFactoryDefaults();
    _backLeft.restoreFactoryDefaults();
    _frontRight.restoreFactoryDefaults();
    _backRight.restoreFactoryDefaults();

    _frontLeft.burnFlash();
    _frontRight.burnFlash();
    _backLeft.burnFlash();
    _backRight.burnFlash();

    _backLeft.follow(_frontLeft);
    _backRight.follow(_frontRight);

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
    double axis4 = controller.getRawAxis(4);
    double axis1 = controller.getRawAxis(1);
    drive(axis4, axis1);
  }

  public void drive(double rotation, double direction) {
    _drive.arcadeDrive(rotation, direction);
  }
}
