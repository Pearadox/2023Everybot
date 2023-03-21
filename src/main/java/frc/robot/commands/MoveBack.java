// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.sql.Timestamp;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MoveBack extends CommandBase {
  /** Creates a new MoveBack. */
  private ADIS16470_IMU _gyro;
  private DriveTrain _driveTrain;

  private int _timeToStop = 3;
  private double _timeElapsed = 0;

  private Timer timeSinceStart = new Timer();

  private boolean isDone = false;

  public MoveBack(DriveTrain driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    _driveTrain = driveTrain;
    addRequirements(_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timeSinceStart.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _timeElapsed = timeSinceStart.get();
    SmartDashboard.putNumber("Time elapsed", _timeElapsed);
    if (_timeElapsed < _timeToStop){
      _driveTrain.drive(0, 0.8);
    }else{
      _driveTrain.drive(0, 0);
      isDone = true;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
