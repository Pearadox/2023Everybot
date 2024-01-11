// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FollowApril extends CommandBase {
  // private final DoubleSubscriber cl;
  // private final DoubleSubscriber tl;

  NetworkTable limeLight = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx;
  double aprilX;
  double aprilY;
  double aprilA;
  double sillyNumber = 0;
  PIDController _turnPID = new PIDController(0.1, 0.2, 0);
  double _turnpower;
  
  /** Creates a new FollowApril. */
  private DriveTrain _driveTrain;
  public FollowApril(DriveTrain driveTrain) {
    _driveTrain = driveTrain;
    addRequirements(_driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled. 
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    tx = limeLight.getEntry("tx");
    NetworkTableEntry ty = limeLight.getEntry("ty");
    NetworkTableEntry ta = limeLight.getEntry("ta");
    aprilX = tx.getDouble(0.0);
    aprilY = ty.getDouble(0.0);
    aprilA = ta.getDouble(0.0);
    SmartDashboard.putNumber("Apriltag X", aprilX);
    SmartDashboard.putNumber("Apriltag Y", aprilY); 
    SmartDashboard.putNumber("Apriltag Size", aprilA);
    SmartDashboard.putNumber("Wait a minute", sillyNumber);
    sillyNumber = sillyNumber + 1;
    _turnpower = _turnPID.calculate(aprilX*-1, 0);
    _driveTrain.drive(_turnpower/2, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
