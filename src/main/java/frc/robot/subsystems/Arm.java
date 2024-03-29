// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.PearadoxSparkMax;
import frc.robot.Constants.*;

public class Arm extends SubsystemBase {
  private PearadoxSparkMax _arm = new PearadoxSparkMax(CANIDs.kArmID, MotorType.kBrushless, IdleMode.kBrake, 30, false);
  private PIDController _armController = new PIDController(ArmConstants.kArmP, ArmConstants.kArmI, ArmConstants.kArmD);
  private double _error;
  private double _power;
  private double _pidOut;
  public String _state;

  /** Creates a new Arm. */
  public Arm() {
  }

  @Override
  public void periodic() {
    checkArmHeight();
    if (Math.abs(_power) > ArmConstants.kArmMax) {
      _power = Math.copySign(ArmConstants.kArmMax, _power);
    }
    if (Math.abs(_power) < ArmConstants.kArmMin) {
      _power = Math.copySign(ArmConstants.kArmMin, _power);
    }
    SmartDashboard.putNumber("Error", _error);
    SmartDashboard.putNumber("PID Out", _pidOut);
    SmartDashboard.putNumber("power", _power);
    _arm.set(_power);
  }

  public void changeArmState(String state) {
    _state = state;
  }

  public void checkArmHeight() {
    _error = _arm.getEncoder().getPosition();
    if (_state == "High") {
      _pidOut = _armController.calculate(_error, ArmConstants.kArmHighRot);
      _power = _pidOut / ArmConstants.kArmHighRot;
    } else if (_state == "Mid") {
      _pidOut = _armController.calculate(_error, ArmConstants.kArmMidRot);
      _power = _pidOut / ArmConstants.kArmMidRot;
    } else if (_state == "Stored") {
      _pidOut = _armController.calculate(_error, ArmConstants.kArmStored);
      _power = _pidOut / ArmConstants.kArmStored;
      if (Math.abs(_error - ArmConstants.kArmStored) < 0.6) {
        _power = 0;
      }
    }
  }
  public double getArmPose(){
    return _arm.getEncoder().getPosition();
  }
}
