// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
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

  private double error;
  private double _power;
  private double pidOut;

  public String _state;


  /** Creates a new Arm. */
  public Arm() {

  }

  @Override
  public void periodic() {
    error = _arm.getEncoder().getPosition();
    SmartDashboard.putNumber("Error", error);
    pidOut = _armController.calculate(error, 0);
    SmartDashboard.putNumber("PID Out", pidOut);
    if (_state == "High"){
      _power = pidOut / ArmConstants.kArmHighRot;
    }else if (_state == "Mid"){
      _power = pidOut / ArmConstants.kArmMidRot;
    }else if (_state == "Stored"){
      _power = pidOut / ArmConstants.kArmStored;
    }
    if (Math.abs(_power) > ArmConstants.kArmMax) {
      _power = Math.copySign(ArmConstants.kArmMax, _power);
    }
    if (Math.abs(_power) < ArmConstants.kArmMin) {
      _power = Math.copySign(ArmConstants.kArmMin, _power);
    }
    _arm.set(_power);
  }
  
  public void changeArmState(String state){
    _state = state;
  }
}
