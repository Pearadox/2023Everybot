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

  private SparkMaxPIDController armController;


  private enum ArmMode{
    HIGH,
    MID,
    STORED
  }

  private ArmMode mode = ArmMode.STORED;

  /** Creates a new Arm. */
  public Arm() {
    armController = _arm.getPIDController();
    armController.setP(ArmConstants.kArmP, 0);
    armController.setI(ArmConstants.kArmI, 0);
    armController.setD(ArmConstants.kArmD, 0);
    armController.setOutputRange(ArmConstants.kArmMin, ArmConstants.kArmMax);
    _arm.burnFlash();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void hold(){
    if (mode == ArmMode.HIGH){
      armController.setReference(ArmConstants.kArmHighRot, ControlType.kPosition, 0);
    }else if (mode == ArmMode.STORED){
      armController.setReference(ArmConstants.kArmStored, ControlType.kPosition, 0);
    }else if (mode == ArmMode.MID){
      armController.setReference(ArmConstants.kArmMidRot, ControlType.kPosition, 0);
    }
  }
  public void armStored(){
    mode = ArmMode.STORED;
  }
  public void armHigh(){
    mode = ArmMode.HIGH;
  }
  public void armMid(){
    mode = ArmMode.MID;
  }
  public String getMode(){
    String modeString = mode.toString();
    return modeString;
  }
}
