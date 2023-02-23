// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.PearadoxSparkMax;
import frc.robot.Constants.*; 

public class Arm extends SubsystemBase {
  private PearadoxSparkMax _arm = new PearadoxSparkMax(CANIDs.kArmID, MotorType.kBrushless, IdleMode.kBrake, 30, false);

  private SparkMaxPIDController armController;


  private enum ArmMode{
    High,
    Mid,
    Stored
  }

  private ArmMode mode = ArmMode.Stored;

  /** Creates a new Arm. */
  public Arm() {
    armController = _arm.getPIDController();
    armController.setP(ArmConstants.kArmP);
    armController.setI(ArmConstants.kArmI);
    armController.setD(ArmConstants.kArmD);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void hold(){
    if (mode == ArmMode.High){
      armController.setReference(ArmConstants.kArmHighRot, ControlType.kPosition);
    }
  }
}
