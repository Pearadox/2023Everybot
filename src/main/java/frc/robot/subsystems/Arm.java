// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.PearadoxSparkMax;
import frc.robot.Constants.*; 

public class Arm extends SubsystemBase {
  private PearadoxSparkMax _arm = new PearadoxSparkMax(CANIDs.kArmID, MotorType.kBrushless, IdleMode.kBrake, 30, false);
  /** Creates a new Arm. */
  public Arm() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void raise(){
    _arm.set(0.3);
  }
  public void lower(){
    _arm.set(-0.3);
  }
  public void stop(){
    _arm.set(0);
  }
}
