// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase; 

public class Arm extends SubsystemBase {
  private CANSparkMax _arm = new CANSparkMax(5, MotorType.kBrushless);
  /** Creates a new Arm. */
  public Arm() {
    _arm.restoreFactoryDefaults();
    _arm.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void raise(){
    _arm.set(0.5);
  }
  public void lower(){
    _arm.set(-0.5);
  }
}
