// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase; 

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
private CANSparkMax _intake = new CANSparkMax(6, MotorType.kBrushless);

  public Intake() {
    _intake.restoreFactoryDefaults();
    _intake.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void intakeIn(){
    _intake.set(1);
  }
  public void intakeOut(){
    _intake.set(-1);
  }
}
