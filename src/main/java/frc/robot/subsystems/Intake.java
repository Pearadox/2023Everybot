// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.PearadoxSparkMax;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase; 

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
private PearadoxSparkMax _intake = new PearadoxSparkMax(6, MotorType.kBrushless, IdleMode.kBrake, 20, false);

  public Intake() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void intakeIn(){
    _intake.set(-0.25);
  }
  public void intakeOut(){
    _intake.set(0.25);
  }
  public void stop(){
    _intake.set(0);
  }
}
