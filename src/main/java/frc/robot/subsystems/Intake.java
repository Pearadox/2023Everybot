// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.PearadoxSparkMax;
import frc.robot.Constants.CANIDs;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private PearadoxSparkMax _intake = new PearadoxSparkMax(CANIDs.kIntakeID, MotorType.kBrushless, IdleMode.kBrake, 20,
      false);

  public enum IntakeState {
    Off,
    CubeIntake,
    CubeHold,
    ConeIntake,
    ConeHold
  }

  public IntakeState _intakeState = IntakeState.Off;

  public Intake() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double power = 0;
    int currentLimit = 25;

    if (_intakeState == IntakeState.Off) {
      power = (0);
      currentLimit = 25;
    } else if (_intakeState == IntakeState.CubeIntake) {
      power = 0.4;
      currentLimit = 25;
    } else if (_intakeState == IntakeState.CubeHold) {
      power = 0.07;
      currentLimit = 5;
    } else if (_intakeState == IntakeState.ConeIntake) {
      power = 0.4;
      currentLimit = 25;
    } else if (_intakeState == IntakeState.ConeHold) {
      power = 0.07;
      currentLimit = 5;
    }
    _intake.set(power);
    _intake.setSmartCurrentLimit(currentLimit);
  }

  public void setState(IntakeState state) {
    _intakeState = state;
  }
}
