// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public final class Autos {
  /** Example static factory for an autonomous command. */

  public static CommandBase exampleAuto(Arm arm, DriveTrain driveTrain, Intake intake) {
    return new AutoRaiseArmHigh(arm)
      .andThen(new AutoCubeOut(intake))
      .andThen(new AutoResetArm(arm))
      .andThen(new MoveBack(driveTrain));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

}
