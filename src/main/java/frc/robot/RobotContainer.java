// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Intake.IntakeState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final ADIS16470_IMU _gyro = new ADIS16470_IMU();
  private final DriveTrain _driveTrain = new DriveTrain(_gyro);
  private final Joystick _driver = new Joystick(0);
  // set to 0 if no operator otherwise set to 1
  private final Joystick _operator = new Joystick(1);
  private final Arm _arm = new Arm();
  private final Intake _intake = new Intake();

  // Replace with CommandPS4Controller or CommandJoystick if needed

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    runContainer();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    new JoystickButton(_operator, JoystickConstants.A).whileTrue(new RunCommand(() -> _arm.changeArmState("Stored")));
    new JoystickButton(_operator, JoystickConstants.Y).whileTrue(new RunCommand(() -> _arm.changeArmState("High")));
    new JoystickButton(_operator, JoystickConstants.X).whileTrue(new RunCommand(() -> _arm.changeArmState("Mid")));
    new JoystickButton(_operator, JoystickConstants.BUMPER_RIGHT)
        .onTrue(new InstantCommand(() -> _intake.setState(IntakeState.CubeIntake)))
        .onFalse(new InstantCommand(() -> _intake.setState(IntakeState.CubeHold)));
    new JoystickButton(_operator, JoystickConstants.BUMPER_LEFT)
        .onTrue(new InstantCommand(() -> _intake.setState(IntakeState.ConeIntake)))
        .onFalse(new InstantCommand(() -> _intake.setState(IntakeState.ConeHold)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(_arm, _driveTrain, _intake);
  }

  private void runContainer() {

    _driveTrain.setDefaultCommand(new ArcadeDrive(_driveTrain, _driver));
    // Configure the trigger bindings
    configureBindings();
  }
}
