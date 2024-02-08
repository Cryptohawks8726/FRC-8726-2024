// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.BetaShooterSubsystem;
import frc.robot.subsystems.BetaShooterSubsystem.toggleMotorsStates;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {
  private CommandXboxController userController = new CommandXboxController(0);
  private BetaShooterSubsystem shooterSubsystem = new BetaShooterSubsystem();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    /*userController.a().onTrue(shooterSubsystem.startIntake());
    userController.y().onTrue(shooterSubsystem.stopMotors(true, true));
    userController.x().onTrue(shooterSubsystem.fireNote());*/

    userController.a().onTrue(shooterSubsystem.startIntake());
    userController.y().onTrue(shooterSubsystem.toggleMotors(toggleMotorsStates.disable, toggleMotorsStates.disable));
    userController.x().onTrue(shooterSubsystem.fireNote());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
