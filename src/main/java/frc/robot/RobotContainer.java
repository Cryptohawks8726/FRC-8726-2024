// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController; 
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {
  private CommandXboxController userController;
  private ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    userController.a().onTrue(shooterSubsystem.toggleFlywheel());
    userController.b().onTrue(shooterSubsystem.shoot());
    
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
