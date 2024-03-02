// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.unmanaged.Unmanaged;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.pathfinding.Pathfinder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.concurrent.Event;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.commands.ActualXboxTeleopDrive;
import frc.robot.subsystems.SwerveDrive;

public class RobotContainer {

  private final CommandXboxController driverController;
  private final SwerveDrive drivetrain;

  public RobotContainer() {

    Unmanaged.setPhoenixDiagnosticsStartTime(-1);
    drivetrain = new SwerveDrive();
    driverController = new CommandXboxController(0);

    NamedCommands.registerCommand("PrintTest1", new PrintCommand("Print @ EventMarker 1"));
    NamedCommands.registerCommand("PrintTest2", new PrintCommand("Print @ EventMarker 2"));
    
    configureBindings();
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(new ActualXboxTeleopDrive(drivetrain,driverController).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
  }

  public Command getAutonomousCommand() {
    PathPlannerPath choreoTraj = PathPlannerPath.fromChoreoTrajectory("EventMarkerTestPath"); 
    drivetrain.setOdometryPosition(choreoTraj.getPreviewStartingHolonomicPose());

    return AutoBuilder.followPath(choreoTraj);
  }

}
