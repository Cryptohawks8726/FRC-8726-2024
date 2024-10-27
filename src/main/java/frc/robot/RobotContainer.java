// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.unmanaged.Unmanaged;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RuntimeType;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Shooter;
import frc.robot.commands.ActualXboxTeleopDrive;
import frc.robot.subsystems.SwerveDrive;

public class RobotContainer {

    private final CommandXboxController driverController;
    private final CommandXboxController operatorController;
    private final GenericHID operatorControllerHID;
    private final GenericHID driverControllerHID;
    private final SwerveDrive drivetrain;
    private final SendableChooser<String> autoChooser;
    private final PowerDistribution pdh;
    private boolean allowRumble;

    public RobotContainer() {

      Unmanaged.setPhoenixDiagnosticsStartTime(-1);
      drivetrain = new SwerveDrive();
      pdh = new PowerDistribution();
      pdh.setSwitchableChannel(true);
      
      driverController = new CommandXboxController(0);
      operatorController = new CommandXboxController(1);
      operatorControllerHID = operatorController.getHID();
      driverControllerHID = driverController.getHID();

      //autoChooser = AutoBuilder.buildAutoChooser();
      autoChooser = new SendableChooser<String>();
      autoChooser.setDefaultOption("2NoteCenterAuto","2NoteCenterAuto");
      autoChooser.addOption("2NoteAmpSideAuto","2NoteAmpSideAuto");
      autoChooser.addOption("2NoteSourceSideAuto","2NoteSourceSideAuto");
      autoChooser.addOption("3NoteRightAuto", "3NoteRightAuto");
      autoChooser.addOption("3NoteLeftAuto", "3NoteLeftAuto");
      //autoChooser.addOption("AmpSideBlank", "AmpSideBlank");
      //autoChooser.addOption("SourceSideBlank", "SourceSideBlank");
      //autoChooser.addOption("CenterBlank", "CenterBlank");

      configureBindings();

      SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {

      drivetrain.setDefaultCommand(new ActualXboxTeleopDrive(drivetrain,driverController).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
      driverController.start().onTrue(drivetrain.resetGyroAngle().withName("Gyro angle reset"));
    }

    public Command getAutonomousCommand() {
      if(!Constants.demoMode){
      System.out.println("Starting auto");

      if (autoChooser.getSelected() != null) {
        if (autoChooser.getSelected().equals("2NoteCenterAuto")) {
          return AutoBuilder.buildAuto("2NoteCenterAuto");
        } else if (autoChooser.getSelected().equals("2NoteSourceSideAuto")) {
          return AutoBuilder.buildAuto("2NoteLeftAuto");
        } else if (autoChooser.getSelected().equals("2NoteAmpSideAuto")) {
          return AutoBuilder.buildAuto("2NoteRightAuto");
        } else if(autoChooser.getSelected().equals("3NoteRightAuto")){
          return AutoBuilder.buildAuto("3NoteRightAuto");
        }else if(autoChooser.getSelected().equals("3NoteLeftAuto")){
          return AutoBuilder.buildAuto("3NoteLeftAuto");
        } else {
          return shooter.fireNote(false); // default path to do if nothing is selected
        }
      }else{
        return null;
      }
      
      

    } else{ return null;}

  }

  public void setControllerRumble(double rumble){
    if(DriverStation.isTeleopEnabled()){
      operatorControllerHID.setRumble(RumbleType.kBothRumble, rumble);
      driverControllerHID.setRumble(RumbleType.kBothRumble, rumble);
    }else{
      operatorControllerHID.setRumble(RumbleType.kBothRumble, 0);
      driverControllerHID.setRumble(RumbleType.kBothRumble, 0);
    }
  }
  
}
