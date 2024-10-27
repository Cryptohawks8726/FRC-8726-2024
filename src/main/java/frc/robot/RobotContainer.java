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
    private final ShooterSubsystem shooter;
    private final ClimberSubsystem climber;
    private final ArmSubsystem arm;
    private final PowerDistribution pdh;
    private boolean allowRumble;

    public RobotContainer() {

      Unmanaged.setPhoenixDiagnosticsStartTime(-1);
      drivetrain = new SwerveDrive();
      shooter = new ShooterSubsystem();
      climber = new ClimberSubsystem();
      arm = new ArmSubsystem();
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

      NamedCommands.registerCommand("ShootFirstNote", arm.rotateToState(Arm.tempShootState).andThen(shooter.fireNote(false)).andThen(arm.rotateToState(Arm.intakeState))); //shooter.fireNote(false) without remy
      NamedCommands.registerCommand("IntakeNoteCmd0", shooter.startIntake());
      NamedCommands.registerCommand("IntakeNoteCmd3", 
        shooter.startIntake().andThen(arm.rotateToState(Arm.driveState)));
      NamedCommands.registerCommand("ShootSecondNote", arm.rotateToState(Arm.tempShootState).andThen(shooter.fireNote(false)));
      NamedCommands.registerCommand("ShootThirdNote", arm.rotateToState(new State(Math.toRadians(7.5), 0)).andThen(shooter.fireNote(false)));
      
    }

    private void configureBindings() {

      drivetrain.setDefaultCommand(new ActualXboxTeleopDrive(drivetrain,driverController).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
      driverController.start().onTrue(drivetrain.resetGyroAngle().withName("Gyro angle reset"));
      
      operatorController.leftTrigger()
          .onTrue(
            new ConditionalCommand(
                  arm.rotateToState(Arm.tempIntakeState),
                  new PrintCommand("Arm not lowered for intake"),
                  () -> arm.getArmDeg() < 40)
          .alongWith(shooter.startIntake()))
      .onFalse(shooter.stopShooter().alongWith(arm.rotateToState(Arm.driveState))); //stopShooter isn't functional atm
      operatorController.rightTrigger()
          .onTrue(
              new ConditionalCommand(
                  new PrintCommand("Already at angle"),
                  arm.rotateToState(Arm.tempShootState),
                  () -> arm.getArmDeg() > 40) // This checks if the Arm is likely going for the amp
          .andThen(shooter.fireNote(arm.getArmDeg()>40)))
          .onFalse(shooter.stopShooter().alongWith(arm.rotateToState(Arm.tempIntakeState)));
      if (Constants.disableBeamBreaks) operatorController.leftBumper().onTrue(shooter.demoPullBackNote()).onFalse(shooter.stopShooter());
      else operatorController.leftBumper().onTrue(shooter.pullBackNote());
      //operatorController.povUp().onTrue(shooter.pullBackNote());
      operatorController.rightBumper().onTrue(shooter.stopShooter());
      operatorController.a().onTrue(arm.rotateToState(Arm.tempIntakeState));
      operatorController.b().onTrue(arm.rotateToState(Arm.ampState));
      //operatorController.y().onTrue(arm.rotateToState(Arm.sourceState));
      //operatorController.x().onTrue(arm.rotateToState(Arm.podiumState));
      operatorController.back().onTrue(climber.smartReleaseClimber());
      operatorController.start().onTrue(climber.smartClimb());
      /*shooter.hasNote()
        .onTrue(new InstantCommand(()->setControllerRumble(0.5)))
        .onFalse(new InstantCommand(()->setControllerRumble(0)));
      */
      //shooter.noteReady()
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
        }else {
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
