package frc.robot.commands;

import java.util.Random;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.internal.DriverStationModeThread;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.Limelights;
import frc.robot.subsystems.SwerveDrive;

public class ActualXboxTeleopDrive extends Command {
    private CommandXboxController controller;
    private final XboxController driverControllerBool;
    private SwerveDrive drivetrain;
    private double lastHeading;
    private boolean isHeadingSet;
    private PIDController headingPID;
    private double translationalSpeed,thetaSpeed;
    private int inversion;
    private Limelights limelight;
    
    public ActualXboxTeleopDrive(SwerveDrive drivetrain, CommandXboxController controller, Limelights newLimelight){
        this.drivetrain = drivetrain;
        this.controller = controller;
        addRequirements(drivetrain);
        headingPID = new PIDController(0.25, Swerve.kHeadingI, Swerve.kHeadingD, 20);
        headingPID.enableContinuousInput(0, 360);
        driverControllerBool = new XboxController(0);
        limelight = newLimelight;
    }
    
    @Override
    public void initialize(){
        isHeadingSet = false;
        headingPID.reset();
        
    } 
     
    @Override
    public void execute(){
        if (driverControllerBool.getAButton()) {
            SmartDashboard.putBoolean("LIMELIGHTWORKING", true);
            drivetrain.drive(limelight.trackTag(1.5),true);

            return;
        }
        /* X axis is forward from driver perspective, and the Y axis is parallel to the driver station wall. 
        See https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html 
        The controller axes have x as left-right and y as up-down
        */

        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
            inversion = -1;
        }else{
            inversion = 1;
        }

        boolean isRobotRelative = controller.leftTrigger().getAsBoolean();
        SmartDashboard.putBoolean("robotRelative", isRobotRelative);

        boolean slowMode = controller.rightTrigger().getAsBoolean();
        double leftX = controller.getLeftX();
        double leftY = controller.getLeftY();
        double rightX = controller.getRightX();

        translationalSpeed = slowMode ? 2.1: Swerve.maxSpeed;
        thetaSpeed = slowMode ? 2.0 : Swerve.maxAngularSpeed;
        
        // Get Controller Values
        double xVel = (Math.abs(leftY) > 0.15 ? leftY : 0.0); 
        double yVel = (Math.abs(leftX) > 0.15 ? leftX : 0.0);
        double thetaVel = (Math.abs(rightX) > 0.20 ? -rightX * thetaSpeed : 0.0);
        xVel = -Math.signum(xVel) * Math.pow(xVel,2) * translationalSpeed ;//square input while preserving sign
        yVel = -Math.signum(yVel) * Math.pow(yVel,2) * translationalSpeed ;

        // maintain heading if there's no rotational input
       if (thetaVel == 0.0 && ((Math.abs(xVel)>0.2) ||(Math.abs(yVel)>0.2))){
            if (isHeadingSet == false){
                headingPID.reset();
                isHeadingSet = true;
                lastHeading = drivetrain.getRobotAngle().getDegrees();
                headingPID.setSetpoint(lastHeading);
                thetaVel = headingPID.calculate(drivetrain.getRobotAngle().getDegrees()%360);
                SmartDashboard.putNumber("Calc heading pid vel", thetaVel);
             }else{
                thetaVel = headingPID.calculate(drivetrain.getRobotAngle().getDegrees()%360);
                SmartDashboard.putNumber("Calc heading pid vel", thetaVel);
            }
        } else{
            isHeadingSet = false;
        }
        drivetrain.drive(
             isRobotRelative ? new ChassisSpeeds(xVel, yVel, thetaVel)
            : ChassisSpeeds.fromFieldRelativeSpeeds(xVel*inversion, yVel*inversion, thetaVel, drivetrain.getRobotAngle())
            ,true
        );
    }

}
