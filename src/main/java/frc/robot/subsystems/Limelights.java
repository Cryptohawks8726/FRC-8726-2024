package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.LimelightHelpers.LimelightTarget_Retro;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;


public class Limelights extends SubsystemBase {
    NetworkTable table;
    public boolean blueOrigin = true;
    public Pose2d robotPose;

    ProfiledPIDController xCont = new ProfiledPIDController(1.25, 0.0, 0.0, new TrapezoidProfile.Constraints(1, 1)); //will add in parameters l8r
    ProfiledPIDController yCont = new ProfiledPIDController(1.25, 0.0, 0.0, new TrapezoidProfile.Constraints(1, 1));
    ProfiledPIDController omegaCont = new ProfiledPIDController(1.5, 0.0, 0.0, new TrapezoidProfile.Constraints(1, 1));

    public Limelights() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        LimelightHelpers.setCameraPose_RobotSpace(getName(), 0.3, 0, 0, 0, 30, 0);
    }

    @Override
    public void periodic() {
        Pose2d botPose = LimelightHelpers.getBotPose2d("limelight");
        

        SmartDashboard.putNumber("FieldPosX", botPose.getX());
        SmartDashboard.putNumber("FieldPosY", botPose.getY());
        SmartDashboard.putNumber("FieldRotDeg", botPose.getRotation().getDegrees());
        double tagID = table.getEntry("tid").getDouble(0.0);
    
        SmartDashboard.putNumber("TagID", tagID);
        
        addVisionMeasurementToPoseEstimator();
    }

    public void addVisionMeasurementToPoseEstimator() { // TODO: add argument SwerveDrivePoseEstimator poseEstimator
        double[] botposeArray = table.getEntry("botpose").getDoubleArray(new double[6]);
        
        double xMeters = botposeArray[0]; // X translation (meters)
        double yMeters = botposeArray[1]; // Y translation (meters)
        //double zMeters = botposeArray[2]; // Z translation (meters)
        //double rollDegrees = botposeArray[3]; // Roll (degrees)
        //double pitchDegrees = botposeArray[4]; // Pitch (degrees)
        double yawDegrees = botposeArray[5]; // Yaw (degrees)

        // Create the Pose2d using the extracted values
        robotPose = new Pose2d(xMeters, yMeters, new Rotation2d(yawDegrees));
        SmartDashboard.putNumber("X Translation", xMeters);
        SmartDashboard.putNumber("Y Translation", yMeters);
        SmartDashboard.putNumber("Rotation", yawDegrees);
        SmartDashboard.putNumberArray("Full Data", table.getEntry("botpose").getDoubleArray(new double[6]));

        //poseEstimator.addVisionMeasurement(robotPose, Timer.getFPGATimestamp());
    }

    public Pose2d getRobotPose(){
        return robotPose;
    }

    public ChassisSpeeds trackTag(double desiredDistance) {
        Transform2d botToTagOffset = new Transform2d(new Translation2d(1.5, 0), new Rotation2d());
        Pose2d botPose = LimelightHelpers.getBotPose2d_wpiBlue("limelight");
        Pose2d relCameraPose = LimelightHelpers.getTargetPose3d_RobotSpace("limelight").toPose2d()
        Pose2d goalPose = botPose.transformBy(new Transform2d(new Translation2d(relCameraPose.getX(), relCameraPose.getY()), new Rotation2d(relCameraPose.getRotation().getRadians());

        xCont.setGoal(goalPose.getX());
        yCont.setGoal(goalPose.getY());
        omegaCont.setGoal(goalPose.getRotation().getRadians());

        SmartDashboard.putBoolean("COMMAND EXECUTED", true);

        return new ChassisSpeeds(xCont.calculate(botPose.getX()), yCont.calculate(botPose.getY()), omegaCont.calculate(botPose.getRotation().getRadians()));
    }

    public ChassisSpeeds getSpeedsForPose(Pose2d desiredPose) {
        Pose2d botPose = LimelightHelpers.getBotPose2d_wpiBlue("limelight");

        xCont.setGoal(desiredPose.getX());
        yCont.setGoal(desiredPose.getY());
        omegaCont.setGoal(desiredPose.getRotation().getRadians());

        return new ChassisSpeeds(xCont.calculate(botPose.getX()), yCont.calculate(botPose.getY()), omegaCont.calculate(botPose.getRotation().getRadians()));
    }
}


