package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Limelights extends SubsystemBase {
    NetworkTable table;
    public boolean blueOrigin = true;
    public Pose2d robotPose;

    ProfiledPIDController xCont = new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0)); //will add in parameters l8r
    ProfiledPIDController yCont = new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0));
    ProfiledPIDController omegaCont = new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0));

    public Limelights() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        LimelightHelpers.setCameraPose_RobotSpace(getName(), 0.3, 0, 0, 0, 40, 0);
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

    public ChassisSpeeds getTagTrackingSpeeds(double desiredDistance) {
        Transform2d botToTagOffset = new Transform2d(new Translation2d(1.5, 0), new Rotation2d();
        Pose2d botPose = LimelightHelpers.getBotPose2d("limelight");
        Pose2d tagPose = LimelightHelpers.getTargetPose_RobotSpace2D("limelight");
        Pose2d goalPose = tagPose.transformBy(botToTagOffset);

        xCont.setGoal(goalPose.getX());
        yCont.setGoal(goalPose.getY());
        omegaCont.setGoal(goalPose.getRotation().getRadians);

        return new ChassisSpeeds(xCont.calculate(botPose.getX()), yCont.calculate(botPose.getY()), omegaCont.calculate(botPose.getRotation.getRadians())));
    }

    public ChassisSpeeds getPoseSpeeds(Pose2d desiredPose) {
        Transform2d botToTagOffset = new Transform2d(new Translation2d(1.5, 0), new Rotation2d();
        Pose2d botPose = LimelightHelpers.getBotPose2d("limelight");

        xCont.setGoal(desiredPose.getX());
        yCont.setGoal(desiredPose.getY());
        omegaCont.setGoal(desiredPose.getRotation().getRadians);

        return new ChassisSpeeds(xCont.calculate(botPose.getX()), yCont.calculate(botPose.getY()), omegaCont.calculate(botPose.getRotation.getRadians())));
    }
}


