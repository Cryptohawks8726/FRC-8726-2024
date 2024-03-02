package frc.robot.subsystems;

import static frc.robot.Constants.Swerve.ModulePosition.BL;
import static frc.robot.Constants.Swerve.ModulePosition.BR;
import static frc.robot.Constants.Swerve.ModulePosition.FL;
import static frc.robot.Constants.Swerve.ModulePosition.FR;

import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;


public class SwerveDrive extends SubsystemBase{
    
    public List<SwerveModule> modules;

    private SwerveModuleState[] modStates;
    private SwerveModuleState[] currentModState;
    private SwerveModulePosition[] modPositionStates;
    private SwerveDriveKinematics kinematics;
    public SwerveDrivePoseEstimator odometry;
    public Pigeon2 gyro;

    private Field2d field; 
    private FieldObject2d[] modPoses;
   
    public SwerveDrive(){
        modules = Arrays.asList(
            new SwerveModule(Constants.Swerve.Module.FR), // 0
            new SwerveModule(Constants.Swerve.Module.BR), // 1
            new SwerveModule(Constants.Swerve.Module.BL), // 2
            new SwerveModule(Constants.Swerve.Module.FL)  // 3
        );

        modPositionStates = new SwerveModulePosition[]{
            modules.get(FR.modPos).getCurrentPosition(),
            modules.get(BR.modPos).getCurrentPosition(),
            modules.get(BL.modPos).getCurrentPosition(),
            modules.get(FL.modPos).getCurrentPosition()
        };

        currentModState = new SwerveModuleState[]{
            modules.get(FR.modPos).getCurrentState(),
            modules.get(BR.modPos).getCurrentState(),
            modules.get(BL.modPos).getCurrentState(),
            modules.get(FL.modPos).getCurrentState()
        };
        
        kinematics = new SwerveDriveKinematics(
            modules.get(FR.modPos).getCenterTransform().getTranslation(),
            modules.get(BR.modPos).getCenterTransform().getTranslation(),
            modules.get(BL.modPos).getCenterTransform().getTranslation(),
            modules.get(FL.modPos).getCenterTransform().getTranslation()
        );
        
        gyro = new Pigeon2(Constants.Swerve.pigeonId);
        gyro.setYaw(0.0);

        odometry = new SwerveDrivePoseEstimator(kinematics, new Rotation2d(), modPositionStates, new Pose2d()); 
        
        field = new Field2d();
        modPoses = new FieldObject2d[]{
            field.getObject("modFR"),
            field.getObject("modBR"),
            field.getObject("modBL"),
            field.getObject("modFL")
        };
        SmartDashboard.putData("Field", field);
        resetOdometry(new Pose2d());

        AutoBuilder.configureHolonomic(
            this::getPoseEstimate, 
            this::setOdometryPosition, 
            this::getRobotRelativeSpeeds, 
            this::drive, 
            new HolonomicPathFollowerConfig( 
                    new PIDConstants(5, 0.0, 0), 
                    new PIDConstants(5, 0, 0),
                    4.5, 
                    Constants.Swerve.driveBaseLength/2, 
                    new ReplanningConfig()
            ),
            () -> {


              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Blue;
              }
              return false;
            },
            this 
        );
    }

    @Override
    public void periodic(){
        odometry.update(
            gyro.getRotation2d(), 
            getSwerveModulePositions()
        );

        modules.forEach(mod->{mod.updateSteerPid();}); 
        
        field.setRobotPose(odometry.getEstimatedPosition());
        logValues(false);

    }
     
    public void drive(ChassisSpeeds robotSpeeds, boolean isClosedLoop){  
        robotSpeeds = ChassisSpeeds.discretize(robotSpeeds, 0.2);
        modStates = kinematics.toSwerveModuleStates(robotSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(modStates,Constants.Swerve.maxSpeed);

        if (isClosedLoop){
            modules.forEach(mod -> {mod.closedLoopDrive(modStates[mod.getModPos().getVal()]);});
        } 
        else if(!isClosedLoop){
            modules.forEach(mod -> {mod.openLoopDrive(modStates[mod.getModPos().getVal()]);});
        }
        
    }
    
    public void drive(ChassisSpeeds robotSpeeds){
        drive(robotSpeeds,false);
        
    }

    public StartEndCommand passiveBrake(){
        SwerveModuleState leftToRight = new SwerveModuleState(0.0,Rotation2d.fromDegrees(45));
        SwerveModuleState rightToLeft = new SwerveModuleState(0.0, Rotation2d.fromDegrees(135));
        return new StartEndCommand(
            () -> modules.forEach(mod -> {mod.closedLoopDrive((mod.getModPos().getVal() %2 == 0) ? leftToRight : rightToLeft).setBrake();}), 
            () -> modules.forEach(mod -> {mod.setCoast();}), 
            this
        );
    }

    private Pigeon2 getGyro(){
        return gyro;
    }

    public void normalZeroModules(){
        modules.forEach(mod -> {mod.closedLoopDrive(new SwerveModuleState(0,Rotation2d.fromDegrees(0)));});
    }

    public Pose2d getPoseEstimate(){
        return odometry.getEstimatedPosition();
    }

    public void setOdometryPosition(Pose2d setPosition){
        odometry.resetPosition(getRobotAngle(), getSwerveModulePositions(), setPosition);
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(Rotation2d.fromDegrees(0.0), getSwerveModulePositions(), pose);
    }

    public Rotation2d getRobotAngle() {
        return odometry.getEstimatedPosition().getRotation();
    }

    public SwerveModulePosition[] getSwerveModulePositions(){
        modules.forEach(mod -> {modPositionStates[mod.getModPos().getVal()] = mod.getCurrentPosition();});
        return modPositionStates;
    }

    public SwerveModuleState[] getSwerveModuleStates(){
        modules.forEach(mod -> {currentModState[mod.getModPos().getVal()] = mod.getCurrentState();});
        return currentModState;
    }

    public ChassisSpeeds getRobotRelativeSpeeds(){
        return kinematics.toChassisSpeeds(getSwerveModuleStates());
    }

    public void setModuleStates(SwerveModuleState[] updatedstates){
        SwerveDriveKinematics.desaturateWheelSpeeds(updatedstates, Constants.Swerve.maxSpeed);
        modules.forEach(mod -> {mod.closedLoopDrive(updatedstates[mod.getModPos().getVal()]);});
    }

    public void setEncoderOffsets(){
        modules.forEach(mod -> {mod.setEncoderOffset();});
    }

    public InstantCommand resetGyroAngle(){
        return new InstantCommand(() -> resetOdometry(new Pose2d()));
    }

    public void logValues(boolean moduleLevel){ 
        Pose2d estimatedPostition = odometry.getEstimatedPosition();

        SmartDashboard.putNumber("xpos", estimatedPostition.getTranslation().getX());
        SmartDashboard.putNumber("ypos", estimatedPostition.getTranslation().getY());
        SmartDashboard.putNumber("robotAngleFull", getRobotAngle().getDegrees());
        SmartDashboard.putNumber("robotAngleAbs", getRobotAngle().getDegrees()%360);
        if(moduleLevel){
            for (SwerveModule module : modules) {
                String modName = module.getModPos().toString();
                module.seedRelativeEncoder();
                SmartDashboard.putNumber(modName + "setvel", module.getLastSetState().speedMetersPerSecond);
                SmartDashboard.putNumber(modName + "actvel", module.getCurrentState().speedMetersPerSecond);
                SmartDashboard.putNumber(modName + "setdeg", module.getSetStateAngle());
                SmartDashboard.putNumber(modName + "actdeg", module.getCurrentState().angle.getDegrees());
            }
        }
        
    }
}