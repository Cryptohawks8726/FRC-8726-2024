package frc.robot.subsystems;

import static frc.robot.Constants.Swerve.ModulePosition.BL;
import static frc.robot.Constants.Swerve.ModulePosition.BR;
import static frc.robot.Constants.Swerve.ModulePosition.FL;
import static frc.robot.Constants.Swerve.ModulePosition.FR;

import java.util.Arrays;
import java.util.List;


import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.proto.Kinematics;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SerialPort;
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
    //public AHRS gyro;
    public Pigeon2 gyro;

    private Field2d field; 
    private FieldObject2d[] modPoses;
    private Pigeon2SimState gyroSim;
    private ChassisSpeeds lastSetSpeeds;
   
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
        gyro.setYaw(180);//TODO: Remove


        if(RobotBase.isSimulation()){
            gyroSim = gyro.getSimState();
            lastSetSpeeds = new ChassisSpeeds();
        }

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

        // Configure AutoBuilder last
        AutoBuilder.configureHolonomic(
            this::getPoseEstimate, // Robot pose supplier
            this::setOdometryPosition, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(1.5, 0.0, 0), // Translation PID constants
                    new PIDConstants(1.25, 0, 0), // Rotation PID constants
                    4.5, // Mgax module speed, in m/s
                    Constants.Swerve.driveBaseLength/2, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
        );
    }

    @Override
    public void periodic(){
        odometry.update(
            gyro.getRotation2d(), 
            getSwerveModulePositions()
        );

        modules.forEach(mod->{mod.updateSteerPid();}); 
        // show estimated robot and mod poses on dashboard
        field.setRobotPose(odometry.getEstimatedPosition());
    
        /*for (int i = 0;i<4;i++){
            modPoses[i].setPose(
                odometry.getEstimatedPosition()
                .plus(
                    modules.get(i).getCenterTransform()
                    .plus
                        (new Transform2d(new Translation2d(),modules.get(i).getCurrentState().angle))
                    )
            );
        }*/
        
        logValues(true);

    }
    @Override
    public void simulationPeriodic(){
        gyroSim.addYaw(Math.toDegrees(0.02*lastSetSpeeds.omegaRadiansPerSecond));
    }

    
    public void drive(ChassisSpeeds robotSpeeds, boolean isClosedLoop){  
        robotSpeeds = ChassisSpeeds.discretize(robotSpeeds, 0.2);
        lastSetSpeeds = robotSpeeds;
        modStates = kinematics.toSwerveModuleStates(robotSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(modStates,Constants.Swerve.maxSpeed);
        //SmartDashboard.putNumber("expectedRotation", robotSpeeds.omegaRadiansPerSecond);
        //SmartDashboard.putNumber("actualRotation", getRobotRelativeSpeeds().omegaRadiansPerSecond);
        //SmartDashboard.putNumber("chassisX", robotSpeeds.vxMetersPerSecond);
        //SmartDashboard.putNumber("actualX", getRobotRelativeSpeeds().vxMetersPerSecond);
        double endPointX = 9.88353157043457; //in meters
        double startPointX = 2.889869213104248;
        
        double endPointY = 7.009701728820801;
        double startPointY = 7.009701728820801;

        //SmartDashboard.putNumber("supposedDistance", endPointX - startPointX);
        //SmartDashboard.putNumber("rotationPosition", gyro.getAngle());
        //SmartDashboard.putNumber("relativeSpeed", getRobotRelativeSpeeds().omegaRadiansPerSecond);
        /*SmartDashboard.putNumber("estimatedPositionX", endPointX - odometry.getEstimatedPosition().getX());
        SmartDashboard.putNumber("estimatedAngle", getRobotAngle().getDegrees() - 180);
        SmartDashboard.putNumber("estimatedPositionY", 7.009701728820801 - odometry.getEstimatedPosition().getY());
        SmartDashboard.putNumber("Gyro angle:", getRobotAngle().getDegrees()%360);
        
        SmartDashboard.putNumber("moduleSpeed0", modules.get(0).getRelativeVel());
        SmartDashboard.putNumber("moduleSpeed1", modules.get(1).getRelativeVel());
        SmartDashboard.putNumber("moduleSpeed2", modules.get(2).getRelativeVel());
        SmartDashboard.putNumber("moduleSpeed3", modules.get(3).getRelativeVel());*/

        SmartDashboard.putNumber("", 0);
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
        odometry.resetPosition(gyro.getRotation2d(), getSwerveModulePositions(), setPosition);
        //gyro.setYaw(setPosition.getRotation().getDegrees()-(gyro.getRotation2d().getDegrees()%360));
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(Rotation2d.fromDegrees(0.0), getSwerveModulePositions(), pose);
        SmartDashboard.putNumber("resetOdometryAngle",getRobotAngle().getDegrees());
        //gyro.setYaw(0.0);
        //gyro.setYaw(gyro.getRotation2d().getDegrees()%360);

        //gyro.reset();
    }

    public Rotation2d getRobotAngle() {
        // return Rotation2d.fromDegrees(gyro.getYaw());
        return odometry.getEstimatedPosition().getRotation();
    }

    public SwerveModulePosition[] getSwerveModulePositions(){
        if(RobotBase.isReal()){
            modules.forEach(mod -> {modPositionStates[mod.getModPos().getVal()] = mod.getCurrentPosition();});
        }else{
            modules.forEach(mod -> {modPositionStates[mod.getModPos().getVal()] = mod.getSimulatedPosition(0.02);});
        }
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
        return new InstantCommand(() -> odometry.resetPosition(gyro.getRotation2d(), getSwerveModulePositions(), new Pose2d(odometry.getEstimatedPosition().getTranslation(),Rotation2d.fromDegrees(0.0)))
);
    }

    public void logValues(boolean moduleLevel){ 
        Pose2d estimatedPostition = odometry.getEstimatedPosition();
        SmartDashboard.putData(field);
        SmartDashboard.putNumber("xpos", estimatedPostition.getTranslation().getX());
        SmartDashboard.putNumber("ypos", estimatedPostition.getTranslation().getY());
        //SmartDashboard.putNumber("estimatedthetaPos",estimatedPostition.getRotation().getDegrees());
        SmartDashboard.putNumber("robotAngleFull", getRobotAngle().getDegrees());//getRotation2d().getDegrees()%360
        SmartDashboard.putNumber("robotAngleFull", getRobotAngle().getDegrees()%360);
        //SmartDashboard.putBoolean("isGyroConnected", gyro.isConnected());
        //SmartDashboard.putNumber("CalcThetaVel", getRobotRelativeSpeeds().omegaRadiansPerSecond);
       // SmartDashboard.putNumber("setXVel", lastSetChassisSpeeds.vxMetersPerSecond);
        //SmartDashboard.putNumber("setYVel", lastSetChassisSpeeds.vyMetersPerSecond);
        if(moduleLevel){
            for (SwerveModule module : modules) {
                String modName = module.getModPos().toString();
                //module.seedRelativeEncoder();
                SmartDashboard.putNumber(modName + "setvel", module.getLastSetState().speedMetersPerSecond);
                SmartDashboard.putNumber(modName + "actvel", module.getCurrentState().speedMetersPerSecond);
                //SmartDashboard.putNumber(modName + "setdeg", module.getSetStateAngle());
                //SmartDashboard.putNumber(modName + "actdeg", module.getCurrentState().angle.getDegrees());
                SmartDashboard.putNumber(modName + "outputPer",module.getOutput());
                SmartDashboard.putNumber(modName + "driveCurrent",module.getDriveCurrent());
                
            // SmartDashboard.putNumber(modName + "absdeg", module.getAbsPos());
            //  SmartDashboard.putNumber(modName + "built in steer", module.getRelativePos());
            // SmartDashboard.putNumber(modName + "built in drive", module.getRelativeVel());
                //SmartDashboard.putNumber(modName + "drive current", module.getDriveCurrent());
                //SmartDashboard.putNumber(modName + "steer current", module.getSteerCurrent());
                
            }
        }
        
    }
}