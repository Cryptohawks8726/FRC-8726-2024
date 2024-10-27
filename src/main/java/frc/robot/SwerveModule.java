package frc.robot;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.Swerve.ModulePosition;

public class SwerveModule{
    private ModulePosition modPos;
    private CANSparkMax driveMotor, steerMotor;
    private CANcoder absEncoder;
    private double canCoderOffset;
    private Transform2d transformationFromCenter;
    private SparkPIDController driveController,steerController;
    private RelativeEncoder driveEncoder, steerEncoder;
    private SwerveModuleState lastSetState;
    private SwerveModulePosition simulatedPosition;
    private PIDController contSteerController;
    MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs();

    private SimpleMotorFeedforward driveFeedforward;
    public double wantedVoltage; //wanted voltage

    public SwerveModule(Constants.Swerve.Module modConstants){
        modPos = modConstants.modPos;
        driveFeedforward = new SimpleMotorFeedforward(modConstants.kS, modConstants.kV);
        wantedVoltage = 0;

        absEncoder = new CANcoder(modConstants.canCoderid);
        lastSetState = new SwerveModuleState();
      
        canCoderOffset = modConstants.canCoderOffset;
        absEncoder.getConfigurator().apply(magnetSensorConfigs.withMagnetOffset(canCoderOffset/360.0).withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1).withSensorDirection(SensorDirectionValue.CounterClockwise_Positive));
        
        transformationFromCenter = modConstants.displacment;

        driveMotor = new CANSparkMax(modConstants.driveMotorid, MotorType.kBrushless);
        steerMotor = new CANSparkMax(modConstants.steerMotorid, MotorType.kBrushless);
        driveMotor.restoreFactoryDefaults();
        steerMotor.restoreFactoryDefaults();
        
        driveMotor.setIdleMode(IdleMode.kBrake);
        steerMotor.setIdleMode(IdleMode.kBrake);
        
        //lower later
        driveMotor.setSmartCurrentLimit(Constants.Swerve.driveMotorStallCurrentLimit,Constants.Swerve.driveMotorFreeCurrentLimit); 
        steerMotor.setSmartCurrentLimit(Constants.Swerve.steerMotorFreeCurrentLimit);
        
        driveMotor.enableVoltageCompensation(12.0);
        steerMotor.enableVoltageCompensation(12.0);

        driveEncoder = driveMotor.getEncoder();
        steerEncoder = steerMotor.getEncoder();

        // config encoders
        driveEncoder.setPositionConversionFactor(Constants.Swerve.driveConversionFactor); // meters
        driveEncoder.setVelocityConversionFactor(Constants.Swerve.driveConversionFactor / 60.0); // m/s
        
        steerEncoder.setPositionConversionFactor(360.0 / Constants.Swerve.steerGearRatio); // degrees
        steerEncoder.setVelocityConversionFactor(360.0 / Constants.Swerve.steerGearRatio / 60.0); // d/s
        
        

        driveController = driveMotor.getPIDController();
        //steerController = steerMotor.getPIDController();
        //steerController.setPositionPIDWrappingEnabled(true);
        //steerController.setPositionPIDWrappingMaxInput(360);
        //steerController.setPositionPIDWrappingMinInput(0);
        //steerController.setP(Constants.Swerve.kSteerP);
        contSteerController = new PIDController(Constants.Swerve.kSteerP, Constants.Swerve.kSteerI, Constants.Swerve.kSteerD);
        //set the reference angle of pid to the current module angle.
        contSteerController.enableContinuousInput(0, 360);

        driveController.setP(Constants.Swerve.kDriveP,0);
        driveController.setP(0.0,1);
        driveController.setI(Constants.Swerve.kDriveI);
        driveController.setD(Constants.Swerve.kDriveD);
        //driveController.setFF(Constants.Swerve.kDriveFF);
        /* 
        steerController.setP(Constants.Swerve.kSteerP);
        steerController.setI(Constants.Swerve.kSteerI);
        steerController.setD(Constants.Swerve.kSteerD);
        steerController.setFF(Constants.Swerve.kSteerFF);
        steerController.setPositionPIDWrappingMaxInput(180);
        steerController.setPositionPIDWrappingMinInput(-180);*/

        driveMotor.burnFlash();
        steerMotor.burnFlash();
        
        // sim setup
        simulatedPosition = new SwerveModulePosition();

        // sketchy delay to make sure cancoder offsets are saved
        double finishTime = System.currentTimeMillis() + 200;
        while (System.currentTimeMillis() < finishTime) {}
        seedRelativeEncoder();
        
    }

    public ModulePosition getModPos(){
        return modPos;
    }

    public double getAbsPos(){
        return absEncoder.getAbsolutePosition().getValueAsDouble();
    }
    
    public double getRelativePos(){
        return steerEncoder.getPosition();
    }

    public double getRelativeVel() {
        return driveEncoder.getVelocity();
    }

    public double getSteerCurrent() {
        return steerMotor.getOutputCurrent();
    }

    public double getDriveCurrent() {
        return driveMotor.getOutputCurrent();
    }

    public double getOutput(){
        return driveMotor.getAppliedOutput();
    }

    public Transform2d getCenterTransform(){
        return transformationFromCenter;
    }

    public void setEncoderOffset(){
        absEncoder.getConfigurator().apply(magnetSensorConfigs.withMagnetOffset(canCoderOffset/360.0));
    }

    public void seedRelativeEncoder(){
        steerEncoder.setPosition(absEncoder.getAbsolutePosition().getValueAsDouble());
    }
    
    public SwerveModule closedLoopDrive(SwerveModuleState setPoint){
        setPoint = SwerveModuleState.optimize(setPoint, Rotation2d.fromDegrees(getAngle()));
        lastSetState = setPoint;
        driveController.setReference(setPoint.speedMetersPerSecond, ControlType.kVelocity,0,driveFeedforward.calculate(setPoint.speedMetersPerSecond),ArbFFUnits.kVoltage);
        steerMotor.set(contSteerController.calculate(getAngle(), MathUtil.inputModulus(setPoint.angle.getDegrees(), 0, 360)));
        return this;
    }

     public SwerveModule openLoopDrive(SwerveModuleState setPoint){
        setPoint = SwerveModuleState.optimize(setPoint, Rotation2d.fromDegrees(getAngle()));
        lastSetState = setPoint;
        driveController.setReference(setPoint.speedMetersPerSecond, ControlType.kVelocity,1,driveFeedforward.calculate(setPoint.speedMetersPerSecond),ArbFFUnits.kVoltage);
        steerMotor.set(contSteerController.calculate(getAngle(), MathUtil.inputModulus(setPoint.angle.getDegrees(), 0, 360)));
        return this;
    }
    
    public void setBrake(){
        driveMotor.setIdleMode(IdleMode.kBrake); 
    }

    public void setCoast(){
        driveMotor.setIdleMode(IdleMode.kCoast);
    }

    public void setPercentOutput(double output){
        driveMotor.set(output);
    }
    
    public void updateSteerPid(){
        steerMotor.set(contSteerController.calculate(getAngle()));
        
    }
    
    public double getAngle(){
        return absEncoder.getAbsolutePosition().getValueAsDouble()*360;
    }
    public SwerveModuleState getCurrentState(){ 
        return new SwerveModuleState(driveEncoder.getVelocity(),Rotation2d.fromDegrees(getAngle()));
    }
    
    public SwerveModulePosition getCurrentPosition(){ // used for odometry
        return new SwerveModulePosition(driveEncoder.getPosition(),Rotation2d.fromDegrees(getAngle()));
    }

    public SwerveModulePosition getSimulatedPosition(double timeStep){
        double newPosition = simulatedPosition.distanceMeters + (lastSetState.speedMetersPerSecond*timeStep);
        simulatedPosition = new SwerveModulePosition(newPosition, lastSetState.angle);
        return simulatedPosition;
    }
    
    public SwerveModuleState getLastSetState(){
        return lastSetState;
    }
    
    public double getLastSetStateSpeed(){
        return lastSetState.speedMetersPerSecond;
    }

    public double getSetStateAngle(){
        return MathUtil.inputModulus(lastSetState.angle.getDegrees(), 0, 360);
    }

}