package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Field;

public class ArmSubsystem extends SubsystemBase{

    int shooterOffset = 135; // TODO: get real offset angle (around 135)

    double shooterVel = 14; // TODO: get initial velocity of shooter
    double maxAmpAngle = 121.5;
   
    private static final double kp = 4.8;//can be increased a bit
    private static final double ki = 0;
    private static final double kd = 0.5;
    //private static final double downKp = 2.5;

    double ks = 0.135;
    double kg = 0.33; //0.33
    double kv = 6.8; //3.62 
    double ka = 0.05; //0.02
    private static final double DEG_TO_RAD = 0.017453292519943295; 
    
    private static final CANSparkMax motorController = new CANSparkMax(Arm.rightMotorId, MotorType.kBrushless);
    private static final CANSparkMax motorController2 = new CANSparkMax(Arm.leftMotorId, MotorType.kBrushless);
    private static final SparkAbsoluteEncoder absoluteEncoder = motorController.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    private static final RelativeEncoder rightEncoder = motorController.getEncoder();
    private static final RelativeEncoder leftEncoder = motorController2.getEncoder();

    private final Constraints constraints = new Constraints(1.5, 15.0); // TODO: find good constraints (in rad/s)
    private final Constraints downconstraints = new Constraints(1, 10.0);
    private static final PIDController pidController = new PIDController(kp, ki, kd); // degrees
    private final ArmFeedforward armFF = new ArmFeedforward(ks, kg, kv, ka);
    private final TrapezoidProfile trapezoidProfile = new TrapezoidProfile(constraints); //TODO: Set down constraints with more limited acceleration
    private final TrapezoidProfile downProfile = new TrapezoidProfile(downconstraints);
    private State calculateAngleState = new State(0, 0); // will be changed based on calculated angle
    private static State goal = new State(); // will be changed to reflect current goal
    private double t;
    private boolean down;

    public ArmSubsystem() {
        absoluteEncoder.setPositionConversionFactor(360);
        absoluteEncoder.setVelocityConversionFactor(360);
        absoluteEncoder.setZeroOffset(Arm.absEncoderOffset);
        absoluteEncoder.setInverted(true);

        motorController.setIdleMode(IdleMode.kBrake); 
        motorController2.setIdleMode(IdleMode.kBrake);

        motorController.setInverted(true);

        motorController2.follow(motorController,true);
        
        goal = new State(getArmRad(),0);
        pidController.setSetpoint(getArmRad());
        t = 0.0;
        motorController.burnFlash();
        motorController2.burnFlash();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("isAtAmp", this.atStatePos(Arm.ampState));
        
            t += 0.02;
            State setpoint;
            if(!down){
                setpoint = trapezoidProfile.calculate(t,
                    new State(getArmRad(), toRads(absoluteEncoder.getVelocity())), goal);
                pidController.setP(kp);
            
            }else{
            setpoint = downProfile.calculate(t,
                    new State(getArmRad(), toRads(absoluteEncoder.getVelocity())), goal);
                pidController.setP(3.0);
            }
            double ff = armFF.calculate(setpoint.position - toRads(17.0), setpoint.velocity);// 14 degrees accounts for
                                                                                             // offset from parallel
            pidController.setSetpoint(setpoint.position);
            double pidOutput = pidController.calculate(toRads(getArmDeg()));

           //if(getArmDeg()<4 && (pidOutput + ff) >0 ){motorController.setVoltage(0)}else{motorControl
            motorController.setVoltage(pidOutput + ff);
            SmartDashboard.putNumber("Applied Voltage", pidOutput + ff);
            SmartDashboard.putNumber("Set Pos", setpoint.position);
            SmartDashboard.putNumber("Goal", goal.position);
            SmartDashboard.putNumber("armVelSetRad", setpoint.velocity);
            SmartDashboard.putNumber("FF", ff);
            SmartDashboard.putNumber("PID", pidOutput);
            SmartDashboard.putNumber("leftArmVel",leftEncoder.getVelocity());
            SmartDashboard.putNumber("rightArmVel", rightEncoder.getVelocity());
            SmartDashboard.putNumber("rightArmCurrent",motorController.getOutputCurrent());
            SmartDashboard.putNumber("leftArmCurrent",motorController2.getOutputCurrent());

            motorController.setIdleMode(IdleMode.kCoast);
            motorController2.setIdleMode(IdleMode.kCoast);
       
            //motorController.setVoltage(0.0);
            //motorController.setIdleMode(IdleMode.kCoast);
            //motorController2.setIdleMode(IdleMode.kCoast);
            //SmartDashboard.putNumber("Set Pos", Arm.intakeState.position);
            if(getArmDeg()<10){
                rotateToState(Arm.tempIntakeState);
                down = false;
                //SmartDashboard.putNumber("Set Pos", Arm.tempIntakeState.position);
            }
            
        
       // motorController2.setVoltage(pidOutput+ff);

        SmartDashboard.putNumber("Arm Rad",getArmRad());
        SmartDashboard.putNumber("armDeg", getArmDeg());
        SmartDashboard.putNumber("armVelRad", toRads(absoluteEncoder.getVelocity()));
        
        SmartDashboard.putNumber("error",pidController.getPositionError());
        SmartDashboard.putNumber("Setpoint diff", Math.abs(getArmRad() - goal.position));
    }

    /*
     * Returns true if the arm is within 5 degrees of the passed angle
     * @param Ideal arm state, in radians
     * @param True if the current angle is within 5 degrees of the goal
     */
    public boolean atStatePos(State goal) { // will check if the arm angle is at the setpoint periodically
        if (Math.abs(getArmRad() - goal.position) < toRads(10)) {
            return true;
        } else {
            return false;
        }
    }

    public static double toRads(double degrees) {
        return degrees * DEG_TO_RAD;
    }

    /**
     * in degrees
     */
    public double getArmDeg() {
        return absoluteEncoder.getPosition();
    }

    public static double getArmRad() {
        return toRads(absoluteEncoder.getPosition());
    }

    /**
     * in degrees
     */
    public double getShooterAngle() {
        return 180 - (getArmDeg() + shooterOffset);
    }

    private Command rotateToIntake() { // sets the set point, angle to intake @ ground
        return new InstantCommand(() -> {
            goal = Arm.intakeState; 
            t = 0.0;
        })
        .andThen(new WaitUntilCommand(()->this.getArmDeg()<75))
        .andThen(new InstantCommand(()->{
            down = true;
            SmartDashboard.putString("At zero", "true");
        })).withName("Rotate to intake"); 
    }


    public Command rotateToState(State goal){
        if(goal == Arm.intakeState){
            return rotateToIntake();
        }else{
            return new InstantCommand(() -> {
                down=false;
                this.goal = goal;
            t=0.0;});
        }
    }

    /**
     * calculates the angle to shoot to the speaker at any field pos
     * @param posX the field position of the robot relative to the speaker
     * @return angle (in degrees)
     */
    double calculateAngle(double posX, double posY) { // position relative to subwoofer
        double horDist = Math.sqrt(posX*posX+posY*posY);

        // the angle that the shooter should be
        double shooterAngle = Math.atan(shooterVel + Math.sqrt(Math.pow(shooterVel, 4) - 9.81*(9.81*horDist*horDist + 2*Field.speakerHeight*shooterVel*shooterVel)) / (9.81*horDist));
        
        double armAngle = 180 - Math.toDegrees(shooterAngle) - shooterOffset;
        
        // TODO: consider shooting at an angle a little more because the hood is pretty big
        return armAngle;
    }

    /**
     * calculates the angle to shoot to the speaker at any field pos and rotates the arm
     */
    public InstantCommand rotateToCalculatedAngle(double posX, double posY) {
        calculateAngleState.position = toRads(calculateAngle(posX, posY));

        return new InstantCommand(() -> goal = calculateAngleState); // in degrees
    }

    public static void initArmSetpoint(){
        goal = new State(getArmRad(),0);
        pidController.setSetpoint(getArmRad());
    }
    
}
