package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

public class BetaShooterSubsystem extends SubsystemBase implements BooleanSupplier {
    //Motor Controllers & Motor Encoders For the Conveyor and Flywheel
    private final CANSparkMax conveyorMotor = new CANSparkMax(8, MotorType.kBrushless);
    private final CANSparkMax topFlywheelMotor = new CANSparkMax(11, MotorType.kBrushless);
    private final CANSparkMax bottomFlywheelMotor = new CANSparkMax(30, MotorType.kBrushless);
    private final RelativeEncoder conveyorEncoder = conveyorMotor.getEncoder();
    private final RelativeEncoder topFlywheelEncoder = topFlywheelMotor.getEncoder();
    private final RelativeEncoder bottomFlywheelEncoder = bottomFlywheelMotor.getEncoder();

    //Conveyor setpoint
    private final double conveyorSetpoint = 8;

    //Feedforward control
    private double flywheelSetpoint = 1425; //5700 (Motor RMP Maximum)/4 (Gearbox Ratio)
    private final double kS = 0;
    private final double kV = 12.0/2448;

    //Feedback control
    private final double kP = 0;
    private final double kI = 0;
    private final double kD = 0;
    SparkPIDController topPID = topFlywheelMotor.getPIDController();
    SparkPIDController bottomPID = bottomFlywheelMotor.getPIDController();

    //Color sensor
    private DigitalInput beamBreakSensor = new DigitalInput(0);

    public enum toggleMotorsStates {
        disable,
        enable,
        proceed
    }

    //Configures flywheel motors
    public BetaShooterSubsystem() {
        //Feedforward configuration
        topPID.setFF(kV);
        bottomPID.setFF(kV);

        //Feedback configuration
        topPID.setP(kP);
        topPID.setI(kI);
        topPID.setD(kD);
        bottomPID.setP(kP);
        bottomPID.setI(kI);
        bottomPID.setD(kD);

        topFlywheelMotor.setInverted(true);
        bottomFlywheelMotor.setInverted(false);
        conveyorMotor.setInverted(false);

        conveyorMotor.setSmartCurrentLimit(25);
        topFlywheelMotor.setSmartCurrentLimit(40);
        bottomFlywheelMotor.setSmartCurrentLimit(40);

        

        bottomFlywheelMotor.follow(topFlywheelMotor, true);
    };

    //Implementation of getAsBoolean
    public boolean getAsBoolean() {
        return beamBreakSensor.get();
        //return true;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("TOPMOTOR:", topFlywheelEncoder.getVelocity());
        SmartDashboard.putNumber("BOTTOMMOTOR:", bottomFlywheelEncoder.getVelocity());
    }

    //Activates the flywheels at a designated speed
    public InstantCommand startFlywheels(float motorSpeed) {
        return new InstantCommand(() -> {
            flywheelSetpoint = motorSpeed;
            topPID.setReference(flywheelSetpoint, ControlType.kVelocity, 0, kS, ArbFFUnits.kVoltage);
            bottomPID.setReference(flywheelSetpoint, ControlType.kVelocity, 0, kS, ArbFFUnits.kVoltage);
        });
    }

    public InstantCommand startFlywheels() {
        return new InstantCommand(() -> {
            topPID.setReference(flywheelSetpoint, ControlType.kVoltage, 0, kS, ArbFFUnits.kVoltage);
            bottomPID.setReference(flywheelSetpoint, ControlType.kVoltage, 0, kS, ArbFFUnits.kVoltage);
        });
    }

    //SequentialCommandGroup which activates the intake and activates the flywheel motors once a ring is detected.
    public SequentialCommandGroup startIntake() {
        return 
            toggleMotors(toggleMotorsStates.disable, toggleMotorsStates.enable)
            .andThen(new WaitUntilCommand(this))
            .andThen(new WaitCommand(1))
            .andThen(startFlywheels())
            .andThen(toggleMotors(toggleMotorsStates.proceed, toggleMotorsStates.proceed /*toggleMotorsStates.disable*/)
            );
    }

    public InstantCommand configureSetpoint(int newSetpoint) {
        return new InstantCommand(() -> {
            flywheelSetpoint = newSetpoint;
        }, this);
    }

    //InstantCommand which activates the index's conveyor belt, transporting a held ring to the flywheel and firing it
    //Only fires if the flywheel is up to speed
    public Command fireNote() {
        return 
            new InstantCommand(() -> {
                if (Math.abs(topFlywheelEncoder.getVelocity() - flywheelSetpoint) < 250 && Math.abs(bottomFlywheelEncoder.getVelocity() - flywheelSetpoint) < 250) {
                    toggleMotors(toggleMotorsStates.proceed, toggleMotorsStates.disable).schedule();
                }
                if (true) {
                    System.out.println(topFlywheelEncoder.getVelocity());
                    System.out.println(bottomFlywheelEncoder.getVelocity());
                }
            }, this)
            .andThen(new WaitCommand(1.5))
            .andThen(toggleMotors(toggleMotorsStates.disable, toggleMotorsStates.disable))
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    public InstantCommand toggleMotors(toggleMotorsStates activateFlywheel, toggleMotorsStates activateConveyor) {
        return new InstantCommand(() -> {
           if (activateConveyor == toggleMotorsStates.enable) {
                conveyorMotor.setVoltage(conveyorSetpoint);
            }
            else if (activateConveyor == toggleMotorsStates.disable) {
                conveyorMotor.setVoltage(0);
            }
            if (activateFlywheel == toggleMotorsStates.enable) {
                topPID.setReference(flywheelSetpoint, ControlType.kVoltage, 0, kS, ArbFFUnits.kVoltage);
                bottomPID.setReference(flywheelSetpoint, ControlType.kVoltage, 0, kS, ArbFFUnits.kVoltage);
            }
            else if (activateFlywheel == toggleMotorsStates.disable) {
                topPID.setReference(0, ControlType.kVoltage, 0, kS, ArbFFUnits.kVoltage);
                bottomPID.setReference(0, ControlType.kVoltage, 0, kS, ArbFFUnits.kVoltage);
            }
        });
    }

}

