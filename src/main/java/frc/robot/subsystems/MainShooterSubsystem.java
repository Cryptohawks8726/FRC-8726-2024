package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

public class MainShooterSubsystem extends SubsystemBase implements BooleanSupplier {
    //Motor Controllers & Motor Encoders For the Conveyor and Flywheel
    private final CANSparkMax conveyorMotor = new CANSparkMax(8, MotorType.kBrushless);
    public final CANSparkMax topFlywheelMotor = new CANSparkMax(11, MotorType.kBrushless);
    public final CANSparkMax bottomFlywheelMotor = new CANSparkMax(30, MotorType.kBrushless);
    private final RelativeEncoder topFlywheelEncoder = topFlywheelMotor.getEncoder();
    private final RelativeEncoder bottomFlywheelEncoder = bottomFlywheelMotor.getEncoder();

    //Conveyor setpoint
    private double conveyorSetpoint = 7;

    //Feedforward control
    private double flywheelSetpoint = 300; //5700 (Motor RMP Maximum)/4 (Gearbox Ratio)
    private final double kSTop = 0.15;
    private final double ksBottom = 0.14;
    private double testSetpoint = 0;
    private final double kV = 12.0/2448;

    //Feedback control
    private final double kP = 1;
    private final double kI = 0;
    private final double kD = 0;
    SparkPIDController topPID = topFlywheelMotor.getPIDController();
    SparkPIDController bottomPID = bottomFlywheelMotor.getPIDController();

    //Color sensor
    private DigitalInput beamBreakSensor = new DigitalInput(0);

    //Enum for potential motor states, used when modifying motor states via toggleMotors
    public enum toggleMotorsStates {
        disable,
        enable,
        proceed
    }

    private boolean isShooting = false;

    //Configures flywheel motors
    public MainShooterSubsystem() {
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

        topFlywheelMotor.setInverted(false);
        bottomFlywheelMotor.setInverted(true);
        conveyorMotor.setInverted(false);

        topFlywheelMotor.setSmartCurrentLimit(40);
        bottomFlywheelMotor.setSmartCurrentLimit(40);
        conveyorMotor.setSmartCurrentLimit(25);
    };

    //Implementation of getAsBoolean
    public boolean getAsBoolean() {
        if (isShooting) {
            return beamBreakSensor.get();
        }
        else {
            return !beamBreakSensor.get();
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("TOPMOTOR:", topFlywheelEncoder.getVelocity());
        SmartDashboard.putNumber("BOTTOMMOTOR:", bottomFlywheelEncoder.getVelocity());
        SmartDashboard.putBoolean("BEAMBREAKSTATE", getAsBoolean());
    }

    //Activates the flywheels at a designated speed
    public InstantCommand startFlywheels(double motorSpeed) {
        return new InstantCommand(() -> {
            flywheelSetpoint = motorSpeed;
            topPID.setReference(flywheelSetpoint, ControlType.kVelocity, 0, kSTop, ArbFFUnits.kVoltage);
            bottomPID.setReference(flywheelSetpoint, ControlType.kVelocity, 0, ksBottom, ArbFFUnits.kVoltage);
        });
    }

    public InstantCommand startFlywheels() {
        return new InstantCommand(() -> {
            topPID.setReference(flywheelSetpoint, ControlType.kVoltage, 0, kSTop, ArbFFUnits.kVoltage);
            bottomPID.setReference(flywheelSetpoint, ControlType.kVoltage, 0, ksBottom, ArbFFUnits.kVoltage);
        });
    }

    //SequentialCommandGroup which activates the intake and activates the flywheel motors once a ring is detected.
    public SequentialCommandGroup startIntake() {
        return 
            toggleMotors(toggleMotorsStates.disable, toggleMotorsStates.enable)
            .andThen(new WaitUntilCommand(this))
            .andThen(toggleMotors(toggleMotorsStates.disable, toggleMotorsStates.disable)
            .andThen(new WaitCommand(1))
            .andThen(startFlywheels()));
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
                if (Math.abs(topFlywheelEncoder.getVelocity() - 5700) < 250 && Math.abs(bottomFlywheelEncoder.getVelocity() - 5700) < 250) {
                    System.out.println("yeag");
                    conveyorSetpoint = 12;
                    isShooting = true;
                    conveyorMotor.setSmartCurrentLimit(35);
                    toggleMotors(toggleMotorsStates.proceed, toggleMotorsStates.enable).schedule();
                }
                else {
                    System.out.println(topFlywheelEncoder.getVelocity());
                    System.out.println(bottomFlywheelEncoder.getVelocity());
                }
            }, this)
            .andThen(new WaitUntilCommand(this))
            .andThen(new InstantCommand(() -> {
                isShooting = false;
                conveyorSetpoint = 6;
                conveyorMotor.setSmartCurrentLimit(25);
            }))
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
                topPID.setReference(flywheelSetpoint, ControlType.kVoltage, 0, kSTop, ArbFFUnits.kVoltage);
                bottomPID.setReference(flywheelSetpoint, ControlType.kVoltage, 0, ksBottom, ArbFFUnits.kVoltage);
            }
            else if (activateFlywheel == toggleMotorsStates.disable) {
                topPID.setReference(0, ControlType.kVoltage, 0, 0, ArbFFUnits.kVoltage);
                bottomPID.setReference(0, ControlType.kVoltage, 0, 0, ArbFFUnits.kVoltage);
            }
        });
    }

    public Command staticGainTest() {
        return new InstantCommand(() -> {
            testSetpoint += 0.01;
            SmartDashboard.putNumber("test setpoint", testSetpoint);
            bottomFlywheelMotor.setVoltage(testSetpoint);
        });
    }
}

