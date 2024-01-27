package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

public class ShooterSubsystem extends SubsystemBase implements BooleanSupplier {
    //Motor Controllers & Motor Encoders For the Conveyor and Flywheel
    private final CANSparkMax conveyorMotor = new CANSparkMax(9, MotorType.kBrushless);
    private final CANSparkMax topFlywheelMotor = new CANSparkMax(30, MotorType.kBrushless);
    private final CANSparkMax bottomFlywheelMotor = new CANSparkMax(11, MotorType.kBrushless);
    private final RelativeEncoder conveyorEncoder = conveyorMotor.getEncoder();
    private final RelativeEncoder topFlywheelEncoder = topFlywheelMotor.getEncoder();
    private final RelativeEncoder bottomFlywheelEncoder = bottomFlywheelMotor.getEncoder();

    //Conveyor setpoint
    private final double conveyorSetpoint = 2;

    //Feedforward control
    private double flywheelSetpoint = 5600; 
    private final double kS = 0;
    private final double kV = 0.00211;
    private final SimpleMotorFeedforward flywheelFeedforward = new SimpleMotorFeedforward(kS, kV);

    //Feedback control
    private final double kP = 0;
    private final double kI = 0;
    private final double kD = 0;
    private final PIDController flywheelFeedback = new PIDController(kP, kI, kD);

    //Manage whether or not the system's motors recieve power
    private boolean flywheelsActive = false;
    private boolean conveyorActive = false;

    //Color sensor
    private ColorSensorV3 noteSensor = new ColorSensorV3(Port.kMXP);

    //Configures Flywheel Motors
    public ShooterSubsystem() {
        topFlywheelMotor.setInverted(true);
        topFlywheelMotor.setSmartCurrentLimit(40);
        bottomFlywheelMotor.setSmartCurrentLimit(40);
        bottomFlywheelMotor.follow(topFlywheelMotor);
    };

    //Implementation of getAsBoolean
    public boolean getAsBoolean() {
        return noteSensor.getColor() == Color.kOrange;
    }

    //Administers voltage to the motors in real-time
    @Override 
    public void periodic() {
        if (flywheelsActive) { 
            //double feedbackOutputVelocity = flywheelFeedback.calculate(flywheelSetpoint);
            double feedforwardOutputVolts = flywheelFeedforward.calculate(flywheelSetpoint);
            topFlywheelMotor.setVoltage(feedforwardOutputVolts);
        }
        else {
            topFlywheelMotor.setVoltage(0);
        }

        if (conveyorActive) {
            conveyorMotor.setVoltage(conveyorSetpoint);
        }
        else {
            conveyorMotor.setVoltage(0);
        }
    }

    //Activates the flywheels at a designated speed
    public InstantCommand startFlywheels(float motorSpeed) {
        return new InstantCommand(() -> {
            flywheelSetpoint = motorSpeed;
            flywheelsActive = true;
        });
    }

    public InstantCommand startFlywheels() {
        return new InstantCommand(() -> {
            flywheelsActive = true;
        });
    }
    

    //SequentialCommandGroup which activates the intake and activates the flywheel motors once a ring is detected.
    public SequentialCommandGroup startIntake() {
        return 
            new InstantCommand(() -> {
                conveyorActive = true;
            }, this)
            .andThen(new WaitUntilCommand(this))
            .andThen(new InstantCommand(() -> {
                startFlywheels();
                stopMotors(false, true);
            })
        );
    }

    public InstantCommand configureSetpoint(int newSetpoint) {
        return new InstantCommand(() -> {
            flywheelSetpoint = newSetpoint;
        }, this);
    }

    //InstantCommand which activates the index's conveyor belt, transporting a held ring to the flywheel and firing it
    //Only fires if the flywheel is up to speed
    public SequentialCommandGroup fireNote() {
        return 
            new InstantCommand(() -> {
                if (Math.abs(topFlywheelEncoder.getVelocity() - flywheelSetpoint) < 250 && Math.abs(bottomFlywheelEncoder.getVelocity() - flywheelSetpoint) < 250) {
                    conveyorActive = true;
                }
                else {
                    System.out.println("FAIL");
                    System.out.println(topFlywheelEncoder.getVelocity());
                    System.out.println(bottomFlywheelEncoder.getVelocity());
                }
            }, this)
            .andThen(new WaitCommand(1.5))
            .andThen(new InstantCommand(() -> {
                stopMotors(true, true);
            }, this)
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );
    }

    public InstantCommand stopMotors(boolean flywheel, boolean conveyor) {
        return new InstantCommand(() -> {
            if (flywheel) {
                flywheelsActive = false;
            }

            if (conveyor) {
                conveyorActive = false;
            }
        });
    }
}

