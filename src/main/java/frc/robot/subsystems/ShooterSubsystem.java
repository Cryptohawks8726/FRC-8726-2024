package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

public class ShooterSubsystem extends SubsystemBase {
    //Motor Controllers & Motor Encoders For the Conveyor and Flywheel
    private final CANSparkMax conveyorMotor = new CANSparkMax(0, MotorType.kBrushless);
    private final CANSparkMax flywheelMotor = new CANSparkMax(0, MotorType.kBrushless);
    private final RelativeEncoder conveyorEncoder = conveyorMotor.getEncoder();
    private final RelativeEncoder flywheelEncoder = flywheelMotor.getEncoder();

    //Feedforward control
    private double flywheelSetpoint = 500;
    private final double kS = 0;
    private final double kV = 0;
    private final SimpleMotorFeedforward flywheelFeedforward = new SimpleMotorFeedforward(kS, kV);

    //Feedback control
    private final double kP = 0;
    private final double kI = 0;
    private final double kD = 0;
    private final PIDController flywheelFeedback = new PIDController(kP, kI, kD);

    //Decides whether or not the flywheel should be given voltage. If no, the flywheel's voltage is set to 0.
    private boolean flywheelActive = false;

    //Administers voltage to the motors in real-time
    @Override 
    public void periodic() {
        if (flywheelActive) {    
            double feedbackOutputVelocity = flywheelFeedback.calculate(flywheelSetpoint);
            double feedforwardOutputVolts = flywheelFeedforward.calculate(feedbackOutputVelocity);
            flywheelMotor.setVoltage(feedforwardOutputVolts);
        }
        else {
            flywheelMotor.setVoltage(0);
        }
    }

    //InstantCommand which deactivates and activates the flywheel in preparation for firing
    //Only toggles the flywheel if the subsystem is not in the process of firing
    public InstantCommand toggleFlywheel() {
        return new InstantCommand(() -> {
            flywheelActive = !flywheelActive;
        }, this);
    }

    public InstantCommand configureSetpoint(int newSetpoint) {
        return new InstantCommand(() -> {
            flywheelSetpoint = newSetpoint;
        }, this);
    }

    //InstantCommand which activates the index's conveyor belt, transporting a held ring to the flywheel and firing it
    //Only fires if the flywheel is up to speed
    public SequentialCommandGroup shoot() {
        return 
            new InstantCommand(() -> {
                if (Math.abs(flywheelEncoder.getVelocity() - flywheelSetpoint) < 0.5) {
                    conveyorMotor.setVoltage(4);
                }
            }, this)
            .andThen(new WaitCommand(3))
            .andThen(new InstantCommand(() -> {
                conveyorMotor.setVoltage(0);
                flywheelActive = false;
            }, this)
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );
    }
}