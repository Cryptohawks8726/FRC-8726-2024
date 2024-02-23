package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

public class MainShooterSubsystem extends SubsystemBase {
    // Motor Controllers & Motor Encoders For the Conveyor and Flywheel
    private final CANSparkMax conveyorMotor = new CANSparkMax(60, MotorType.kBrushless);
    public final CANSparkMax topFlywheelMotor = new CANSparkMax(62, MotorType.kBrushless);
    public final CANSparkMax bottomFlywheelMotor = new CANSparkMax(61, MotorType.kBrushless);
    private final RelativeEncoder topFlywheelEncoder = topFlywheelMotor.getEncoder();
    private final RelativeEncoder bottomFlywheelEncoder = bottomFlywheelMotor.getEncoder();

    private double conveyorSetpoint = 7; // TODO: Since this and flywheelSetpoint change during operation, they should
                                         // be initalized at the end of the constructor to make their inital values
                                         // clear.

    // Feedforward control
    private double flywheelSetpoint = 5700; // Theoretical, get experimental value
    private final double kSTop = 0.0;
    private final double ksBottom = 0.0;
    private double testSetpoint = 0;
    private final double kV = 1.0 / 5700.0;

    // Feedback control
    private final double kP = 0.0001;
    private final double kI = 0;
    private final double kD = 0; // 0.05 works, but causes the motors to tick. This could be resolved by a
                                 // bang-bang controller.;
    SparkPIDController topPID = topFlywheelMotor.getPIDController();
    SparkPIDController bottomPID = bottomFlywheelMotor.getPIDController();

    private DigitalInput beamBreakSensor = new DigitalInput(3); // TODO: Wire the emitter to signal - ground to allow
                                                                // for self-tests of the sensor

    // Enum for potential motor states, used when modifying motor states via
    // toggleMotors
    public enum toggleMotorsStates {
        disable,
        enable,
        proceed
    }

    /*
     * TODO: Refactor to remove toggleMotors().
     * This makes the code unnecesarily indirect and verbose, and could easily be
     * replaced by direct setVoltage() calls to the conveyor motors and a
     * configureSetpoint() call instead. The conveyor setVoltage() call could be
     * wrapped to allow for easy setpoint logging.
     */

    // Configures flywheel motors
    public MainShooterSubsystem() {

        topPID.setFF(kV);
        bottomPID.setFF(kV);

        topPID.setP(kP);
        topPID.setI(kI);
        topPID.setD(kD);
        bottomPID.setP(kP);
        bottomPID.setI(kI);
        bottomPID.setD(kD);

        topFlywheelMotor.setInverted(false);
        bottomFlywheelMotor.setInverted(false);
        conveyorMotor.setInverted(false);

        topFlywheelMotor.setSmartCurrentLimit(40);
        bottomFlywheelMotor.setSmartCurrentLimit(40);
        conveyorMotor.setSmartCurrentLimit(25);

        topFlywheelMotor.enableVoltageCompensation(12.0);
        bottomFlywheelMotor.enableVoltageCompensation(12.0);
        conveyorMotor.enableVoltageCompensation(12.0);
    };

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Top Flywheel Vel", topFlywheelEncoder.getVelocity());
        SmartDashboard.putNumber("Bottom Flywheel Vel", bottomFlywheelEncoder.getVelocity());
        SmartDashboard.putBoolean("Beam Broken", !beamBreakSensor.get());
    }

    /*
     * Returns an InstantCommand to set the flywheel speed reference.
     * Warning: This reference is currently followed by a PID controller, so setting
     * a speed substantially slower than or in the opposite direction
     * of current motion may cause damage to the shooter belts
     * 
     * @param motorSpeed the desired RPM (-5700,5700) acheivable
     * 
     * @return The InstantCommand to set the flywheel reference
     */
    // TODO: Prevent potentially damaging refrences from being set by checking
    // current flywheel velocity OR rewrite with a bang-bang controller.
    private InstantCommand startFlywheels(double motorSpeed) {
        return new InstantCommand(() -> {
            flywheelSetpoint = motorSpeed;
            topPID.setReference(flywheelSetpoint, ControlType.kVelocity, 0, kSTop, ArbFFUnits.kVoltage);
            bottomPID.setReference(flywheelSetpoint, ControlType.kVelocity, 0, ksBottom, ArbFFUnits.kVoltage);
        });
    }

    /*
     * Returns a SequentialCommandGroup that starts the intake conveyor until the
     * beam break sensor is triggered.
     * 
     * @Return The SequentialCommandGroup to run the intake sequence
     */
    public SequentialCommandGroup startIntake() {
        return toggleMotors(toggleMotorsStates.proceed, toggleMotorsStates.enable)
                .andThen(new WaitUntilCommand(() -> isBeamBroken()))
                .andThen(new WaitCommand(0.0625))
                .andThen(toggleMotors(toggleMotorsStates.disable, toggleMotorsStates.disable));
    }

    public InstantCommand configureSetpoint(int newSetpoint) {
        return new InstantCommand(() -> {
            flywheelSetpoint = newSetpoint;
        }, this);
    }

    // InstantCommand which activates the index's conveyor belt, transporting a held
    // ring to the flywheel and firing it
    // Only fires if the flywheel is up to speed
    public Command fireNote(boolean isAmp) {
        return startFlywheels(isAmp ? 1000.0 : 5700.0)
                .andThen(new WaitUntilCommand(() -> Math.abs(topFlywheelEncoder.getVelocity() - flywheelSetpoint) < 500
                        && Math.abs(bottomFlywheelEncoder.getVelocity() - flywheelSetpoint) < 500)) // Lower tolerance
                                                                                                    // range in the
                                                                                                    // future
                .andThen(new InstantCommand(() -> {
                    conveyorSetpoint = 12;
                    conveyorMotor.setSmartCurrentLimit(35);
                    System.out.println(topFlywheelEncoder.getVelocity());
                    System.out.println(bottomFlywheelEncoder.getVelocity());
                }, this))
                .andThen(toggleMotors(toggleMotorsStates.proceed, toggleMotorsStates.enable)) // proceed seems to do
                                                                                              // nothing here. What was
                                                                                              // the intended logic?
                .andThen(new WaitUntilCommand(() -> !isBeamBroken()))
                .andThen(new InstantCommand(() -> {
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
            } else if (activateConveyor == toggleMotorsStates.disable) {
                conveyorMotor.setVoltage(0);
            }
            if (activateFlywheel == toggleMotorsStates.enable) {
                topPID.setReference(flywheelSetpoint, ControlType.kVelocity, 0, kSTop, ArbFFUnits.kVoltage);
                bottomPID.setReference(flywheelSetpoint, ControlType.kVelocity, 0, ksBottom, ArbFFUnits.kVoltage);
            } else if (activateFlywheel == toggleMotorsStates.disable) {
                // Control mode is set to kVoltage to let the flywheels coast to zero. This is
                // unneccesary for a bang-bang controller
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

    public boolean isBeamBroken() {
        return !beamBreakSensor.get();
    }
}
