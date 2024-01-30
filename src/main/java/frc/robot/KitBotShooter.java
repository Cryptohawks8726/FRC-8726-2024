// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class KitBotShooter extends SubsystemBase implements BooleanSupplier{
  
  /** Creates a new KitBotShooter. */

  private CANSparkMax TopMotor = new CANSparkMax(0,MotorType.kBrushless);
  private CANSparkMax BottomMotor = new CANSparkMax(0,MotorType.kBrushless);

  private XboxController XBOXcontroller = new XboxController(0);

  private double desiredSpeed;

  public void ChooseSpeed() {
    if (XBOXcontroller.getBButtonPressed()) {
      desiredSpeed = -0.4;
    }
    else if (XBOXcontroller.getXButtonPressed()) {
      desiredSpeed = -0.6;
    }
    else if (XBOXcontroller.getYButtonPressed()) {
      desiredSpeed = -0.2;
    }
  }

  public boolean getAsBoolean() {
    if (XBOXcontroller.getAButtonPressed()) {
      if (TopMotor.getEncoder().getVelocity() > 2000 && TopMotor.getEncoder().getVelocity() < 2100) {
        return true;
      }
    }
    else if (XBOXcontroller.getBButtonPressed()) {
      if (TopMotor.getEncoder().getVelocity() > 2000 && TopMotor.getEncoder().getVelocity() < 2100) {
        return true;
      }
    }
    else if (XBOXcontroller.getXButtonPressed()) {
      if (TopMotor.getEncoder().getVelocity() > 4000 && TopMotor.getEncoder().getVelocity() < 4100) {
        return true;
      }
    }
    else if (XBOXcontroller.getYButtonPressed()) {
      if (TopMotor.getEncoder().getVelocity() > 4000 && TopMotor.getEncoder().getVelocity() < 4100) {
        return true;
      }
    }
    return false;
  }

  public KitBotShooter() {

  }


  public SequentialCommandGroup StartIntake() {
    return
      new InstantCommand(() -> {
          if (XBOXcontroller.getAButtonPressed()) {
            TopMotor.set(0.4);
          }
        }, this)
      .andThen(new WaitUntilCommand(this))
      .andThen(new InstantCommand(() -> {
        BottomMotor.set(0.4);
      },this)

      );
  }

  public SequentialCommandGroup StartShooter(double goal) {
    return
        new InstantCommand(() -> {
          if (XBOXcontroller.getBButtonPressed() || XBOXcontroller.getXButtonPressed() || XBOXcontroller.getYButtonPressed()) {
            TopMotor.set(desiredSpeed);
          }
        }, this)
        .andThen(new WaitUntilCommand(this))
        .andThen(new InstantCommand(() -> {
          BottomMotor.set(desiredSpeed);
        },this)
        
        );
  }

  public void StopMotors() {
    if (XBOXcontroller.getAButtonReleased() || XBOXcontroller.getBButtonReleased() || XBOXcontroller.getXButtonReleased() || XBOXcontroller.getYButtonReleased()) {
      TopMotor.set(0);
      BottomMotor.set(0);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
