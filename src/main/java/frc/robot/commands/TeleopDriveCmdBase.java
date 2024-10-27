// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.SwerveDrive;

public abstract class TeleopDriveCmdBase extends CommandBase {
  private final SwerveDrive drivetrain;
  private double lastHeading;
  private boolean isHeadingSet;
  private PIDController headingPID;
  
  public TeleopDriveCmdBase(SwerveDrive drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
    headingPID = new PIDController(Swerve.kHeadingP,Swerve.kHeadingI, Swerve.kHeadingD, 20);
    headingPID.enableContinuousInput(0, 360);
  }

  public abstract boolean isRobotRelative();

  public abstract double[] applyMultipliers(double[] rawVel);

  public abstract double[] getRawInputs();

  public abstract double[] applyDeadbands(double[] rawInputs);

  public double[] getComputedVelocities(){
    return applyMultipliers(applyDeadbands(getRawInputs()));
  }

  public double getHeadingOutput(){
    if (isHeadingSet == false){
      headingPID.reset();
      isHeadingSet = true;
      lastHeading = (drivetrain.gyro.getYaw().getValue())+180%360;
      headingPID.setSetpoint(lastHeading);
      return headingPID.calculate(drivetrain.gyro.getYaw().getValue()+180%360);
   }else{
      return headingPID.calculate(drivetrain.gyro.getYaw().getValue()+180%360);
   }
  }

  @Override
  public void initialize() {
    isHeadingSet = false;
    headingPID.reset();
    
  }

  @Override
  public abstract void execute();

  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(new ChassisSpeeds(0.0,0.0,0.0),true);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
  
}
