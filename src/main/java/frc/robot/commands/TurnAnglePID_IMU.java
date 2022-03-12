// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import frc.robot.Constants.DriveConstants;
//import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;




/** A command that will turn the robot to the specified angle. */
public class TurnAnglePID_IMU extends PIDCommand { 
  
  /**
   * Turns to robot to the specified angle.
   *
   * @param targetAngleDegrees The angle to turn to
   * @param drive The drive subsystem to use
   */
  double currentheading;
  double m_targetDegree;
  int m_count;

  private final DriveTrain m_drive;

  public TurnAnglePID_IMU(DriveTrain drive, double targetAngleDegrees) {
       super(
        new PIDController( Constants.kp, Constants.ki, Constants.kd),
        // Close loop on heading
        drive::getHeading,
        // Set reference to target
        targetAngleDegrees,
        // Pipe output to turn robot
        output -> drive.arcadeDrive(0, output),
        // Require the drive
        drive);

        m_drive = drive;
        addRequirements(m_drive);
        
        m_targetDegree = targetAngleDegrees;
        m_count = 0;
       
    // Set the controller to be continuous (because it is an angle controller)
    getController().enableContinuousInput(-360, 360);
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController().setTolerance(3,20.0);



  }




  @Override
  public boolean isFinished() {
  //SmartDashboard.putNumber("Current Angle is: ",   getAdjustedYaw());
    // End when the controller is at the reference.
    //return false;
   if (getController().atSetpoint() )
   //if ((Math.abs(m_drive.getAdjustedYaw() - m_targetDegree) <= 13 ) && m_count++ > 20)
    //if ((Math.abs(m_drive.getAdjustedYaw() - m_targetDegree) <= 3 ) && m_count++ > 20)
    
    {
      
      try {
        Thread.sleep(250);
    } catch (InterruptedException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
    }
    
      return true;
    }
    else
    {
      return false;
    }
    
    
    
   // return getController().atSetpoint();
  }
}
