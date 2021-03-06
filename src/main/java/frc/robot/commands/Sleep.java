// RobotBuilder Version: 4.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.subsystems.ShooterDrive;
import frc.robot.subsystems.IntakeFront;


public class Sleep extends CommandBase {

        private int m_msSleep;
        private int m_repeat;
        private int m_count;

    public Sleep(int m_msSleep) {

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        try {
            Thread.sleep(m_msSleep);
        } catch (InterruptedException e) {
            // TODO Auto-generated catch block
            e.printStackTrace(); 
        }

        m_repeat = m_msSleep/20;
                 
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
  
    
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

      
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (m_count ++ > m_repeat)
        {
            return true;
        }

        else 
        {
            return false;
        }
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}

