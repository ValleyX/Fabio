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
import frc.robot.subsystems.BeanBreakFront;
import frc.robot.subsystems.BeanBreakMid;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.FeederSystemFront;
import frc.robot.subsystems.IntakeFront;

import java.lang.Thread;
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import frc.robot.subsystems.ShooterDrive;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

/**
 *
 */
public class TakeSpeedShot extends CommandBase {
    private Joystick m_joystickDriver;
    private double m_velocityRPM;
    private boolean m_doubleShot;
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
        private final ShooterDrive m_shooterDrive;
        private final Conveyor m_conveyor;
        private final FeederSystemFront m_feederFront;
        private final BeanBreakMid m_beanBreakMid;
        private final BeanBreakFront m_beanBreakFront;
        //private final IntakeFront
        //private final BeanBreakFront m_beanBreakFront;
 
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS


    public TakeSpeedShot(BeanBreakMid beanBreakMid, BeanBreakFront beanBreakFront, Conveyor conveyor,  ShooterDrive subsystem, 
                        FeederSystemFront feederFront, double velocityRPM, boolean doubleShot) {


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES

        m_velocityRPM = velocityRPM;
        m_doubleShot = doubleShot;
        m_shooterDrive = subsystem;
        m_conveyor = conveyor;
        m_feederFront = feederFront;
        
        m_beanBreakMid = beanBreakMid;
        m_beanBreakFront = beanBreakFront;
        //m_beanBreakFront = beanBreakFront;
        addRequirements(m_shooterDrive);
        addRequirements(m_conveyor);
        addRequirements(m_beanBreakMid);
        addRequirements(m_beanBreakFront);
        addRequirements(m_feederFront);

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
                
                   SmartDashboard.getNumber("Velocity RPM", m_velocityRPM);
                   SmartDashboard.putNumber("Velocity RPM got", m_velocityRPM);
                   m_shooterDrive.getLeftTalon().set(ControlMode.Velocity, m_velocityRPM);
                   m_shooterDrive.getRightTalon().set(ControlMode.Velocity, m_velocityRPM);

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
  
        if  (m_shooterDrive.getRightTalon().getSelectedSensorVelocity(0) >= m_velocityRPM)
        {
          m_conveyor.getConveyorMotor().set(0.5);
          m_feederFront.getFeederFront().set(0.5);

        }
        SmartDashboard.putNumber("Velocity RPM meas", m_shooterDrive.getRightTalon().getSelectedSensorVelocity(0));
        SmartDashboard.putBoolean("BeanBreakMid", m_beanBreakMid.getBeanBreakMid().get());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
       try {
        Thread.sleep(700);
    } catch (InterruptedException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
    }
        m_shooterDrive.getLeftTalon().set(ControlMode.Velocity, 0);
        m_shooterDrive.getRightTalon().set(ControlMode.Velocity, 0);
       m_shooterDrive.getLeftTalon().set(0);
       m_shooterDrive.getRightTalon().set(0);
       m_conveyor.getConveyorMotor().set(0);
       m_feederFront.getFeederFront().set(0);


    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (m_beanBreakMid.getBeanBreakMid().get() == true && m_doubleShot == false)
        {
            return true;
        }

        else if (m_beanBreakFront.getBeanBreakFront().get() == true && m_beanBreakMid.getBeanBreakMid().get() == true 
                 && m_doubleShot == true)
        {
            return true;
        }

        else
        {
            return false;
        }

        //return (m_beanBreakMid.getBeanBreakMid().get() == true);
    }

    @Override
    public boolean runsWhenDisabled() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
        return false;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
    }
}
