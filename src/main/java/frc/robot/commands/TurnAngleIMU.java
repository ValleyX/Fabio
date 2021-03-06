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
import javax.lang.model.util.ElementScanner6;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Imu;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

/**
 *
 */
public class TurnAngleIMU extends CommandBase {

    private double m_Speed;
    private double m_Heading;
    private boolean Left;
    private boolean Right;

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
        private final DriveTrain m_driveTrain;
        private final Imu m_imu;
 
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS

    public double getAdjustedYaw()
    {
        return -m_imu.getImu().getYaw();
    }

    public TurnAngleIMU(DriveTrain driveTrain,Imu imu, double Speed, double Heading) {

        m_Heading = Heading;
        m_Speed = Speed;

        
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES

        m_driveTrain = driveTrain;
        m_imu = imu;
        addRequirements(m_driveTrain);
        addRequirements(m_imu);

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
        
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        m_driveTrain.getDifferentialDrive().setSafetyEnabled(false);
        m_driveTrain.SetPercentOutput(1, 30);

        double currentYaw = getAdjustedYaw();
        double wantAngle = m_Heading - currentYaw;


        if(wantAngle < 0)
        {
            m_driveTrain.getDifferentialDrive().tankDrive(-m_Speed, m_Speed);
            Left = true;
            Right = false;
        }
        else
        {
            m_driveTrain.getDifferentialDrive().tankDrive(m_Speed, -m_Speed);
            Left = false;
            Right = true; 
        }
        SmartDashboard.putNumber("Wanted Angle is: ", wantAngle);
        SmartDashboard.putString("You are turning:", Left ? "Left" : "Right" );
        
    
    }





    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    
        

        SmartDashboard.putNumber("Current Angle is: ",  getAdjustedYaw());
  
  
        
            
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_driveTrain.getDifferentialDrive().tankDrive(0,0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
     {
            if(getAdjustedYaw() <=  m_Heading && Left == true)
            {
                return true;
            }
            else if(getAdjustedYaw() >=  m_Heading && Right == true)
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
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
        return false;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
    }
}
