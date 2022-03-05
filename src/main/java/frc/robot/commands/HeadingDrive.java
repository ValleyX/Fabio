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

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Imu;
import frc.robot.Constants;
import frc.robot.commands.TurnAngleIMU;


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

/**
 *
 */
public class HeadingDrive extends CommandBase {

    private double m_Speed;
    private double m_Heading;
    private double m_Distance;
    private double m_MoveCount;
    private boolean Left;
    private boolean Right;
    private double newLeftTarget;
    private double newRightTarget;
    private double speed;
    private double count;
   

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
        private final DriveTrain m_driveTrain;
        private final Imu m_imu;
 
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS

    public double getAdjustedYaw()
    {
        return -m_imu.getImu().getYaw();
    }
    public double getError(double targetAngle)
    {

        double robotError;
           
        robotError = targetAngle - getAdjustedYaw(); // the - on the target determines the direction

        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
      
        return robotError;
    }

    public double getSteer(double error, double PCoeff) {
        return DriveTrain.clip(error * PCoeff, -1, 1);

    }


    public HeadingDrive(DriveTrain driveTrain, Imu imu, double Speed, double Distance, double Heading) 
    {

        m_Heading = Heading;
        m_Speed = Speed;
        m_Distance = Distance; 
        
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
        count = 0;

        m_driveTrain.getDifferentialDrive().setSafetyEnabled(false);

        m_MoveCount = (int) (m_Distance * m_driveTrain.getCountsPerFoot());
        newLeftTarget = (m_MoveCount + m_driveTrain.getCountsPerFoot()); 
        newRightTarget = (m_MoveCount + m_driveTrain.getRightCurrentPos());

        m_driveTrain.getRightBack().set(ControlMode.Position, newRightTarget);
        m_driveTrain.getLeftBack().set(ControlMode.Position, newLeftTarget);

        //driveTrain.SetLeftRightDistance(m_Speed, m_Distance, m_Distance);
         
        speed = DriveTrain.clip(Math.abs(m_Speed), 0.0, 1.0);
        m_driveTrain.getLeftFront().configPeakOutputForward(speed, Constants.kTimeoutMs);
        m_driveTrain.getLeftFront().configPeakOutputReverse(-speed, Constants.kTimeoutMs);
        m_driveTrain.getLeftBack().configPeakOutputForward(speed, Constants.kTimeoutMs);
        m_driveTrain.getLeftBack().configPeakOutputReverse(-speed, Constants.kTimeoutMs);
        m_driveTrain.getRightFront().configPeakOutputForward(speed, Constants.kTimeoutMs);
        m_driveTrain.getRightFront().configPeakOutputReverse(-speed, Constants.kTimeoutMs);
        m_driveTrain.getRightBack().configPeakOutputForward(speed, Constants.kTimeoutMs);
        m_driveTrain.getRightBack().configPeakOutputReverse(-speed, Constants.kTimeoutMs);
    
    }





    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    
        

        double error = getError(m_Heading);
        double steer = getSteer(error, 0.015);

        // if driving in reverse, the motor correction also needs to be reversed
        if (m_Distance < 0)
            steer *= -1.0;

        double leftFrontSpeed = speed + steer;
        double leftBackSpeed = speed + steer;
        double rightFrontSpeed = speed - steer;
        double rightBackSpeed = speed - steer;


        // Normalize speeds if either one exceeds +/- 1.0;
       double max = Math.max(Math.abs(leftFrontSpeed), Math.abs(rightFrontSpeed));
       

        if (max > 1.0/* && maxback > 1.0*/) {
            leftFrontSpeed /= max;
            rightBackSpeed /= max;
            leftBackSpeed /= max;
            rightFrontSpeed /= max;
        }



        m_driveTrain.getLeftBack().set(leftBackSpeed);
        m_driveTrain.getLeftFront().set(leftFrontSpeed);
        m_driveTrain.getRightBack().set(rightBackSpeed);
        m_driveTrain.getRightFront().set(rightFrontSpeed);

  
  
        
            
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
        if (count++ > 20)
        return (m_driveTrain.IsLeftClose(600) && m_driveTrain.IsRightClose(600));
      else
        return true;
            
    }

    @Override
    public boolean runsWhenDisabled() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
        return false;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
    }
}