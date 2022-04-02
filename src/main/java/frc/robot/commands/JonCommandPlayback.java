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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

import javax.lang.model.util.ElementScanner6;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.time.StopWatch;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
//import frc.robot.subsystems.Sorter;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.FeederSystemBack;
import frc.robot.subsystems.FeederSystemFront;
import frc.robot.subsystems.FrontIntakeArmSub;
import frc.robot.subsystems.BackIntakeArmSub;
import frc.robot.subsystems.Imu;
import frc.robot.subsystems.IntakeFront;
import frc.robot.subsystems.IntakeBack;
import frc.robot.subsystems.IntakeFrontArm;
import frc.robot.subsystems.ShooterDrive;
import frc.robot.subsystems.Sorter;
import frc.robot.subsystems.BeanBreakBack;
import frc.robot.subsystems.BeanBreakFront;
import frc.robot.subsystems.BeanBreakMid;
import frc.robot.subsystems.DriveBaseBlinkin1;
import frc.robot.subsystems.DriveBaseBlinkin2;
import frc.robot.commands.TakeSpeedShot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;

import frc.robot.JoyReadWrite;
import frc.robot.JoyStorage;



    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

/**
 *
 */
public class JonCommandPlayback extends CommandBase {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
        private final Sorter m_sorter;
 
        private final FrontIntakeArmSub m_frontIntakeArmSub;
        private final BackIntakeArmSub m_backIntakeArmSub;

        private final FeederSystemBack m_feederSystemBack;
        private final Conveyor m_conveyor;
        private final FeederSystemFront m_feederSystemFront;
        private final IntakeBack m_intakeBack;
        private final IntakeFront m_intakeFront;
        private final ShooterDrive m_shooterDrive;
        private final DriveTrain m_driveTrain;
        private final BeanBreakMid m_beanBreakMid;
        private final BeanBreakBack m_beanBreakBack;
        private final BeanBreakFront m_beanBreakFront;
        private final Climbers m_deathClimbers;
        private final DriveBaseBlinkin1 m_driveBaseBlinkin1;
        private final DriveBaseBlinkin2 m_driveBaseBlinkin2;


        private Joystick m_joystickDriver = new Joystick(0);
        private Joystick m_joystickManipulator = new Joystick(1);

        private String m_fileName;
 

        private double intakein = 1;
        private double intakeout = -1;
        private boolean everythingrun = false;
        private boolean frontIntakeEnable = false;
        private boolean shooton = false;
        private boolean deathClimbersOut = false;
        private final double velocityRPMhigh = 7300;
        private final double velocityRPMlow = 3800;
        private double heading;
 
        private JoyStorage m_joy[];
        private int m_joyCount;
        private final int m_joyCountmax = 750;
    




    public JonCommandPlayback(String fileName ,DriveTrain driveTrain, ShooterDrive shooterDrive, IntakeFront intakeFront, IntakeBack intakeBack, Conveyor conveyor,
    FeederSystemFront feederSystemFront, FeederSystemBack feederSystemBack, Sorter sorter, FrontIntakeArmSub frontIntakeArmSub, BackIntakeArmSub backIntakeArmSub, BeanBreakMid beanBreakMid,
    BeanBreakBack beanBreakBack, BeanBreakFront beanBreakFront, Climbers deathClimbers, DriveBaseBlinkin1 driveBaseBlinkin1, DriveBaseBlinkin2 driveBaseBlinkin2) {


        m_driveTrain = driveTrain;
        m_sorter = sorter;
        m_shooterDrive = shooterDrive;
        m_intakeFront = intakeFront;
        m_intakeBack = intakeBack;
        m_conveyor = conveyor;
        m_feederSystemFront = feederSystemFront;
        m_feederSystemBack = feederSystemBack;
        m_frontIntakeArmSub = frontIntakeArmSub;
        m_backIntakeArmSub = backIntakeArmSub;
        m_beanBreakMid = beanBreakMid;
        m_beanBreakBack = beanBreakBack;
        m_beanBreakFront = beanBreakFront;
        m_deathClimbers = deathClimbers;
        m_driveBaseBlinkin1 = driveBaseBlinkin1;
        m_driveBaseBlinkin2 = driveBaseBlinkin2;
        //m_joy = new JoyStorage[m_joyCountmax];

        m_fileName = fileName;
        m_joy = JoyReadWrite.readObject(m_fileName);

        m_joyCount = 0;
    
    
        
        addRequirements(m_intakeBack);
        addRequirements(m_beanBreakBack);
        addRequirements(m_beanBreakMid);
        addRequirements(m_beanBreakFront);
        addRequirements(m_driveTrain);
        addRequirements(m_sorter);
        addRequirements(m_shooterDrive);
        addRequirements(m_intakeFront);
        addRequirements(m_conveyor);
        addRequirements(m_feederSystemFront);
        addRequirements(m_feederSystemBack);
        addRequirements(m_frontIntakeArmSub);
        addRequirements(m_backIntakeArmSub);
        addRequirements(m_deathClimbers);
        addRequirements(m_driveBaseBlinkin1);
        addRequirements(m_driveBaseBlinkin2);


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES

       


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    }



    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
        SmartDashboard.putString("joncommand", "in init");
        m_driveTrain.SetPercentOutput(1, 30);
      /*  
        CameraServer.startAutomaticCapture();
        CameraServer.addAxisCamera("USBCamera y");
        CameraServer.putVideo("USBCamera y", 320, 240);
        */
        /*
        CvSink cvSink = CameraServer.getVideo();
        CvSource outputStream = CameraServer.putVideo("camera 1", 320, 240);
        */
    }
    
    public void PrimaryIntakeEnable()
    {
       //if (frontIntakeEnable == false)
        {
            m_frontIntakeArmSub.getFrontIntakeArm().set(true);
            m_intakeFront.getFrontIntakeMotor().set(intakein);
            m_feederSystemFront.getFeederFront().set(0.75);
          //m_sorter.getSorter().set(1);
            //m_conveyor.getConveyorMotor().set(0.75);
            frontIntakeEnable = true;
        }
    }

    public void BackIntakeEnable()
    {
        m_backIntakeArmSub.getBackIntakeArm().set(true);
        m_intakeBack.getBackIntakeMotor().set(intakein);
        m_feederSystemBack.getFeederBack().set(1);
        //m_feederSystemFront.getFeederFront().set(-1);
        //m_sorter.getSorter().set(-1);
        //m_conveyor.getConveyorMotor().set(1);
    }
    
    public void PrimaryIntakeExpel()
    {
        //if (frontIntakeEnable == false)
        {
           // m_intakeFront.getFrontIntakeMotor().set(-1);
            m_feederSystemFront.getFeederFront().set(-1);
            m_feederSystemBack.getFeederBack().set(-1);
            //m_sorter.getSorter().set(-1);
            m_conveyor.getConveyorMotor().set(-0.6);
            frontIntakeEnable = true;
        }
    }

    public void PrimaryIntakeDisable()
    {
        //if (frontIntakeEnable == true)
        {
            m_frontIntakeArmSub.getFrontIntakeArm().set(false);
            m_intakeFront.getFrontIntakeMotor().set(0);
            m_feederSystemFront.getFeederFront().set(0);
            m_feederSystemBack.getFeederBack().set(0);
           // m_sorter.getSorter().set(0);
            m_conveyor.getConveyorMotor().set(0);
            frontIntakeEnable = false;
        }
    }
    public void BackIntakeDisable()
    {
        m_backIntakeArmSub.getBackIntakeArm().set(false);
        m_intakeBack.getBackIntakeMotor().set(0);
        m_feederSystemBack.getFeederBack().set(0);
        m_feederSystemFront.getFeederFront().set(0);
       // m_sorter.getSorter().set(0);
        m_conveyor.getConveyorMotor().set(0);
    }

    public void shooteron(double direction)
    {
        //if (shooton == false)
        //{
            m_feederSystemFront.getFeederFront().set(1);
           // m_sorter.getSorter().set(1);
            m_conveyor.getConveyorMotor().set(1);
            m_shooterDrive.getLeftTalon().set(0.5);
            m_shooterDrive.getRightTalon().set(0.5);
            shooton = true;
        //}
    }

    public void shooteroff()
    {
        //if (shooton == true)
        //{
            PrimaryIntakeDisable();
            m_shooterDrive.getLeftTalon().set(0.5);
            m_shooterDrive.getRightTalon().set(0.5);
            shooton = false;
     //  }
    }

    public void SwitchLights(double test)
    {
        m_driveBaseBlinkin1.getDriveBlinkin1().setSpeed(test);
        m_driveBaseBlinkin2.getDriveBlinkin2().setSpeed(test);
    }




    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        //want to be recorded 
		/*double leftYstick = m_joystickDriver.getRawAxis(1);  //left y stick
		double rightYstick = m_joystickDriver.getRawAxis(5);  //right y stick
        boolean buttonlb = m_joystickManipulator.getRawButton(5); // back intake
        boolean buttonrb = m_joystickManipulator.getRawButton(6);// front intake 
        boolean buttonX = m_joystickManipulator.getRawButton(3); // shooooot high
        boolean buttonReleaseX = m_joystickManipulator.getRawButtonReleased(3); // shoot high off
        boolean buttonReleaserb = m_joystickManipulator.getRawButtonReleased(6); // turn off intake
        //want to be recorded 
*/

        if (m_joyCount >= m_joyCountmax)
        {
            return;
        }

        double leftYstick = m_joy[m_joyCount].leftYstick;
        double rightYstick = m_joy[m_joyCount].rightYstick;
        boolean buttonlb = m_joy[m_joyCount].buttonlb;
        boolean buttonrb = m_joy[m_joyCount].buttonrb;
        boolean buttonX = m_joy[m_joyCount].buttonX;
        boolean buttonReleaseX = m_joy[m_joyCount].buttonReleaseX;
        boolean buttonReleaserb = m_joy[m_joyCount].buttonReleaserb;

        m_joyCount++;




        boolean buttonA = m_joystickManipulator.getRawButton(1); //deathclimbers p1 (manipulator)
        boolean buttonB = m_joystickManipulator.getRawButton(2); //shoooot low
        boolean buttonY = m_joystickManipulator.getRawButton(4); //expell

        
        boolean buttonrbd = m_joystickDriver.getRawButton(6); // deathclimbers p2 (driver)

        boolean buttonReleaseB = m_joystickManipulator.getRawButtonReleased(2); //
        boolean buttonReleaseA = m_joystickManipulator.getRawButtonReleased(1);
        boolean buttonReleaseY = m_joystickManipulator.getRawButtonReleased(4);
        boolean buttonReleaselb = m_joystickManipulator.getRawButtonReleased(5);
       

        boolean beanBreakMid = m_beanBreakMid.getBeanBreakMid().get();
        boolean beanBreakFront = m_beanBreakFront.getBeanBreakFront().get();
        boolean beanBreakBack = m_beanBreakBack.getBeanBreakBack().get();
        SmartDashboard.putNumber("Drive On Joystick leftYstick", leftYstick);
        SmartDashboard.putNumber("Drive On Joystick rightYstick", rightYstick);
        double velocityRPM = SmartDashboard.getNumber("Velocity RPM", 4000);
        SmartDashboard.putNumber("Velocity RPM got", velocityRPM);


        SmartDashboard.putBoolean("getBeanBreakMid", beanBreakMid);
        SmartDashboard.putBoolean("getBeanBreakFront", beanBreakFront);
        SmartDashboard.putBoolean("getBeanBreakBack", beanBreakBack);
        SmartDashboard.putBoolean("buttonA", buttonA);
        SmartDashboard.putBoolean("buttonB", buttonB);
        SmartDashboard.putBoolean("buttonX", buttonX);
        SmartDashboard.putBoolean("buttonY", buttonY);
        SmartDashboard.putBoolean("frontIntakeEnable", frontIntakeEnable);



        m_driveTrain.getDifferentialDrive().tankDrive(-leftYstick, -rightYstick);

    
        if (beanBreakFront == false && beanBreakMid == false)
        {
            SwitchLights(0.57);
        }

        else if (beanBreakBack == false && beanBreakMid == false)
        {
            SwitchLights(0.57);
        }
        
        else if (beanBreakMid == false)
        {
            SwitchLights(0.67);
        }

        else 
        {
            SwitchLights(-0.15);
        }



        if ((beanBreakFront == false || beanBreakBack == false) && beanBreakMid == true)
        {
            if (beanBreakFront == false)
            {
                m_feederSystemFront.getFeederFront().set(0.6);
            }

            else if (beanBreakBack == false)
            {
                m_feederSystemBack.getFeederBack().set(0.6);
            }

            m_conveyor.getConveyorMotor().set(0.6);
            SwitchLights(0.67);
        }

        else if (beanBreakMid == false)
        {
            m_conveyor.getConveyorMotor().set(0);
            m_feederSystemBack.getFeederBack().set(0);
            m_feederSystemFront.getFeederFront().set(0);

        }



        if (buttonrbd == true && buttonA == true )
        {
        
                m_deathClimbers.getDeathClimbers().set(true);
                
        }

        else
        {
            m_deathClimbers.getDeathClimbers().set(false);
        }

    
/*
        if (beanBreakMid == false)
        {
            SwitchLights(0.67);
        }

        
        else if (beanBreakBack == false || beanBreakFront == false)
        {
            SwitchLights(0.67);
        }
        */
    

        
        
        
        if (buttonB == true /*&& Math.abs(leftYstick) <= 0.1 && Math.abs(rightYstick) <= 0.1*/)
        { 

            
            SmartDashboard.putNumber("Velocity RPM got", velocityRPM);
            m_shooterDrive.getLeftTalon().set(ControlMode.Velocity, velocityRPMlow);
            m_shooterDrive.getRightTalon().set(ControlMode.Velocity, velocityRPMlow);
                        
            if (beanBreakFront == false)
            {
                m_feederSystemFront.getFeederFront().set(0.8); 
            }
  
            else if (beanBreakBack == false)
            {
                m_feederSystemBack.getFeederBack().set(0.8);
            }
            else
            {
                m_feederSystemBack.getFeederBack().set(0);
                m_feederSystemFront.getFeederFront().set(0);
            }
            
            if ( m_shooterDrive.getRightTalon().getSelectedSensorVelocity(0) >= (velocityRPMlow - 100))
            {
                m_conveyor.getConveyorMotor().set(0.6);
            }

            else
            {
                //m_feederSystemFront.getFeederFront().set(0);
                //m_conveyor.getConveyorMotor().set(0);
            }

        }
        else if (buttonReleaseB)
        {
            m_feederSystemFront.getFeederFront().set(0);
            m_feederSystemBack.getFeederBack().set(0);

            m_conveyor.getConveyorMotor().set(0);
            m_shooterDrive.getLeftTalon().set(0);
            m_shooterDrive.getRightTalon().set(0);
        }
    

        if (buttonY == true)
        {
            SwitchLights(0.61);
            //PrimaryIntakeExpel();
        }

        else if (buttonReleaseY)
        {
            //PrimaryIntakeDisable();
        }


        if (buttonX == true /*&& Math.abs(leftYstick) <= 0.1 && Math.abs(rightYstick) <= 0.1*/)
        { 

            SmartDashboard.putNumber("Velocity RPM got", velocityRPM);
            m_shooterDrive.getLeftTalon().set(ControlMode.Velocity, velocityRPMhigh);
            m_shooterDrive.getRightTalon().set(ControlMode.Velocity, velocityRPMhigh);
            //m_feederSystemBack.getFeederBack().set(0.5);
            //m_feederSystemFront.getFeederFront().set(0.5);
            
            if (beanBreakFront == false)
            {
                m_feederSystemFront.getFeederFront().set(0.5); 
            }
            
            
            else if (beanBreakBack == false)
            {
                m_feederSystemBack.getFeederBack().set(0.5);
            }

            
            //SmartDashboard.putNumber("Actual velocity", m_shooterDrive.getLeftTalon().get());
            
            if ( m_shooterDrive.getRightTalon().getSelectedSensorVelocity(0) >= (velocityRPMhigh - 50))
            {
                m_conveyor.getConveyorMotor().set(0.6);
            }

            else
            {
                //m_feederSystemFront.getFeederFront().set(0);
                //m_conveyor.getConveyorMotor().set(0);
            }

        }
        else if (buttonReleaseX)
        {
            m_feederSystemFront.getFeederFront().set(0);
            m_feederSystemBack.getFeederBack().set(0);

            m_conveyor.getConveyorMotor().set(0);
            m_shooterDrive.getLeftTalon().set(0);
            m_shooterDrive.getRightTalon().set(0);
        }
/*
            velocityRPM = 7300;
            SmartDashboard.putNumber("Velocity RPM got", velocityRPM);
            m_shooterDrive.getLeftTalon().set(ControlMode.Velocity, velocityRPM);
            m_shooterDrive.getRightTalon().set(ControlMode.Velocity, velocityRPM);

        
            if (beanBreakBack == false)
            {
                if (m_shooterDrive.getLeftTalon().get() >= 7200 )
                {
                    m_feederSystemBack.getFeederBack().set(1);
                    m_conveyor.getConveyorMotor().set(1);
                    m_sorter.getSorter().set(-1);

                }

                else
                {
                    m_feederSystemBack.getFeederBack().set(0);
                    m_conveyor.getConveyorMotor().set(0);
                    m_sorter.getSorter().set(0);
                }

            }

            else if (beanBreakFront == false)
            {
                if (m_shooterDrive.getLeftTalon().get() >= 7200)
                {
                    m_feederSystemFront.getFeederFront().set(1);
                    m_conveyor.getConveyorMotor().set(1);
                    m_sorter.getSorter().set(1);
                }

                else
                {
                    m_feederSystemBack.getFeederBack().set(0);
                    m_conveyor.getConveyorMotor().set(0);
                    m_sorter.getSorter().set(0);
                }

            }

            else 
            {
                 if (m_shooterDrive.getLeftTalon().get() >= 7200)
                {
                   
                    m_conveyor.getConveyorMotor().set(1);
                   
                }
                
                else
                {
                   
                    m_conveyor.getConveyorMotor().set(0);
      
                }

            }

            */
            //new TakeSpeedShot(m_shooterDrive, 4000);
           
          
        

        if (buttonY == true )
        {
           PrimaryIntakeExpel();
        }
        else if (buttonReleaseY)
        {
            PrimaryIntakeDisable();
        }
        

        if (buttonlb == true && beanBreakFront) //left bummper is back
        {
            if ((beanBreakFront == true || beanBreakBack == true) && beanBreakMid == true)
            {
                BackIntakeEnable();
            }   

            else if (beanBreakBack == false && beanBreakMid == true )
            {
                BackIntakeEnable();
                m_conveyor.getConveyorMotor().set(0.8);
            }

            else if (beanBreakBack == true && beanBreakMid == false)
            {
                BackIntakeEnable();
                m_conveyor.getConveyorMotor().set(0);
            }

            else if ((beanBreakBack == false || beanBreakFront == false) && beanBreakMid == false)
            {
                BackIntakeDisable();
            }

            else
            {
                BackIntakeDisable();
            }
        }

        else if (buttonReleaselb) 
        {
            BackIntakeDisable();
        }

        if (buttonrb == true && beanBreakBack == true ) //right bumper is frotn 
        {
            if (beanBreakFront == true && beanBreakMid == true)
            {
                PrimaryIntakeEnable();
            }   

            else if (beanBreakFront == false && beanBreakMid == true )
            {
                PrimaryIntakeEnable();
                m_conveyor.getConveyorMotor().set(0.8);
            }

            else if (beanBreakFront == true && beanBreakMid == false)
            {
                PrimaryIntakeEnable();
                m_conveyor.getConveyorMotor().set(0);
            }

            else if (beanBreakFront == false  && beanBreakMid == false)
            {
                PrimaryIntakeDisable();
            }

            else
            {
                PrimaryIntakeDisable();
            }
        }

        else if (buttonReleaserb)
        {
            PrimaryIntakeDisable();
        }

        

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_sorter.getSorter().set(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (m_joyCount >= m_joyCountmax)
        {
            return true;
        }
        return false;
    }

    

    @Override
    public boolean runsWhenDisabled() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
        return false;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
    }
}
