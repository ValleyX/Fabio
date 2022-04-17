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

import org.opencv.core.Mat;

import com.ctre.phoenix.motorcontrol.NeutralMode;

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
import frc.robot.Robot;
import frc.robot.commands.TakeSpeedShot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import frc.robot.JoyReadWrite;
import frc.robot.JoyStorage;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

/**
 * 1. constants 
 * 2. methods
 * 
 */
public class JonWorldRecord extends CommandBase {

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
        private final Climbers m_climbers;
        private final DriveBaseBlinkin1 m_driveBaseBlinkin1;
        private final DriveBaseBlinkin2 m_driveBaseBlinkin2;


        private Joystick m_joystickDriver = new Joystick(0);
        private Joystick m_joystickManipulator = new Joystick(1);
        private String m_fileName;
 
        //  1. constants 
        private double intakein = 1;
        private double intakeout = -1;
        private boolean everythingrun = false;
        private boolean frontIntakeEnable = false;
        private boolean shooton = false;
        private boolean deathClimbersOut = false;
        private final double velocityRPMhigh = 6500;
        private final double velocityRPMlow = 3800;
        private double heading;
        private double leftSpeed = 0;
        private double rightSpeed = 0; 
        private double drivingSensitivty = 0.1;

        private final double climberSpeed = 1.0;
        private final double climberDistance = 8; //inches
        private boolean climbing = false;
        private final double climbingSensetivity = 50;

        private final double colorBlue = 0.57;
        private final double colorGold = 0.67;
        private final double colorPink = -0.15;

        private final int povUp = 0;
        private final int povDown = 180;

		//joy storage
        private JoyStorage m_joy[];
        private int m_joyCount;
        private final int m_joyCountmax = 750;
		
        //  1. constants 




    public JonWorldRecord(String fileName , DriveTrain driveTrain, ShooterDrive shooterDrive, IntakeFront intakeFront, IntakeBack intakeBack, Conveyor conveyor,
    FeederSystemFront feederSystemFront, FeederSystemBack feederSystemBack, Sorter sorter, FrontIntakeArmSub frontIntakeArmSub, BackIntakeArmSub backIntakeArmSub, BeanBreakMid beanBreakMid,
    BeanBreakBack beanBreakBack, BeanBreakFront beanBreakFront, Climbers climbers, DriveBaseBlinkin1 driveBaseBlinkin1, DriveBaseBlinkin2 driveBaseBlinkin2) {
        m_joy = new JoyStorage[m_joyCountmax];
        m_joyCount = 0;

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
        m_climbers = climbers;
        m_driveBaseBlinkin1 = driveBaseBlinkin1;
        m_driveBaseBlinkin2 = driveBaseBlinkin2;
        m_fileName = fileName;
    
        
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
        addRequirements(m_climbers);
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
     //   SmartDashboard.putNumber("JonVelInput", 2000);
        
        SmartDashboard.putString("joncommand Record", "in init");
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
        m_joyCount = 0;
    }
    
    //2. methods
    public void PrimaryIntakeEnable() // turns the front intake system on 
    {
        m_frontIntakeArmSub.getFrontIntakeArm().set(true);
        m_intakeFront.getFrontIntakeMotor().set(intakein);
        m_feederSystemFront.getFeederFront().set(0.75);
    }

    public void BackIntakeEnable() // turns the back intake system on
    {
        m_backIntakeArmSub.getBackIntakeArm().set(true);
        m_intakeBack.getBackIntakeMotor().set(intakein);
        m_feederSystemBack.getFeederBack().set(1);
    }
    
    public void PrimaryIntakeExpel() // spits the ball out 
    {
        m_feederSystemFront.getFeederFront().set(-1);
        m_feederSystemBack.getFeederBack().set(-1);
        m_conveyor.getConveyorMotor().set(-0.6);
    }

    public void PrimaryIntakeDisable() // turns the front system off
    {
        m_frontIntakeArmSub.getFrontIntakeArm().set(false);
        m_intakeFront.getFrontIntakeMotor().set(0);
        m_feederSystemFront.getFeederFront().set(0);
        m_feederSystemBack.getFeederBack().set(0);
        m_conveyor.getConveyorMotor().set(0);
    }
    public void BackIntakeDisable() // turns the back system off
    {
        m_backIntakeArmSub.getBackIntakeArm().set(false);
        m_intakeBack.getBackIntakeMotor().set(0);
        m_feederSystemBack.getFeederBack().set(0);
        m_feederSystemFront.getFeederFront().set(0);
        m_conveyor.getConveyorMotor().set(0);
    }

    public void shooteroff() // turns the shooter off
    {
        PrimaryIntakeDisable();
        m_shooterDrive.getLeftTalon().set(0.5);
        m_shooterDrive.getRightTalon().set(0.5);
    }

    public void SwitchLights(double test) // controls the blinkin lights
    {
        m_driveBaseBlinkin1.getDriveBlinkin1().setSpeed(test);
        m_driveBaseBlinkin2.getDriveBlinkin2().setSpeed(test);
    }

    public double velocityCalculate(double diagDistance) // returns velocity
    {
        double velocity;

        velocity =-3.7033e-8*Math.pow(diagDistance,6)+
                3.40972e-5*Math.pow(diagDistance,5)+ 
                -0.012827582*Math.pow(diagDistance,4)+
                2.520262315*Math.pow(diagDistance,3)+ 
                -272.6389002*Math.pow(diagDistance,2)+
                15443.17929*diagDistance+ 
                -352149.2735;
/*
                velocity =0.0000674777560920603*Math.pow(diagDistance,6)- 
                0.0474845690722852*Math.pow(diagDistance,5)+ 
                13.8745937546658*Math.pow(diagDistance,4)- 
                2154.6147755598*Math.pow(diagDistance,3)+ 
                187555.023312217*Math.pow(diagDistance,2)- 
                8677276.73722726*diagDistance+ 
                166706850.6687;
                /*

        velocity =  (7e-5 * Math.pow(diagDistance, 6))-
                    (0.0475 * Math.pow(diagDistance, 5))+
                    (13.875 * Math.pow(diagDistance, 4))-
                    (2154.6 * Math.pow(diagDistance, 3))+
                    (187555 * Math.pow(diagDistance, 2))-
                    (9e+06 * Math.pow(diagDistance, 1))+
                    (2e+08);
                    */
        return velocity;
    }
    //2. methods



    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
	
	SmartDashboard.putNumber("joy count", m_joyCount);

        if(m_joyCount == 750) {

            
            m_driveTrain.allpower(0);
            m_intakeFront.getFrontIntakeMotor().set(0); //self oofus big die
            m_intakeBack.getBackIntakeMotor().set(0); //DEATH TO ALL MOTORS
            m_conveyor.getConveyorMotor().set(0);
            m_shooterDrive.getRightTalon().set(0);
            m_shooterDrive.getRightTalon().set(0);
            m_driveTrain.getDifferentialDrive().tankDrive(0, 0);
            m_joyCount++;
            JoyReadWrite.writeObject(m_joy, m_fileName);
            
            return;
        }
    else if (m_joyCount > 750) {
        m_driveTrain.getDifferentialDrive().tankDrive(0, 0);
        return;
    }
	
	
        double velocity;
        //double velocity = SmartDashboard.getNumber("JonVelInput", 4000);
        //SmartDashboard.putNumber("currentVel", velocity);


        //entering climbing mode
        if (climbing && m_climbers.IsLeftClose(climbingSensetivity) && m_climbers.IsRightClose(climbingSensetivity))
        {
            climbing = false;
        }

        if (climbing == true)
        {
            return;
        }
        //entering climbing mode

        //want to be recorded 
		double leftYstick = m_joystickDriver.getRawAxis(1);  //left y stick
		double rightYstick = m_joystickDriver.getRawAxis(5);  //right y stick
        boolean buttonlb = m_joystickManipulator.getRawButton(5); // back intake
        boolean buttonrb = m_joystickManipulator.getRawButton(6);// front intake 
        boolean buttonX = m_joystickManipulator.getRawButton(3); // shooooot high
        boolean buttonReleaseX = m_joystickManipulator.getRawButtonReleased(3); // shoot high off
        boolean buttonReleaserb = m_joystickManipulator.getRawButtonReleased(6); // turn off intake
        //want to be recorded 
        leftYstick = leftYstick * 0.75;
        rightYstick = rightYstick * 0.75;
        m_joy[m_joyCount] = new JoyStorage(
        leftYstick,
        rightYstick,
        buttonlb,
        buttonrb,
        buttonX,
        buttonReleaseX,
        buttonReleaserb);
        
        
        m_joyCount++;
        SmartDashboard.putNumber("leftYstick", leftYstick);

        boolean buttonADriver = m_joystickDriver.getRawButtonPressed(1); //climberpivot true
        boolean buttonReleaseADriver = m_joystickDriver.getRawButtonReleased(1); //climberpivot false

        boolean buttonB = m_joystickManipulator.getRawButton(2); //shoooot low
        boolean buttonY = m_joystickManipulator.getRawButton(4); //expell

        boolean buttonReleaseB = m_joystickManipulator.getRawButtonReleased(2); // turn off shoot low 
        boolean buttonReleaseY = m_joystickManipulator.getRawButtonReleased(4); // turn off expel
        boolean buttonReleaselb = m_joystickManipulator.getRawButtonReleased(5); // turn off back intake 

        int povDriver =  m_joystickDriver.getPOV(0);

        //climber buttons
        boolean buttonlbDriver = m_joystickDriver.getRawButton(5); // left bumper hold for pivot pneumatic true
        boolean buttonrbDriver = m_joystickDriver.getRawButton(6);// right bumper hold for extender going out
        boolean buttonlbReleaseDriver = m_joystickDriver.getRawButtonReleased(5); // left bumper release for pivot pneumatic false
        boolean buttonrbReleaseDriver = m_joystickDriver.getRawButtonReleased(6);// right bumper release for extender going in
        boolean buttonYDriver = m_joystickDriver.getRawButton(4); //Climber automatic distance 
        //climber buttons
        
        //beambreaks that are used to know how many balls are in the robot
        boolean beanBreakMid = m_beanBreakMid.getBeanBreakMid().get();
        boolean beanBreakFront = m_beanBreakFront.getBeanBreakFront().get();
        boolean beanBreakBack = m_beanBreakBack.getBeanBreakBack().get();
        //beambreaks that are used to know how many balls are in the robot

        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");             
        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");
        NetworkTableEntry ta = table.getEntry("ta");

        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);
        double heightGoal = 104; //104
        double heightCamera = 22; //to be changed in the future 
        double angle = 5.7; //in degrees 

        double distance = ((heightGoal-heightCamera))/Math.tan(Math.toRadians(angle+y));
        double diagDistance = ((heightGoal-heightCamera))/Math.sin(Math.toRadians(angle+y));


        SmartDashboard.putNumber("Drive On Joystick leftYstick", leftYstick);
        SmartDashboard.putNumber("Drive On Joystick rightYstick", rightYstick);
        double velocityRPM = SmartDashboard.getNumber("Velocity RPM", 4000);
        SmartDashboard.putNumber("Velocity RPM got", velocityRPM);


        SmartDashboard.putBoolean("getBeanBreakMid", beanBreakMid);
        SmartDashboard.putBoolean("getBeanBreakFront", beanBreakFront);
        SmartDashboard.putBoolean("getBeanBreakBack", beanBreakBack);
        SmartDashboard.putBoolean("buttonA", buttonADriver);
        SmartDashboard.putBoolean("buttonB", buttonB);
        SmartDashboard.putBoolean("buttonX", buttonX);
        SmartDashboard.putBoolean("buttonY", buttonY);
        

    
        // test code to limit the speed progression of the robot
    
        if (Math.abs(leftYstick) > Math.abs(leftSpeed))
        {
            leftSpeed += drivingSensitivty*(leftYstick-leftSpeed);

        }
        else 
        {
            leftSpeed = leftYstick;
        }


        if (Math.abs(rightYstick) > Math.abs(rightSpeed))
        {
          
            rightSpeed += drivingSensitivty*(rightYstick-rightSpeed);
        }
        else 
        {
            rightSpeed = rightYstick;
        }

        if (Math.abs(rightSpeed) < 0.05 && Math.abs(leftSpeed) < 0.05)
        
        {
          m_driveTrain.getLeftBack().setNeutralMode(NeutralMode.Brake);
          m_driveTrain.getLeftFront().setNeutralMode(NeutralMode.Brake);
          m_driveTrain.getRightBack().setNeutralMode(NeutralMode.Brake);
          m_driveTrain.getLeftBack().setNeutralMode(NeutralMode.Brake);
        }
        else
        {
         m_driveTrain.getLeftBack().setNeutralMode(NeutralMode.Coast);
         m_driveTrain.getLeftFront().setNeutralMode(NeutralMode.Coast);
         m_driveTrain.getRightBack().setNeutralMode(NeutralMode.Coast);
         m_driveTrain.getLeftBack().setNeutralMode(NeutralMode.Coast);

        }

        // test code to limit the speed progression of the robot


        m_driveTrain.getDifferentialDrive().tankDrive(-leftSpeed, -rightSpeed);

        if (tx.getDouble(0) < 8 && tx.getDouble(0) > -2 && tx.getDouble(0) != 0)
        {
            SwitchLights(0.77);
        }
        else 
        {
                if (beanBreakFront == false && beanBreakMid == false)
            {
                SwitchLights(colorBlue);  // no ball = blue
            }

            else if (beanBreakBack == false && beanBreakMid == false) 
            {
                SwitchLights(colorBlue); // no ball = blue
            }
            
            else if (beanBreakMid == false)
            {
                SwitchLights(colorGold); // 1 ball in mid switch to gold
            }

            else 
            {
                SwitchLights(colorPink); // two balls in robot switch to pink
            }
        }
        // automated lights determined by ball placement in robot
        
        // automated lights determined by ball placement in robot

        // automated ball movement determined by the beam breaks
        if ((beanBreakFront == false || beanBreakBack == false) && beanBreakMid == true) // if a ball is in either the front or back enter this loop
        {
            if (beanBreakFront == false)
            {
                m_feederSystemFront.getFeederFront().set(0.6); //if the ball is in the front, turn on feeder front to feed it up to mid
            }

            else if (beanBreakBack == false)
            {
                m_feederSystemBack.getFeederBack().set(0.6); // if the ball is in the back, turn on feeder back to feed it up to mid
            }

            m_conveyor.getConveyorMotor().set(0.6); 
            SwitchLights(colorGold); 
        }

        else if (beanBreakMid == false) //once mid is triped automation turns off.
        {
            m_conveyor.getConveyorMotor().set(0);
            m_feederSystemBack.getFeederBack().set(0);
            m_feederSystemFront.getFeederFront().set(0);

        }
        // automated ball movement determined by the beam breaks


        //Climber buttons
        if(buttonlbDriver == true)
        {
            m_climbers.getClimerPivots().set(true);        
        }
        else if(buttonlbReleaseDriver)
        {
            m_climbers.getClimerPivots().set(false);
        }

        //if(buttonYDriver == true)
        //{
           //                                                                                                                                m_climbers.goToDistance(climberDistance);
         //   climbing = true;
        //}

        //Winches do later
        if(povDriver == povUp)
        {
            m_climbers.getLeftTalon().set(climberSpeed);
            m_climbers.getRightTalon().set(climberSpeed);
        }
        else if(povDriver == povDown)
        {
            m_climbers.getLeftTalon().set(-climberSpeed);
            m_climbers.getRightTalon().set(-climberSpeed);
        }

        else 
        {
            m_climbers.getLeftTalon().set(0);
            m_climbers.getRightTalon().set(0);
        }
        //Climmber buttons 
    
        
        //low goal shot (no need to change with new math)
        if (buttonB == true)
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


        }
        else if (buttonReleaseB)
        {
            m_feederSystemFront.getFeederFront().set(0);
            m_feederSystemBack.getFeederBack().set(0);

            m_conveyor.getConveyorMotor().set(0);
            m_shooterDrive.getLeftTalon().set(0);
            m_shooterDrive.getRightTalon().set(0);
        }
        //low goal shot (no need to change with new math)


        if (diagDistance > 99 && diagDistance < 200)
        {
            velocity = velocityCalculate(diagDistance);
            
        }
        else 
        {
            velocity = 5650;
        }

        SmartDashboard.putNumber("Velocity RPM realtime", velocity);


        // high shot has to be redone with the new math and the limelite 
        if (buttonX == true)
        { 
            
            
            m_shooterDrive.getLeftTalon().set(ControlMode.Velocity, velocity);
            m_shooterDrive.getRightTalon().set(ControlMode.Velocity, velocity);
            
            if ((beanBreakFront == false) && (beanBreakMid == true))
            {
                m_intakeFront.getFrontIntakeMotor().set(0.5);

                m_feederSystemFront.getFeederFront().set(0.5); 
            }
            
            
            else if ((beanBreakBack == false) && (beanBreakMid == true))
            {
                m_feederSystemBack.getFeederBack().set(0.5);
                m_intakeBack.getBackIntakeMotor().set(0.5);

            }
            
            if ( m_shooterDrive.getRightTalon().getSelectedSensorVelocity(0) >= (velocity - 10))
            {
                m_conveyor.getConveyorMotor().set(0.6);
            }

        }
        else if (buttonReleaseX)
        {
            m_feederSystemFront.getFeederFront().set(0);
            m_feederSystemBack.getFeederBack().set(0);

            m_intakeFront.getFrontIntakeMotor().set(0);
            m_intakeBack.getBackIntakeMotor().set(0);


            m_conveyor.getConveyorMotor().set(0);
            m_shooterDrive.getLeftTalon().set(0);
            m_shooterDrive.getRightTalon().set(0);
        }
        // high shot has to be redone with the new math and the limelite

        //expel balls
        if (buttonY == true )
        {
           PrimaryIntakeExpel();
        }
        else if (buttonReleaseY)
        {
            PrimaryIntakeDisable();
        }
        //expel balls

        //back intake 
        if (buttonlb == true && beanBreakFront) //left bummper is back
        {
            if ((beanBreakFront == true || beanBreakBack == true) && beanBreakMid == true)
            {
                BackIntakeEnable(); // if there are no balls in the robot activate the whole back intake system
            }   

            else if (beanBreakBack == false && beanBreakMid == true )
            {
                BackIntakeEnable(); // if there is one ball in the back enable everything including conveyor
                m_conveyor.getConveyorMotor().set(0.8);
            }

            else if (beanBreakBack == true && beanBreakMid == false)
            {
                BackIntakeEnable(); // if the ball is only one ball in the robot in mid 
                m_conveyor.getConveyorMotor().set(0); // turn off the conveyor
            }

            else if ((beanBreakBack == false || beanBreakFront == false) && beanBreakMid == false)
            {
                BackIntakeDisable(); // there is either a ball in the front or in the back with a ball in mid turn off everything
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
        //back intake 

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
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
        return false;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
    }
}