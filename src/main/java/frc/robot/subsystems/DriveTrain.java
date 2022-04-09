// RobotBuilder Version: 4.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;


import frc.robot.commands.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import java.util.concurrent.TimeoutException;
import java.lang.Thread;

import javax.print.CancelablePrintJob;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.can.*;
import frc.robot.Constants;
import frc.robot.Gains;
import frc.robot.subsystems.*;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS


/**
 *
 */
public class DriveTrain extends SubsystemBase {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
private WPI_TalonFX leftFront;
private WPI_TalonFX leftBack;
private MotorControllerGroup motorCtrolGroupLeft;
private WPI_TalonFX rightFront;
private WPI_TalonFX rightBack;
private MotorControllerGroup motorCtrlGroupRight;
private DifferentialDrive differentialDrive1;
private CANCoder cANCoderRight;
private CANCoder cANCoderLeft;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    

    private static final double c_wheelDiameterIn = 4.0;
    private static final double c_OneWheelRevDistanceIn = c_wheelDiameterIn * Math.PI;
    private static final double c_countsPerMotorRev = 4096;
    private static final double c_countsPerInch = c_countsPerMotorRev / c_OneWheelRevDistanceIn;
    private static final double c_countsPerFoot = (c_countsPerMotorRev / c_OneWheelRevDistanceIn) * 12;
    
  // The gyro sensor
  private final WPI_Pigeon2  m_gyro = new WPI_Pigeon2 (14);
  
    /**
    *
    */
    public DriveTrain() {
        

        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
leftFront = new WPI_TalonFX(0);
 
 

leftBack = new WPI_TalonFX(1);
 

 

motorCtrolGroupLeft = new MotorControllerGroup(leftFront, leftBack  );
 addChild("MotorCtrolGroupLeft",motorCtrolGroupLeft);
 

rightFront = new WPI_TalonFX(3);
 
 

rightBack = new WPI_TalonFX(2);
 
 

motorCtrlGroupRight = new MotorControllerGroup(rightFront, rightBack  );
 addChild("MotorCtrlGroupRight",motorCtrlGroupRight);
 

differentialDrive1 = new DifferentialDrive(motorCtrolGroupLeft, motorCtrlGroupRight);
 addChild("Differential Drive 1",differentialDrive1);
 differentialDrive1.setSafetyEnabled(true);
differentialDrive1.setExpiration(0.1);
differentialDrive1.setMaxOutput(1.0);


cANCoderRight = new CANCoder(5);
 
 

cANCoderLeft = new CANCoder(4);
 
 


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS


    cANCoderLeft.configSensorDirection(true);
    cANCoderRight.configSensorDirection(true);

    

    leftBack.configFactoryDefault();
    leftFront.configFactoryDefault();
    rightBack.configFactoryDefault();
    rightFront.configFactoryDefault();
    
    TalonFXConfiguration allConfigs = new TalonFXConfiguration();
    
    leftBack.getAllConfigs(allConfigs);
    allConfigs.remoteFilter0.remoteSensorDeviceID = 4;
    allConfigs.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;
    allConfigs.remoteFilter1.remoteSensorDeviceID = 5;
    allConfigs.remoteFilter1.remoteSensorSource = RemoteSensorSource.CANCoder;

    allConfigs.diff0Term = FeedbackDevice.RemoteSensor0;
    allConfigs.diff1Term = FeedbackDevice.RemoteSensor1;

    allConfigs.closedloopRamp = 0;
    allConfigs.primaryPID.selectedFeedbackCoefficient= 1;
    allConfigs.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;
    allConfigs.auxiliaryPID.selectedFeedbackCoefficient = 1;
    allConfigs.auxiliaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor1;
    
    leftBack.configAllSettings(allConfigs);
    leftFront.configAllSettings(allConfigs);

rightBack.getAllConfigs(allConfigs);
    allConfigs.remoteFilter0.remoteSensorDeviceID = 5;
    allConfigs.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;
    allConfigs.remoteFilter1.remoteSensorDeviceID = 4;
    allConfigs.remoteFilter1.remoteSensorSource = RemoteSensorSource.CANCoder;

    allConfigs.diff0Term = FeedbackDevice.RemoteSensor0;
    allConfigs.diff1Term = FeedbackDevice.RemoteSensor1;

    allConfigs.closedloopRamp = 0;
    allConfigs.primaryPID.selectedFeedbackCoefficient= 1;
    allConfigs.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;
    allConfigs.auxiliaryPID.selectedFeedbackCoefficient = 1;
    allConfigs.auxiliaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor1;

rightBack.configAllSettings(allConfigs);
rightFront.configAllSettings(allConfigs);
    

    
    /* Config the sensor used for Primary PID and sensor direction */
    /*
    _talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 
                                        Constants.kPIDLoopIdx,
                                        Constants.kTimeoutMs);
                                        */

    leftBack.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, 
    Constants.kPIDLoopIdx,
    Constants.kTimeoutMs);
    leftFront.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, 
    Constants.kPIDLoopIdx,
    Constants.kTimeoutMs);

rightBack.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, 
    Constants.kPIDLoopIdx,
    Constants.kTimeoutMs);
rightFront.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, 
    Constants.kPIDLoopIdx,
    Constants.kTimeoutMs);


    /* Ensure sensor is positive when output is positive */
    leftBack.setSensorPhase(Constants.kSensorPhase);
    leftFront.setSensorPhase(Constants.kSensorPhase);
rightBack.setSensorPhase(Constants.kSensorPhase);
rightFront.setSensorPhase(Constants.kSensorPhase);

    /**
     * Set based on what direction you want forward/positive to be.
     * This does not affect sensor phase. 
     */ 
    leftBack.setInverted(Constants.kMotorInvert);
    leftFront.setInverted(Constants.kMotorInvert);
rightBack.setInverted(!Constants.kMotorInvert);
rightFront.setInverted(!Constants.kMotorInvert);

/*
    leftBack.setNeutralMode(NeutralMode.Brake);
    leftFront.setNeutralMode(NeutralMode.Brake);
    rightBack.setNeutralMode(NeutralMode.Brake);
    rightFront.setNeutralMode(NeutralMode.Brake);
*/
leftBack.setNeutralMode(NeutralMode.Coast);
leftFront.setNeutralMode(NeutralMode.Coast);
rightBack.setNeutralMode(NeutralMode.Coast);
rightFront.setNeutralMode(NeutralMode.Coast);


    /* Config the peak and nominal outputs, 12V means full */
    leftBack.configNominalOutputForward(0, Constants.kTimeoutMs);
    leftBack.configNominalOutputReverse(0, Constants.kTimeoutMs);
    leftBack.configPeakOutputForward(1, Constants.kTimeoutMs);
    leftBack.configPeakOutputReverse(-1, Constants.kTimeoutMs);

    leftFront.configNominalOutputForward(0, Constants.kTimeoutMs);
    leftFront.configNominalOutputReverse(0, Constants.kTimeoutMs);
    leftFront.configPeakOutputForward(1, Constants.kTimeoutMs);
    leftFront.configPeakOutputReverse(-1, Constants.kTimeoutMs);

rightBack.configNominalOutputForward(0, Constants.kTimeoutMs);
rightBack.configNominalOutputReverse(0, Constants.kTimeoutMs);
rightBack.configPeakOutputForward(1, Constants.kTimeoutMs);
rightBack.configPeakOutputReverse(-1, Constants.kTimeoutMs);

rightFront.configNominalOutputForward(0, Constants.kTimeoutMs);
rightFront.configNominalOutputReverse(0, Constants.kTimeoutMs);
rightFront.configPeakOutputForward(1, Constants.kTimeoutMs);
rightFront.configPeakOutputReverse(-1, Constants.kTimeoutMs);

    /**
     * Config the allowable closed-loop error, Closed-Loop output will be
     * neutral within this range. See Table in Section 17.2.1 for native
     * units per rotation.
     */
    leftBack.configAllowableClosedloopError(Constants.kPIDLoopIdx,100, Constants.kTimeoutMs);

    /* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
    leftBack.config_kF(Constants.kPIDLoopIdx, Constants.kGains.kF, Constants.kTimeoutMs);
    leftBack.config_kP(Constants.kPIDLoopIdx, Constants.kGains.kP, Constants.kTimeoutMs);
    leftBack.config_kI(Constants.kPIDLoopIdx, Constants.kGains.kI, Constants.kTimeoutMs);
    leftBack.config_kD(Constants.kPIDLoopIdx, Constants.kGains.kD, Constants.kTimeoutMs);

    leftFront.configAllowableClosedloopError(Constants.kPIDLoopIdx,100, Constants.kTimeoutMs);

    /* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
    leftFront.config_kF(Constants.kPIDLoopIdx, Constants.kGains.kF, Constants.kTimeoutMs);
    leftFront.config_kP(Constants.kPIDLoopIdx, Constants.kGains.kP, Constants.kTimeoutMs);
    leftFront.config_kI(Constants.kPIDLoopIdx, Constants.kGains.kI, Constants.kTimeoutMs);
    leftFront.config_kD(Constants.kPIDLoopIdx, Constants.kGains.kD, Constants.kTimeoutMs);

rightBack.configAllowableClosedloopError(Constants.kPIDLoopIdx,100, Constants.kTimeoutMs);

    /* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
rightBack.config_kF(Constants.kPIDLoopIdx, Constants.kGains.kF, Constants.kTimeoutMs);
rightBack.config_kP(Constants.kPIDLoopIdx, Constants.kGains.kP, Constants.kTimeoutMs);
rightBack.config_kI(Constants.kPIDLoopIdx, Constants.kGains.kI, Constants.kTimeoutMs);
rightBack.config_kD(Constants.kPIDLoopIdx, Constants.kGains.kD, Constants.kTimeoutMs);

rightFront.configAllowableClosedloopError(Constants.kPIDLoopIdx,100, Constants.kTimeoutMs);

    /* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
rightFront.config_kF(Constants.kPIDLoopIdx, Constants.kGains.kF, Constants.kTimeoutMs);
rightFront.config_kP(Constants.kPIDLoopIdx, Constants.kGains.kP, Constants.kTimeoutMs);
rightFront.config_kI(Constants.kPIDLoopIdx, Constants.kGains.kI, Constants.kTimeoutMs);
rightFront.config_kD(Constants.kPIDLoopIdx, Constants.kGains.kD, Constants.kTimeoutMs);

    leftFront.follow(leftBack);
rightFront.follow(rightBack);

    
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }
    
    
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    public DifferentialDrive getDifferentialDrive()
    {
        return differentialDrive1;
    }

    public void SetPercentOutput(double speed, int TimeoutMs)
	{
		leftFront.configPeakOutputForward(speed, TimeoutMs);
		leftFront.configPeakOutputReverse(-speed, TimeoutMs);


		leftBack.configPeakOutputForward(speed, TimeoutMs);
		leftBack.configPeakOutputReverse(-speed, TimeoutMs);


		rightFront.configPeakOutputForward(speed, TimeoutMs);
		rightFront.configPeakOutputReverse(-speed, TimeoutMs);

		rightBack.configPeakOutputForward(speed, TimeoutMs);
		rightBack.configPeakOutputReverse(-speed, TimeoutMs);
	}

   


    public void SetLeftRightDistance(double speed, double numberLeftFeet, double numberRightFeet)
    {


        SetPercentOutput(speed, Constants.kTimeoutMs);
        
        //cANCoderLeft.setPosition(0);
        //talonSRX1.stopMotor();
        


        
        double currentPositionLeft = leftBack.getSelectedSensorPosition(0);
		double targetPositionRotationsLeft = currentPositionLeft + (c_countsPerFoot * numberLeftFeet);
	//	double targetPositionRotationsLeft = currentPositionLeft + (10 * 4096);
		leftBack.set(ControlMode.Position, targetPositionRotationsLeft);
		
       // talonSRX3.stopMotor();
        
        //cANCoderRight.setPosition(0);
		double currentPositionRight = rightBack.getSelectedSensorPosition(0);
		double targetPositionRotationsRight = currentPositionRight + (c_countsPerFoot * numberRightFeet);
		//double targetPositionRotationsRight = currentPositionRight + (10 * 4096);
		rightBack.set(ControlMode.Position, targetPositionRotationsRight);



        
          /*      
        while (leftBack.isAlive() && rightBack.isAlive())
        {

            if (numberLeftFeet == numberRightFeet)
            {
                while ( currentPositionLeft - currentPositionRight > 0.1)
                {
                    currentPositionLeft = leftBack.getSelectedSensorPosition(0);
                    currentPositionRight = rightBack.getSelectedSensorPosition(0);
                    leftBack.set(speed -= 0.1);
                }

                while (currentPositionLeft - currentPositionRight < 0.1)
                {
                    currentPositionLeft = leftBack.getSelectedSensorPosition(0);
                    currentPositionRight = rightBack.getSelectedSensorPosition(0);
                    rightBack.set(speed -= 0.1);
                }

                leftBack.set(1);
                rightBack.set(1);


            }
        }
        */
      
    }

    public void allpower(double speed)
    {
        leftBack.set(speed);
        rightBack.set(speed);

    }
    public void sleep(int sleep)
    {
        try {
            Thread.sleep(sleep);
        } catch (InterruptedException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }

    public boolean IsLeftClose(double value)
    {
        return Math.abs(leftBack.getClosedLoopError()) < value;
    }
    public boolean IsRightClose(double value)
    {
        return Math.abs(rightBack.getClosedLoopError()) < value;
    }

    public double getLeftCurrentPos() {
        return leftBack.getClosedLoopError();
        
    }

    public double getRightCurrentPos() {
        return rightBack.getClosedLoopError();
    }

    public double getCountsPerFoot() {
        return c_countsPerFoot;
    }

    public WPI_TalonFX getLeftBack(){
        return leftBack;
    }

    public WPI_TalonFX getRightBack(){
        return rightBack;
    }

    public WPI_TalonFX getLeftFront(){
        return leftFront;
    }

    public WPI_TalonFX getRightFront(){
        return rightFront;
    }

    public WPI_Pigeon2 getImu()
    {
        return m_gyro;
    }

    //test 
    public static double clip(double number, double min, double max) {
        if (number < min) {
            return min;
        } else {
            return number > max ? max : number;
        }
    }

    public static float clip(float number, float min, float max) {
        if (number < min) {
            return min;
        } else {
            return number > max ? max : number;
        }
    }

    public static int clip(int number, int min, int max) {
        if (number < min) {
            return min;
        } else {
            return number > max ? max : number;
        }
    }

    public static short clip(short number, short min, short max) {
        if (number < min) {
            return min;
        } else {
            return number > max ? max : number;
        }
    }

    public static byte clip(byte number, byte min, byte max) {
        if (number < min) {
            return min;
        } else {
            return number > max ? max : number;
        }
    }
    //test

    WaitCommand m_wait = new WaitCommand(.02);
 /** Zeroes the heading of the robot. */
 public void zeroHeading() {
    m_gyro.setYaw(0);
    
    while (Math.abs(m_gyro.getYaw()) > 0.1)
    {
      m_wait.execute();
    }

  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from 180 to 180
   */
  public double getHeading() {
    return Math.IEEEremainder(m_gyro.getYaw(), 360) * (Constants.kGyroReversed ? -1.0 : 1.0);

  }

  public double getAdjustedYaw()
  {
    return -m_gyro.getYaw();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (Constants.kGyroReversed ? -1.0 : 1.0);
    //return 0;
  }

  public void arcadeDrive(double fwd, double rot) {
    differentialDrive1.arcadeDrive(fwd, rot);
   
  }

  

}

