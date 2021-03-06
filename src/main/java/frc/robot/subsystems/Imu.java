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


// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS


/**
 *
 */
public class Imu extends SubsystemBase {
 
//private Pigeon2 pigeonIMU1;
private WPI_Pigeon2 pigeonIMU1;

 

    /**
    *
    */
    public Imu() {
       
        pigeonIMU1 = new WPI_Pigeon2(14);
        pigeonIMU1.setYaw(0);
        

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
    public Pigeon2 getImu()
    {
        
        return pigeonIMU1;
    }
    // y = yaw    only needs yaw
    // p = pitch if need pitch or roll your in trouble
    // r = roll
    /*
    public double getYaw()
    {
      
            double[] ypr = new double[3];
            pigeonIMU1.getYawPitchRoll(ypr);
            return -ypr[0];
    } 
    public double getCompassHeading()
    {
        return pigeonIMU1.getCompassHeading();
    }
    */




}

