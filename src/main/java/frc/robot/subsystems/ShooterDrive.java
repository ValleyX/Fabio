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


import frc.robot.Constants;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS


/**
 *
 */
public class ShooterDrive extends SubsystemBase {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
private WPI_TalonFX shooterRight;
private WPI_TalonFX shooterLeft;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    
    /**
    *
    */
    public ShooterDrive() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
shooterRight = new WPI_TalonFX(6);
 
 

shooterLeft = new WPI_TalonFX(7);
 
 


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    shooterLeft.setInverted(true);
    shooterRight.setInverted(false);

    /* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
    shooterLeft.config_kF(Constants.kPIDLoopIdx, 0, Constants.kTimeoutMs);
    shooterLeft.config_kP(Constants.kPIDLoopIdx, .125, Constants.kTimeoutMs);
    shooterLeft.config_kI(Constants.kPIDLoopIdx, .00025, Constants.kTimeoutMs);
    shooterLeft.config_kD(Constants.kPIDLoopIdx, 0, Constants.kTimeoutMs);

    shooterLeft.configAllowableClosedloopError(Constants.kPIDLoopIdx,0, Constants.kTimeoutMs);

    /* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
    shooterRight.config_kF(Constants.kPIDLoopIdx, 0, Constants.kTimeoutMs);
    shooterRight.config_kP(Constants.kPIDLoopIdx, .125, Constants.kTimeoutMs);
    shooterRight.config_kI(Constants.kPIDLoopIdx, .00025, Constants.kTimeoutMs);
    shooterRight.config_kD(Constants.kPIDLoopIdx, 0, Constants.kTimeoutMs);

    shooterRight.configAllowableClosedloopError(Constants.kPIDLoopIdx,0, Constants.kTimeoutMs);

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
    public WPI_TalonFX getLeftTalon()
    {
        return shooterLeft;
    }



    public WPI_TalonFX getRightTalon()
    {
        return shooterRight;
    }
}

