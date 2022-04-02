// RobotBuilder Version: 4.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;
import org.ejml.dense.block.MatrixOps_DDRB;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.DriveBaseBlinkin1;
import frc.robot.subsystems.DriveBaseBlinkin2;
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.FeederSystemBack;
import frc.robot.subsystems.FeederSystemFront;
import frc.robot.subsystems.FrontIntakeArmSub;
import frc.robot.subsystems.Imu;
import frc.robot.subsystems.IntakeFront;
import frc.robot.subsystems.IntakeBack;
import frc.robot.subsystems.IntakeFrontArm;
import frc.robot.subsystems.ShooterDrive;
import frc.robot.subsystems.Sorter;
import frc.robot.subsystems.BackIntakeArmSub;
import frc.robot.subsystems.BeanBreakBack;
import frc.robot.subsystems.BeanBreakFront;
import frc.robot.subsystems.BeanBreakMid;
import frc.robot.subsystems.Climbers;

/**
 *
 */
public class SequentialCommandGroupPlayBack extends SequentialCommandGroup {

  

    public SequentialCommandGroupPlayBack(String filename, DriveTrain driveTrain, FrontIntakeArmSub frontarm, 
                                     ShooterDrive shoot, Sorter sort, IntakeFront intakeFront,
                                    Conveyor conveyor, FeederSystemFront feederfront, BeanBreakMid beanBreakMid,
                                    BeanBreakBack beanBreakBack, BeanBreakFront beanBreakFront,
                                    IntakeBack intakeBack, FeederSystemBack feederSystemBack, 
                                    BackIntakeArmSub backIntakeArm, Climbers deathClimbers,
                                     DriveBaseBlinkin1 driveBaseBlinkin1, DriveBaseBlinkin2 driveBaseBlinkin2)
    {

    addCommands(
        // Add Commands here:
        // Also add parallel commands using the
        //
        // addCommands(
        //      new command1(argsN, subsystem),
        //      parallel(
        //          new command2(argsN, subsystem),
        //          new command3(argsN, subsystem)
        //      )    
     
        //  );


       new JonCommandPlayback(filename,
       driveTrain,
       shoot, 
       intakeFront,
       intakeBack,
       conveyor,
       feederfront,
       feederSystemBack,
       sort,
       frontarm, 
       backIntakeArm,
       beanBreakMid,
       beanBreakBack,
       beanBreakFront,
       deathClimbers,
       driveBaseBlinkin1,
       driveBaseBlinkin2)

    
        );
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}
