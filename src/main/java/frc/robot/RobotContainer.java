// RobotBuilder Version: 4.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: RobotContainer.

package frc.robot;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.*;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS


/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private static RobotContainer m_robotContainer = new RobotContainer();
  private final SequentialCommandGroup1ball m_SeqCmdGrp1ball;
  private final SequentialCommandGroup2ball m_SeqCmdGrp2ball;
  private final SequentialCommandGroup3ball m_SeqCmdGrp3ball;
  private final SequentialCommandGroup4ball m_SeqCmdGrp4ball;
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
// The robot's subsystems
    //public final Cimbers m_cimbers = new Cimbers();
    public final BeanBreakFront m_beanBreakFront = new BeanBreakFront();
    public final BeanBreakBack m_beanBreakBack = new BeanBreakBack();
    public final BeanBreakMid m_beanBreakMid = new BeanBreakMid();
    public final FrontIntakeArmSub m_frontIntakeArmSub = new FrontIntakeArmSub();
    public final Sorter m_sorter = new Sorter();
    public final FeederSystemBack m_feederSystemBack = new FeederSystemBack();
    public final Conveyor m_conveyor = new Conveyor();
    public final FeederSystemFront m_feederSystemFront = new FeederSystemFront();
    public final IntakeBack m_intakeBack = new IntakeBack();
    public final IntakeFront m_intakeFront = new IntakeFront();
    public final ShooterDrive m_shooterDrive = new ShooterDrive();
    public final DriveTrain m_driveTrain = new DriveTrain();
    public final BackIntakeArmSub m_backIntakeArm= new BackIntakeArmSub();
    public final Climbers  m_deathClimbers = new Climbers();
    public final DriveBaseBlinkin1 m_driveBaseBlinkin1 = new DriveBaseBlinkin1();
    public final DriveBaseBlinkin2 m_driveBaseBlinkin2 = new DriveBaseBlinkin2();

// Joysticks
private final XboxController xboxController1 = new XboxController(0);

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    public final Imu m_imu = new Imu();
  
  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
  * The container for the robot.  Contains subsystems, OI devices, and commands.
  */
  private RobotContainer() {
    m_SeqCmdGrp1ball = new SequentialCommandGroup1ball(m_driveTrain,
                                              m_frontIntakeArmSub, 
                                              m_shooterDrive, 
                                              m_sorter,
                                              m_intakeFront,
                                              m_conveyor, 
                                              m_feederSystemFront, 
                                              m_beanBreakMid,
                                              m_beanBreakBack,
                                              m_beanBreakFront);
    m_SeqCmdGrp2ball = new SequentialCommandGroup2ball(m_driveTrain,
                                              m_frontIntakeArmSub, 
                                              m_shooterDrive, 
                                              m_sorter,
                                              m_intakeFront,
                                              m_conveyor, 
                                              m_feederSystemFront, 
                                              m_beanBreakMid,
                                              m_beanBreakBack,
                                              m_beanBreakFront);
    m_SeqCmdGrp3ball = new SequentialCommandGroup3ball(m_driveTrain,
                                              m_frontIntakeArmSub, 
                                              m_shooterDrive, 
                                              m_sorter,
                                              m_intakeFront,
                                              m_conveyor, 
                                              m_feederSystemFront, 
                                              m_beanBreakMid,
                                              m_beanBreakBack,
                                              m_beanBreakFront);
    m_SeqCmdGrp4ball = new SequentialCommandGroup4ball(m_driveTrain,
                                              m_frontIntakeArmSub, 
                                              m_shooterDrive, 
                                              m_sorter,
                                              m_intakeFront,
                                              m_conveyor, 
                                              m_feederSystemFront, 
                                              m_beanBreakMid,
                                              m_beanBreakBack,
                                              m_beanBreakFront);


        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SMARTDASHBOARD
    // Smartdashboard Subsystems
    SmartDashboard.putData(m_shooterDrive);


    // SmartDashboard Buttons
    SmartDashboard.putData("Autonomous Command", new AutonomousCommand());
    SmartDashboard.putData("DriveOnJoystick", new DriveOnJoystick( m_driveTrain ));
    SmartDashboard.putData("Sequential Command Group 1", new SequentialCommandGroup2ball(m_driveTrain,
    m_frontIntakeArmSub, 
    m_shooterDrive, 
    m_sorter,
    m_intakeFront,
    m_conveyor, 
    m_feederSystemFront, 
    m_beanBreakMid,
    m_beanBreakBack,
    m_beanBreakFront));
    SmartDashboard.putData("TakeShot", new TakeShot( m_shooterDrive, m_conveyor ));
    //SmartDashboard.putData("IntakeFromFront", new IntakeFromFront( m_intakeFront ));
    SmartDashboard.putData("RobotPractice", new RobotPractice( m_driveTrain ));
    SmartDashboard.putData("ConveyorDriver", new ConveyorDriver( m_conveyor ));
    SmartDashboard.putData("FeederFrontDriver", new FeederFrontDriver( m_feederSystemFront ));
    SmartDashboard.putData("SorterDriver", new SorterDriver( m_sorter ));
    SmartDashboard.putData("FrontIntakeArmsExtend", new FrontIntakeArmsExtend( m_frontIntakeArmSub ));
    SmartDashboard.putData("Sequential Command Group 2", new SequentialCommandGroup2( m_driveTrain ));
    SmartDashboard.putData("IMUTest", new IMUTest());

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SMARTDASHBOARD
    //SmartDashboard.putData("JonCommand", new JonCommand(m_shooterDrive, m_intakeFront, m_conveyor, m_feederSystemFront, m_sorter, m_frontIntakeArmSub));
    
    m_driveTrain.setDefaultCommand(new JonCommand(m_driveTrain, m_shooterDrive, m_intakeFront, m_intakeBack, 
                                m_conveyor, m_feederSystemFront, m_feederSystemBack, m_sorter, m_frontIntakeArmSub, m_backIntakeArm,
                                m_beanBreakMid, m_beanBreakBack, m_beanBreakFront, m_deathClimbers, m_driveBaseBlinkin1, m_driveBaseBlinkin2));
                                
    m_imu.setDefaultCommand(new imuCommand(m_imu));
    // Configure the button bindings
    configureButtonBindings();


    // Configure default commands
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SUBSYSTEM_DEFAULT_COMMAND


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SUBSYSTEM_DEFAULT_COMMAND

    // Configure autonomous sendable chooser
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMOUS

        //m_chooser.setDefaultOption("Autonomous Command", new AutonomousCommand());
        m_chooser.setDefaultOption("One Ball", m_SeqCmdGrp1ball);
      m_chooser.addOption("Two Ball", m_SeqCmdGrp2ball);
        m_chooser.addOption("Three Ball", m_SeqCmdGrp3ball);
      //  m_chooser.addOption("Four Ball", m_SeqCmdGrp4ball);

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMOUS

    SmartDashboard.putData("Auto Mode", m_chooser);
  }

  public static RobotContainer getInstance() {
    return m_robotContainer;
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=BUTTONS
// Create some buttons

/*
final JoystickButton intakeArmExtendA = new JoystickButton(xboxController1, XboxController.Button.kStart.value);        
intakeArmExtendA.toggleWhenPressed(new FrontIntakeArmsExtend( m_frontIntakeArmSub ).withTimeout(0.0) ,true);
    SmartDashboard.putData("IntakeArmExtendA",new FrontIntakeArmsExtend( m_frontIntakeArmSub ).withTimeout(0.0) );

final JoystickButton conveyorButtonY = new JoystickButton(xboxController1, XboxController.Button.kRightBumper.value);        
conveyorButtonY.whenPressed(new ConveyorDriver( m_conveyor ) ,true);
    SmartDashboard.putData("ConveyorButtonY",new ConveyorDriver( m_conveyor ) );

final JoystickButton xboxButtonB = new JoystickButton(xboxController1, XboxController.Button.kLeftBumper.value);        
xboxButtonB.whileHeld(new TakeShot( m_shooterDrive, m_conveyor ) ,true);
    SmartDashboard.putData("Xbox Button B",new TakeShot( m_shooterDrive, m_conveyor ) );
    */

    final JoystickButton aButton = new JoystickButton(xboxController1, XboxController.Button.kA.value);        
aButton.whileHeld(new DriveDistance(m_driveTrain, 1, 4) ,true);
    //SmartDashboard.putData("Xbox Button B",new TakeShot( m_shooterDrive, m_conveyor ) );
  



    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=BUTTONS
  }

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS
public XboxController getXboxController1() {
      return xboxController1;
    }


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
  */
  public Command getAutonomousCommand() {
    // The selected command will be run in autonomous
    //return m_chooser.getSelected();
   // return m_SeqCmdGrp2ball;
    return m_SeqCmdGrp3ball;
  }
  

}

