// RobotBuilder Version: 4.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public class Constants {
   /**
    * public static final class DriveConstants {
    *   public static final int kLeftMotor1Port = 0;
    *   public static final int kLeftMotor2Port = 1;
    *   public static final int kRightMotor1Port = 2;
    *   public static final int kRightMotor2Port = 3; 
    * }
    */ 
	/**
	 * Which PID slot to pull gains from. Starting 2018, you can choose from
	 * 0,1,2 or 3. Only the first two (0,1) are visible in web-based
	 * configuration.
	 */
	public static final int kSlotIdx = 0;

	/**
	 * Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops. For
	 * now we just want the primary one.
	 */
	public static final int kPIDLoopIdx = 0;

	/**
	 * Set to zero to skip waiting for confirmation, set to nonzero to wait and
	 * report to DS if action fails.
	 */
	public static final int kTimeoutMs = 120;//30
	
	/* Choose so that Talon does not report sensor out of phase */
	public static boolean kSensorPhase = true;

	/**
	 * Choose based on what direction you want to be positive,
	 * this does not affect motor invert. 
	 */
	public static boolean kMotorInvert = false;

	/**
	 * Gains used in Positon Closed Loop, to be adjusted accordingly
     * Gains(kp, ki, kd, kf, izone, peak output);
     */
    //static final Gains kGains = new Gains(0.15, 0.0, 1.0, 0.0, 0, 1.0);
	//static final Gains kGains = new Gains(0.125, 0.0001, 1.25, 0.0, 0, 1.0);
	public static final Gains kGains = new Gains(0.25, 0.0000, 0, 0.0, 0, 1.0); // 0.125, 0.000, 0, 0.0, 0, 1.0

	public static boolean kGyroReversed = true;

	//decrease = less sensitive 
	public static final double kp = .05 * 2; //.01
	public static final double ki = 0.0004 * 2; //0.02
	public static final double kd = 0.005 * 2; //0.
/*
	//best so far
	public static final double kp = .05; //.01
	public static final double ki = 0.0004; //0.02
	public static final double kd = 0.005; //0.
*/
}

