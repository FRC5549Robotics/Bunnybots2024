package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
// import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final int DRIVE_CONTROLLER = 0;
    public static final int OPERATOR_CONTROLLER = 1;

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 4; 
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 3;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 10; 

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 2;  
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 1;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 13; 

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 6; 
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 5; 
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 12; 

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 8; 
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 7; 
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 11;  

    // Define the conventional order of our modules when putting them into arrays
    public static final int FRONT_LEFT =0;
    public static final int FRONT_RIGHT =1;
    public static final int REAR_LEFT =2;
    public static final int REAR_RIGHT =3;

    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kRearLeftDriveEncoderReversed = true;
    public static final boolean kRearRightDriveEncoderReversed = true;

    public static final boolean kFrontLeftTurningEncoderReversed = false;
    public static final boolean kFrontRightTurningEncoderReversed = false;
    public static final boolean kRearLeftTurningEncoderReversed = true;
    public static final boolean kRearRightTurningEncoderReversed = true;

    public static final double kWheelDiameterMeters = 0.1016; //0.098; // 0.09398; // 3.7 in

    // The drive encoder reports in RPM by default. Calculate the conversion factor
    // to make it report in meters per second.
    public static final double kDriveGearRatio = 8.143;
    public static final double kDriveConversionFactor = (kWheelDiameterMeters * Math.PI) / kDriveGearRatio;

    public static final double kTurnPositionConversionFactor = 12.8;

    public static final double kMaxSpeedMetersPerSecond = 6.0;
    // Units are meters.
    // Distance between centers of right and left wheels on robot
    public static final double kTrackWidth = 0.56515;
    
    // Distance between front and back wheels on robot
    public static final double kWheelBase = 0.56515;

    // Units are meters per second
    public static final double kMaxTranslationalVelocity = 6784 / 60.0 *
    (1/kDriveGearRatio) *
    kWheelDiameterMeters * Math.PI;

    // Units are radians per second
    public static final double kMaxRotationalVelocity = kMaxTranslationalVelocity /
    Math.hypot(kTrackWidth / 2.0, kWheelBase / 2.0);; //max 5.0

    //The locations f
    //*or the modules must be relative to the center of the robot. 
    // Positive x values represent moving toward the front of the robot 
    // Positive y values represent moving toward the left of the robot.
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0),   // front left
            new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0),  // front right
            new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0),  // rear left
            new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0)  // rear right
            );

    public static final boolean kGyroReversed = false;

    public static final double kDriveP = 0.05;
    public static final double kDriveI = 0.0;
    public static final double kDriveD = 0.0;
    public static final double kDriveFF = 0.1;

    public static final double kTurningP = 0.005;
    public static final double kTurningI = 0.0;
    public static final double kTurningD = 0.0;
    public static final double kAcceleration = 4;

    public static final int RESET_NAVX_BUTTON = 4;
    public static final double ROBOT_ORIENTED_DEADBAND = 0.15;


    //Pivot Constants
    public static final double PIVOT_DEPLOY_SETPOINT = 1;
    public static final double PIVOT_STOW_SETPOINT = 0;
    public static final int DEPLOY_PIVOT_BUTTON = 1;
    public static final int RETRACT_PIVOT_BUTTON = 4;
    public static final int PIVOT_MOTOR_LEFT = 20;
    public static final int PIVOT_MOTOR_RIGHT = 21;
    public static final int PIVOT_JOYSTICK = 1;
    public static final double PIVOT_DEADBAND = 0.05;
    public static final double PIVOT_SCALING_FACTOR = 0.15;
    public static final double PIVOT_RIGHT_RETRACTED_SETPOINT = 1;
    public static final double PIVOT_LEFT_RETRACTED_SETPOINT = -1;
    public static final double PIVOT_RIGHT_INTAKE_SETPOINT = 102.5;
    public static final double PIVOT_LEFT_INTAKE_SETPOINT = -107.2;
    public static final int PIVOT_ENCODER_RIGHT = 4;
    public static final int PIVOT_ENCODER_LEFT = 0;
    public static final double PIVOT_OFFSET_RIGHT = 0.331;
    public static final double PIVOT_OFFSET_LEFT = 0.769;
    public static final double PIVOT_RIGHT_AMP_SETPOINT = 0;
    public static final double PIVOT_LEFT_AMP_SETPOINT = 0;
    public static final int AMP_SHOOTER_BUTTON = 3;


    //Indexer Constants
    public static final int INDEXER_MOTOR = 22;
    public static final double INDEXER_SPEED = 1;
    public static final double INDEXER_SPEED_OUT = 0.5;
    public static final int INDEXER_IN_BUTTON = 2;
    public static final int INDEXER_OUT_BUTTON = 5;
    public static final double SENSOR_VOLTAGE_THRESHOLD = 0.18;
    public static final int INDEXER_LED_STRIP_LENGTH = 69;

 //Shintake
 public static final int SHINTAKE_MOTOR = 21;
 public static final double kP = 0.46416;
 public static final double kI = 0;
 public static final double kD = 0;
 public static final double kS = 0.020963;
 public static final double kV = 0.18825;
 public static final double kA = 0.013596;
 public static final double tP = 1/27;
 public static final int SHOOT_TRIGGER = 2;
 public static final int INTAKE_BUTTON = 3;
 public static final int SHOOT_BUTTON = 4;
 public static final int INDEX_OUT = 1;

 //Shooter Solenoids
 public static final int SHOOTER_SOLENOID_FORWARD = 6;
 public static final int SHOOTER_SOLENOID_REVERSE = 7;
 public static final int PNEUMATIC_BUTTON  = 8;

 public static final double SHINTAKE_SPEED = 0.75;
    //Shooter Constants
    public static final int SHOOTER_LEFT_MOTOR = 25;
    public static final int SHOOTER_RIGHT_MOTOR = 26;
    public static final int SHOOTER_AMP_BUTTON = 2;
    public static final int SHOOTER_INTAKE_BUTTON = 6;
    public static final double SHOOTER_kP = 0;
    public static final double SHOOTER_kI = 0;
    public static final double SHOOTER_kD = 0;
    public static final double SHOOTER_kS = 0;
    public static final double SHOOTER_kV = 0;
    public static final double SHOOTER_kA = 0;
    public static final double SHOOTER_TRIGGER_THRESHOLD = 0.075;
    public static final int SHOOTER_TRIGGER = 3;
    public static final double SHOOTER_SET_SPEED = 1;

    //Climber Constants
    public static final int CLIMBER_MOTOR_LEFT = 27;
    public static final int CLIMBER_MOTOR_RIGHT = 28;
    public static final double CLIMBER_SCALING_FACTOR = 1;// placeholder
    public static final int CLIMBER_LEFT_JOYSTICK = 1;
    public static final int CLIMBER_RIGHT_JOYSTICK = 5;
    public static final double CLIMBER_DEADBAND = 0.05;
    public static final int CLIMBER_BUTTON = 1;
    public static final double CLIMBER_LEFT_ENCODER_MAX = 1;// placeholder
    public static final double CLIMBER_RIGHT_ENCODER_MAX = 1;// placeholder

    //Deflectorinator Constants
    public static final int DEFLECTORINATOR_MOTOR = 29;
    public static final double kDeflectGearRatio = 20;
    public static final int DEFLECTORINATOR_IN_BUTTON = 6;
    public static final int DEFLECTORINATOR_OUT_BUTTON = 5;
    public static final double DEFLECTORINATOR_SPEED = 0.1;
    public static final double DEFLECTORINATOR_OUT_SETPOINT = -0.317857;
    public static final double DEFLECTORINATOR_RETRACTED_SETPOINT = 0;


    //Limelight Constants
    public static final double Kp = 1/27;
    public static final int AUTO_ALIGN_BUTTON = 8;
    public static final double CAMERA_HEIGHT_METERS = 0.8;
    public static final double TARGET_HEIGHT_METERS = 1.22;
    public static final double ALIGN_kP = 0;
    public static final double ALIGN_kI = 0;
    public static final double ALIGN_kD = 0;

}