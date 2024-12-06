// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;
// import java.io.Console;


// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import frc.robot.Constants;
// import java.lang.Math;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.Encoder;
// import edu.wpi.first.wpilibj.XboxController;
// import org.photonvision.PhotonCamera;
// import org.photonvision.targeting.PhotonPipelineResult;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Translation3d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;

// import com.revrobotics.CANSparkLowLevel.MotorType;
// import edu.wpi.first.wpilibj.DriveCommand;




// public class Limelight extends SubsystemBase {
//   /** Creates a new Limelight. */
  
//   NetworkTable limelightTable;
//   double ty, tv, tx, angle, distance;
//   double min_command = 0.05;
//   PIDController alignController = new PIDController(Constants.ALIGN_kP, Constants.ALIGN_kI, Constants.ALIGN_kD);
//   PhotonCamera  camera;
//   PIDController pid = new PIDController(1, 1, 1);

  
//   double steering_adjust = 0.0;
//   private static Limelight limelight = null;
//   CommandXboxController xboxcontroller;
//   DrivetrainSubsystem m_drivetrain;
 

//   public Limelight(CommandXboxController m_controller, DrivetrainSubsystem drivetrain) {
//     limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
//     camera = new PhotonCamera("photonvision");
//     xboxcontroller = m_controller;
//     m_drivetrain = drivetrain;
//     //Kp = inputkP;
//   }

//   public double getAngle() {
//     var result = camera.getLatestResult();
//     return result.getBestTarget().getYaw();

//     // SmartDashboard.putNumber("Horizontal Angle:", tx);
//     // if (tx != 0) {
//     //   return tx;
//     // } else {
//     // return 0;
//     // }
//   }



//   public double getDistance() {
//     return 0;  
//   }

//   public static Limelight getInstance(){
//     return Limelight.limelight;
//   }

//   public double getDesiredRPM(){
//     // double a = 1.3;
//     // double b = 24.18; //+ Constants.PIDB_CONSTANT;
//     // return a*getDistance() + b;
//     // // return 0.149827*(Math.pow(1.04964, (4.99985*this.getDistance()) + 29.9996) + 28.4836);
//     // //Add implementation
//     return 0;
//   }

//   public double getSpeakerTheta(){
//     return alignController.calculate(getAngle(), 0);
//   }
  
//   public double getAmpTheta(){
//     return alignController.calculate(getAngle(), getAmpDesiredAngle());
//   }

//   public double getAmpDesiredAngle(){
//     var result = camera.getLatestResult();
//     var res = result.getBestTarget().getBestCameraToTarget();
//     double x = res.gettX();
//     double y = res.getY();
//     angle = Math.atan(x/y);
    
    
//     return angle;
//   }

//   @Override
//   public void periodic() {
//     ty = limelightTable.getEntry("ty").getDouble(0);
//     tv = limelightTable.getEntry("tv").getDouble(0);
//     tx = limelightTable.getEntry("tx").getDouble(0);
//     Encoder encoder = new Encoder(0, 1); // this thing translates tx to physical

//     //motor.set(pid.calculate(encoder.getDistance(), tx));
    
//     SmartDashboard.putNumber("Horizontal", tx); 
//     SmartDashboard.putNumber("Vertical:", ty);

//     if ((xboxcontroller.getAButton()== true) && (tv==1)){
//       m_drivetrain.drive(new ChassisSpeeds(3.0, -2.0, pid.calculate(tx)));


//     }

  
//   }

//   public void autoAlignJawntius() {
    
//     double heading_error = -tx;
     
//     if (tx > 1.0 || tx < -1.0)
//     {
//       // Drivetrain.getInstance().rightGroup.set(-heading_error/70);        
//       // Drivetrain.getInstance().leftGroup.set(heading_error/70);
//     }
//   }

// }