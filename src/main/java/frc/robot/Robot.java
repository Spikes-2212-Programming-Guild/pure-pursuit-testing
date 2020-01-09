/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.spikes2212.command.drivetrains.TankDrivetrain;
import com.spikes2212.control.FeedForwardController;
import com.spikes2212.path.*;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private interface RobotMap {
      int LEFT_MOTOR_TALON = 5;
      int LEFT_MOTOR_VICTOR = 6;
      int RIGHT_MOTOR_TALON = 4;
      int RIGHT_MOTOR_VICTOR = 1;

      int ENCODER_LEFT_POS = 0;
      int ENCODER_LEFT_NEG = 1;
      int ENCODER_RIGHT_POS = 3;
      int ENCODER_RIGHT_NEG = 2;
  }

  public static Encoder right, left;
  public static IMotorController leftVictor, rightVictor, leftTalon, rightTalon;
  public static TankDrivetrain drivetrain;
  public static OdometryHandler handler;
  public static ADXRS450_Gyro gyro;
  public static Waypoint start = new Waypoint(0,0), middle = new Waypoint(0,2),
    end = new Waypoint(1.5, 2);
  public static Path path;
  public static PurePursuitController controller;
  public static final double kV = 0.2/3.05, kA = 0.002, kB = 0.04;

  @Override
  public void robotInit() {
      drivetrain = new TankDrivetrain((WPI_TalonSRX)
              (leftTalon = new WPI_TalonSRX(RobotMap.LEFT_MOTOR_TALON))
              , (WPI_TalonSRX)(rightTalon = new WPI_TalonSRX(RobotMap.RIGHT_MOTOR_TALON)));
      (leftVictor = new WPI_VictorSPX(RobotMap.LEFT_MOTOR_VICTOR)).follow(leftTalon);
      (rightVictor = new WPI_VictorSPX(RobotMap.RIGHT_MOTOR_VICTOR)).follow(rightTalon);
      right = new Encoder(RobotMap.ENCODER_RIGHT_POS, RobotMap.ENCODER_RIGHT_NEG);
      left = new Encoder(RobotMap.ENCODER_LEFT_POS, RobotMap.ENCODER_LEFT_NEG);
      right.setDistancePerPulse(6*0.0254*Math.PI/360);
      left.setDistancePerPulse(6*0.0254*Math.PI/360);
      gyro = new ADXRS450_Gyro();
      handler = new OdometryHandler(left::getDistance, right::getDistance,
              gyro::getAngle, 0, 0, 0);
      path = new Path(0.15, 0.98,
              0.001, 3.05, 3, 18, start,
              middle, end);
      controller = new PurePursuitController(handler, path, 0.4, 0.7);
  }

  @Override
  public void robotPeriodic() {
      handler.calculate();
      SmartDashboard.putNumber("odometry x position", handler.getX());
      SmartDashboard.putNumber("odometry y position", handler.getY());
      SmartDashboard.putNumber("odometry yaw position", handler.getYaw());
  }

  @Override
  public void disabledInit() {drivetrain.stop();}

  public static double previousRate = 0;
  public static double currentMaximum = Double.MIN_VALUE;
    @Override
    public void teleopPeriodic() {
        drivetrain.tankDrive(1, -1);
        if((-previousRate + left.getRate())/getPeriod() > currentMaximum)
        SmartDashboard.putNumber("max acceleration", currentMaximum = (-previousRate + left.getRate())/getPeriod());
        SmartDashboard.putNumber("period", getPeriod());
        previousRate = left.getRate();
    }

    @Override
    public void autonomousInit() {
        left.reset();
        right.reset();
        handler.set(0,0);
        gyro.reset();
        controller.reset();
    }

  @Override
  public void autonomousPeriodic() {
      try {
          double[] speeds = controller.getTargetSpeeds();
          FeedForwardController leftController = new FeedForwardController(kV, kA, getPeriod());
          FeedForwardController rightController = new FeedForwardController(kV, kA, getPeriod());
          double leftSpeed = leftController.calculate(speeds[0]) +
                  kB * (speeds[0] - left.getRate());
          double rightSpeed = rightController.calculate(speeds[1]) +
                          kB * (speeds[1] - right.getRate());
         drivetrain.tankDrive(leftSpeed, rightSpeed);
          SmartDashboard.putNumber("speed left", speeds[0]);
          SmartDashboard.putNumber("speed right", speeds[1]);
          SmartDashboard.putNumber("speed left converted", leftSpeed);
          SmartDashboard.putNumber("speed right converted", rightSpeed);
      } catch (LookaheadPointNotFoundException e) {
          drivetrain.stop();
      }
  }
}