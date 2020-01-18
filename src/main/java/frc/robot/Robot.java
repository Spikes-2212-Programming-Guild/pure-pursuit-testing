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
import com.spikes2212.command.drivetrains.OdometryDrivetrain;
import com.spikes2212.command.drivetrains.commands.FollowPath;
import com.spikes2212.control.FeedForwardController;
import com.spikes2212.control.PIDVASettings;
import com.spikes2212.path.OdometryHandler;
import com.spikes2212.path.Path;
import com.spikes2212.path.PurePursuitController;
import com.spikes2212.path.Waypoint;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

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

      int ENCODER_LEFT_POS = 1;
      int ENCODER_LEFT_NEG = 0;
      int ENCODER_RIGHT_POS = 2;
      int ENCODER_RIGHT_NEG = 3;
  }

  private Joystick leftJ = new Joystick(0), rightJ = new Joystick(1);

  public static Encoder right, left;
  public static IMotorController leftVictor, rightVictor, leftTalon, rightTalon;
  public static OdometryDrivetrain drivetrain;
  public static OdometryHandler handler;
  public static ADXRS450_Gyro gyro;
  public static Path path;
  public static PurePursuitController controller;
  public static final double kV = 0.5/3.05, kA = 0.0, kB = 0.0;
  public static FeedForwardController leftController, rightController;

  @Override
  public void robotInit() {
      right = new Encoder(RobotMap.ENCODER_RIGHT_POS, RobotMap.ENCODER_RIGHT_NEG);
      left = new Encoder(RobotMap.ENCODER_LEFT_POS, RobotMap.ENCODER_LEFT_NEG);
      right.setDistancePerPulse(6*0.0254*Math.PI/360);
      left.setDistancePerPulse(6*0.0254*Math.PI/360);
      gyro = new ADXRS450_Gyro();
      drivetrain = new OdometryDrivetrain((WPI_TalonSRX)
              (leftTalon = new WPI_TalonSRX(RobotMap.LEFT_MOTOR_TALON))
              , (WPI_TalonSRX)(rightTalon = new WPI_TalonSRX(RobotMap.RIGHT_MOTOR_TALON))) {
          private OdometryHandler handler = new OdometryHandler(left::getDistance, right::getDistance,
                  gyro::getAngle, 0,0);
          @Override
          public OdometryHandler getHandler() {
              return handler;
          }

          @Override
          public double getWidth() {
              return 0.7;
          }

          @Override
          public void zeroSensors() {
              left.reset();
              right.reset();
              gyro.reset();
          }

          @Override
          public double getLeftRate() {
              return left.getRate();
          }

          @Override
          public double getRightRate() {
              return right.getRate();
          }
      };
      (leftVictor = new WPI_VictorSPX(RobotMap.LEFT_MOTOR_VICTOR)).follow(leftTalon);
      (rightVictor = new WPI_VictorSPX(RobotMap.RIGHT_MOTOR_VICTOR)).follow(rightTalon);
      handler = new OdometryHandler(left::getDistance, right::getDistance,
              gyro::getAngle, 0, 0);
      path = new Path(0.075, 0.6,
              0.6, 3.05, 3, 18, new Waypoint(0, 0), new Waypoint(0, 1),  new Waypoint(-1, 1.5), new Waypoint(-1, 2), new Waypoint(0, 5));
      controller = new PurePursuitController(handler, path, 0.4, 18, 0.7);
      leftController = new FeedForwardController(kV, kA, getPeriod());
      rightController = new FeedForwardController(kV, kA, getPeriod());
      SmartDashboard.putData("autonomous", new FollowPath(drivetrain, path, 0.35,
              new PIDVASettings(kB, 0, 0, kV, kA), 9));
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

    @Override
    public void teleopPeriodic() {

    }

    @Override
    public void autonomousInit() {
        left.reset();
        right.reset();
        gyro.reset();
        handler.set(0,0);
        controller.reset();
    }

  @Override
  public void autonomousPeriodic() {
          double[] speeds = controller.getTargetSpeeds();
          double leftSpeed = leftController.calculate(speeds[0]) +
                  kB * (speeds[0] - left.getRate());
          double rightSpeed = rightController.calculate(speeds[1]) +
                          kB * (speeds[1] - right.getRate());
          drivetrain.tankDrive(leftSpeed, rightSpeed);
          SmartDashboard.putNumber("speed left", speeds[0]);
          SmartDashboard.putNumber("speed right", speeds[1]);
          SmartDashboard.putNumber("speed left converted", leftSpeed);
          SmartDashboard.putNumber("speed right converted", rightSpeed);
  }

    @Override
    public void testPeriodic() {
        CommandScheduler.getInstance().run();
    }
}