/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
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
  public static Waypoint start = new Waypoint(0,0), middle = new Waypoint(2,0),
          end = new Waypoint(1,1);
  public static Path path;
  public static PurePursuitController controller;
  public static final double kV = 0, kA = 0, kB = 0;

  @Override
  public void robotInit() {
      drivetrain = new TankDrivetrain((WPI_TalonSRX)
              (leftTalon = new WPI_TalonSRX(RobotMap.LEFT_MOTOR_TALON))
              , (WPI_TalonSRX)(rightTalon = new WPI_TalonSRX(RobotMap.RIGHT_MOTOR_TALON)));
      (leftVictor = new WPI_VictorSPX(RobotMap.LEFT_MOTOR_VICTOR)).follow(leftTalon);
      (rightVictor = new WPI_VictorSPX(RobotMap.RIGHT_MOTOR_VICTOR)).follow(rightTalon);
      right = new Encoder(RobotMap.ENCODER_RIGHT_POS, RobotMap.ENCODER_RIGHT_NEG);
      left = new Encoder(RobotMap.ENCODER_LEFT_POS, RobotMap.ENCODER_LEFT_NEG);
      gyro = new ADXRS450_Gyro();
      handler = new OdometryHandler(left::getDistance, right::getDistance,
              gyro::getAngle, 0, 0, 0);
      path = new Path(2000, 0.3, 0.7,
              0.04, 1, 3, 1, start,
              middle, end);
      controller = new PurePursuitController(handler, path, 0.2, 0.7);
  }

  @Override
  public void robotPeriodic() {
      handler.calculate();
  }

  @Override
  public void autonomousPeriodic() {
      try {
          double[] speeds = controller.getTargetSpeeds();
          FeedForwardController left = new FeedForwardController(kV, kA, getPeriod());
          FeedForwardController right = new FeedForwardController(kV, kA, getPeriod());
          drivetrain.tankDrive(left.calculate(speeds[0]) +
                  kB * (speeds[0] - leftTalon.getSelectedSensorVelocity(0)),
                  right.calculate(speeds[1]) +
                  kB * (speeds[1] - rightTalon.getSelectedSensorVelocity(0)));
      } catch (LookaheadPointNotFoundException lpnfe) {
          lpnfe.printStackTrace();
      }
  }
}