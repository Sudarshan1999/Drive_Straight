package org.usfirst.frc.team1165.robot;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;

/**
 * This is a demo program showing how to use Mecanum control with the RobotDrive class.
 */
public class Robot extends SampleRobot {

	private Gyro gyro;
	double kp=3;
	Joystick stick;
	BuiltInAccelerometer accelerometer = new BuiltInAccelerometer();
	// Channels for the wheels
	final int frontLeftChannel	= 0;
	final int frontRightChannel	= 1;
	final int rearLeftChannel	= 2;
	final int rearRightChannel	= 3;

	// The channel on the driver station that the joystick is connected to
	final int joystickChannel	= 0;

	public RobotDrive robotDrive;

	public Robot() {
		robotDrive = new RobotDrive(new Victor(frontLeftChannel), new Victor(frontRightChannel), new Victor(rearLeftChannel), new Victor(rearRightChannel));
		robotDrive.setInvertedMotor(MotorType.kFrontLeft, true);	// invert the left side motors
		robotDrive.setInvertedMotor(MotorType.kRearLeft, true);		// you may need to change or remove this to match your robot
		robotDrive.setExpiration(0.1);
		stick = new Joystick(joystickChannel);
		gyro=new Gyro(0); // Gyro on Analog Channel 0
	}


	/**
	 * Runs the motors with Mecanum drive.
	 */
	public void autonomous() {
	
	
	}
	public void operatorControl() {
		robotDrive.setSafetyEnabled(true);
		gyro.reset();
		gyro.setSensitivity(7.0/1000.0);
		while (isOperatorControl() && isEnabled()) {

			// Drive train jaguars on PWM 1 and 2
			// Use the joystick X axis for lateral movement, Y axis for forward movement, and Z axis for rotation.
			// This sample does not use field-oriented drive, so the gyro input is set to zero.
			robotDrive.mecanumDrive_Cartesian(stick.getY(), stick.getX(), stick.getTwist(),0);
			SmartDashboard.putNumber("Accelerometer X", accelerometer.getX());
			String GyroArray="";
			gyro.reset();
			//while (true) {
				double angle = gyro.getAngle(); // get current heading
				// drive towards heading 0
				//Timer.delay(0.005);
				
				
				SmartDashboard.putNumber("Gyro",angle);
				Timer.delay(0.005);
			//}
				// wait 5ms to avoid hogging CPU cycles
		}
	}

}
