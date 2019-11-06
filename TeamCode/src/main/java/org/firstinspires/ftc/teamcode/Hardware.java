package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

//makes a robot, including motors, servos, imu
public class Hardware {

    final double WHEEL_CIRCUMFERENCE = 4 * Math.PI;
    final double DRIVE_GEAR_RATIO = 24/16.0;
    final double TICKS_PER_REV = 1440; //tetrix encoder

    public Hdrive drivetrain;

    public double left_drive_zero, right_drive_zero, strafe1_drive_zero, strafe2_drive_zero;

    public DcMotor elevator;
    public DcMotor arm;

    public Servo grabLeft;
    public Servo grabRight;
    public Servo grabFoundationLeft;
    public Servo grabFoundationRight;

    private double left_servo_start;
    private double right_servo_start;

    public double armPosStart;

    public IMU imu;
    public VuforiaPhone vuforiaPhone;


    public Hardware(HardwareMap hardwareMap, Telemetry telemetry){
        drivetrain = new Hdrive(hardwareMap.get(DcMotor.class, "1-0"),
                                hardwareMap.get(DcMotor.class, "1-1"),
                                hardwareMap.get(DcMotor.class, "1-2"),
                                hardwareMap.get(DcMotor.class, "1-3"));
        drivetrain.leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        drivetrain.strafeMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        drivetrain.strafeMotor2.setDirection(DcMotorSimple.Direction.REVERSE);


        elevator = hardwareMap.get(DcMotor.class, "2-0");
        arm = hardwareMap.get(DcMotor.class, "2-1");

        // stone intake
        grabLeft = hardwareMap.get(Servo.class, "grabLeft");
        grabRight = hardwareMap.get(Servo.class, "grabRight");
        // foundation grabber
        grabFoundationLeft = hardwareMap.get(Servo.class, "grabFoundationLeft");
        grabFoundationRight = hardwareMap.get(Servo.class, "grabFoundationRight");

        left_servo_start = grabLeft.getPosition();
        right_servo_start = grabRight.getPosition();

        armPosStart = arm.getCurrentPosition();

        imu = new IMU(hardwareMap.get(BNO055IMU.class,"imu"));
        imu.setHeadingAxis(IMU.HeadingAxis.YAW);
        imu.initialize();

        vuforiaPhone = new VuforiaPhone(hardwareMap, telemetry);


        telemetry.addData("Status", "Initialized");
        telemetry.update(); //needs to be run every time you send something
    }

    public void clampStone(){
        grabLeft.setPosition(0);
        grabRight.setPosition(1);
    }

    public void dropStone(){
        grabLeft.setPosition(0.7);
        grabRight.setPosition(0.3);
    }

    public void openClaw(){
        grabLeft.setPosition(1);
        grabRight.setPosition(0);
    }

    public void setClawPos(double pos){
        grabLeft.setPosition(1-pos);
        grabRight.setPosition(pos);
    }



    public void armApplyAntigrav(double power){
        double armPos = (arm.getCurrentPosition() - armPosStart) * (16/24.0) * (360/1440.0) - 80;
        arm.setPower(power + 0.15 * Math.cos(Math.toRadians(armPos)));
    }


    public void clampFoundation(){
        grabFoundationLeft.setPosition(-1);
        grabFoundationRight.setPosition(1);
    }

    public void releaseFoundation(){
        grabFoundationLeft.setPosition(1);
        grabFoundationRight.setPosition(-1);
    }

    public void resetMotors(){
        drivetrain.clear();
        arm.setPower(0);
        elevator.setPower(0);
    }

    public void resetEncoders(){
        left_drive_zero = drivetrain.leftMotor.getCurrentPosition();
        right_drive_zero = drivetrain.rightMotor.getCurrentPosition();
        strafe1_drive_zero = drivetrain.strafeMotor1.getCurrentPosition();
        strafe2_drive_zero = drivetrain.strafeMotor1.getCurrentPosition();
    }

    public double getLeftDrivePos(){ //inches
        return (drivetrain.leftMotor.getCurrentPosition() - left_drive_zero) * DRIVE_GEAR_RATIO * WHEEL_CIRCUMFERENCE * (1/TICKS_PER_REV);
    }
    public double getRightDrivePos(){ //inches
        return (drivetrain.rightMotor.getCurrentPosition() - right_drive_zero) * DRIVE_GEAR_RATIO * WHEEL_CIRCUMFERENCE * (1/TICKS_PER_REV);
    }
    public double getStrafeDrive1Pos(){ //inches
        return (drivetrain.strafeMotor1.getCurrentPosition() - strafe1_drive_zero) * DRIVE_GEAR_RATIO * WHEEL_CIRCUMFERENCE * (1/TICKS_PER_REV);
    }


    public void steadyForward(double magnitude){
        double error = imu.getHeading();
        double turnpower = (1/90.0)*error + Math.copySign(0.1, error);

        drivetrain.setleftPower(magnitude + turnpower);
        drivetrain.setrightPower(magnitude - turnpower);
    }

    public void steadyLeft(double magnitude){
        double error = imu.getHeading();
        double turnpower = (1/90.0)*error + Math.copySign(0.1, error);

        drivetrain.setrightPower(magnitude);
        drivetrain.setleftPower(turnpower);
        drivetrain.setrightPower(-turnpower);
    }

    public void steadyTranslation(double magnitudeX, double magnitudeY){
        double error = imu.getHeading();
        double turnpower = (1/90.0)*error + Math.copySign(0.1, error);

        drivetrain.setstrafePower(magnitudeX);
        drivetrain.setleftPower(magnitudeY + turnpower);
        drivetrain.setrightPower(magnitudeY - turnpower);
    }







}
