package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

//makes a robot, including motors, servos, imu
public class Hardware {

    public Hdrive drivetrain;

    public DcMotor elevator;
    public DcMotor arm;

    public Servo grabLeft;
    public Servo grabRight;
    public Servo grabFoundationLeft;
    public Servo grabFoundationRight;

    private double left_servo_start;
    private double right_servo_start;

    public double armPosStart = 0;

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
        imu.initialize();

        vuforiaPhone = new VuforiaPhone(hardwareMap, telemetry);


        telemetry.addData("Status", "Initialized");
        telemetry.update(); //needs to be run every time you send something
    }

    public void clampStone(){
        grabLeft.setPosition(0.2);
        grabRight.setPosition(0.55);
    }

    public void dropStone(){
        grabLeft.setPosition(0.45);
        grabRight.setPosition(0.3);
    }

    public void openClaw(){
        grabLeft.setPosition(0.6);
        grabRight.setPosition(0.15);
    }

    public void armApplyAntigrav(double power){
        double armPos = (arm.getCurrentPosition() - armPosStart) * (16/24.0) * (360/1440.0) - 80;
        arm.setPower(power + 0.15 * Math.cos(Math.toRadians(armPos)));
    }


    public void clampFoundation(double grabPos){
        grabFoundationLeft.setPosition(grabPos);
        grabFoundationRight.setPosition(-grabPos);
    }

    public void resetMotors(){
        drivetrain.clear();
        arm.setPower(0);
        elevator.setPower(0);
    }

}
