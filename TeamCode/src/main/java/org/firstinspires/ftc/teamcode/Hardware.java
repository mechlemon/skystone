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
    public DcMotor cMech;
    public DcMotor dMech;

    public Servo grabLeft;
    public Servo grabRight;

    private double left_servo_start = 0.5;
    private double right_servo_start = 0.25;

    private double armPosStart = 0;

    public IMU imu;
    public VuforiaPhone vuforiaPhone;


    public Hardware(HardwareMap hardwareMap, Telemetry telemetry){
        drivetrain = new Hdrive(hardwareMap.get(DcMotor.class, "1-0"),
                                hardwareMap.get(DcMotor.class, "1-1"),
                                hardwareMap.get(DcMotor.class, "1-2"),
                                hardwareMap.get(DcMotor.class, "1-3"));
        drivetrain.leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        elevator = hardwareMap.get(DcMotor.class, "2-0");
        arm = hardwareMap.get(DcMotor.class, "2-1");
        cMech = hardwareMap.get(DcMotor.class, "2-2");
        dMech = hardwareMap.get(DcMotor.class, "2-3");

        grabLeft = hardwareMap.get(Servo.class, "grabLeft");
        grabRight = hardwareMap.get(Servo.class, "grabRight");

        left_servo_start = grabLeft.getPosition();
        right_servo_start = grabRight.getPosition();

        armPosStart = arm.getCurrentPosition();

        imu = new IMU(hardwareMap.get(BNO055IMU.class,"imu"));
        imu.initialize();

        vuforiaPhone = new VuforiaPhone(hardwareMap, telemetry);


        telemetry.addData("Status", "Initialized");
        telemetry.update(); //needs to be run every time you send something
    }

}
