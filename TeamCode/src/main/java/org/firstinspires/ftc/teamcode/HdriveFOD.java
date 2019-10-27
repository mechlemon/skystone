package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.bosch.BNO055IMU;



//This annotation tells the driverstation phone about the program.
@TeleOp(name = "HdriveFOD", group = "concept")

//we extend OpMode which is like a library that gives us functions to interact with the FTC hardware.
public class HdriveFOD extends OpMode {

    private Hdrive drivetrain = null;

    private DcMotor elevator = null;
    private DcMotor arm = null;

    private Servo grabLeft = null;
    private Servo grabRight = null;

    private double grabStartPosL = 0;
    private double grabStartPosR = 0;
    private double grabPos = 0;

    private double armPosStart = 0;


    private IMU imu = null;

    /** Tuner shows up at the bottom of the driverstation phone that
     *  allows you to adjust values while running the program. This is
     *  meant to be temporary in order to test out constants.
     */
    private String[] titles = new String[] {"forwardCoeff", "turnCoeff", "strafeCoeff", "elevatorCoeff", "armCoeff", "antigrav" , "left_servo" , "right_servo", "FOD_angle"}; //names of the tuner values
    private double[] values = new double[] {     1        ,    0.7     ,     0.9      ,         1      ,    0.3    ,     0.15   ,      0.5     ,       0.25   ,      0     }; //default tuner values

    private Tuner tuner;

    //the init() function runs when you press the init button on the driverstation before starting.
    //Here, we put things that need to be run right before starting.
    @Override
    public void init() {
        //INITIALIZE
        //MOTORS
        telemetry.addData("Status", "Initializing Motors"); telemetry.update();

        drivetrain = new Hdrive(hardwareMap.get(DcMotor.class, "1-0"),
                                hardwareMap.get(DcMotor.class, "1-1"),
                                hardwareMap.get(DcMotor.class, "1-2"),
                                hardwareMap.get(DcMotor.class, "1-3"));
        drivetrain.clear(); //makes sure motors don't run before hitting start!


        elevator = hardwareMap.get(DcMotor.class, "2-0");

        arm = hardwareMap.get(DcMotor.class, "2-1");
        armPosStart = arm.getCurrentPosition();


        grabLeft = hardwareMap.get(Servo.class, "grabLeft");
        grabRight = hardwareMap.get(Servo.class, "grabRight");

        grabStartPosL = grabLeft.getPosition();
        grabStartPosR = grabRight.getPosition();



        //IMU
        telemetry.addData("Status", "Initializing IMU"); telemetry.update();

        imu = new IMU(hardwareMap.get(BNO055IMU.class,"imu"));
        imu.initialize();
        telemetry.addData("Status", "Initializing DONE"); telemetry.update();



        //initialize the tuner object. Telemetry is a parameter, meaning we send the tuner options
        //though telemetry.
        tuner = new Tuner(titles, values, gamepad1, telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.update(); //needs to be run every time you send something
    }

    @Override
    public void loop() {

        //get the constants from the tuner
        tuner.tune();
        double forwardCoeff = tuner.get("forwardCoeff");
        double strafeCoeff = tuner.get("strafeCoeff");
        double turnCoeff = tuner.get("turnCoeff");
        double armCoeff = tuner.get("armCoeff");
        double antigrav = tuner.get("antigrav");
        double elevatorCoeff = tuner.get("elevatorCoeff");
        double left_servo_start = tuner.get("left_servo");
        double right_servo_start = tuner.get("right_servo");
        double FODangle = tuner.get("FOD_angle") / 100.0;


        //apply the constants to calculate values
        double forward = -gamepad1.left_stick_y * forwardCoeff; //joysticks usually returns negative for up
        double strafe = -gamepad1.left_stick_x * strafeCoeff; //I think right is positive and left is negative
        double turn = -gamepad1.right_stick_x * turnCoeff;

        drivetrain.moveGlobalVector(forward, strafe, imu.getHeading() + FODangle, turn);


        //servos
        if(gamepad2.x && grabPos < 1){
            grabPos += 0.05;
        }
        if(gamepad2.y && grabPos > -1){
            grabPos -= 0.05;
        }

        grabLeft.setPosition(left_servo_start - grabPos);
        grabRight.setPosition(right_servo_start + grabPos);

        if(gamepad2.dpad_up){ elevator.setPower(elevatorCoeff); }
        else if(gamepad2.dpad_down){ elevator.setPower(-elevatorCoeff); }
        else{ elevator.setPower(0); }

        double armPos = (arm.getCurrentPosition() - armPosStart) * (16/24.0) * (360/1440.0) - 80;
        arm.setPower(armCoeff * -gamepad2.left_stick_y + antigrav * Math.cos(Math.toRadians(armPos)));

        telemetry.addData("arm pos",armPos);
        telemetry.addData("heading", imu.getHeading());

        telemetry.update();

    }



}
