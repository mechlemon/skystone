package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;


//This annotation tells the driverstation phone about the program.
@TeleOp(name = "Teleop1", group = "comp")

//we extend OpMode which is like a library that gives us functions to interact with the FTC hardware.
public class Teleop1 extends OpMode {

    //create motor instances, but don't initialize them.
    private DcMotor lDriveMotor = null;
    private DcMotor rDriveMotor = null;
    private DcMotor strafeMotor1 = null;
    private DcMotor strafeMotor2 = null;
    private DcMotor extraMotor = null;

    private DcMotor elevator = null;
    private DcMotor arm = null;
    private DcMotor cMech = null;
    private DcMotor dMech = null;

    private Servo grabLeft = null;
    private Servo grabRight = null;

    private double grabStartPosL = 0;
    private double grabStartPosR = 0;
    private double grabPos = 0;

    private double armPosStart = 0;




    /** Tuner shows up at the bottom of the driverstation phone that
     *  allows you to adjust values while running the program. This is
     *  meant to be temporary in order to test out constants.
     */
    private String[] titles = new String[] {"forwardCoeff", "turnCoeff", "strafeCoeff", "elevatorCoeff", "armCoeff", "antigrav" , "left_servo" , "right_servo"}; //names of the tuner values
    private double[] values = new double[] {     1        ,    0.7     ,     1      ,         1      ,    0.3    ,     0.15   ,      0.5     ,       0.25    }; //default tuner values

    private Tuner tuner;

    //the init() function runs when you press the init button on the driverstation before starting.
    //Here, we put things that need to be run right before starting.
    @Override
    public void init() {
        //telemetry is like System.out.println() but since this code is run on the robot controller phone,
        //the messages need to be transmitted over to the driver station phone to be displayed. These
        //messages show up on the bottom of the driver station phone.
        telemetry.addData("Status", "Initializing");

        //initialize the motors here and map them to the ports as configured by the robot controller phone.
        //the device name format is <hub number> - <motor port>
        lDriveMotor = hardwareMap.get(DcMotor.class, "1-0");
        lDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE); //may need to flipped based on motor configuration
        rDriveMotor = hardwareMap.get(DcMotor.class, "1-1");
        rDriveMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        strafeMotor1 = hardwareMap.get(DcMotor.class, "1-2");
        strafeMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        strafeMotor2 = hardwareMap.get(DcMotor.class, "1-3");
        strafeMotor2.setDirection(DcMotorSimple.Direction.FORWARD);

        extraMotor = hardwareMap.get(DcMotor.class, "1-3");

        elevator = hardwareMap.get(DcMotor.class, "2-0");
        arm = hardwareMap.get(DcMotor.class, "2-1");
        cMech = hardwareMap.get(DcMotor.class, "2-2");
        dMech = hardwareMap.get(DcMotor.class, "2-3");

        grabLeft = hardwareMap.get(Servo.class, "grabLeft");
        grabRight = hardwareMap.get(Servo.class, "grabRight");

        grabStartPosL = grabLeft.getPosition();
        grabStartPosR = grabRight.getPosition();

        armPosStart = arm.getCurrentPosition();

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

        //apply the constants to calculate values
        double forward = -gamepad1.left_stick_y * forwardCoeff; //joysticks usually returns negative for up
        double strafe = gamepad1.left_stick_x * strafeCoeff; //I think right is positive and left is negative
        double turn = -gamepad1.right_stick_x * turnCoeff;

        //Range.clip() limits the power so that 1.1 is 1 and -1.1 is -1.
        //In this simple drivetrain algorithm, the turning is caused by subtracting or adding
        //the turn from the forward value. This is probably the simplest way to make it work but
        //is usually not very smooth. We could try a constant radius turn later.
        lDriveMotor.setPower(Range.clip(forward - turn,-1,1));
        strafeMotor1.setPower(Range.clip(strafe,-1,1));
        strafeMotor2.setPower(Range.clip(strafe,-1,1));
        rDriveMotor.setPower(Range.clip(forward + turn,-1,1));


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
        telemetry.addData("L pos",lDriveMotor.getCurrentPosition());
        telemetry.addData("R pos",rDriveMotor.getCurrentPosition());

        telemetry.addData("grabLeft pos",grabLeft.getPosition()); //the encoder position readings
        telemetry.addData("grabRight pos",grabRight.getPosition());
        telemetry.update();

    }



}
