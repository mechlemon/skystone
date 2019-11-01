package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

//This annotation tells the driverstation phone about the program.
@TeleOp(name = "HdriveWgrav", group = "Joystick Opmode")
@Disabled

//we extend OpMode which is like a library that gives us functions to interact with the FTC hardware.
public class HdriveWgrav extends OpMode {

    //create motor instances, but don't initialize them.
    private DcMotor lDriveMotor = null;
    private DcMotor rDriveMotor = null;
    private DcMotor strafeMotor = null;
    private DcMotor extraMotor = null;

    private DcMotor aMech = null;
    private DcMotor bMech = null;
    private DcMotor cMech = null;
    private DcMotor dMech = null;


    /** Tuner shows up at the bottom of the driverstation phone that
     *  allows you to adjust values while running the program. This is
     *  meant to be temporary in order to test out constants.
     */
    private String[] titles = new String[] {"forwardCoeff", "turnCoeff", "strafeCoeff"}; //names of the tuner values
    private double[] values = new double[] {     0.7      ,    0.2     ,     0.9      }; //default tuner values
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
        strafeMotor = hardwareMap.get(DcMotor.class, "1-2");
        strafeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        extraMotor = hardwareMap.get(DcMotor.class, "1-3");

        aMech = hardwareMap.get(DcMotor.class, "2-0");
        bMech = hardwareMap.get(DcMotor.class, "2-1");
        cMech = hardwareMap.get(DcMotor.class, "2-2");
        dMech = hardwareMap.get(DcMotor.class, "2-3");

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

        double antigrav = 0;

        //75 is 90-c, c is 15 degrees for now.
        double anglestrafe = strafeMotor.getCurrentPosition()-75;

        //strafecoeff (for now) is power needed to hold up at 0 degrees; is mg.

        if((anglestrafe)<0){
            antigrav = Math.sin(anglestrafe)*strafeCoeff;

        }
        if((anglestrafe)>0){
            antigrav = Math.cos(anglestrafe)*strafeCoeff;
        }



        //apply the constants to calculate values
        double forward = -gamepad1.left_stick_y * forwardCoeff; //joysticks usually returns negative for up
        double strafe = gamepad1.left_stick_x * strafeCoeff + antigrav;
        double turn = gamepad1.right_stick_x * turnCoeff;

        //Range.clip() limits the power so that 1.1 is 1 and -1.1 is -1.
        //In this simple drivetrain algorithm, the turning is caused by subtracting or adding
        //the turn from the forward value. This is probably the simplest way to make it work but
        //is usually not very smooth. We could try a constant radius turn later.
        lDriveMotor.setPower(Range.clip(forward - turn,-1,1));
        strafeMotor.setPower(Range.clip(strafe,-1,1));
        rDriveMotor.setPower(Range.clip(forward + turn,-1,1));


        //various buttons that run the motors at ±90% speed for testing
        if(gamepad1.dpad_up){ extraMotor.setPower(0.9); }
        else if(gamepad2.dpad_down){ extraMotor.setPower(-0.9); }
        else{ extraMotor.setPower(0); }

        if(gamepad2.dpad_up){ aMech.setPower(0.9); }
        else if(gamepad2.dpad_down){ aMech.setPower(-0.9); }
        else{ aMech.setPower(0); }

        if(gamepad2.dpad_right){ bMech.setPower(0.9); }
        else if(gamepad2.dpad_left){ bMech.setPower(-0.9); }
        else{ bMech.setPower(0); }

        if(gamepad2.y){ cMech.setPower(0.9); }
        else if(gamepad2.a){ cMech.setPower(-0.9); }
        else{ cMech.setPower(0); }

        if(gamepad2.b){ dMech.setPower(0.9); }
        else if(gamepad2.x){ dMech.setPower(-0.9); }
        else{ dMech.setPower(0); }

        //some debug info sent back
        telemetry.addData("forward",forward); //the power values
        telemetry.addData("strafe",turn);
        telemetry.addData("turn",turn);

        telemetry.addData("left pos",lDriveMotor.getCurrentPosition()); //the encoder position readings
        telemetry.addData("strafe position",strafeMotor.getCurrentPosition());
        telemetry.addData("right pos",rDriveMotor.getCurrentPosition());
        telemetry.update();



    }



}
