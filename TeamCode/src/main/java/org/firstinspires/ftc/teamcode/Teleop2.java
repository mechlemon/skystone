package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


//This annotation tells the driverstation phone about the program.
@TeleOp(name = "Teleop2", group = "comp")

//we extend OpMode which is like a library that gives us functions to interact with the FTC hardware.
public class Teleop2 extends OpMode {


    private double grabPos = 0;

    private String[] titles = new String[] {"forwardCoeff", "turnCoeff", "strafeCoeff", "armCoeff",  "left_servo" , "right_servo"}; //names of the tuner values
    private double[] values = new double[] {     1        ,    0.7     ,     1        ,    0.3    ,        0.5    ,      0.25    }; //default tuner values

    private Tuner tuner;
    private Hardware hardware;

    //the init() function runs when you press the init button on the driverstation before starting.
    //Here, we put things that need to be run right before starting.
    @Override
    public void init() {
        hardware = new Hardware(hardwareMap, telemetry);
        tuner = new Tuner(titles, values, gamepad1, telemetry);
    }

    @Override
    public void loop() {

        //get the constants from the tuner
        tuner.tune();
        double forwardCoeff = tuner.get("forwardCoeff");
        double strafeCoeff = tuner.get("strafeCoeff");
        double turnCoeff = tuner.get("turnCoeff");
        double armCoeff = tuner.get("armCoeff");

        //apply the constants to calculate values
        double forward = -gamepad1.left_stick_y * forwardCoeff; //joysticks usually returns negative for up
        double strafe = -gamepad1.left_stick_x * strafeCoeff; //I think right is positive and left is negative
        double turn = gamepad1.right_stick_x * turnCoeff;


        if(Math.abs(turn) < 0.05){
            hardware.steadyTranslation(strafe, forward);
        }else{
            hardware.drivetrain.setPowers(forward + turn, forward - turn, strafe);
        }


        //servos
        if(gamepad2.x && grabPos < 1){
            grabPos += 0.05;
        }
        if(gamepad2.y && grabPos > -0.5){
            grabPos -= 0.05;
        }

        hardware.setClawPos(grabPos);

        if(gamepad2.dpad_up){ hardware.elevator.setPower(1); }
        else if(gamepad2.dpad_down){ hardware.elevator.setPower(-1); }
        else{ hardware.elevator.setPower(0); }

        hardware.armApplyAntigrav(armCoeff * -gamepad2.left_stick_y);

        hardware.drivetrain.execute();
        telemetry.addData("arm pos", hardware.arm.getCurrentPosition());
        telemetry.addData("L pos", hardware.getLeftDrivePos());
        telemetry.addData("R pos", hardware.getRightDrivePos());
        telemetry.update();

    }



}
