package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "swervetest", group = "test")

public class SwerveTest extends OpMode {

    DcMotor leftTop = null;
    DcMotor leftBottom = null;
    DcMotor rightTop = null;
    DcMotor rightBottom = null;

    DcMotor rightEncoder = null;



    @Override
    public void init() {
        leftTop = hardwareMap.get(DcMotor.class, "2-0");
        leftTop.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBottom = hardwareMap.get(DcMotor.class, "2-1");
        leftBottom.setDirection(DcMotorSimple.Direction.FORWARD);
        rightTop = hardwareMap.get(DcMotor.class, "2-2");
        rightTop.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBottom = hardwareMap.get(DcMotor.class, "2-3");
        rightBottom.setDirection(DcMotorSimple.Direction.FORWARD);

        rightEncoder = hardwareMap.get(DcMotor.class, "1-0");
    }

    @Override
    public void loop() {
        double wheelSpin = gamepad1.right_stick_x;
        double moduleSpin = gamepad1.left_stick_y;

        leftTop.setPower(moduleSpin - wheelSpin);
        leftBottom.setPower(moduleSpin + wheelSpin);
        rightTop.setPower(moduleSpin + wheelSpin);
        rightBottom.setPower(moduleSpin - wheelSpin);

        telemetry.addData("LT", leftTop.getCurrentPosition());
        telemetry.addData("LB", leftBottom.getCurrentPosition());
        telemetry.addData("RT", rightTop.getCurrentPosition());
        telemetry.addData("RB", rightBottom.getCurrentPosition());
        telemetry.addData("RE", rightEncoder.getCurrentPosition());
        telemetry.addData( "RMotorAngle", determineAngle(rightTop.getCurrentPosition(), rightBottom.getCurrentPosition()));
        telemetry.addData( "LMotorAngle", determineAngle(leftTop.getCurrentPosition(), leftBottom.getCurrentPosition()));
        telemetry.update();
    }

    public double determineAngle(double motorTop, double motorBottom){

        double gearRatio = 1024;

        double avgMotor = ((motorTop + motorBottom)/2.0);
        double angleValue = (avgMotor/gearRatio) * 360.0;

        return angleValue;
    }



}
